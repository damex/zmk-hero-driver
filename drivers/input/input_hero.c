// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/*
 * Logitech HERO mouse optical sensor. Zephyr input driver (reverse-engineered).
 * SPI: 4 MHz typical (DT-configurable), CS active-low.
 * Addressing: bit7=1 read, bit7=0 write.
 * Motion: read regs 0x05..0x08 = dy_hi, dy_lo, dx_hi, dx_lo (int16, 2's comp).
 *   0x05 must be read first to latch.
 * Blob: a proprietary firmware blob MUST be uploaded via regs 0x2A-0x2F at
 *   power-up, or the sensor won't track.
 * No motion-ready output in run mode, so this driver polls at poll-rate-hz
 *   rather than using an IRQ.
 */

#define DT_DRV_COMPAT logitech_hero

#include <zmk/input/hero.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(input_hero, CONFIG_INPUT_HERO_LOG_LEVEL);

/* SPI framing */
#define HERO_READ_BIT   0x80
#define HERO_WRITE_MASK 0x7F   /* register address with bit 7 cleared = write transaction */
#define HERO_DUMMY_BYTE 0x80   /* MOSI byte clocked out between read frames; stock firmware uses 0x80 */

#define HERO_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA)

/* Chip needs mode 3 (CPOL=1/CPHA=1) MSB-first; trap any future edit that drops a flag.
 * MSB-first is Zephyr's default (SPI_TRANSFER_MSB == 0), so assert LSB isn't set. */
_Static_assert((HERO_SPI_OPERATION & SPI_MODE_CPOL) != 0, "HERO requires CPOL=1");
_Static_assert((HERO_SPI_OPERATION & SPI_MODE_CPHA) != 0, "HERO requires CPHA=1");
_Static_assert((HERO_SPI_OPERATION & SPI_TRANSFER_LSB) == 0, "HERO requires MSB-first");

/* Register addresses (sorted by address) */
#define HERO_REGISTER_MOTION               0x02
#define HERO_REGISTER_MODE                 0x03
#define HERO_REGISTER_MOTION_DY_HIGH       0x05
#define HERO_REGISTER_MOTION_DY_LOW        0x06
#define HERO_REGISTER_MOTION_DX_HIGH       0x07
#define HERO_REGISTER_MOTION_DX_LOW        0x08
#define HERO_REGISTER_POWER_UP             0x0A
#define HERO_REGISTER_DPI_X                0x0C
#define HERO_REGISTER_DPI_Y                0x0D
#define HERO_REGISTER_MAX_FRAME_PERIOD     0x20  /* period = 20us * value, floor 100us; 0x32 = 1000 fps */
#define HERO_REGISTER_RUN_TO_REST_TIMEOUT  0x22  /* timeout = (0.5 * value + 1) s */
#define HERO_REGISTER_BLOB_LOAD            0x2A
#define HERO_REGISTER_BLOB_DATA_A          0x2E
#define HERO_REGISTER_BLOB_DATA_B          0x2F
#define HERO_REGISTER_CALIBRATION_PAGE     0x40
#define HERO_REGISTER_CALIBRATION_TRIGGER  0x43
#define HERO_REGISTER_CALIBRATION_RESULT_A 0x4A
#define HERO_REGISTER_CALIBRATION_RESULT_B 0x4B
#define HERO_REGISTER_CALIBRATION_CONTROL  0x76

/* Register values written by the driver (grouped by feature) */
#define HERO_POWER_UP_RESET              0x40
#define HERO_MOTION_CLEAR                0x80
#define HERO_MODE_RUN_TRANSITIONAL       0x40
#define HERO_MODE_RUN                    0x20
#define HERO_BLOB_LOAD_ENABLE            0xCF
#define HERO_BLOB_LOAD_DONE              0x00
#define HERO_BLOB_FLUSH                  0x80
#define HERO_CALIBRATION_CONTROL_ENABLE  0x88
#define HERO_CALIBRATION_CONTROL_DISABLE 0x00
#define HERO_CALIBRATION_TRIGGER_FIRE    0x01
#define HERO_CALIBRATION_TRIGGER_CLEAR   0x00
#define HERO_CALIBRATION_FIRST_PAGE      0x82
#define HERO_CALIBRATION_LAST_PAGE       0x85

/* Motion read layout (5-byte burst).
 * RX[0] is the slave's pre-address byte; each be16 starts one byte later. */
#define HERO_MOTION_TRANSFER_LENGTH 5
#define HERO_MOTION_DY_OFFSET       1  /* RX[1..2] = dy big-endian */
#define HERO_MOTION_DX_OFFSET       3  /* RX[3..4] = dx big-endian */

/* Conversions */
#define HERO_CPI_REGISTER_STEP     50
#define HERO_CPI_MIN               50
#define HERO_CPI_MAX               12000
#define HERO_FRAME_PERIOD_STEP_US  20  /* reg 0x20 unit: period = value * 20 us */
#define HERO_FRAME_PERIOD_MIN      6   /* 120 us floor; low-rate tracking degrades below */
#define HERO_REST_MIN_SEC          1   /* value 0 = ~1 s */
#define HERO_REST_STEP_PER_SEC     2   /* 0.5 s per reg step */
#define HERO_POLL_INTERVAL_MIN_US  100 /* 10 kHz ceiling: leaves headroom above the SPI floor */

_Static_assert((HERO_CPI_MAX / HERO_CPI_REGISTER_STEP) - 1 <= UINT8_MAX,
               "CPI max overflows the DPI register width");

/* Driver-internal axis bitmap */
#define HERO_AXIS_FLAG_INVERT_X BIT(0)
#define HERO_AXIS_FLAG_INVERT_Y BIT(1)
#define HERO_AXIS_FLAG_SWAP_XY  BIT(2)

struct hero_config {
    struct spi_dt_spec spi;

    /* Chip defaults */
    uint32_t cpi;
    uint32_t poll_rate_hz;
    uint32_t min_frame_rate_hz;
    uint32_t run_to_rest_sec;

    /* Reporting */
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint8_t event_type;

    /* Axis transforms */
    bool swap_xy;
    bool invert_x;
    bool invert_y;

    /* Poll thread */
    k_thread_stack_t *thread_stack;
    size_t thread_stack_size;
};

struct hero_data {
    struct k_thread thread;

    /* Deferred SPI writes: setter stores value + arms atomic; poll thread CAS-claims and writes. */
    uint32_t pending_cpi;
    atomic_t cpi_pending;
    uint8_t pending_frame_period;
    atomic_t frame_period_pending;
    uint8_t pending_rest_period;
    atomic_t rest_period_pending;

    /* Poll-thread parameters (single-word, lock-free) */
    uint32_t poll_interval_us;
    uint16_t x_input_code;
    uint16_t y_input_code;
    uint8_t event_type;

    /* Axis bitmap: aligned 32-bit store/load is hardware-atomic on ARM. */
    uint32_t axis_flags;
};

/* Read-shaped transfer: one CS assertion, equal-length tx/rx. spi_buf.buf lacks
 * const; the cast is safe because tx is read-only on the wire. */
static int hero_xfer(const struct hero_config *config, const uint8_t *tx, uint8_t *rx,
                     size_t len) {
    const struct spi_buf tx_buf = {.buf = (uint8_t *)tx, .len = len};
    const struct spi_buf rx_buf = {.buf = rx, .len = len};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
    return spi_transceive_dt(&config->spi, &tx_set, &rx_set);
}

/* Write under a caller-held CS (SPI_HOLD_ON_CS) for atomic multi-write blocks.
 * Use hero_write for normal single-write transactions. */
static int hero_write_held(const struct hero_config *config, const struct spi_config *spi_config,
                           uint8_t register_address, uint8_t value) {
    __ASSERT(register_address < HERO_READ_BIT, "write register %#x must have bit 7 clear",
             register_address);
    const uint8_t tx[2] = {register_address & HERO_WRITE_MASK, value};
    const struct spi_buf tx_buf = {.buf = (uint8_t *)tx, .len = sizeof(tx)};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    return spi_transceive(config->spi.bus, spi_config, &tx_set, NULL);
}

static int hero_write(const struct hero_config *config, uint8_t register_address, uint8_t value) {
    return hero_write_held(config, &config->spi.config, register_address, value);
}

static int hero_read(const struct hero_config *config, uint8_t register_address, uint8_t *value) {
    const uint8_t tx[2] = {register_address | HERO_READ_BIT, HERO_DUMMY_BYTE};
    uint8_t rx[2] = {0};
    int error = hero_xfer(config, tx, rx, sizeof(tx));
    if (error < 0) {
        return error;
    }
    *value = rx[1];
    return 0;
}

/* Value discarded; the read is needed to advance the chip's state machine. */
static int hero_read_discard(const struct hero_config *config, uint8_t register_address) {
    uint8_t scratch;
    return hero_read(config, register_address, &scratch);
}

static int hero_read_motion(const struct hero_config *config, int16_t *delta_x, int16_t *delta_y) {
    const uint8_t tx[HERO_MOTION_TRANSFER_LENGTH] = {
        HERO_REGISTER_MOTION_DY_HIGH | HERO_READ_BIT,
        HERO_REGISTER_MOTION_DY_LOW | HERO_READ_BIT,
        HERO_REGISTER_MOTION_DX_HIGH | HERO_READ_BIT,
        HERO_REGISTER_MOTION_DX_LOW | HERO_READ_BIT,
        HERO_DUMMY_BYTE,
    };
    uint8_t rx[HERO_MOTION_TRANSFER_LENGTH] = {0};
    int error = hero_xfer(config, tx, rx, sizeof(tx));
    if (error < 0) {
        return error;
    }
    *delta_y = (int16_t)sys_get_be16(&rx[HERO_MOTION_DY_OFFSET]);
    *delta_x = (int16_t)sys_get_be16(&rx[HERO_MOTION_DX_OFFSET]);
    return 0;
}

/* Sample discarded; the read is needed to advance the chip's state machine. */
static int hero_read_motion_discard(const struct hero_config *config) {
    int16_t delta_x;
    int16_t delta_y;
    return hero_read_motion(config, &delta_x, &delta_y);
}

static int hero_set_cpi_registers(const struct hero_config *config, uint32_t cpi) {
    if (cpi < HERO_CPI_MIN || cpi > HERO_CPI_MAX) {
        LOG_WRN("cpi %u out of range [%u, %u]", cpi, HERO_CPI_MIN, HERO_CPI_MAX);
        return -EINVAL;
    }
    const uint8_t value = (uint8_t)((cpi / HERO_CPI_REGISTER_STEP) - 1);
    int error = hero_write(config, HERO_REGISTER_DPI_X, value);
    if (error < 0) {
        return error;
    }
    return hero_write(config, HERO_REGISTER_DPI_Y, value);
}

/* Higher rate = shorter period = smaller register value; clamped to range. */
static uint8_t hero_min_frame_rate_to_period(uint32_t hz) {
    if (hz == 0) {
        return UINT8_MAX;
    }
    const uint32_t value = USEC_PER_SEC / (HERO_FRAME_PERIOD_STEP_US * hz);
    return (uint8_t)CLAMP(value, HERO_FRAME_PERIOD_MIN, UINT8_MAX);
}

/* Convert poll rate Hz to interval microseconds; clamp to the SPI floor. */
static uint32_t hero_poll_rate_to_interval_us(uint32_t hz) {
    if (hz == 0) {
        return UINT32_MAX;
    }
    return MAX(USEC_PER_SEC / hz, (uint32_t)HERO_POLL_INTERVAL_MIN_US);
}

/* Sensor enters low-power rest after this many seconds of inactivity; reg step is 0.5 s. */
static uint8_t hero_rest_seconds_to_register(uint32_t seconds) {
    if (seconds <= HERO_REST_MIN_SEC) {
        return 0;
    }
    return (uint8_t)MIN((seconds - HERO_REST_MIN_SEC) * HERO_REST_STEP_PER_SEC,
                        (uint32_t)UINT8_MAX);
}

/* Sensor needs a read before the mode write to latch the state. */
static int hero_set_mode(const struct hero_config *config, uint8_t mode) {
    int error = hero_read_discard(config, HERO_REGISTER_MODE);
    if (error < 0) {
        return error;
    }
    return hero_write(config, HERO_REGISTER_MODE, mode);
}

struct hero_register_value {
    uint8_t register_address;
    uint8_t value;
};

/* Write a table of register/value pairs, one CS per write. */
static int hero_write_registers(const struct hero_config *config,
                                const struct hero_register_value *registers, size_t count) {
    for (size_t entry = 0; entry < count; entry++) {
        int error = hero_write(config, registers[entry].register_address,
                               registers[entry].value);
        if (error < 0) {
            return error;
        }
    }
    return 0;
}

/* Defined in sensor_blob.S via .incbin of sensor.bin. */
extern const uint8_t sensor_blob[];
extern const uint8_t sensor_blob_end[];
#define SENSOR_BLOB_SIZE      ((size_t)(sensor_blob_end - sensor_blob))
#define HERO_BLOB_CHUNK_BYTES 256

/* Interleave [0x2E, blob[0], 0x2F, blob[1], ...] and send as one transceive. */
static int hero_blob_write_chunk(const struct hero_config *config,
                                 const struct spi_config *held, const uint8_t *blob,
                                 size_t bytes) {
    __ASSERT(bytes > 0 && bytes <= HERO_BLOB_CHUNK_BYTES,
             "blob chunk out of bounds: %u (max %u)", (unsigned)bytes,
             (unsigned)HERO_BLOB_CHUNK_BYTES);
    __ASSERT(bytes % 2 == 0, "blob chunk must be 2-byte aligned, got %u", (unsigned)bytes);
    uint8_t buffer[HERO_BLOB_CHUNK_BYTES * 2];
    size_t buffer_position = 0;
    for (size_t blob_offset = 0; blob_offset < bytes; blob_offset += 2) {
        buffer[buffer_position++] = HERO_REGISTER_BLOB_DATA_A & HERO_WRITE_MASK;
        buffer[buffer_position++] = blob[blob_offset];
        buffer[buffer_position++] = HERO_REGISTER_BLOB_DATA_B & HERO_WRITE_MASK;
        buffer[buffer_position++] = blob[blob_offset + 1];
    }
    const struct spi_buf tx_buf = {.buf = buffer, .len = buffer_position};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    return spi_transceive(config->spi.bus, held, &tx_set, NULL);
}

/* Loader pre-config regs 0x2A-0x2D: arm the loader before the data stream. */
static const struct hero_register_value hero_blob_preload[] = {
    {HERO_REGISTER_BLOB_LOAD, HERO_BLOB_LOAD_ENABLE},
    {0x2B, 0x06},
    {0x2C, 0x08},
    {0x2D, 0x00},
};

/* Single 0x80 byte under the held CS terminates the blob-load stream. */
static int hero_blob_send_flush(const struct hero_config *config, const struct spi_config *held) {
    const uint8_t flush = HERO_BLOB_FLUSH;
    const struct spi_buf tx_buf = {.buf = (uint8_t *)&flush, .len = sizeof(flush)};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    return spi_transceive(config->spi.bus, held, &tx_set, NULL);
}

/* Upload the sensor blob. The loader needs the whole stream under ONE held CS
 * (SPI_HOLD_ON_CS); a per-write CS toggle corrupts it and the sensor never tracks. */
static int hero_upload_blob(const struct hero_config *config) {
    struct spi_config held = config->spi.config;
    held.operation |= SPI_HOLD_ON_CS | SPI_LOCK_ON;

    int error = 0;
    for (size_t entry = 0; entry < ARRAY_SIZE(hero_blob_preload); entry++) {
        error = hero_write_held(config, &held, hero_blob_preload[entry].register_address,
                                hero_blob_preload[entry].value);
        if (error < 0) {
            goto out_release;
        }
    }
    for (size_t offset = 0; offset < SENSOR_BLOB_SIZE; offset += HERO_BLOB_CHUNK_BYTES) {
        const size_t chunk = MIN(SENSOR_BLOB_SIZE - offset, (size_t)HERO_BLOB_CHUNK_BYTES);
        error = hero_blob_write_chunk(config, &held, &sensor_blob[offset], chunk);
        if (error < 0) {
            goto out_release;
        }
    }
    error = hero_blob_send_flush(config, &held);
out_release:
    spi_release(config->spi.bus, &held);
    return error;
}

/* Performance regs 0x30-0x35: tuning constants applied after calibration. */
static const struct hero_register_value hero_performance_init[] = {
    {0x30, 0x17},
    {0x31, 0x40},
    {0x32, 0x09},
    {0x33, 0x09},
    {0x34, 0x0F},
    {0x35, 0x0B},
};

/* Run-mode head: regs 0x0B, 0x0E. Precedes the rest-timeout write. */
static const struct hero_register_value hero_run_configuration_head[] = {
    {0x0B, 0x40},
    {0x0E, 0x11},
};

/* Run-mode tail: regs 0x23-0x29 and 0x64. Sensor reports motion after this. */
static const struct hero_register_value hero_run_configuration_tail[] = {
    {0x23, 0x97},
    {0x24, 0xB6},
    {0x25, 0x36},
    {0x26, 0xDA},
    {0x27, 0x50},
    {0x28, 0xC4},
    {0x29, 0x00},
    {0x64, 0x06},
};

/* One calibration page: enable, select page, fire+clear trigger, read both
 * result registers (read clocks the state machine), disable. */
static int hero_calibrate_page(const struct hero_config *config, uint8_t page) {
    int error = hero_write(config, HERO_REGISTER_CALIBRATION_CONTROL, HERO_CALIBRATION_CONTROL_ENABLE);
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_CALIBRATION_PAGE, page);
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_CALIBRATION_TRIGGER, HERO_CALIBRATION_TRIGGER_FIRE);
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_CALIBRATION_TRIGGER, HERO_CALIBRATION_TRIGGER_CLEAR);
    if (error < 0) {
        return error;
    }
    error = hero_read_discard(config, HERO_REGISTER_CALIBRATION_RESULT_A);
    if (error < 0) {
        return error;
    }
    error = hero_read_discard(config, HERO_REGISTER_CALIBRATION_RESULT_B);
    if (error < 0) {
        return error;
    }
    return hero_write(config, HERO_REGISTER_CALIBRATION_CONTROL, HERO_CALIBRATION_CONTROL_DISABLE);
}

static int hero_power_on(const struct hero_config *config) {
    int error = hero_write(config, HERO_REGISTER_POWER_UP, HERO_POWER_UP_RESET);
    if (error < 0) {
        return error;
    }
    /* Blob upload is mandatory for tracking. */
    error = hero_upload_blob(config);
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_BLOB_LOAD, HERO_BLOB_LOAD_DONE);
    if (error < 0) {
        return error;
    }
    for (uint8_t page = HERO_CALIBRATION_FIRST_PAGE; page <= HERO_CALIBRATION_LAST_PAGE; page++) {
        error = hero_calibrate_page(config, page);
        if (error < 0) {
            return error;
        }
    }
    return hero_write_registers(config, hero_performance_init, ARRAY_SIZE(hero_performance_init));
}

/* Clear motion, drop into the transitional mode, apply CPI, then discard the
 * sample latched during the state change. Pairs with hero_finalize_run below. */
static int hero_arm_run(const struct hero_config *config) {
    int error = hero_read_discard(config, HERO_REGISTER_MOTION);
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_MOTION, HERO_MOTION_CLEAR);
    if (error < 0) {
        return error;
    }
    error = hero_set_mode(config, HERO_MODE_RUN_TRANSITIONAL);
    if (error < 0) {
        return error;
    }
    error = hero_set_cpi_registers(config, config->cpi);
    if (error < 0) {
        return error;
    }
    return hero_read_motion_discard(config);
}

/* Promote to full run mode, write the run-config tables and rest timeout,
 * discard the post-write sample, and program the max frame period. */
static int hero_finalize_run(const struct hero_config *config) {
    int error = hero_write(config, HERO_REGISTER_MODE, HERO_MODE_RUN);
    if (error < 0) {
        return error;
    }
    error = hero_write_registers(config, hero_run_configuration_head,
                                 ARRAY_SIZE(hero_run_configuration_head));
    if (error < 0) {
        return error;
    }
    error = hero_write(config, HERO_REGISTER_RUN_TO_REST_TIMEOUT,
                       hero_rest_seconds_to_register(config->run_to_rest_sec));
    if (error < 0) {
        return error;
    }
    error = hero_write_registers(config, hero_run_configuration_tail,
                                 ARRAY_SIZE(hero_run_configuration_tail));
    if (error < 0) {
        return error;
    }
    error = hero_read_motion_discard(config);
    if (error < 0) {
        return error;
    }
    return hero_write(config, HERO_REGISTER_MAX_FRAME_PERIOD,
                      hero_min_frame_rate_to_period(config->min_frame_rate_hz));
}

static int hero_enter_run(const struct hero_config *config) {
    int error = hero_arm_run(config);
    if (error < 0) {
        return error;
    }
    return hero_finalize_run(config);
}

static int hero_configure(const struct device *dev) {
    const struct hero_config *config = dev->config;

    LOG_INF("HERO configure: start");
    int error = hero_power_on(config);
    if (error < 0) {
        LOG_ERR("HERO power-on failed (%d)", error);
        return error;
    }
    error = hero_enter_run(config);
    if (error < 0) {
        LOG_ERR("HERO enter-run failed (%d)", error);
        return error;
    }
    LOG_INF("HERO configure: done");
    return 0;
}

/* Non-SPI setters: the poll thread reads each value once per iteration. */

/* Three bools packed into one uint32 so the poll thread reads the full set
 * atomically (no half-updated observation). */
void hero_set_axis(const struct device *dev, bool invert_x, bool invert_y, bool swap_xy) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    uint32_t flags = 0;
    if (invert_x) {
        flags |= HERO_AXIS_FLAG_INVERT_X;
    }
    if (invert_y) {
        flags |= HERO_AXIS_FLAG_INVERT_Y;
    }
    if (swap_xy) {
        flags |= HERO_AXIS_FLAG_SWAP_XY;
    }
    data->axis_flags = flags;
}

void hero_set_report_rate(const struct device *dev, uint32_t hz) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    if (hz == 0) {
        LOG_WRN("report rate 0 ignored");
        return;
    }
    data->poll_interval_us = hero_poll_rate_to_interval_us(hz);
}

void hero_set_event_type(const struct device *dev, uint8_t event_type) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    data->event_type = event_type;
}

void hero_set_x_code(const struct device *dev, uint16_t code) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    data->x_input_code = code;
}

void hero_set_y_code(const struct device *dev, uint16_t code) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    data->y_input_code = code;
}

/* Always arm the atomic; skip-if-equal would sticky-suppress retries after SPI failure. */

void hero_set_min_frame_rate(const struct device *dev, uint32_t hz) {
    __ASSERT_NO_MSG(dev != NULL);
    if (hz == 0) {
        LOG_WRN("min frame rate 0 ignored");
        return;
    }
    struct hero_data *data = dev->data;
    data->pending_frame_period = hero_min_frame_rate_to_period(hz);
    atomic_set(&data->frame_period_pending, 1);
}

void hero_set_rest_timeout(const struct device *dev, uint32_t seconds) {
    __ASSERT_NO_MSG(dev != NULL);
    struct hero_data *data = dev->data;
    data->pending_rest_period = hero_rest_seconds_to_register(seconds);
    atomic_set(&data->rest_period_pending, 1);
}

void hero_set_cpi(const struct device *dev, uint32_t cpi) {
    __ASSERT_NO_MSG(dev != NULL);
    if (cpi < HERO_CPI_MIN || cpi > HERO_CPI_MAX) {
        LOG_WRN("cpi %u out of range [%u, %u]", cpi, HERO_CPI_MIN, HERO_CPI_MAX);
        return;
    }
    struct hero_data *data = dev->data;
    data->pending_cpi = cpi;
    atomic_set(&data->cpi_pending, 1);
}

/* Read motion, apply axis transforms, report. No-op on read failure or no movement. */
static void hero_emit_motion(const struct device *dev) {
    const struct hero_config *config = dev->config;
    struct hero_data *data = dev->data;

    int16_t delta_x = 0;
    int16_t delta_y = 0;
    if (hero_read_motion(config, &delta_x, &delta_y) != 0 || (delta_x == 0 && delta_y == 0)) {
        return;
    }
    const uint32_t axis_flags = data->axis_flags;
    if (axis_flags & HERO_AXIS_FLAG_SWAP_XY) {
        int16_t original_delta_x = delta_x;
        delta_x = delta_y;
        delta_y = original_delta_x;
    }
    if (axis_flags & HERO_AXIS_FLAG_INVERT_X) {
        delta_x = (int16_t)-delta_x;
    }
    if (axis_flags & HERO_AXIS_FLAG_INVERT_Y) {
        delta_y = (int16_t)-delta_y;
    }
    input_report(dev, data->event_type, data->x_input_code, delta_x, false, K_NO_WAIT);
    input_report(dev, data->event_type, data->y_input_code, delta_y, true, K_NO_WAIT);
}

/* Own thread: init SPI + logging need far more stack than the shared system
 * workqueue's, and a stuck SPI here must not wedge the rest. */
static void hero_thread(void *device_handle, void *unused_param_1, void *unused_param_2) {
    ARG_UNUSED(unused_param_1);
    ARG_UNUSED(unused_param_2);
    const struct device *dev = device_handle;
    const struct hero_config *config = dev->config;
    struct hero_data *data = dev->data;

    if (hero_configure(dev) != 0) {
        LOG_ERR("HERO configure failed");
        return;
    }
    LOG_INF("HERO ready, polling every %u us, cpi %u", data->poll_interval_us, data->pending_cpi);

    while (1) {
        if (atomic_cas(&data->cpi_pending, 1, 0) &&
            hero_set_cpi_registers(config, data->pending_cpi) < 0) {
            LOG_DBG("cpi write failed");
        }
        if (atomic_cas(&data->frame_period_pending, 1, 0) &&
            hero_write(config, HERO_REGISTER_MAX_FRAME_PERIOD, data->pending_frame_period) < 0) {
            LOG_DBG("frame-period write failed");
        }
        if (atomic_cas(&data->rest_period_pending, 1, 0) &&
            hero_write(config, HERO_REGISTER_RUN_TO_REST_TIMEOUT, data->pending_rest_period) < 0) {
            LOG_DBG("rest-period write failed");
        }
        hero_emit_motion(dev);
        k_sleep(K_USEC(data->poll_interval_us));
    }
}

static int hero_init(const struct device *dev) {
    const struct hero_config *config = dev->config;
    struct hero_data *data = dev->data;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    hero_set_axis(dev, config->invert_x, config->invert_y, config->swap_xy);
    data->poll_interval_us = hero_poll_rate_to_interval_us(config->poll_rate_hz);
    data->event_type = config->event_type;
    data->x_input_code = config->x_input_code;
    data->y_input_code = config->y_input_code;
    /* Seed the cache; hero_enter_run does the chip writes. */
    data->pending_cpi = config->cpi;
    data->pending_frame_period = hero_min_frame_rate_to_period(config->min_frame_rate_hz);
    data->pending_rest_period = hero_rest_seconds_to_register(config->run_to_rest_sec);

    k_thread_create(&data->thread, config->thread_stack, config->thread_stack_size, hero_thread,
                    (void *)dev, NULL, NULL, CONFIG_INPUT_HERO_THREAD_PRIORITY, 0,
                    K_NO_WAIT);
    return 0;
}

#define HERO_INST(instance)                                                                         \
    K_THREAD_STACK_DEFINE(hero_thread_stack_##instance, CONFIG_INPUT_HERO_THREAD_STACK_SIZE);       \
    static struct hero_data hero_data_##instance;                                                   \
    static const struct hero_config hero_config_##instance = {                                      \
        .spi = SPI_DT_SPEC_INST_GET(instance, HERO_SPI_OPERATION, 0),                               \
        .cpi = DT_INST_PROP(instance, cpi),                                                         \
        .poll_rate_hz = DT_INST_PROP(instance, poll_rate_hz),                                       \
        .min_frame_rate_hz = DT_INST_PROP(instance, min_frame_rate_hz),                             \
        .run_to_rest_sec = DT_INST_PROP(instance, run_to_rest_sec),                                 \
        .event_type = DT_INST_PROP(instance, event_type),                                           \
        .x_input_code = DT_INST_PROP(instance, x_input_code),                                       \
        .y_input_code = DT_INST_PROP(instance, y_input_code),                                       \
        .swap_xy = DT_INST_PROP(instance, swap_xy),                                                 \
        .invert_x = DT_INST_PROP(instance, invert_x),                                               \
        .invert_y = DT_INST_PROP(instance, invert_y),                                               \
        .thread_stack = hero_thread_stack_##instance,                                               \
        .thread_stack_size = K_THREAD_STACK_SIZEOF(hero_thread_stack_##instance),                   \
    };                                                                                              \
    DEVICE_DT_INST_DEFINE(instance, hero_init, NULL, &hero_data_##instance,                         \
                          &hero_config_##instance, POST_KERNEL,                                     \
                          CONFIG_INPUT_HERO_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HERO_INST)
