// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include "hero_timing.h"

#include <zephyr/sys/util.h>
#include <zephyr/sys_clock.h>

/* Higher rate = shorter period = smaller register value, clamped to range. */
uint8_t hero_min_frame_rate_to_period(uint32_t hz) {
    if (hz == 0) {
        return UINT8_MAX;
    }
    const uint32_t value = USEC_PER_SEC / (HERO_FRAME_PERIOD_STEP_US * hz);
    return (uint8_t)CLAMP(value, HERO_FRAME_PERIOD_MIN, UINT8_MAX);
}

/* Clamped to SPI floor. */
uint32_t hero_poll_rate_to_interval_us(uint32_t hz) {
    if (hz == 0) {
        return UINT32_MAX;
    }
    return MAX(USEC_PER_SEC / hz, (uint32_t)HERO_POLL_INTERVAL_MIN_US);
}

uint8_t hero_rest_seconds_to_register(uint32_t seconds) {
    if (seconds <= HERO_REST_MIN_SEC) {
        return 0;
    }
    return (uint8_t)MIN((seconds - HERO_REST_MIN_SEC) * HERO_REST_STEP_PER_SEC,
                        (uint32_t)UINT8_MAX);
}
