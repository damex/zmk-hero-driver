# zmk-hero-driver

Zephyr input driver for the Logitech **HERO** mouse sensor (reverse-engineered).

## Verified sensors

| Sensor | Found in |
|---|---|
| HERO Hg11 (VBQ32917) | Logitech G305, G603 |

## Install

Add it to your `config/west.yml`:
```yaml
  remotes:
    - name: damex
      url-base: https://github.com/damex
  projects:
    - name: zmk-hero-driver
      remote: damex
      revision: v0.1.0
```
Then:
```
west update
```
For a local checkout, build with `-DZMK_EXTRA_MODULES=<path>/zmk-hero-driver`
instead.

## `sensor.bin`

Drop your own dump as `sensor.bin` in this directory. It is gitignored;
missing it is a hard CMake error.

## Configure

### Devicetree (`logitech,hero`)

| property | default | description |
| --- | --- | --- |
| `cpi` | `1000` | Resolution, 50-12000 (step 50). Applies to both axes unless overridden. |
| `cpi-x` | `cpi` | Per-axis X resolution override. |
| `cpi-y` | `cpi` | Per-axis Y resolution override. |
| `poll-rate-hz` | `1000` | Motion poll rate. Clamped to 10000 Hz (100 us SPI floor); rounds down for non-divisor rates. |
| `poll-timer` | required | Hardware timer (`nordic,nrf-timer` counter) that paces the poll. |
| `min-frame-rate-hz` | `1000` | Frame-rate floor when idle. Settable ~32-8333 Hz; see below for the auto-scale behavior. |
| `run-to-rest-sec` | `6` | Inactivity seconds before rest mode. |
| `x-input-code` | required | Input code for X. |
| `y-input-code` | required | Input code for Y. |

### Axis orientation

Driver emits raw sensor axes. Fix mounting (invert X/Y, swap X/Y) with ZMK
[input-processor-transform](https://zmk.dev/docs/keymaps/input-processors/transformer),
attached where sensor events are consumed. Combine flags with `|`:
`INPUT_TRANSFORM_X_INVERT`, `INPUT_TRANSFORM_Y_INVERT`, `INPUT_TRANSFORM_XY_SWAP`.

Both forms need includes:

```dts
#include <dt-bindings/zmk/input_transform.h>
#include <input/processors.dtsi>
```

Standalone board, on the input listener:

```dts
hero_listener {
    compatible = "zmk,input-listener";
    device = <&hero>;
    input-processors = <&zip_xy_transform (INPUT_TRANSFORM_Y_INVERT | INPUT_TRANSFORM_XY_SWAP)>;
};
```

Split, on the peripheral's `zmk,input-split` node (transforms before forwarding):

```dts
&your_input_split {
    input-processors = <&zip_xy_transform INPUT_TRANSFORM_Y_INVERT>;
};
```

### Frame rate

`min-frame-rate-hz` is the floor. Sensor auto-scales up to ~12000 Hz under
motion. Higher floor = better tracking at idle, more current draw. Max useful
value 8333 Hz (register 6); higher writes degrade slow-speed tracking.

Upper rate is not exposed; sensor firmware decides.

### Limitations

No motion-ready IRQ. HERO does not drive MISO (or any other pin) to signal
motion in run mode; MISO sits at idle level, no chip-driven edges. The driver
polls at `poll-rate-hz`.

### C API

`<zmk/input/hero.h>` declares runtime control functions callable from any thread:

| Function | Effect |
|---|---|
| `hero_set_cpi` | per-axis CPI: X then Y (each 50-12000, step 50) |
| `hero_set_report_rate` | poll rate in Hz (capped to 10 kHz) |
| `hero_set_min_frame_rate` | frame-rate floor in Hz (capped to ~8333) |
| `hero_set_rest_timeout` | inactivity seconds before rest mode |
| `hero_set_x_code` | input code for X axis |
| `hero_set_y_code` | input code for Y axis |
| `hero_park` | deepsleep + stop poll thread |
| `hero_unpark` | wake + resume poll thread |

When `CONFIG_PM_DEVICE=y`, the driver also implements Zephyr device PM:
`PM_DEVICE_ACTION_SUSPEND` calls `hero_park`, `PM_DEVICE_ACTION_RESUME` calls
`hero_unpark`. No effect otherwise.

## Load order

Each step overrides the previous:

1. **Boot defaults:** the DT props program the sensor at startup.
2. **Saved values:** any persisted in NVS apply on top (needs `CONFIG_SETTINGS`).
3. **Live changes:** pushed at runtime via `settings_runtime_set`, no reflash.

Runtime-overridable keys: `hero/rate`, `hero/min_frame_rate`, `hero/x_code`,
`hero/y_code`, `hero/rest_timeout`.

## License

This module is MIT.

Dependencies (each keeps its own license):

| Dependency | License |
|---|---|
| ZMK | MIT |
| Zephyr | Apache-2.0 |

The HERO sensor blob (`sensor.bin`) is Logitech proprietary firmware. Users
supply their own dump; this repository does not redistribute it.
