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
| `cpi` | `1000` | Resolution, 50-12000 (step 50). |
| `poll-rate-hz` | `1000` | Motion poll rate. Clamped to 10000 Hz (100 us SPI floor); rounds down for non-divisor rates. |
| `min-frame-rate-hz` | `1000` | Frame-rate floor when idle. Settable ~32-8333 Hz; see below for the auto-scale behavior. |
| `run-to-rest-sec` | `6` | Inactivity seconds before rest mode. |
| `event-type` | required | Input event type (e.g. `INPUT_EV_REL`). |
| `x-input-code` | required | Input code for X. |
| `y-input-code` | required | Input code for Y. |
| `swap-xy` | off | Swap X/Y at report time. |
| `invert-x` | off | Negate X at report time. |
| `invert-y` | off | Negate Y at report time. |

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

`<zmk/input/hero.h>` declares runtime setters callable from any thread:

| Setter | Sets |
|---|---|
| `hero_set_cpi` | CPI (50-12000, step 50) |
| `hero_set_axis` | invert-X, invert-Y, swap-X/Y flags |
| `hero_set_report_rate` | poll rate in Hz (capped to 10 kHz) |
| `hero_set_min_frame_rate` | frame-rate floor in Hz (capped to ~8333) |
| `hero_set_rest_timeout` | inactivity seconds before rest mode |
| `hero_set_event_type` | input event type |
| `hero_set_x_code` | input code for X axis |
| `hero_set_y_code` | input code for Y axis |

## License

This module is MIT.

Dependencies (each keeps its own license):

| Dependency | License |
|---|---|
| ZMK | MIT |
| Zephyr | Apache-2.0 |

The HERO sensor blob (`sensor.bin`) is Logitech proprietary firmware. Users
supply their own dump; this repository does not redistribute it.
