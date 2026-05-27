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
| `poll-interval-us` | `1000` | Motion poll period in microseconds (`1000` = 1000 Hz). |
| `max-frame-rate` | `1000` | Sensor frame rate in fps. |
| `run-to-rest-sec` | `6` | Inactivity seconds before rest mode. |
| `event-type` | required | Input event type (e.g. `INPUT_EV_REL`). |
| `x-input-code` | required | Input code for X. |
| `y-input-code` | required | Input code for Y. |
| `swap-xy` | off | Swap X/Y at report time. |
| `invert-x` | off | Negate X at report time. |
| `invert-y` | off | Negate Y at report time. |

### C API

`<zmk/input/hero.h>` declares runtime setters callable from any thread:

| Setter | Sets |
|---|---|
| `hero_set_cpi` | CPI (50-12000, step 50) |
| `hero_set_axis` | invert-X, invert-Y, swap-X/Y flags |
| `hero_set_report_rate` | poll rate in Hz (capped to 10 kHz) |
| `hero_set_frame_rate` | sensor frame rate (fps) |
| `hero_set_rest_timeout` | inactivity seconds before rest mode |
| `hero_set_event_type` | input event type |
| `hero_set_x_code` | input code for X axis |
| `hero_set_y_code` | input code for Y axis |
