// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/* Public control API for the HERO sensor driver. All setters are callable from any thread. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

void hero_set_cpi(const struct device *dev, uint32_t cpi);
void hero_set_axis(const struct device *dev, bool invert_x, bool invert_y, bool swap_xy);
void hero_set_report_rate(const struct device *dev, uint32_t hz);
void hero_set_frame_rate(const struct device *dev, uint32_t hz);
void hero_set_event_type(const struct device *dev, uint8_t event_type);
void hero_set_x_code(const struct device *dev, uint16_t code);
void hero_set_y_code(const struct device *dev, uint16_t code);
void hero_set_rest_timeout(const struct device *dev, uint32_t seconds);
