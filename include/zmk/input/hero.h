// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/* Public control API for the HERO sensor driver. All setters are callable from any thread. */
#pragma once

#include <stdint.h>

#include <zephyr/device.h>

void hero_set_cpi(const struct device *dev, uint32_t cpi_x, uint32_t cpi_y);
void hero_set_report_rate(const struct device *dev, uint32_t hz);
void hero_set_min_frame_rate(const struct device *dev, uint32_t hz);
void hero_set_x_code(const struct device *dev, uint16_t code);
void hero_set_y_code(const struct device *dev, uint16_t code);
void hero_set_rest_timeout(const struct device *dev, uint32_t seconds);
void hero_park(const struct device *dev);
void hero_unpark(const struct device *dev);
