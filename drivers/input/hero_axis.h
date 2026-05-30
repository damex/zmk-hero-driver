// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#include <zephyr/sys/util.h>

#define HERO_AXIS_FLAG_INVERT_X BIT(0)
#define HERO_AXIS_FLAG_INVERT_Y BIT(1)
#define HERO_AXIS_FLAG_SWAP_XY  BIT(2)

void hero_apply_axis_transform(uint32_t axis_flags, int16_t *delta_x, int16_t *delta_y);
