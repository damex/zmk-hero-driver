// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include "hero_axis.h"

#include <zephyr/sys/__assert.h>

void hero_apply_axis_transform(uint32_t axis_flags, int16_t *delta_x, int16_t *delta_y) {
    __ASSERT_NO_MSG(delta_x != NULL);
    __ASSERT_NO_MSG(delta_y != NULL);
    if (axis_flags & HERO_AXIS_FLAG_SWAP_XY) {
        int16_t original_delta_x = *delta_x;
        *delta_x = *delta_y;
        *delta_y = original_delta_x;
    }
    if (axis_flags & HERO_AXIS_FLAG_INVERT_X) {
        *delta_x = (int16_t)-*delta_x;
    }
    if (axis_flags & HERO_AXIS_FLAG_INVERT_Y) {
        *delta_y = (int16_t)-*delta_y;
    }
}
