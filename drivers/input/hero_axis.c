// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include "hero_axis.h"

#include <zephyr/sys/__assert.h>

uint32_t hero_axis_flags(bool invert_x, bool invert_y, bool swap_xy) {
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
    return flags;
}

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
