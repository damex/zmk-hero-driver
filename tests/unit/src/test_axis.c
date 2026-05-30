// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include <stdint.h>

#include <zephyr/ztest.h>

#include "hero_axis.h"

ZTEST_SUITE(hero_axis, NULL, NULL, NULL, NULL, NULL);

static void assert_transform(uint32_t flags, int16_t in_x, int16_t in_y, int16_t expect_x,
                             int16_t expect_y) {
    int16_t delta_x = in_x;
    int16_t delta_y = in_y;
    hero_apply_axis_transform(flags, &delta_x, &delta_y);
    zassert_equal(delta_x, expect_x, "x: flags=%u got=%d want=%d", flags, delta_x, expect_x);
    zassert_equal(delta_y, expect_y, "y: flags=%u got=%d want=%d", flags, delta_y, expect_y);
}

ZTEST(hero_axis, test_axis_flags) {
    zassert_equal(hero_axis_flags(false, false, false), 0, "no flags");
    zassert_equal(hero_axis_flags(true, false, false), HERO_AXIS_FLAG_INVERT_X, NULL);
    zassert_equal(hero_axis_flags(false, true, false), HERO_AXIS_FLAG_INVERT_Y, NULL);
    zassert_equal(hero_axis_flags(false, false, true), HERO_AXIS_FLAG_SWAP_XY, NULL);
    zassert_equal(hero_axis_flags(true, true, true),
                  HERO_AXIS_FLAG_INVERT_X | HERO_AXIS_FLAG_INVERT_Y | HERO_AXIS_FLAG_SWAP_XY,
                  "all flags");
}

ZTEST(hero_axis, test_axis_transform) {
    assert_transform(0, 3, 7, 3, 7);
    assert_transform(HERO_AXIS_FLAG_INVERT_X, 3, 7, -3, 7);
    assert_transform(HERO_AXIS_FLAG_INVERT_Y, 3, 7, 3, -7);
    assert_transform(HERO_AXIS_FLAG_INVERT_X | HERO_AXIS_FLAG_INVERT_Y, 3, 7, -3, -7);
    assert_transform(HERO_AXIS_FLAG_SWAP_XY, 3, 7, 7, 3);
    /* swap happens before invert, so invert acts on the swapped axis */
    assert_transform(HERO_AXIS_FLAG_SWAP_XY | HERO_AXIS_FLAG_INVERT_X, 3, 7, -7, 3);
    assert_transform(HERO_AXIS_FLAG_SWAP_XY | HERO_AXIS_FLAG_INVERT_Y, 3, 7, 7, -3);
    assert_transform(HERO_AXIS_FLAG_SWAP_XY | HERO_AXIS_FLAG_INVERT_X | HERO_AXIS_FLAG_INVERT_Y,
                     3, 7, -7, -3);
}
