// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include <stdint.h>

#include <zephyr/ztest.h>

#include "hero_timing.h"

ZTEST_SUITE(hero_timing, NULL, NULL, NULL, NULL, NULL);

ZTEST(hero_timing, test_min_frame_rate_to_period) {
    zassert_equal(hero_min_frame_rate_to_period(0), UINT8_MAX, "0 Hz disables the floor");
    zassert_equal(hero_min_frame_rate_to_period(1000), 50, "1000 fps -> 50");
    zassert_equal(hero_min_frame_rate_to_period(500), 100, "500 fps -> 100");
    zassert_equal(hero_min_frame_rate_to_period(100), UINT8_MAX, "low rate clamps to register max");
    zassert_equal(hero_min_frame_rate_to_period(10000), HERO_FRAME_PERIOD_MIN,
                  "high rate clamps to the 120 us floor");
    zassert_equal(hero_min_frame_rate_to_period(UINT32_MAX), HERO_FRAME_PERIOD_MIN,
                  "wrap-range rate clamps to the floor");
}

ZTEST(hero_timing, test_poll_rate_to_interval_us) {
    zassert_equal(hero_poll_rate_to_interval_us(0), UINT32_MAX, "0 Hz parks the poll");
    zassert_equal(hero_poll_rate_to_interval_us(1000), 1000, "1 kHz -> 1000 us");
    zassert_equal(hero_poll_rate_to_interval_us(8000), 125, "8 kHz -> 125 us");
    zassert_equal(hero_poll_rate_to_interval_us(10000), HERO_POLL_INTERVAL_MIN_US,
                  "10 kHz sits on the floor");
    zassert_equal(hero_poll_rate_to_interval_us(20000), HERO_POLL_INTERVAL_MIN_US,
                  "above the ceiling clamps to the floor");
}

ZTEST(hero_timing, test_rest_seconds_to_register) {
    zassert_equal(hero_rest_seconds_to_register(0), 0, "0 s -> 0");
    zassert_equal(hero_rest_seconds_to_register(1), 0, "min floor -> 0");
    zassert_equal(hero_rest_seconds_to_register(2), 2, "2 s -> 2");
    zassert_equal(hero_rest_seconds_to_register(5), 8, "5 s -> 8");
    zassert_equal(hero_rest_seconds_to_register(200), UINT8_MAX, "large value clamps");
    zassert_equal(hero_rest_seconds_to_register(UINT32_MAX), UINT8_MAX,
                  "wrap-range seconds clamp to register max");
}
