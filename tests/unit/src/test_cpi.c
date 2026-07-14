// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include <zephyr/ztest.h>

#include "hero_cpi.h"

ZTEST_SUITE(hero_cpi, NULL, NULL, NULL, NULL, NULL);

ZTEST(hero_cpi, test_cpi_in_range) {
    zassert_false(hero_cpi_in_range(HERO_CPI_MIN - 1), "below min must be rejected");
    zassert_true(hero_cpi_in_range(HERO_CPI_MIN), "min is in range");
    zassert_true(hero_cpi_in_range(1800), "typical value is in range");
    zassert_true(hero_cpi_in_range(HERO_CPI_MAX), "max is in range");
    zassert_false(hero_cpi_in_range(HERO_CPI_MAX + 1), "above max must be rejected");
    zassert_false(hero_cpi_in_range(0), "zero must be rejected");
}

ZTEST(hero_cpi, test_cpi_pair_in_range) {
    zassert_true(hero_cpi_pair_in_range(HERO_CPI_MIN, HERO_CPI_MAX), "both valid");
    zassert_false(hero_cpi_pair_in_range(0, 1800), "x invalid rejects pair");
    zassert_false(hero_cpi_pair_in_range(1800, HERO_CPI_MAX + 1), "y invalid rejects pair");
    zassert_false(hero_cpi_pair_in_range(0, 0), "both invalid");
}

ZTEST(hero_cpi, test_cpi_to_register) {
    /* register value = (cpi / 50) - 1 */
    zassert_equal(hero_cpi_to_register(50), 0, NULL);
    zassert_equal(hero_cpi_to_register(100), 1, NULL);
    zassert_equal(hero_cpi_to_register(800), 15, NULL);
    zassert_equal(hero_cpi_to_register(1800), 35, NULL);
    zassert_equal(hero_cpi_to_register(2100), 41, NULL);
    zassert_equal(hero_cpi_to_register(12000), 239, NULL);
}
