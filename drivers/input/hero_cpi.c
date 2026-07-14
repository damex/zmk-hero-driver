// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#include "hero_cpi.h"

#include <zephyr/toolchain.h>

BUILD_ASSERT((HERO_CPI_MAX / HERO_CPI_STEP) - 1 <= UINT8_MAX,
             "CPI max overflows the DPI register width");

bool hero_cpi_in_range(uint32_t cpi) {
    return cpi >= HERO_CPI_MIN && cpi <= HERO_CPI_MAX;
}

bool hero_cpi_pair_in_range(uint32_t cpi_x, uint32_t cpi_y) {
    return hero_cpi_in_range(cpi_x) && hero_cpi_in_range(cpi_y);
}

uint8_t hero_cpi_to_register(uint32_t cpi) {
    return (uint8_t)((cpi / HERO_CPI_STEP) - 1);
}
