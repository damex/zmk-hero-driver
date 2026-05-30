// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <dt-bindings/zmk/hero_cpi.h>

/* Per-axis CPI packed into one word (x low, y high) so the poll thread reads
 * the pair atomically. Each value fits 16 bits. */
#define HERO_CPI_PACK(cpi_x, cpi_y) ((uint32_t)(cpi_x) | ((uint32_t)(cpi_y) << 16))
#define HERO_CPI_UNPACK_X(word)     ((word) & 0xFFFF)
#define HERO_CPI_UNPACK_Y(word)     ((word) >> 16)

bool hero_cpi_in_range(uint32_t cpi);

bool hero_cpi_pair_in_range(uint32_t cpi_x, uint32_t cpi_y);

uint8_t hero_cpi_to_register(uint32_t cpi);
