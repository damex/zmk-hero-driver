// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <dt-bindings/zmk/hero_cpi.h>

bool hero_cpi_in_range(uint32_t cpi);

bool hero_cpi_pair_in_range(uint32_t cpi_x, uint32_t cpi_y);

uint8_t hero_cpi_to_register(uint32_t cpi);
