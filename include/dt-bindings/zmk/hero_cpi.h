// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/* HERO CPI scale. DT-safe (defines only) so both the driver and DT-bindings
 * wire encoders share one source of truth. */
#pragma once

#define HERO_CPI_STEP 50
#define HERO_CPI_MIN  50
#define HERO_CPI_MAX  12000
