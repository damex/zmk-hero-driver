// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

#define HERO_FRAME_PERIOD_STEP_US  20  /* reg 0x20 unit: period = value * 20 us */
#define HERO_FRAME_PERIOD_MIN      6   /* 120 us floor, tracking degrades below */
#define HERO_REST_MIN_SEC          1   /* value 0 = ~1 s */
#define HERO_REST_STEP_PER_SEC     2   /* 0.5 s per reg step */
#define HERO_POLL_INTERVAL_MIN_US  100 /* 10 kHz ceiling, headroom above SPI floor */

uint8_t hero_min_frame_rate_to_period(uint32_t hz);

uint32_t hero_poll_rate_to_interval_us(uint32_t hz);

uint8_t hero_rest_seconds_to_register(uint32_t seconds);
