# Copyright 2026 Roman Kuzmitskii (@damex)
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.20.0)

project(hero_unit_test)
find_package(Zephyr COMPONENTS unittest HINTS $ENV{ZEPHYR_BASE})

if(NOT EXISTS "/usr/include/gnu/stubs-32.h")
  string(REPLACE "-m32" "-m64" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  string(REPLACE "-m32" "-m64" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()

set(hero_driver_root ${CMAKE_CURRENT_SOURCE_DIR}/../..)

target_include_directories(testbinary PRIVATE
  ${hero_driver_root}/include
  ${hero_driver_root}/drivers/input
)

target_sources(testbinary PRIVATE
  src/test_cpi.c
  src/test_timing.c
  ${hero_driver_root}/drivers/input/hero_cpi.c
  ${hero_driver_root}/drivers/input/hero_timing.c
)
