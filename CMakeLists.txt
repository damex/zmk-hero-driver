# Copyright 2026 Roman Kuzmitskii (@damex)
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_compile_options(
    -Werror
    -Wextra
    -Wshadow
    -Wformat=2
    -Wlogical-op
    -Wduplicated-cond
    -Wduplicated-branches
    -Wmissing-prototypes
    -Wjump-misses-init
    -Wwrite-strings
    -Winit-self
    -Wold-style-definition
    -fanalyzer
)
zephyr_include_directories(include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_library_sources_ifdef(CONFIG_INPUT_HERO drivers/input/input_hero.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_HERO drivers/input/hero_axis.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_HERO drivers/input/hero_cpi.c)
zephyr_library_sources_ifdef(CONFIG_INPUT_HERO drivers/input/hero_timing.c)

# HERO sensor blob is Logitech proprietary firmware (gitignored sensor.bin, see README).
# -Wa,-I lets the assembler find the blob; OBJECT_DEPENDS re-runs on change.
if(CONFIG_INPUT_HERO)
  set(sensor_bin ${CMAKE_CURRENT_SOURCE_DIR}/sensor.bin)
  if(NOT EXISTS ${sensor_bin})
    message(FATAL_ERROR
            "HERO sensor blob missing: ${sensor_bin}. It is Logitech proprietary "
            "firmware and is not redistributed; supply your own dump.")
  endif()
  zephyr_library_sources(drivers/input/sensor_blob.S)
  set_source_files_properties(drivers/input/sensor_blob.S PROPERTIES
                              COMPILE_OPTIONS "-Wa,-I${CMAKE_CURRENT_SOURCE_DIR}"
                              OBJECT_DEPENDS "${sensor_bin}")
endif()
