// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/* HERO sensor blob embedded from sensor.bin (proprietary, not redistributed).
 * Exports sensor_blob (start) and sensor_blob_end for the upload loop. */
    .section .rodata,"a"
    .balign 4
    .global sensor_blob
sensor_blob:
    .incbin "sensor.bin"
    .global sensor_blob_end
sensor_blob_end:
