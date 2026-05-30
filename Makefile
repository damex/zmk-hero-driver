# Copyright 2026 Roman Kuzmitskii (@damex)
# SPDX-License-Identifier: MIT

PYTHON ?= python3
ZEPHYR_BASE ?= $(error ZEPHYR_BASE is not set)
ZEPHYR_TOOLCHAIN_VARIANT ?= host

.PHONY: unit-test
unit-test:
	ZEPHYR_TOOLCHAIN_VARIANT=$(ZEPHYR_TOOLCHAIN_VARIANT) $(PYTHON) $(ZEPHYR_BASE)/scripts/twister --testsuite-root tests/unit --platform unit_testing --inline-logs --clobber-output
