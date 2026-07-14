// Copyright 2026 Roman Kuzmitskii (@damex)
// SPDX-License-Identifier: MIT

/*
 * Applies "hero/" settings to the sensor: settings_load at boot, settings_runtime_set
 * live. Value is a uint32, cast per key.
 */
#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/input/hero.h>

LOG_MODULE_DECLARE(input_hero, CONFIG_INPUT_HERO_LOG_LEVEL);

static const struct device *const hero_device = DEVICE_DT_GET_ONE(logitech_hero);

static int hero_config_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    uint32_t value;
    if (len != sizeof(value)) {
        return -EINVAL;
    }
    if (read_cb(cb_arg, &value, sizeof(value)) < 0) {
        return -EIO;
    }
    if (settings_name_steq(name, "cpi_x", &next) && next == NULL) {
        hero_set_cpi_x(hero_device, value);
        return 0;
    }
    if (settings_name_steq(name, "cpi_y", &next) && next == NULL) {
        hero_set_cpi_y(hero_device, value);
        return 0;
    }
    if (settings_name_steq(name, "rate", &next) && next == NULL) {
        hero_set_report_rate(hero_device, value);
        return 0;
    }
    if (settings_name_steq(name, "min_frame_rate", &next) && next == NULL) {
        hero_set_min_frame_rate(hero_device, value);
        return 0;
    }
    if (settings_name_steq(name, "x_code", &next) && next == NULL) {
        hero_set_x_code(hero_device, (uint16_t)value);
        return 0;
    }
    if (settings_name_steq(name, "y_code", &next) && next == NULL) {
        hero_set_y_code(hero_device, (uint16_t)value);
        return 0;
    }
    if (settings_name_steq(name, "rest_timeout", &next) && next == NULL) {
        hero_set_rest_timeout(hero_device, value);
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(hero_config, "hero", NULL, hero_config_set, NULL, NULL);

/* Runs after the hero device init (POST_KERNEL), so a restored value applies live. */
static int hero_config_init(void) {
    int error = settings_subsys_init();
    if (error < 0) {
        LOG_ERR("settings init failed (%d)", error);
        return error;
    }
    error = settings_load_subtree("hero");
    if (error < 0) {
        LOG_ERR("hero settings load failed (%d)", error);
    }
    return error;
}
SYS_INIT(hero_config_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
