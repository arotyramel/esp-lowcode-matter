// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0
//
// app_driver.cpp — Garage Side-Door Controller hardware layer
//
// GPIO19 : Relay hardwired on board (NO — LOW=locked, HIGH=unlocked)
// GPIO9  : Internal programmable LED (on = gate open)
// GPIO5  : Reset button (active-low)
// GPIO6  : SICK LIDAR contact (active-low, pull-up)
// GPIO7  : Gate door contact (active-low, pull-up)

#include <stdio.h>
#include <stdbool.h>

#include <low_code.h>
#include <relay_driver.h>
#include <button_driver.h>

#include "app_priv.h"

static const char *TAG = "app_driver";

static struct {
    bool lock_unlocked;
    bool lidar_active;
    bool gate_open;
} g_state = {};

static button_handle_t s_btn_reset = NULL;
static button_handle_t s_btn_lidar = NULL;
static button_handle_t s_btn_gate  = NULL;

/* LED mirrors gate_open state */
static void led_update(void)
{
    relay_driver_set_power(GPIO_LED, g_state.gate_open);
}

/* ── Relay ──────────────────────────────────────────────────────────────── */

int app_driver_set_lock_state(bool unlocked)
{
    g_state.lock_unlocked = unlocked;
    printf("%s: Relay → %s\n", TAG, unlocked ? "UNLOCKED" : "LOCKED");
    relay_driver_set_power(GPIO_RELAY, unlocked);
    return 0;
}

/* ── Sensor reporting ───────────────────────────────────────────────────── */

int app_driver_report_lidar(bool active)
{
    g_state.lidar_active = active;
    printf("%s: LIDAR → %s\n", TAG, active ? "ACTIVE" : "CLEAR");

    bool val = active;
    low_code_feature_data_t feature = {};
    feature.details.endpoint_id = EP_LIDAR_SENSOR;
    feature.details.feature_id  = LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE;
    feature.value.type           = LOW_CODE_VALUE_TYPE_BOOLEAN;
    feature.value.value_len      = sizeof(bool);
    feature.value.value          = (uint8_t *)&val;
    return low_code_feature_update_to_system(&feature);
}

int app_driver_report_gate(bool open)
{
    g_state.gate_open = open;
    printf("%s: Gate → %s\n", TAG, open ? "OPEN" : "CLOSED");
    led_update();

    bool val = open;
    low_code_feature_data_t feature = {};
    feature.details.endpoint_id = EP_GATE_SENSOR;
    feature.details.feature_id  = LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE;
    feature.value.type           = LOW_CODE_VALUE_TYPE_BOOLEAN;
    feature.value.value_len      = sizeof(bool);
    feature.value.value          = (uint8_t *)&val;
    return low_code_feature_update_to_system(&feature);
}

/* ── Button callbacks ───────────────────────────────────────────────────── */

static void btn_reset_cb(void *handle, void *usr_data)
{
    printf("%s: Reset button → factory reset\n", TAG);
    low_code_event_t event = {};
    event.event_type = LOW_CODE_EVENT_FACTORY_RESET;
    low_code_event_to_system(&event);
}

static void btn_lidar_down_cb(void *handle, void *usr_data) { app_driver_report_lidar(true);  }
static void btn_lidar_up_cb  (void *handle, void *usr_data) { app_driver_report_lidar(false); }
static void btn_gate_down_cb (void *handle, void *usr_data) { app_driver_report_gate(true);   }
static void btn_gate_up_cb   (void *handle, void *usr_data) { app_driver_report_gate(false);  }

/* ── Event handler ──────────────────────────────────────────────────────── */

int app_driver_event_handler(low_code_event_t *event)
{
    printf("%s: Event: %d\n", TAG, event->event_type);
    return 0;
}

/* ── Driver init ────────────────────────────────────────────────────────── */

int app_driver_init()
{
    printf("%s: Initializing garage controller driver\n", TAG);

    relay_driver_init(GPIO_RELAY);
    relay_driver_set_power(GPIO_RELAY, false);  /* start locked */

    relay_driver_init(GPIO_LED);
    relay_driver_set_power(GPIO_LED, false);    /* LED off at boot */

    button_config_t reset_cfg = {
        .long_press_time  = 3000,
        .short_press_time = 50,
        .gpio_num         = GPIO_RESET_BTN,
        .pullup_en        = 1,
        .active_level     = 0,
    };
    s_btn_reset = button_driver_create(&reset_cfg);
    button_driver_register_cb(s_btn_reset, BUTTON_SINGLE_CLICK, btn_reset_cb, NULL);
    button_driver_register_cb(s_btn_reset, BUTTON_LONG_PRESS_UP, btn_reset_cb, NULL);

    button_config_t lidar_cfg = {
        .short_press_time = 50,
        .gpio_num         = GPIO_LIDAR_IN,
        .pullup_en        = 1,
        .active_level     = 0,
    };
    s_btn_lidar = button_driver_create(&lidar_cfg);
    button_driver_register_cb(s_btn_lidar, BUTTON_PRESS_DOWN, btn_lidar_down_cb, NULL);
    button_driver_register_cb(s_btn_lidar, BUTTON_PRESS_UP,   btn_lidar_up_cb,   NULL);

    button_config_t gate_cfg = {
        .short_press_time = 50,
        .gpio_num         = GPIO_GATE_IN,
        .pullup_en        = 1,
        .active_level     = 0,
    };
    s_btn_gate = button_driver_create(&gate_cfg);
    button_driver_register_cb(s_btn_gate, BUTTON_PRESS_DOWN, btn_gate_down_cb, NULL);
    button_driver_register_cb(s_btn_gate, BUTTON_PRESS_UP,   btn_gate_up_cb,   NULL);

    printf("%s: Driver init complete\n", TAG);
    return 0;
}
