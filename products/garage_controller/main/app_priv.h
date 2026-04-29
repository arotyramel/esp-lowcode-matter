// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <low_code.h>

/* ── GPIO pin assignments — ESP32-C6 Relay X1 V1.1 board ───────────────── */
/* LP core can only drive LP GPIOs (0-7) — all outputs must be in that range*/
/*                                                                           */
/*   Left header (top→bottom):                                               */
/*   GND  | 5V                                                               */
/*   G12  | G13                                                              */
/*   G11  | ?                                                                */
/*   G8   | G1                                                               */
/*   G0   | G7   ← G0 hardwired to relay coil (NO, safe to boot on GPIO0)  */
/*   G6   | G5                                                               */
/*   G4   | EN                                                               */
/*   3V3  | GND                                                              */

#define GPIO_RELAY          19  /* Output: relay IN — hardwired on board    */
                                /*   NO contact: LOW=off, HIGH=on           */
                                /*   HP GPIO — must use system_digital_write*/
                                /*   not relay_driver (LP-only API)         */
#define GPIO_LED            2   /* Output: internal programmable LED        */
#define GPIO_RESET_BTN      5   /* Input:  reset button, external, to GND   */
#define GPIO_LIDAR_IN       6   /* Input:  SICK LIDAR potential-free        */
#define GPIO_GATE_IN        7   /* Input:  garage gate door contact         */

/* ── Matter endpoint IDs (must match data_model_thread.zap) ────────────── */
#define EP_LIDAR_SENSOR     2   /* Occupancy Sensor — SICK LIDAR            */
#define EP_GATE_SENSOR      3   /* Occupancy Sensor — gate door             */
#define EP_DOOR_LOCK        4   /* On/Off Plugin Unit — relay output        */

/* ── Driver init ────────────────────────────────────────────────────────── */
int app_driver_init();

/* ── Driver setters (called from feature_update_from_system) ────────────── */
int app_driver_set_lock_state(bool unlocked);

/* ── Driver sensor reporting (called from button/timer callbacks) ────────── */
int app_driver_report_lidar(bool active);
int app_driver_report_gate(bool open);

/* ── Event handler ──────────────────────────────────────────────────────── */
int app_driver_event_handler(low_code_event_t *event);

/* ── System callbacks (defined in app_main.cpp) ─────────────────────────── */
int feature_update_from_system(low_code_feature_data_t *data);
int event_from_system(low_code_event_t *event);
