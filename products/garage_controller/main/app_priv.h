// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <low_code.h>

/* ── GPIO pin assignments — ESP32-C6 Relay X1 V1.1 board ───────────────── */
/*                                                                           */
/*   Left header (top→bottom):                                               */
/*   GND  | 5V                                                               */
/*   G12  | G13                                                              */
/*   G11  | ?                                                                */
/*   G8   | G1   ← G8 external button to GND                               */
/*   G19  | G7   ← G19 hardwired to relay coil, G7 gate contact            */
/*   G6   | G5   ← G6 LIDAR contact                                        */
/*   G4   | EN                                                               */
/*   3V3  | GND                                                              */
#define GPIO_RELAY          19  /* Output: relay IN — hardwired on board    */
                                /*   NO contact: LOW=off, HIGH=on           */
                                /*   HP GPIO — must use system_digital_write*/
                                /*   not relay_driver (LP-only API)         */
#define GPIO_LED            2   /* Output: internal programmable LED        */
#define GPIO_RESET_BTN      8   /* Input:  toggle/reset button, to GND      */
#define GPIO_LIDAR_IN       6   /* Input:  SICK LIDAR potential-free        */
#define GPIO_GATE_IN        7   /* Input:  garage gate door contact         */

/* ── Matter endpoint IDs as seen by the LowCode framework ──────────────── */
/* The LowCode framework numbers endpoints by 0-based array index, not by  */
/* the Matter endpoint ID in the ZAP file. Our ZAP has [EP0, EP2, EP3, EP4]*/
/* so the indices are: EP0=0, EP2=1, EP3=2, EP4=3.                        */
#define EP_LIDAR_SENSOR     1   /* Occupancy Sensor — SICK LIDAR  (ZAP EP2) */
#define EP_GATE_SENSOR      2   /* Occupancy Sensor — gate door   (ZAP EP3) */
#define EP_DOOR_LOCK        3   /* On/Off Plugin Unit — relay     (ZAP EP4) */

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
