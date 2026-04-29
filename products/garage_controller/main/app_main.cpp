// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0
//
// app_main.cpp — Garage Side-Door Controller
//
// Matter endpoints (must match data_model_thread.zap):
//   EP1: Root Node          — mandatory, managed by system firmware
//   EP2: Occupancy Sensor   — SICK LIDAR GPIO6 (potential-free contact)
//   EP3: Occupancy Sensor   — Gate door  GPIO7 (potential-free contact)
//   EP4: On/Off Plugin Unit — relay GPIO19 (NO, hardwired on board)

#include <stdio.h>
#include <system.h>
#include <low_code.h>

#include "app_priv.h"

static const char *TAG = "app_main";

/* ── setup() — called once before the main loop ─────────────────────────── */
static void setup()
{
    /* Register system callbacks first */
    low_code_register_callbacks(feature_update_from_system, event_from_system);

    /* Initialize all hardware drivers */
    app_driver_init();
}

/* ── loop() — called every iteration of the main loop ──────────────────── */
static void loop()
{
    /* Poll the transport layer for incoming feature updates and events.
     * These will fire the registered callbacks synchronously. */
    low_code_get_feature_update_from_system();
    low_code_get_event_from_system();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * feature_update_from_system
 *
 * Called when a Matter controller (Home Assistant) changes a device attribute.
 * We handle:
 *   EP4 + POWER feature → relay on/off (door unlock / lock)
 *
 * EP2 and EP3 are input-only sensors — they push updates TO the system,
 * they never receive feature updates from the controller.
 * ═════════════════════════════════════════════════════════════════════════ */
int feature_update_from_system(low_code_feature_data_t *data)
{
    uint16_t endpoint_id = data->details.endpoint_id;
    low_code_feature_id_t feature_id = data->details.feature_id;

    printf("%s: Feature update — ep=%d feature=%d\n", TAG, endpoint_id, (int)feature_id);

    if (endpoint_id == EP_DOOR_LOCK) {
        if (feature_id == LOW_CODE_FEATURE_ID_POWER) {
            bool unlock = *(bool *)data->value.value;
            printf("%s: Lock command: %s\n", TAG, unlock ? "UNLOCK" : "LOCK");
            app_driver_set_lock_state(unlock);
        }
    }
    /* EP2/EP3 feature updates from system are not expected — ignore */

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * event_from_system
 *
 * Passes system events (commissioning, network, OTA, identify) to the
 * driver layer which handles LED indication.
 * ═════════════════════════════════════════════════════════════════════════ */
int event_from_system(low_code_event_t *event)
{
    return app_driver_event_handler(event);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * main — LowCode entry point
 * ═════════════════════════════════════════════════════════════════════════ */
extern "C" int main()
{
    printf("%s: Garage Controller starting\n", TAG);

    /* Pre-initialisation — must be called first */
    system_setup();

    setup();

    /* Main loop */
    while (1) {
        system_loop();
        loop();
    }

    return 0;
}