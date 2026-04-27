// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0
//
// app_driver.cpp — Garage Side-Door Controller hardware layer
//
// Board: ESP32-C6 Relay X1 V1.1 (single left header, 5V powered)
//
// Hardware:
//   GPIO0  : Relay hardwired on board (NO — LOW=locked, HIGH=unlocked)
//             Boot safety: relay is NO → GPIO0 not pulled low at boot
//   GPIO4  : WS2812B LED chain DIN (3 LEDs, via 330Ω series resistor)
//   GPIO5  : Reset button (active-low, pull-up, external tactile to GND)
//             single click  → factory reset via LOW_CODE_EVENT_FACTORY_RESET
//             long press    → (same, with LED warning)
//   GPIO6  : SICK LIDAR contact input (active-low, pull-up)
//   GPIO7  : Garage gate door contact input (active-low, pull-up)

#include <stdio.h>
#include <stdbool.h>

#include <low_code.h>
#include <relay_driver.h>
#include <button_driver.h>
#include <ws2812_driver.h>
#include <sw_timer.h>

#include "app_priv.h"

static const char *TAG = "app_driver";

/* ═══════════════════════════════════════════════════════════════════════════
 * LED colour definitions
 * WS2812B channels: R=0, G=1, B=2 per ws2812_channel_enum_t
 * Each LED is a separately registered channel group on its own GPIO.
 * We drive each LED with full R/G/B by registering it three times isn't
 * possible — instead we use the single ws2812_driver_regist_channel per GPIO
 * and set brightness only. For colour we set R/G/B channels directly via
 * ws2812_driver_set_channel on the per-LED channels.
 *
 * The ws2812 driver supports one strip per GPIO. We register three separate
 * GPIO pins, each controlling one LED. Each call to ws2812_driver_regist_channel
 * maps a logical channel index to a GPIO. We use three channel indices (0,1,2)
 * one per LED, and set R/G/B by writing the GRB buffer directly via the
 * RED/GREEN/BLUE channel enums on each registered channel.
 *
 * Simpler approach used here: we register each LED GPIO as its own "channel"
 * using WS2812_CHANNEL_RED=0/GREEN=1/BLUE=2 offset per LED:
 *   LED_LIDAR: channels 0,1,2  → GPIO5
 *   LED_GATE:  channels 3,4,5  → GPIO6  (not directly supported by driver)
 *
 * The ws2812 driver as shipped only supports a SINGLE strip (one GPIO).
 * For three separate single LEDs we therefore use one strip of 3 LEDs on
 * a single data line (GPIO5, chained), which is the recommended wiring.
 * GPIO6 and GPIO7 are then unused for LEDs.
 *
 * WIRING NOTE: Chain all 3 WS2812B LEDs on GPIO5 only:
 *   GPIO5 → 330Ω → LED0 DIN
 *                   LED0 DOUT → LED1 DIN
 *                               LED1 DOUT → LED2 DIN
 * GPIO6 and GPIO7 are freed for future use.
 * ═════════════════════════════════════════════════════════════════════════ */

/* LED indices in the chain */
#define LED_LIDAR_IDX   0
#define LED_GATE_IDX    1
#define LED_LOCK_IDX    2

/* Colour helpers — writes to the 3-byte GRB buffer for one LED in the chain.
 * ws2812_driver_set_channel operates on a single-LED strip registered on
 * one GPIO. To address individual LEDs in a chain we use the light_driver
 * approach: set R/G/B channels then call update.
 *
 * Since ws2812_driver_regist_channel maps ONE channel enum to ONE GPIO,
 * and our three LEDs share one GPIO in a chain, we drive them via the
 * light_driver WS2812 path which handles multi-LED strips.
 * We use ws2812_driver_set_channel for the single registered strip.
 */

/* Application state */
static struct {
    bool lock_unlocked;     /* true = relay on = door unlocked              */
    bool lidar_active;      /* true = LIDAR contact closed = person present */
    bool gate_open;         /* true = gate contact open = door open         */
    bool commissioning;     /* true = in setup/pairing mode                 */
} g_state = {};

/* Button handles */
static button_handle_t s_btn_reset  = NULL;
static button_handle_t s_btn_lidar  = NULL;
static button_handle_t s_btn_gate   = NULL;

/* Commissioning animation timer */
static sw_timer_handle_t s_anim_timer = NULL;
static uint8_t           s_anim_phase = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 * LED helpers
 * ═════════════════════════════════════════════════════════════════════════ */

/* Set one LED in the chain by index to an R,G,B value.
 * The ws2812 driver stores one RGB buffer. For a multi-LED chain we write
 * sequentially: each call to ws2812_driver_set_channel + update drives the
 * currently selected pixel. The driver shipped supports a single pixel strip
 * per call, so we call regist+set+update three times with the three GPIOs
 * being the same GPIO (chained). This is done by calling update once after
 * setting all three LEDs using the R/G/B channel writes.
 *
 * Practical approach: use ws2812_driver_set_channel for R, G, B channels
 * (channels 0, 1, 2) which maps to the single registered strip on GPIO5,
 * then call ws2812_driver_update_channels() once.
 * This drives LED0 only (single LED on strip).
 *
 * For a 3-LED chain, the driver would need to be extended. As a pragmatic
 * workaround for the beta SDK, we register the same GPIO three times on
 * channels 0..8 and rely on the RMT sending a 9-byte (3 LED) GRB stream.
 * Until multi-LED support is confirmed, we implement leds_set() to write
 * all three LEDs the same colour (simplest valid approach), and differentiate
 * by encoding state in blink patterns instead.
 */

static void leds_write_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_driver_set_channel(WS2812_CHANNEL_RED,   r);
    ws2812_driver_set_channel(WS2812_CHANNEL_GREEN, g);
    ws2812_driver_set_channel(WS2812_CHANNEL_BLUE,  b);
    ws2812_driver_update_channels();
}

/*
 * Update all three LEDs to reflect current application state.
 * Because the ws2812 driver in the beta SDK drives a single-pixel strip,
 * we encode state into a combined colour visible on LED0. This can be
 * upgraded to per-LED control when Espressif extends the multi-LED API.
 *
 * Encoding (priority order):
 *   Commissioning  → breathing blue (handled by timer, not here)
 *   Lock unlocked  → blue
 *   Gate open      → red
 *   LIDAR active   → amber
 *   All clear      → green
 */
static void leds_update_state(void)
{
    if (g_state.commissioning) {
        return; /* handled by animation timer */
    }
    if (g_state.lock_unlocked) {
        leds_write_rgb(0, 0, 200);      /* blue  = unlocked */
    } else if (g_state.gate_open) {
        leds_write_rgb(200, 0, 0);      /* red   = gate open */
    } else if (g_state.lidar_active) {
        leds_write_rgb(200, 80, 0);     /* amber = person detected */
    } else {
        leds_write_rgb(0, 150, 0);      /* green = all clear */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Commissioning animation timer callback (~50ms tick → 2s breathe cycle)
 * ═════════════════════════════════════════════════════════════════════════ */
static void anim_timer_cb(sw_timer_handle_t handle, void *arg)
{
    if (!g_state.commissioning) {
        sw_timer_stop(handle);
        leds_update_state();
        return;
    }
    /* Triangular brightness on 40-step cycle (40 × 50ms = 2s) */
    s_anim_phase = (s_anim_phase + 1) % 40;
    uint8_t bright = (s_anim_phase < 20) ? (s_anim_phase * 10)
                                         : ((39 - s_anim_phase) * 10);
    leds_write_rgb(0, 0, bright);   /* breathing blue */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Sensor reporting helpers — push state up to Matter via low_code
 * ═════════════════════════════════════════════════════════════════════════ */

/*
 * Report LIDAR contact state to Matter.
 * We use LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE (6001).
 * Value: true = occupied/active, false = clear.
 */
int app_driver_report_lidar(bool active)
{
    g_state.lidar_active = active;
    printf("%s: LIDAR → %s\n", TAG, active ? "ACTIVE" : "CLEAR");

    bool val = active;
    low_code_feature_data_t feature = {};
    feature.details.endpoint_id        = EP_LIDAR_SENSOR;
    feature.details.feature_id         = LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE;
    feature.value.type                 = LOW_CODE_VALUE_TYPE_BOOLEAN;
    feature.value.value_len            = sizeof(bool);
    feature.value.value                = (uint8_t *)&val;

    int ret = low_code_feature_update_to_system(&feature);
    leds_update_state();
    return ret;
}

/*
 * Report gate door contact state to Matter.
 * We re-use LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE on EP3.
 * Value: true = gate open, false = gate closed.
 */
int app_driver_report_gate(bool open)
{
    g_state.gate_open = open;
    printf("%s: Gate → %s\n", TAG, open ? "OPEN" : "CLOSED");

    bool val = open;
    low_code_feature_data_t feature = {};
    feature.details.endpoint_id        = EP_GATE_SENSOR;
    feature.details.feature_id         = LOW_CODE_FEATURE_ID_OCCUPANCY_SENSOR_VALUE;
    feature.value.type                 = LOW_CODE_VALUE_TYPE_BOOLEAN;
    feature.value.value_len            = sizeof(bool);
    feature.value.value                = (uint8_t *)&val;

    int ret = low_code_feature_update_to_system(&feature);
    leds_update_state();
    return ret;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Relay (door lock)
 * ═════════════════════════════════════════════════════════════════════════ */
int app_driver_set_lock_state(bool unlocked)
{
    g_state.lock_unlocked = unlocked;
    printf("%s: Relay → %s\n", TAG, unlocked ? "UNLOCKED" : "LOCKED");
    relay_driver_set_power(GPIO_RELAY, unlocked);
    leds_update_state();
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Button callbacks
 * ═════════════════════════════════════════════════════════════════════════ */

/* Reset button — single click triggers factory reset */
static void btn_reset_single_click_cb(void *handle, void *usr_data)
{
    printf("%s: Reset button — single click → factory reset\n", TAG);
    /* Flash white 3 times as visual warning */
    for (int i = 0; i < 3; i++) {
        leds_write_rgb(180, 180, 180);
        /* busy-wait ~200ms — acceptable on LP core since this is a shutdown path */
        for (volatile int d = 0; d < 800000; d++) {}
        leds_write_rgb(0, 0, 0);
        for (volatile int d = 0; d < 800000; d++) {}
    }
    low_code_event_t event = {};
    event.event_type = LOW_CODE_EVENT_FACTORY_RESET;
    low_code_event_to_system(&event);
}

/* Reset button — long press also triggers factory reset (belt and braces) */
static void btn_reset_long_press_cb(void *handle, void *usr_data)
{
    printf("%s: Reset button — long press → factory reset\n", TAG);
    btn_reset_single_click_cb(handle, usr_data);
}

/* LIDAR input — press down = contact closed = active */
static void btn_lidar_down_cb(void *handle, void *usr_data)
{
    app_driver_report_lidar(true);
}

static void btn_lidar_up_cb(void *handle, void *usr_data)
{
    app_driver_report_lidar(false);
}

/* Gate input — press down = contact closed = gate open
 * (wire NC contact so that "gate open" = contact opens = GPIO goes high
 *  OR wire NO contact so that "gate open" = contact closes = GPIO goes low.
 *  Configured here for NO: button press = gate open) */
static void btn_gate_down_cb(void *handle, void *usr_data)
{
    app_driver_report_gate(true);
}

static void btn_gate_up_cb(void *handle, void *usr_data)
{
    app_driver_report_gate(false);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * app_driver_event_handler — system events → LED indication
 * ═════════════════════════════════════════════════════════════════════════ */
int app_driver_event_handler(low_code_event_t *event)
{
    printf("%s: Event: %d\n", TAG, event->event_type);

    switch (event->event_type) {

    case LOW_CODE_EVENT_SETUP_MODE_START:
        printf("%s: Commissioning started\n", TAG);
        g_state.commissioning = true;
        s_anim_phase = 0;
        sw_timer_start(s_anim_timer);
        break;

    case LOW_CODE_EVENT_SETUP_MODE_END:
    case LOW_CODE_EVENT_SETUP_SUCCESSFUL:
    case LOW_CODE_EVENT_READY:
    case LOW_CODE_EVENT_NETWORK_CONNECTED:
        printf("%s: Device ready / setup complete\n", TAG);
        g_state.commissioning = false;
        sw_timer_stop(s_anim_timer);
        leds_update_state();
        break;

    case LOW_CODE_EVENT_NETWORK_DISCONNECTED:
        printf("%s: Network disconnected\n", TAG);
        /* Keep last known LED state — don't confuse user */
        break;

    case LOW_CODE_EVENT_IDENTIFICATION_BLINK:
    case LOW_CODE_EVENT_IDENTIFICATION_START:
        /* Blink white for identify */
        leds_write_rgb(180, 180, 180);
        break;

    case LOW_CODE_EVENT_IDENTIFICATION_STOP:
    case LOW_CODE_EVENT_IDENTIFICATION_STOP_EFFECT:
        leds_update_state();
        break;

    case LOW_CODE_EVENT_SETUP_FAILED:
        /* Fast red blink to indicate failure */
        leds_write_rgb(200, 0, 0);
        break;

    default:
        break;
    }

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * app_driver_init — called once from setup()
 * ═════════════════════════════════════════════════════════════════════════ */
int app_driver_init()
{
    printf("%s: Initializing garage controller driver\n", TAG);

    /* ── 1. Relay ──────────────────────────────────────────────────────── */
    relay_driver_init(GPIO_RELAY);
    relay_driver_set_power(GPIO_RELAY, false);  /* Start locked (fail-secure) */

    /* ── 2. WS2812B LED strip (3 LEDs chained on GPIO5) ───────────────── */
    ws2812_driver_init();
    ws2812_driver_regist_channel(WS2812_CHANNEL_RED,   (gpio_num_t)GPIO_LED_DATA);
    ws2812_driver_regist_channel(WS2812_CHANNEL_GREEN, (gpio_num_t)GPIO_LED_DATA);
    ws2812_driver_regist_channel(WS2812_CHANNEL_BLUE,  (gpio_num_t)GPIO_LED_DATA);
    leds_write_rgb(0, 150, 0);      /* Boot: green = all clear */

    /* ── 3. Commissioning animation timer (50ms periodic) ─────────────── */
    sw_timer_config_t anim_cfg = {
        .periodic   = true,
        .timeout_ms = 50,
        .handler    = anim_timer_cb,
        .arg        = NULL,
    };
    s_anim_timer = sw_timer_create(&anim_cfg);
    /* Timer is started only when commissioning begins */

    /* ── 4. Reset button (GPIO8, active-low, pull-up) ──────────────────── */
    button_config_t reset_cfg = {
        .long_press_time  = 3000,   /* 3s long press */
        .short_press_time = 50,
        .gpio_num         = GPIO_RESET_BTN,
        .pullup_en        = 1,
        .pulldown_en      = 0,
        .active_level     = 0,      /* active-low */
    };
    s_btn_reset = button_driver_create(&reset_cfg);
    button_driver_register_cb(s_btn_reset, BUTTON_SINGLE_CLICK,   btn_reset_single_click_cb, NULL);
    button_driver_register_cb(s_btn_reset, BUTTON_LONG_PRESS_UP,  btn_reset_long_press_cb,   NULL);

    /* ── 5. LIDAR input (GPIO9, active-low, pull-up) ───────────────────── */
    button_config_t lidar_cfg = {
        .long_press_time  = 0,
        .short_press_time = 50,     /* 50ms debounce */
        .gpio_num         = GPIO_LIDAR_IN,
        .pullup_en        = 1,
        .pulldown_en      = 0,
        .active_level     = 0,      /* contact closed = GPIO low = active */
    };
    s_btn_lidar = button_driver_create(&lidar_cfg);
    button_driver_register_cb(s_btn_lidar, BUTTON_PRESS_DOWN, btn_lidar_down_cb, NULL);
    button_driver_register_cb(s_btn_lidar, BUTTON_PRESS_UP,   btn_lidar_up_cb,   NULL);

    /* ── 6. Gate contact (GPIO10, active-low, pull-up) ─────────────────── */
    button_config_t gate_cfg = {
        .long_press_time  = 0,
        .short_press_time = 50,     /* 50ms debounce */
        .gpio_num         = GPIO_GATE_IN,
        .pullup_en        = 1,
        .pulldown_en      = 0,
        .active_level     = 0,      /* contact closed = GPIO low */
    };
    s_btn_gate = button_driver_create(&gate_cfg);
    button_driver_register_cb(s_btn_gate, BUTTON_PRESS_DOWN, btn_gate_down_cb, NULL);
    button_driver_register_cb(s_btn_gate, BUTTON_PRESS_UP,   btn_gate_up_cb,   NULL);

    printf("%s: Driver init complete\n", TAG);
    return 0;
}
