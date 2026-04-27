# ESP32-C6 Garage Side-Door Controller

A Matter over Thread smart home device built with [ESP LowCode Matter](https://github.com/espressif/esp-lowcode-matter), running on an **ESP32-C6 Relay X1 V1.1** — a compact PCB with the relay hardwired on-board, USB-C power, and a wide-voltage (7–60V) auxiliary input. Controls a garage side-door electric strike, monitors a SICK LIDAR presence sensor and a gate contact, and reports everything natively to Home Assistant via the Matter protocol — no local toolchain installation required.

---

## Table of Contents

- [Features](#features)
- [Hardware](#hardware)
  - [Bill of Materials](#bill-of-materials)
  - [GPIO Assignments](#gpio-assignments)
  - [Wiring](#wiring)
- [Software Architecture](#software-architecture)
  - [Matter Data Model](#matter-data-model)
  - [Firmware Structure](#firmware-structure)
  - [LED Status Indicators](#led-status-indicators)
- [Build & Flash](#build--flash)
  - [Prerequisites](#prerequisites)
  - [Step 1 — Fork and set up Codespaces](#step-1--fork-and-set-up-codespaces)
  - [Step 2 — Create the product](#step-2--create-the-product)
  - [Step 3 — Generate the ZAP data model](#step-3--generate-the-zap-data-model)
  - [Step 4 — Prepare device](#step-4--prepare-device)
  - [Step 5 — Upload configuration](#step-5--upload-configuration)
  - [Step 6 — Upload code](#step-6--upload-code)
- [Home Assistant Integration](#home-assistant-integration)
  - [Commissioning](#commissioning)
  - [Entities](#entities)
  - [Suggested Automations](#suggested-automations)
- [Reset & Re-pairing](#reset--re-pairing)
- [Project File Structure](#project-file-structure)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)

---

## Features

- **Matter over Thread** — native integration with Home Assistant, Apple Home, Google Home, and Amazon Alexa via your existing Thread Border Router
- **Thread Router** — the ESP32-C6 acts as a Thread Router (not a Border Router), extending the Thread mesh for other devices
- **Door lock relay** — controls a 12V/24V electric strike via an optocoupled relay module
- **SICK LIDAR input** — monitors a potential-free contact from an industrial SICK LIDAR distance sensor for presence/intrusion detection
- **Gate contact input** — monitors the garage door open/closed state via a potential-free magnetic contact
- **WS2812B status LEDs** — three addressable RGB LEDs indicate device and sensor state at a glance
- **Reset button** — short press reboots the device; 5-second hold triggers Matter factory reset for re-pairing
- **No local toolchain** — built entirely in GitHub Codespaces (browser-based VS Code), flashed via WebUSB

---

## Hardware

### Bill of Materials

| Component | Specification | Qty |
|---|---|---|
| ESP32-C6 DevKit | ESP32-C6, USB-C, 5V via VIN | 1 |
| ~~Relay module~~ | **Hardwired on board** (Songle SRD-05VDC-SL-C) | — |
| WS2812B LED | 5V addressable RGB, chainable | 3 |
| Tactile button | SPST NO, 6mm | 1 |
| SICK LIDAR | Potential-free NC/NO contact output | 1 |
| Gate contact | Magnetic, potential-free NC/NO | 1 |
| Resistor 330 Ω | 1/4W | 1 |
| Capacitor 100 µF | Electrolytic, 10V+ | 1 |
| Capacitor 100 nF | Ceramic | 3 |
| Resistor 10 kΩ | 1/4W (optional, external pull-up) | 3 |
| Power supply | 5V, ≥1A regulated | 1 |
| Door strike | 12V or 24V electric strike/solenoid | 1 |
| Strike PSU | 12V or 24V, sized to strike current | 1 |

### GPIO Assignments

All pins selected to avoid ESP32-C6 boot/flash strapping pins (GPIO0, GPIO2, GPIO12, GPIO15).

| GPIO | Direction | Function |
|---|---|---|
| GPIO0 | Output | Relay IN — **hardwired on board** (NO contact) |
| GPIO4 | Output | WS2812B DIN — LED chain (via 330 Ω) |
| GPIO5 | Input | Reset button — external tactile switch to GND |
| GPIO6 | Input | SICK LIDAR contact (active-low, pull-up) |
| GPIO7 | Input | Gate door contact (active-low, pull-up) |

> **GPIO0 / Boot pin note:** The relay is Normally Open (NO), so GPIO0 is not pulled low by the relay at power-on. The ESP32-C6 boots normally. Avoid energising the relay programmatically during the first 100ms after boot.

### Wiring

```
Left header (single connector — solder only this one):

Board 5V  ──────────────────────────── +5V LED strip
Board GND ──────────────────────────── GND (LEDs, button, contacts)
GPIO0     ──── hardwired to relay ────  (no external wire needed)
GPIO4     ── 330Ω ─── LED0 DIN
                       LED0 DOUT ─── LED1 DIN
                                    LED1 DOUT ─── LED2 DIN
GPIO5     ──────────────────────────── Reset button ── GND
GPIO6     ──────────────────────────── LIDAR contact ── GND
GPIO7     ──────────────────────────── Gate contact ── GND

Board relay terminals:
  NO  ──── Door strike ──── Strike PSU +
  COM ──── Strike PSU −
           (relay energised = door unlocked)
```

**Key wiring rules:**
- Relay is **NO (Normally Open)** and **hardwired to GPIO0** on the board — no relay wiring needed. LOW = locked (fail-secure), HIGH = unlocked.
- All inputs are **active-low**: contact closed → GPIO pulled to GND → sensor active.
- LIDAR and gate contacts must be **potential-free** (galvanic isolation). If the sensor provides powered contacts, add an opto-isolator between the contact and the GPIO.
- **Single header:** All signals (GPIO4–7), 5V, and GND are on the left header — solder only this one connector.
- Add a **100 µF** electrolytic cap across the 5V rail at the LED strip entry point, plus a **100 nF** ceramic cap per LED.
- The strike actuator is powered by its own dedicated supply — the relay contacts are galvanically isolated from the ESP32.

---

## Software Architecture

### Matter Data Model

The device exposes four Matter endpoints:

| Endpoint | Device Type | Matter Code | Maps to |
|---|---|---|---|
| EP1 | Root Node | 0x0016 | Managed by system firmware |
| EP2 | Occupancy Sensor | 0x0107 | SICK LIDAR (GPIO9) |
| EP3 | Occupancy Sensor | 0x0107 | Gate door contact (GPIO10) |
| EP4 | On/Off Plugin Unit | 0x010A | Relay / door lock (GPIO4) |

The data model is defined in `configuration/data_model_thread.zap` and generated by `generate_zap.py` from the existing `occupancy_sensor` and `socket_2_channel` reference products.

### Firmware Structure

The firmware follows the ESP LowCode Matter split architecture:

- **System Firmware (HP core)** — pre-built binary provided by Espressif. Handles the full Matter/Thread stack, BLE commissioning, OTA, and security. Never modified.
- **Application Firmware (LP core)** — our code (~20KB). Runs on the low-power RISC-V core without an OS, in a simple `setup()` / `loop()` style.

```
main/
├── app_main.cpp      # Entry point: main(), setup(), loop(), system callbacks
├── app_driver.cpp    # All hardware logic: relay, LEDs, buttons, sensor reporting
└── app_priv.h        # Pin definitions, endpoint IDs, function declarations
configuration/
├── data_model_thread.zap   # Matter data model (Thread variant)
├── data_model_wifi.zap     # Matter data model (Wi-Fi variant, unused)
├── product_info.json       # Vendor/product metadata for cert generation
├── product_config.json     # Test mode and device management config
└── cd_cert_fff1_8000.der   # Certification Declaration certificate
generate_zap.py             # Script to (re)generate ZAP files from reference products
CMakeLists.txt              # Build configuration
sdkconfig.defaults          # SDK configuration defaults
```

**Driver components used:**

| Component | Used for |
|---|---|
| `relay_driver` | GPIO4 relay control |
| `button_driver` | GPIO8/9/10 debounced input with event callbacks |
| `ws2812_driver` | WS2812B LED strip via RMT |
| `sw_timer` | 50ms periodic timer for commissioning animation |
| `low_code` | Matter feature updates and system events |

### LED Status Indicators

Three WS2812B LEDs chained on GPIO5 show device state via combined colour encoding:

| LED Colour | Meaning |
|---|---|
| 🟢 Green | All clear — door locked, no presence, gate closed |
| 🔵 Blue | Door unlocked (relay energised) |
| 🔴 Red | Gate open |
| 🟠 Amber | LIDAR triggered — person detected |
| ⚪ White blink | Reset button held — release for reboot |
| 💙 Breathing blue | Commissioning / pairing mode |

Priority order when multiple states are active: unlocked > gate open > LIDAR active > all clear.

---

## Build & Flash

### Prerequisites

- GitHub account (free tier includes 60h/month Codespaces)
- Chrome or Edge browser (required for WebUSB/WebSerial)
- ESP32-C6 DevKit connected to your laptop via USB-C

### Step 1 — Fork and set up Codespaces

1. Fork [espressif/esp-lowcode-matter](https://github.com/espressif/esp-lowcode-matter)
2. In your fork, click **Code → Codespaces → Create codespace on main**
3. Wait ~2 minutes for the environment to initialise
4. When the terminal shows **"LowCode is Ready"**, proceed

### Step 2 — Create the product

In the Codespaces terminal:

```bash
mkdir -p /workspaces/esp-lowcode-matter/products/garage_controller/main
mkdir -p /workspaces/esp-lowcode-matter/products/garage_controller/configuration
```

Copy the following files from this repository into the Codespace:

```
products/garage_controller/
├── main/
│   ├── app_main.cpp
│   ├── app_driver.cpp
│   └── app_priv.h
│   └── CMakeLists.txt
├── CMakeLists.txt
├── sdkconfig.defaults
├── generate_zap.py
└── configuration/
    ├── product_info.json
    ├── product_config.json
    └── cd_cert_fff1_8000.der
```

Copy the cert and config files from the reference product:

```bash
cp /workspaces/esp-lowcode-matter/products/occupancy_sensor/configuration/cd_cert_fff1_8000.der \
   /workspaces/esp-lowcode-matter/products/garage_controller/configuration/

cp /workspaces/esp-lowcode-matter/products/occupancy_sensor/configuration/product_config.json \
   /workspaces/esp-lowcode-matter/products/garage_controller/configuration/
```

Copy the working sdkconfig:

```bash
cp /workspaces/esp-lowcode-matter/products/light_cw_pwm/sdkconfig \
   /workspaces/esp-lowcode-matter/products/garage_controller/sdkconfig
```

### Step 3 — Generate the ZAP data model

```bash
cd /workspaces/esp-lowcode-matter/products/garage_controller
python3 generate_zap.py
```

Expected output:
```
Written: .../data_model_thread.zap  (~115000 bytes)
Written: .../data_model_wifi.zap    (~115000 bytes)
```

### Step 4 — Prepare device

1. Connect the ESP32-C6 via USB
2. In the LowCode status bar: **Select Product** → `garage_controller`
3. Click **Select Port** → choose your device port
4. Click **Prepare Device** — flashes the pre-built system firmware (~30s, once per device)

### Step 5 — Upload configuration

Click **Upload Configuration** in the status bar. This generates:
- A unique test DAC certificate for your device
- A unique QR code and manual pairing code
- All manufacturing binaries flashed to the device

Note the manual pairing code from the terminal output, or find it at:
```bash
cat /workspaces/esp-lowcode-matter/products/garage_controller/configuration/output/<MAC>/manual_code.info
```

### Step 6 — Upload code

Click **Upload Code**. The LP core application builds (~20KB) and flashes. Monitor output:

```
Garage Controller starting
Driver init complete
Device ready on Thread network
```

LEDs should show steady green (all clear).

---

## Home Assistant Integration

### Commissioning

1. In Home Assistant: **Settings → Devices & Services → Add Integration → Matter**
2. Click **Commission a new device**
3. Enter the manual pairing code or scan the QR code
4. HA discovers the device via Thread through your existing Thread Border Router

> **Note:** If commissioning fails, verify the manual code from `manual_code.info` — it is generated fresh each time **Upload Configuration** runs and may differ from earlier terminal output.

### Entities

After commissioning, three entities appear under the **Garage Controller** device:

| Entity | Type | Description |
|---|---|---|
| `binary_sensor.garage_controller_occupancy` | Binary sensor | SICK LIDAR — person detected in area |
| `binary_sensor.garage_controller_occupancy_2` | Binary sensor | Gate door — open/closed state |
| `switch.garage_controller` | Switch | Relay — door lock (on = unlocked) |

Rename these in HA to meaningful names like `binary_sensor.lidar_presence`, `binary_sensor.gate_open`, and `switch.door_strike`.

### Suggested Automations

**Alert when gate is open and LIDAR detects presence:**
```yaml
trigger:
  - platform: state
    entity_id: binary_sensor.gate_open
    to: "on"
condition:
  - condition: state
    entity_id: binary_sensor.lidar_presence
    state: "on"
action:
  - service: notify.mobile_app
    data:
      message: "Gate open with presence detected!"
```

**Auto-lock door after 30 seconds:**
```yaml
trigger:
  - platform: state
    entity_id: switch.door_strike
    to: "on"
    for: "00:00:30"
action:
  - service: switch.turn_off
    target:
      entity_id: switch.door_strike
```

---

## Reset & Re-pairing

| Action | How |
|---|---|
| Soft reboot | Press reset button briefly (< 5s) |
| Factory reset / re-pair | Hold reset button for > 5 seconds — LEDs flash white, then amber |

After factory reset the device re-enters commissioning mode (LEDs breathe blue). Remove the old device from Home Assistant before re-pairing.

To re-pair after a factory reset, run **Upload Configuration** again in Codespaces to generate new certificates, then re-commission in HA with the new pairing code.

---

## Project File Structure

```
products/garage_controller/
├── main/
│   ├── app_main.cpp          # Main loop, system callbacks, feature dispatch
│   ├── app_driver.cpp        # Hardware: relay, LEDs, buttons, sensor reporting
│   ├── app_priv.h            # Pin defs, endpoint IDs, function declarations
│   └── CMakeLists.txt        # Component registration (low_code, relay, button, light, sw_timer)
├── configuration/
│   ├── data_model_thread.zap # Matter data model — Thread (generated)
│   ├── data_model_wifi.zap   # Matter data model — Wi-Fi (generated, unused)
│   ├── product_info.json     # Device metadata for DAC generation
│   ├── product_config.json   # Test mode config
│   └── cd_cert_fff1_8000.der # Certification Declaration (Espressif test cert)
├── generate_zap.py           # Generates ZAP files from reference products
├── CMakeLists.txt            # Top-level build (includes low_code.cmake, component dirs)
├── sdkconfig                 # Full SDK config (copied from light_cw_pwm)
└── sdkconfig.defaults        # Minimal overrides
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Stuck in download mode after flash | GPIO0 held low at boot | Press EN/RST button, or unplug and replug USB |
| LEDs don't light | Missing 330Ω resistor or 5V rail issue | Check wiring, confirm 5V at LED strip |
| Relay doesn't click | Wrong relay type or GPIO level | Confirm relay is 5V optocoupled; check GPIO4 with multimeter |
| Only occupancy sensors visible in HA | EP4 ZAP compliance error | Re-run `generate_zap.py` using `socket_2_channel` as relay source |
| Wrong pairing code | Multiple Upload Configuration runs | Always use code from `manual_code.info`, not terminal history |
| Device not found during commissioning | Not in commissioning mode | Check LEDs for breathing blue; press reset button to reboot |
| Thread device not reachable | Thread Border Router offline | Verify border router is online in HA Thread panel |
| `low_code` component not found | Missing `low_code.cmake` include | Verify top-level `CMakeLists.txt` matches the pattern above |

---

## Known Limitations

- **Beta SDK:** ESP LowCode Matter is currently in beta. APIs may change between releases.
- **Single-pixel WS2812B:** The beta `ws2812_driver` drives a single-pixel strip. Three LEDs share one colour state rather than displaying independently per sensor. This can be upgraded when Espressif extends the multi-LED API.
- **On/Off Plugin Unit:** The relay appears as a switch/plug in HA rather than a door lock entity, as the Matter Door Lock device type is not yet available in the LowCode SDK.
- **Test certificates:** The generated DAC certificates are Espressif test certificates suitable for development. Production deployment requires Espressif's Matter manufacturing service for production-grade certificates.
- **Re-pairing after factory reset:** A new Upload Configuration run is required to generate fresh certificates after each factory reset.
