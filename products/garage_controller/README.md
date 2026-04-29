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
  - [Relay Logic](#relay-logic)
  - [LED Status Indicator](#led-status-indicator)
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
- **Thread Router** — the ESP32-C6 acts as a Thread Router, extending the Thread mesh for other devices
- **Door lock relay** — controls a 12V/24V electric strike via the on-board optocoupled relay
- **SICK LIDAR door sensor** — detects presence at the side door; automatically activates the relay while detection is active
- **Gate contact sensor** — monitors the garage gate open/closed state via a potential-free magnetic contact; reported to HA only
- **Relay priority logic** — LIDAR detection overrides the HA switch: if LIDAR is active, the relay stays on even if the switch is turned off
- **Internal status LED** — single on-board LED mirrors the gate sensor state
- **Reset button** — short press triggers factory reset for re-pairing
- **No local toolchain** — built entirely in GitHub Codespaces (browser-based VS Code), flashed via WebUSB

---

## Hardware

### Bill of Materials

| Component | Specification | Qty |
|---|---|---|
| ESP32-C6 Relay X1 V1.1 | ESP32-C6, USB-C, on-board relay | 1 |
| SICK LIDAR | Potential-free NC/NO contact output | 1 |
| Gate contact | Magnetic, potential-free NC/NO | 1 |
| Tactile button | SPST NO, 6mm | 1 |
| Resistor 10 kΩ | 1/4W (optional, external pull-up for inputs) | 3 |
| Door strike | 12V or 24V electric strike/solenoid | 1 |
| Strike PSU | 12V or 24V, sized to strike current | 1 |

### GPIO Assignments

> **LP core GPIO note:** The ESP32-C6 application runs on the LP RISC-V core. LP GPIOs (0–7) can be driven via the LP IO controller (`relay_driver`). HP GPIOs (8+) require the HP GPIO controller — use `system_digital_write()` from `system.h` for those. GPIO19 (relay) uses `system_digital_write`; GPIO2 (LED) uses `relay_driver`.

| GPIO | Direction | Function |
|---|---|---|
| GPIO19 | Output | Relay IN — **hardwired on board** (NO contact) |
| GPIO2 | Output | Internal programmable LED |
| GPIO5 | Input | Reset button — external tactile switch to GND |
| GPIO6 | Input | SICK LIDAR contact (active-low, pull-up) — door sensor |
| GPIO7 | Input | Gate door contact (active-low, pull-up) — gate sensor |

> **GPIO0 / relay note:** The relay is Normally Open (NO), so the coil does not pull GPIO0 low at power-on. The ESP32-C6 boots normally. GPIO0 is not a strapping pin on the ESP32-C6.

### Wiring

```
Left header (single connector):

Board GND ──────────────────────────── GND (button, contacts)
GPIO19    ──── hardwired to relay ────  (no external wire needed)
GPIO5     ──────────────────────────── Reset button ── GND
GPIO6     ──────────────────────────── LIDAR contact ── GND
GPIO7     ──────────────────────────── Gate contact ── GND

Board relay terminals:
  NO  ──── Door strike ──── Strike PSU +
  COM ──── Strike PSU −
           (relay energised = door unlocked)
```

**Key wiring rules:**
- Relay is **NO (Normally Open)** and **hardwired to GPIO19** on the board. LOW = off (door locked), HIGH = on (door unlocked).
- All inputs are **active-low**: contact closed → GPIO pulled to GND → sensor active.
- LIDAR and gate contacts must be **potential-free** (galvanic isolation). If the sensor provides powered contacts, add an opto-isolator between the contact and the GPIO.
- The strike actuator is powered by its own dedicated supply — the relay contacts are galvanically isolated from the ESP32.

---

## Software Architecture

### Matter Data Model

The device exposes four Matter endpoints:

| Endpoint | Device Type | Matter Code | Maps to |
|---|---|---|---|
| EP1 | Root Node | 0x0016 | Managed by system firmware |
| EP2 | Occupancy Sensor | 0x0107 | SICK LIDAR door sensor (GPIO6) |
| EP3 | Occupancy Sensor | 0x0107 | Gate door contact (GPIO7) |
| EP4 | On/Off Plugin Unit | 0x010A | Relay / door lock (GPIO19) |

The data model is defined in `configuration/data_model_thread.zap` and generated by `generate_zap.py` from the existing `occupancy_sensor` and `socket_2_channel` reference products.

### Firmware Structure

The firmware follows the ESP LowCode Matter split architecture:

- **System Firmware (HP core)** — pre-built binary provided by Espressif. Handles the full Matter/Thread stack, BLE commissioning, OTA, and security. Never modified.
- **Application Firmware (LP core)** — our code (~20KB). Runs on the low-power RISC-V core without an OS, in a simple `setup()` / `loop()` style.

```
main/
├── app_main.cpp      # Entry point: main(), setup(), loop(), system callbacks
├── app_driver.cpp    # All hardware logic: relay, LED, button, sensor reporting
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
| `system` / `relay_driver` | GPIO19 relay (`system_digital_write`), GPIO2 LED (`relay_driver`) |
| `button_driver` | GPIO5/6/7 debounced input with event callbacks |
| `low_code` | Matter feature updates and system events |

### Relay Logic

The relay (EP4) is controlled by two sources with the following priority:

| State | Relay |
|---|---|
| LIDAR detecting (door sensor active) | **ON** — overrides everything |
| HA switch ON, LIDAR not detecting | ON |
| HA switch OFF, LIDAR not detecting | OFF |
| HA switch OFF, LIDAR detecting | **ON** — LIDAR has priority |

In code: `relay_on = ha_switch_on || lidar_active`

The HA switch represents the user's manual command. LIDAR detection temporarily overrides it — the relay turns on for the duration of the detection and turns off again when the LIDAR clears (unless the HA switch is also on).

The gate contact (EP3/GPIO7) does **not** affect the relay — it is reported to HA as an informational sensor only.

### LED Status Indicator

The on-board LED (GPIO2) mirrors the gate sensor state:

| LED | Meaning |
|---|---|
| OFF | Gate closed |
| ON  | Gate open |

---

## Build & Flash

### Prerequisites

- GitHub account (free tier includes 60h/month Codespaces)
- Chrome or Edge browser (required for WebUSB/WebSerial)
- ESP32-C6 Relay X1 V1.1 connected to your laptop via USB-C

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
│   ├── app_priv.h
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
```

The on-board LED should be off (gate closed state).

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
| `binary_sensor.garage_controller_occupancy` | Binary sensor | SICK LIDAR — presence detected at door |
| `binary_sensor.garage_controller_occupancy_2` | Binary sensor | Gate contact — gate open/closed |
| `switch.garage_controller` | Switch | Relay — door strike (on = unlocked) |

Rename these in HA to meaningful names like `binary_sensor.door_presence`, `binary_sensor.gate_open`, and `switch.door_strike`.

### Suggested Automations

**Alert when gate is open:**
```yaml
trigger:
  - platform: state
    entity_id: binary_sensor.gate_open
    to: "on"
action:
  - service: notify.mobile_app
    data:
      message: "Gate is open!"
```

**Auto-lock door after 30 seconds if manually unlocked:**
```yaml
trigger:
  - platform: state
    entity_id: switch.door_strike
    to: "on"
    for: "00:00:30"
condition:
  - condition: state
    entity_id: binary_sensor.door_presence
    state: "off"
action:
  - service: switch.turn_off
    target:
      entity_id: switch.door_strike
```

---

## Reset & Re-pairing

| Action | How |
|---|---|
| Factory reset / re-pair | Short press reset button |

After factory reset the device re-enters commissioning mode. Remove the old device from Home Assistant before re-pairing.

To re-pair after a factory reset, run **Upload Configuration** again in Codespaces to generate new certificates, then re-commission in HA with the new pairing code.

---

## Project File Structure

```
products/garage_controller/
├── main/
│   ├── app_main.cpp          # Main loop, system callbacks, feature dispatch
│   ├── app_driver.cpp        # Hardware: relay, LED, buttons, sensor reporting
│   ├── app_priv.h            # Pin defs, endpoint IDs, function declarations
│   └── CMakeLists.txt        # Component registration
├── configuration/
│   ├── data_model_thread.zap # Matter data model — Thread (generated)
│   ├── data_model_wifi.zap   # Matter data model — Wi-Fi (generated, unused)
│   ├── product_info.json     # Device metadata for DAC generation
│   ├── product_config.json   # Test mode config
│   └── cd_cert_fff1_8000.der # Certification Declaration (Espressif test cert)
├── generate_zap.py           # Generates ZAP files from reference products
├── CMakeLists.txt            # Top-level build
├── sdkconfig                 # Full SDK config (copied from light_cw_pwm)
└── sdkconfig.defaults        # Minimal overrides
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Relay doesn't click | HP GPIO driven via wrong API | `GPIO_RELAY` must be 19; relay init must use `system_set_pin_mode` + `system_digital_write`, not `relay_driver` (LP-only) |
| Stuck in download mode after flash | GPIO0 held low at boot | Press EN/RST button, or unplug and replug USB |
| Sensors not triggering | Wiring or pull-up issue | Short GPIO6 or GPIO7 to GND with a wire to test; check serial for sensor log |
| Relay on but HA switch shows off | Expected — LIDAR has priority | Relay stays on while LIDAR detects regardless of HA switch state |
| Only occupancy sensors visible in HA | EP4 ZAP `endpointTypeIndex` mismatch | Re-run `generate_zap.py` and re-commission |
| Wrong pairing code | Multiple Upload Configuration runs | Always use code from `manual_code.info`, not terminal history |
| Device not found during commissioning | Not in commissioning mode | Press reset button to reboot into commissioning mode |
| Thread device not reachable | Thread Border Router offline | Verify border router is online in HA Thread panel |

---

## Known Limitations

- **Beta SDK:** ESP LowCode Matter is currently in beta. APIs may change between releases.
- **On/Off Plugin Unit:** The relay appears as a switch/plug in HA rather than a door lock entity, as the Matter Door Lock device type is not yet available in the LowCode SDK.
- **HA switch state during LIDAR override:** When LIDAR activates the relay, EP4 in HA continues to show the last switch state rather than the actual relay state. The relay is physically on, but HA may show the switch as off.
- **Test certificates:** The generated DAC certificates are Espressif test certificates suitable for development. Production deployment requires Espressif's Matter manufacturing service for production-grade certificates.
- **Re-pairing after factory reset:** A new Upload Configuration run is required to generate fresh certificates after each factory reset.
