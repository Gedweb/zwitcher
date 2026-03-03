# ZWitcher 4CH - Zigbee Solid State Relay Controller

ESP32-C6 based 4-channel Zigbee 3.0 relay controller for Home Assistant ZHA.
Based on the light-mqtt project architecture, simplified for on/off SSR control (no PWM, no dimming).

## Hardware

- **MCU:** ESP32-C6-DevKitM-1 (RISC-V, built-in IEEE 802.15.4 radio)
- **Relay outputs (active LOW):**
  - Channel 1: GPIO 23
  - Channel 2: GPIO 22
  - Channel 3: GPIO 21
  - Channel 4: GPIO 20
- **Status LED:** GPIO 8 (WS2812 RGB)
- **Boot button:** GPIO 9 (hold 3s for factory reset)

## Zigbee Architecture

- **Device role:** Router (ZCZR, max 10 children)
- **4 endpoints** (1–4), each `ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID`
- Endpoint 1 carries: Basic (manufacturer/model), Identify, On/Off, OTA
- Endpoints 2–4 carry: Basic, Identify, On/Off
- Each endpoint appears as a separate switch in Home Assistant

## Data Flow

```
HA ZHA → Zigbee On/Off command to endpoint N
  → zigbee_action_handler (ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID)
  → RelayEvent queued to relayQueue
  → TaskRelayHandler: gpio_set_level + report attribute back
```

## FreeRTOS Tasks

| Task             | Priority | Stack | Purpose                                  |
|------------------|----------|-------|------------------------------------------|
| TaskZigbeeStack  | 5        | 8192  | Zigbee init, endpoint creation, main loop |
| TaskRelayHandler | 2        | 4096  | Dequeue relay events, set GPIO, report   |
| TaskStatusLED    | 1        | 2048  | Blue blink (pairing) / green (paired)    |
| TaskBootButton   | 1        | 4096  | 3s hold → erase zb_storage/zb_fct, restart |

## Project Structure

```
platformio.ini          - PlatformIO build config (ESP-IDF framework)
sdkconfig.defaults      - Zigbee, IEEE802154, OTA, 4MB flash, custom partitions
partitions_ota.csv      - Flash layout: nvs, factory, ota_0, zb_storage, zb_fct
CMakeLists.txt          - Root ESP-IDF CMake
src/
  CMakeLists.txt        - Component registration (Zigbee + GPIO drivers)
  idf_component.yml     - Dependencies: esp-zboss-lib, esp-zigbee-lib, led_strip
  secrets.h             - DEVICE_MANUFACTURER "HomeLab", DEVICE_MODEL_ID "ZWitcher4CH"
  dto.h / dto.cpp       - Relay (pin, state) and RelayEvent (endpoint, state) classes
  main.cpp              - Application: GPIO init, Zigbee stack, 4 tasks
```

## Key Differences from light-mqtt

- No PWM / LEDC — GPIO on/off only
- No sine wave ornament effect
- No master brightness / Level Control cluster
- No power supply relay — each SSR switches its own load
- 4 independent Zigbee endpoints instead of 1 dimmable light
- Simpler data model: Relay + RelayEvent instead of Led + IncomingEvent

## Device Identification

- Manufacturer: `HomeLab`
- Model: `ZWitcher4CH`
- OTA manufacturer code: `0x131B` (Espressif)
- OTA image type: `0x0000`, version: `0x00000001`
