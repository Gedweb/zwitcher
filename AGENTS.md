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
- **Boot button:** GPIO 9 (hold 3s, release → factory reset)

## Zigbee Architecture

- **Device role:** Router (ZCZR, max 10 children)
- **Endpoints 1–4:** relay channels, each `ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID`
  - Endpoint 1 carries: Basic (manufacturer/model/SW build ID), Identify, On/Off, OTA client
  - Endpoints 2–4 carry: Basic, Identify, On/Off
- **Endpoint 5:** internal chip temperature sensor (`ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID`), updated every 30s
- Each relay endpoint appears as a separate switch in Home Assistant

**Changing the endpoint/cluster layout requires bumping `ZB_LAYOUT_VERSION`** (`main.cpp`):
reporting config persisted in `zb_storage` under the old layout crashes ZBOSS after an OTA.
On first boot with a new layout version the firmware erases `zb_storage` and re-enters
pairing. Behavior-only releases must NOT bump it; pairing then survives OTA.

## Data Flow

```
HA ZHA → Zigbee On/Off command to endpoint N
  → zigbee_action_handler (ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) → attr_value_handler
  → latest RelayStates snapshot overwrites single-slot relayQueue (nonblocking)
  → TaskRelayHandler: gpio_set_level + report attribute back (under esp_zb_lock)
```

Command bursts coalesce intermediate transitions while retaining the latest requested
state for every channel. The Zigbee callback must never block waiting for queue space:
the relay worker acquires the Zigbee lock to report changes.

## FreeRTOS Tasks

| Task             | Priority | Stack | Purpose                                  |
|------------------|----------|-------|------------------------------------------|
| TaskZigbeeStack  | 5        | 8192  | Zigbee init, endpoint creation, main loop |
| TaskRelayHandler | 2        | 4096  | Apply latest relay states, report changes |
| TaskStatusLED    | 1        | 2048  | Network status indication (see below)    |
| TaskBootButton   | 1        | 4096  | 3s hold + release → esp_zb_factory_reset |
| TaskTemperature  | 1        | 3072  | Chip temp → endpoint 5 every 30s         |

## Status LED

| State | Color | Behavior |
|-------|-------|----------|
| Not paired / searching | Blue | 500ms blink |
| Paired | Green | Short heartbeat every ~3.75s |
| Factory reset pending (BOOT held 3s) | Red | Rapid blink, release to reset |

The factory reset runs only after the button is released: rebooting with GPIO 9 low
would enter ROM download mode.

## Project Structure

```
platformio.ini          - PlatformIO build config (ESP-IDF framework)
sdkconfig.defaults      - Zigbee, IEEE802154, OTA, 4MB flash, custom partitions
partitions_ota.csv      - Flash layout: nvs, otadata, ota_0, ota_1, zb_storage, zb_fct
CMakeLists.txt          - Root ESP-IDF CMake
src/
  CMakeLists.txt        - Component registration (Zigbee, GPIO, tsens, app_update)
  idf_component.yml     - Dependencies: esp-zboss-lib ~1.6, esp-zigbee-lib ~1.6, led_strip ^3.0
  secrets.h             - DEVICE_MANUFACTURER "HomeLab", DEVICE_MODEL_ID "ZWitcher4CH"
  dto.h / dto.cpp       - Relay (pin, state) and RelayEvent (endpoint, state) classes
  main.cpp              - Application: GPIO init, Zigbee stack, OTA, 5 tasks
```

### Partition Table (4MB flash)

Two OTA slots (no factory partition): each OTA writes the inactive slot and `otadata`
selects the boot slot. Changing the layout requires a serial reflash and re-pairing
(zb_storage moves).

```csv
nvs,        data, nvs,      0x9000,   0x6000
otadata,    data, ota,      0xf000,   0x2000
phy_init,   data, phy,      0x11000,  0x1000
ota_0,      app,  ota_0,    0x20000,  0x180000
ota_1,      app,  ota_1,    0x1a0000, 0x180000
nvs_key,    data, nvs_keys, 0x320000, 0x1000
zb_storage, data, fat,      0x321000, 0x8000
zb_fct,     data, fat,      0x329000, 0x1000
```

## Zigbee OTA Updates

A raw `firmware.bin` will NOT work - ZHA matches images by the Zigbee OTA file header
(manufacturer 0x131B, image type 0x0000, file version), so the binary must be wrapped:

1. Bump `OTA_FW_VERSION` in `src/main.cpp` (served version must be greater than the running one)
2. Build: `pio run`
3. Wrap the binary with esp-zigbee-sdk's
   [image_builder_tool](https://github.com/espressif/esp-zigbee-sdk/tree/main/tools/image_builder_tool):
   ```bash
   python3 image_builder_tool.py -m 0x131B -i 0x0000 -v 0x00000002 \
       --tag 0x0000 .pio/build/esp32-c6-devkitm-1/firmware.bin
   ```
4. Copy the generated `.zigbee`/`.ota` file into ZHA's local OTA directory —
   the directory must exist or ZHA fails to start (`configuration.yaml`):
   ```yaml
   zha:
     zigpy_config:
       ota:
         extra_providers:
           - type: advanced
             path: /config/zigbee_ota
             warning: <zigpy's exact consent sentence - the ZHA startup error prints it>
   ```
5. Restart Home Assistant, open the ZHA device page → "Update Firmware"
6. Device downloads via Zigbee, writes to the inactive OTA slot, verifies, and reboots

The OTA client queries the server within a minute of joining/reboot, then hourly.
Firmware version is shown on the ZHA device page (SW build ID).

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
- OTA image type: `0x0000`, version: `OTA_FW_VERSION` in `main.cpp`

## Development Commands

```bash
pio run                    # Build
pio run --target upload    # Build and flash (serial)
pio device monitor         # Serial monitor, 115200 baud
pio run --target fullclean # Full clean (removes sdkconfig)
```
