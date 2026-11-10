# ZWitcher 4CH

ESP32-C6 Zigbee 3.0 four-channel solid-state relay controller for Home Assistant ZHA. Built with ESP-IDF and PlatformIO.

- Four independent switches, plus internal MCU temperature reporting.
- Zigbee router with OTA updates; no Wi-Fi or MQTT required.
- Device: `HomeLab` / `ZWitcher4CH`.

## Hardware

**ESP32-C6-DevKitM-1**, 4 MB flash.

| Function | GPIO |
| --- | --- |
| Relays 1–4 (active LOW) | 23, 22, 21, 20 |
| WS2812 status LED | 8 |
| BOOT button | 9 |

Relays initialize OFF. Use suitable pull-ups/input circuitry to keep them OFF during reset. Mains loads require properly rated isolation, protection, and an enclosure.

## Build and flash

Install [PlatformIO](https://docs.platformio.org/en/latest/core/installation/index.html), connect the board over USB, then run:

```sh
pio run                    # Build
pio run --target upload    # Flash over serial
pio device monitor         # Logs at 115200 baud
```

## Pairing

Enable discovery in ZHA and power on the device. To reset pairing, **hold BOOT for 3 seconds, then release** when the LED blinks red.

- **Blue blink:** searching / not paired
- **Green heartbeat:** paired
- **Rapid red blink:** release BOOT to factory-reset

## OTA updates

Increase `OTA_FW_VERSION` in `src/main.cpp`, build, then wrap `firmware.bin` with Espressif's [image builder tool](https://github.com/espressif/esp-zigbee-sdk/tree/main/tools/image_builder_tool). Use manufacturer `0x131B`, **image type `0x0001` (relay)**, tag `0x0000`, and the same version as the firmware. Serve the generated image through ZHA's local OTA provider; a raw `.bin` will not work.

**Migration:** Older relay firmware shares image type `0x0000` with `zwitcher-lamp`. Remove legacy relay OTA images from ZHA's provider and serial-flash the new relay firmware before enabling mixed-project OTA updates. Keep lamp images at type `0x0000`; never wrap relay firmware with that type again. Different model names do not prevent this OTA collision.

Keep `ZB_LAYOUT_VERSION` unchanged for behavior-only updates. Bump it for endpoint/cluster changes, which require re-pairing. Partition changes require a serial reflash and re-pairing.

## Tests and privacy

See [test/README](test/README) for host parser tests and hardware checks.

`src/secrets.h` contains public device labels, not credentials. Zigbee keys live on the device; treat flash dumps as sensitive. Review Git author metadata and logs before sharing them.
