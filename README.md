# An embedded-hal implementation for Rust on ESP32 and ESP-IDF

![CI](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml/badge.svg)

* This crate is intended for usage in ESP32 embedded projects that utilize and link with the **ESP-IDF SDK**.
* For embedded projects that don't need Rust STD support, WiFi or BLE (and thus don't link with the ESP-IDF SDK), please check [esp32-hal](https://github.com/esp-rs/esp32-hal).

For more information, check out:
* The [Rust ESP32 STD compiler fork](https://github.com/ivmarkov/rust)
* The ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello)
* The [esp-idf-svc](https://github.com/ivmarkov/esp-idf-svc) project
* The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
* The [esp32-hal](https://github.com/esp-rs/esp32-hal) project

## Hardware Notes

Each chip has a number of GPIO pins which are generally used by the `SPI0` and `SPI1` peripherals in order to connect external PSRAM and/or SPI Flash memory. The datasheets explicitly state that these are not recommended for use, however this crate includes them anyways for completeness.

Please refer to the table below to determine the pins which are not recommended for use for your chip.

| Chip         |       GPIOs        |
| ------------ | :----------------: |
| **ESP32**    |  6 - 11, 16 - 17   |
| **ESP32-C3** |      12 - 17       |
| **ESP32-S2** |      26 - 32       |
| **ESP32-S3** | 26 - 32, 33 - 37\* |

_\* When using Octal Flash and/or Octal PSRAM_
