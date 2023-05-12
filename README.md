# An [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementation for ESP32[-XX] + ESP-IDF

[![CI](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml/badge.svg)](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml)
![crates.io](https://img.shields.io/crates/v/esp-idf-hal.svg)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://esp-rs.github.io/esp-idf-hal/esp_idf_hal/index.html)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)

* This crate is intended for usage in ESP32[-XX] embedded projects that utilize and link with the **ESP-IDF SDK**.
* For embedded projects that don't need Rust STD support, WiFi or BLE (and thus don't link with the ESP-IDF SDK), please check [esp-hal](https://github.com/esp-rs/esp-hal).

For more information, check out:
* The [Rust on ESP Book](https://esp-rs.github.io/book/)
* The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
* The [esp-hal](https://github.com/esp-rs/esp-hal) project
* The [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project
* The [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys) project
* The [embedded-svc](https://github.com/esp-rs/embedded-svc) project
* The [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) project
* The [Rust for Xtensa toolchain](https://github.com/esp-rs/rust-build)
* The [Rust-with-STD demo](https://github.com/ivmarkov/rust-esp32-std-demo) project

## Hardware Notes

Each chip has a number of GPIO pins which are generally used by the `SPI0` and `SPI1` peripherals in order to connect external PSRAM and/or SPI Flash memory. The datasheets explicitly state that these are not recommended for use, however this crate includes them anyways for completeness.

Please refer to the table below to determine the pins which are not recommended for use for your chip.

| Chip         |       GPIOs        |
| ------------ | :----------------: |
| **ESP32**    |  6 - 11, 16 - 17   |
| **ESP32-C2** |      12 - 17       |
| **ESP32-C3** |      12 - 17       |
| **ESP32-C6** |      24 - 30       |
| **ESP32-H2** |      15 - 21       |
| **ESP32-S2** |      26 - 32       |
| **ESP32-S3** | 26 - 32, 33 - 37\* |

_\* When using Octal Flash and/or Octal PSRAM_

## Examples

The examples could be built and flashed conveniently with [`cargo-espflash`](https://github.com/esp-rs/espflash/). To run `ledc_simple` on an ESP32-C3:
```
$ cargo espflash --release --target riscv32imc-esp-espidf --example ledc_simple --monitor /dev/ttyUSB0
```

In order to run the examples on other chips you will most likely need to adapt at least the used pins.
