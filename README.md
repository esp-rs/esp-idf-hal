# Safe Rust wrappers for the drivers in the [ESP IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)

[![CI](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml/badge.svg)](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/esp-idf-hal.svg)](https://crates.io/crates/esp-idf-hal)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://esp-rs.github.io/esp-idf-hal/esp_idf_hal/index.html)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)
[![Wokwi](https://img.shields.io/endpoint?url=https%3A%2F%2Fwokwi.com%2Fbadge%2Fclick-to-simulate.json)](https://wokwi.com/projects/332188235906155092)

## Highlights

* Implements the traits of [embedded-hal](https://github.com/rust-embedded/embedded-hal) `V0.2` as well as those of `V1.0.alpha`
* Supports almost all ESP IDF drivers: GPIO, SPI, I2C, TIMER, PWM, I2S, UART, etc.
* Blocking and `async` mode for each driver (`async` support in progress)

**You might want to also check out the ESP IDF [Services](https://github.com/esp-rs/esp-idf-svc) wrappers, and the raw bindings to ESP IDF in the [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys) crate!**

(For baremetal Rust ESP projects please check [esp-hal](https://github.com/esp-rs/esp-hal).)

## Build Prerequisites

Follow the [Prerequisites](https://github.com/esp-rs/esp-idf-template#prerequisites) section in the `esp-idf-template` crate.

## Examples

The examples could be built and flashed conveniently with [`cargo-espflash`](https://github.com/esp-rs/espflash/). To run e.g. the `ledc_simple` on an e.g. ESP32-C3:
(Swap the Rust target and example name with the target corresponding for your ESP32 MCU and with the example you would like to build)

with cargo-esptool v1.7:
```
$ ESP_IDF_VERSION=release/v4.4 cargo espflash --target riscv32imc-esp-espidf --example ledc_simple --monitor /dev/ttyUSB0
```

with cargo-esptool v2.0:
```
$ ESP_IDF_VERSION=release/v4.4 cargo espflash flash --target riscv32imc-esp-espidf --example ledc_simple --monitor
```

In order to run the examples on other chips you will most likely need to adapt at least the used pins.

## Setting up a "Hello, world!" binary crate with ESP IDF

Use the [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project. Everything would be arranged and built for you automatically - no need to manually clone the ESP IDF repository.

## More information

For more information, check out:
* The [Rust on ESP Book](https://esp-rs.github.io/book/)
* The [ESP Embedded Training](https://github.com/esp-rs/espressif-trainings)
* The [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project
* The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
* The [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) project
* The [embedded-svc](https://github.com/esp-rs/embedded-svc) project
* The [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys) project
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
