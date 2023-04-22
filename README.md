# An [embedded-hal](https://github.com/rust-embedded/embedded-hal) implementation for ESP32[-XX] + ESP-IDF

[![CI](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml/badge.svg)](https://github.com/esp-rs/esp-idf-hal/actions/workflows/ci.yml)
![crates.io](https://img.shields.io/crates/v/esp-idf-hal.svg)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://esp-rs.github.io/esp-idf-hal/esp_idf_hal/index.html)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)
![Wokwi](https://img.shields.io/endpoint?label=wokwi&url=https%3A%2F%2Fwokwi.com%2Fbadge%2Fsimulate-in-wokwi.json)

- Run ESP-IDF's FreeRTOS using safe Rust code
- Provides wrappers and abstractions for ESP Peripherals like GPIO, SPI, I2C, TIMER, PWM etc..
- Enables running Rust standard library code on a ESP
- Contains implementations of portable embedded traits defined in the [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
- For embedded projects that don't need Rust STD support, please check [esp-hal](https://github.com/esp-rs/esp-hal).



# Getting Starded

To get started quickly, you have two options:

## Option 1: Zero Setup with the Wokwi Online Simulator

You can use the [Wokwi online simulator](https://wokwi.com/projects/332188235906155092) to experiment with this crate.

## Option 2: Use cargo-generate
Please make sure you have installed all [prerequisites](https://github.com/esp-rs/esp-idf-hal#prerequisites) first!
### Generate
```bash
cargo generate --git https://github.com/esp-rs/esp-idf-template
```
### Build
```bash 
cd <your-project-name>
cargo build
```
### Flash + Run + Monitor
```bash
cargo run
```

### Prerequisites

To use this crate, you will need:

1. Install Rust via rustup
2. Install the `cargo-generate` and `ldproxy` tools via cargo
    ```bash
    cargo install cargo-generate ldproxy
    ```
3. (Linux & macOS) Install libuv
    ```bash
    # macOS
    brew install libuv
    # Debian/Ubuntu/etc.
    apt-get install libuv-dev
    # Fedora
    dnf install systemd-devel
    ```
4. Install toolchain
    *  For RISCV-based chips:
        - Have Clang11 and Python 3.7 or greater installed
        - Install the nightly Rust toolchain:
            ```bash
             rustup toolchain install nightly --component rust-src
             ```
    * For XTENSA-based chips:
        - Install `espup` either by building it with Cargo or by downloading the binary from https://github.com/esp-rs/espup/releases
            ```bash
            cargo install espup
            ```
       - Run it
            ```bash
            espup install
            ```

For a comprehansive setup guide check out the [template](https://github.com/esp-rs/esp-idf-template#prerequisites) or the [book](https://esp-rs.github.io/book/)

## Removing the project + toolchain
To remove the project generated using cargo generate, simply delete the directory that was created. For the RISC-V case, no additional cleanup is needed.

For the XTENSA case, you need to remove the XTENSA toolchain installed via espup. You can do this by running `espup uninstall`.

# Chat
Join the ESP-RS community on Matrix chat for help or questions: https://matrix.to/#/#esp-rs:matrix.org

# Aditional Information

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
| **ESP32-C3** |      12 - 17       |
| **ESP32-S2** |      26 - 32       |
| **ESP32-S3** | 26 - 32, 33 - 37\* |

_\* When using Octal Flash and/or Octal PSRAM_

