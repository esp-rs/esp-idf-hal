# Raw Rust bindings for the [ESP IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)

[![CI](https://github.com/esp-rs/esp-idf-sys/actions/workflows/ci.yml/badge.svg)](https://github.com/esp-rs/esp-idf-sys/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/esp-idf-sys.svg)](https://crates.io/crates/esp-idf-sys)
[![Documentation](https://img.shields.io/badge/docs-esp--rs-brightgreen)](https://esp-rs.github.io/esp-idf-sys/esp_idf_sys/index.html)
[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)
[![Wokwi](https://img.shields.io/endpoint?url=https%3A%2F%2Fwokwi.com%2Fbadge%2Fclick-to-simulate.json)](https://wokwi.com/projects/332188235906155092)

## Highlights

- Build is `cargo` driven and **automatically downloads & configures everything by default**; no need to download the ESP IDF SDK manually, or set up a C toolchain
- Supports both native ESP IDF build (default), as well as a PlatformIO build
- Option to use in a mixed Rust/C project. Check the documentation in the [esp-idf-template](https://github.com/esp-rs/esp-idf-template) crate

**You might want to also check out the type safe Rust wrappes built on top of these raw bindings:**
- [Type safe wrappers for ESP IDF Services](https://github.com/esp-rs/esp-idf-svc)
- [Type safe wrappers for ESP IDF Drivers](https://github.com/esp-rs/esp-idf-hal)

> **Note**  
> `esp-idf-sys`'s [build
> script](https://doc.rust-lang.org/cargo/reference/build-scripts.html) will download the
> esp-idf, its gcc toolchain, and build it. To show progress and build information about
> this process run cargo with the `-vv` (very verbose) flag, so that build script output
> is also displayed. This is especially useful since the initial build will take a while.

## Build Prerequisites

Follow the [Prerequisites](https://github.com/esp-rs/esp-idf-template#prerequisites) section in the `esp-idf-template` crate.

## Customizing how the ESP IDF SDK is built

Read the [documentation here](BUILD-OPTIONS.md).

## Examples

The examples could be built and flashed conveniently with [`cargo-espflash`](https://github.com/esp-rs/espflash/). To run e.g. `std_basics` on an e.g. ESP32-C3:
(Swap the Rust target and example name with the target corresponding for your ESP32 MCU and with the example you would like to build)

with `cargo-espflash`:
```sh
$ MCU=esp32c3 cargo espflash flash --target riscv32imc-esp-espidf --example std_basics --monitor
```

| MCU | "--target" |
| --- | ------ |
| esp32c2 | riscv32imc-esp-espidf |
| esp32c3| riscv32imc-esp-espidf |
| esp32c6| riscv32imac-esp-espidf |
| esp32h2 | riscv32imac-esp-espidf |
| esp32p4 | riscv32imafc-esp-espidf |
| esp32 | xtensa-esp32-espidf |
| esp32s2 | xtensa-esp32s2-espidf |
| esp32s3 | xtensa-esp32s3-espidf |


## Setting up a "Hello, world!" binary crate with ESP IDF

Use the [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project. Everything would be arranged and built for you automatically - no need to manually clone the ESP IDF repository.

## More information

For more information, check out:
- The [Rust on ESP Book](https://esp-rs.github.io/book/)
- The [ESP Embedded Training](https://github.com/esp-rs/espressif-trainings)
- The [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project
- The [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) project
- The [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal) project
- The [embedded-svc](https://github.com/esp-rs/embedded-svc) project
- The [embedded-hal](https://github.com/rust-embedded/embedded-hal) project
- The [Rust for Xtensa toolchain](https://github.com/esp-rs/rust-build)
- The [Rust-with-STD demo](https://github.com/ivmarkov/rust-esp32-std-demo) project
