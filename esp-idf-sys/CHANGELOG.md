# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [?.??.?] - ????-??-??
* #264 - Copy the bootloader and partition table binaries to the target folder
* #262 - Bindings for the `esp_lcd` driver component
* #259 - Bindings for the temperature sensor driver
* #261 - Build time optimization - do not download Rust crates not needed for the host platform
* #257 - Make builds utilizing the `esp_app_desc` component reproducible
* (Bugfix) Re-expose raw bindings for the `esp_flash` component on ESP IDF 5+

## [0.33.7] - 2023-11-08
* Workaround for https://github.com/esp-rs/esp-idf-svc/issues/312
* Include gptimer headers (#255)

## [0.33.6] - 2023-11-07
* Export esp_netif_sntp APIs from esp-idf 5.1
* Fix compile error for esp32c6 with NimBLE
* Support for the [symlink to xtensa Clang library](https://github.com/esp-rs/espup/releases/tag/v0.8.0) installed by latest `espup` ([esp-idf-svc issue 319](https://github.com/esp-rs/esp-idf-hal/issues/319))

## [0.33.5] - 2023-10-28
* Support for latest ESP IDF 5.2 dev (master)

## [0.33.4] - 2023-10-27
* The `MCU` environment variable was failing the `pio` build if the MCU was not uppercased
* Better error message for the `native` build in case the MCU was not recognized

## [0.33.3] - 2023-10-17
* Support for ESP IDF Component Manager - check the documentation in BUILD-OPTIONS.md
* ESP32H2 and ESP32C5 now properly assigned to the `riscv32imac-esp-espidf`
* All ESP IDF WPA supplicant APIs exposed
* Build is now checking for the presence of certain environment variables (e.g. CXXFLAGS) that might fail the ESP IDF C build and removing those
* Build is now checking if the project path might fail the ESP IDF C build (i.e. too long on Windows or containing spaces on Unix) and failing if so

## [0.33.2] - 2023-08-18
* Band-aid solution that fixes the build with recent Rust nightlies and ESP IDF < 5.1 (https://github.com/esp-rs/esp-idf-template/issues/149)
* Raw bindings for the continuous ADC driver (ESP IDF >= 5.0)
* Raw bindings for bootloader random functions
* Raw bindings for all available classic BT APIs
* Raw bindings for esp_freertos_hooks.h

## [0.33.1] - 2023-06-11

* Raw bindings for the I2S driver
* Raw bindings for CRC ROM functions

## [0.33.0] - 2023-05-13

* (In theory) no API breakage, yet the minor version is raised just in case
* Support for new chips: esp32c2, esp32h2, esp32c6 and future proofed for esp32c5 and esp32p4
* Support for ESP IDF 5.0, 5.1 and 5.2 (master)
* New raw bindings: esp-transport, himem, psram, esp-dpp, i2s, a2dp, wpa2

## [0.32.1] - 2022-12-13

* Fix an erroneous cast to `u32` in `Esp32Alloc`, causing `no_std` builds using the allocator to fail (#158)
* Apply Niche optimization to `EspError` (`NonZeroI32`), add `from_infallible` associated function, (#159)

## [0.32] - 2022-12-09

* Remove the custom `c_types` module in favor of `core::ffi`
* Switch to `embuild` 0.31 and `bindgen` 0.63. Since 0.61, `bindgen` has the `--size_t-is-usize` flag is enabled by default. This removes a lot of unnecessary casting from `usize` to `u32` and makes the `esp-idf-sys` bindings more ergonomic
