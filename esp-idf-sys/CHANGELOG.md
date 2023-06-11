# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
