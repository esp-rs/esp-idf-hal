# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

[![crate](http://meritbadge.herokuapp.com/esp-idf-sys)](https://crates.io/crates/esp-idf-sys)
[![docs](https://docs.rs/esp-idf-sys/badge.svg)](https://docs.rs/esp-idf-sys)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP8266 etc.) conditionally.

Currently, the bindings specific for each chip are enabled conditionally using Cargo features (e.g. feature = "esp32", feature = "esp32s2", etc.) with "esp32" being the default,
but that might change in future to be automatically derived from the Rust/Cargo target configuration.

## Version

* The bindings are generated against ESP-IDF V4.1 (latest stable as of Nov 2020).
* For ESP8266, the "idf-like" ESP8266 RTOS SDK V3.3 is used (latest stable as of Nov 2020).

## Bindings Regeneration

If you would like to re-generate the bindings (for example, to include your own "sdkconfig.h" in the relevant src/idf-target/esp*/ directory),
you'll need:
* **Clang** with support for the ESP-family targets. I.e. you should build it from the [Espressif fork of LLVM](https://github.com/espressif/llvm-project) sources.
* The **ESP-IDF GCC Toolchain** corresponding to your chip(s). For e.g. ESP32, you can fetch it from [here](https://github.com/espressif/crosstool-NG/releases); for ESP8266 [here](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html).
* **Bindgen** (`cargo install bindgen`)
* **Bash**

Once you compile/install the above denendencies, make sure that the custom Clang compiler and the ESP-IDF GCC Toolchain(s) are in your PATH.
Then go to the project root and execute:
```sh
./bindgen.sh
```

## STD

This crate is (obviously) no_std.
