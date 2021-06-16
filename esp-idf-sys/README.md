# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP32C3 etc.) based on the Rust target

## Build

* The build requires the [Rust ESP32 STD compiler fork](https://github.com/ivmarkov/rust) to be configured and installed as per the instructions there.
* The relevant Espressif toolchain, as well as the ESP-IDF itself are all automatically downloaded during the build by utilizing the [cargo-pio](https://github.com/ivmarkov/cargo-pio) library crate.
* Check the ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello) for how to use and build this crate

## More info

If you are interested how it all works under the hood, check the [build.rs](https://github.com/ivmarkov/esp-idf-sys/blob/master/build.rs) script of this crate.