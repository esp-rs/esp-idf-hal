# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP32C3 etc.) based on the Rust target

## Build

- The build requires the [Rust ESP32 STD compiler fork](https://github.com/ivmarkov/rust) to be configured and installed as per the instructions there.
- The relevant Espressif toolchain, as well as the ESP-IDF itself are all automatically downloaded during the build by utilizing the [cargo-pio](https://github.com/ivmarkov/cargo-pio) library crate.
- Check the ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello) for how to use and build this crate

## Bluetooth Support

In order to enable Bluetooth support with either Bluedroid or NimBLE, there is some additional work:
* Go to the root of your **binary crate** project (e.g., the ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello))
* Create a `.cargo/config.toml` file if it does not exist there yet. You can copy it from the ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello)
* Include in it the following:
```toml
[env]
...
ESP_IDF_SYS_GLOB_0 = { value = "/sdkconfig" }
```
* Next, create a file `sdkconfig` at the root of binary crate your project. This could be generated with `cargo pio espidf menuconfig` if you install `cargo-pio` (see below) but a minimal manual example follows:
```c
CONFIG_BT_ENABLED=y
CONFIG_BTDM_CTRL_MODE_BLE_ONLY=y
CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=n
CONFIG_BTDM_CTRL_MODE_BTDM=n
// Uncomment whichever of these you need
//CONFIG_BT_BLUEDROID_ENABLED=y
//CONFIG_BT_NIMBLE_ENABLED=y
```

## Using cargo-pio to interactively modify ESP-IDF's `sdkconfig` file

To enable Bluetooth, or do other configurations to the ESP-IDF sdkconfig you might take advantage of the cargo-pio Cargo subcommand:
* To install it, issue `cargo install cargo-pio --git https://github.com/ivmarkov/cargo-pio`
* To open the ESP-IDF interactive menuconfig system, issue `cargo pio espidf menuconfig` in the root of your **binary crate** project
* To use the generated/updated `sdkconfig` file, follow the steps described in the "Bluetooth Support" section

## More info

If you are interested how it all works under the hood, check the [build.rs](https://github.com/ivmarkov/esp-idf-sys/blob/master/build.rs) script of this crate.

