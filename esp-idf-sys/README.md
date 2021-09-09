# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP32C3 etc.) based on the Rust target

## Build

- The build requires the [Rust ESP32 STD compiler fork](https://github.com/esp-rs/rust) to be configured and installed as per the instructions there.
- The relevant Espressif toolchain, as well as the `esp-idf` itself are all automatically
  downloaded during the build by 
    - with the feature `pio` (default): utilizing [platformio](https://platformio.org/) (via
        the [embuild](https://github.com/ivmarkov/embuild) crate) or
    - with the feature `native` (*experimental*): utilizing native `esp-idf` tooling.
- Check the ["Hello, World" demo](https://github.com/ivmarkov/rust-esp32-std-hello) for how to use and build this crate

## Feature `platformio`
This is currently the default for installing all build tools and building the `esp-idf` C
library. It uses [platformio](https://platformio.org/) via the
[embuild](https://github.com/ivmarkov/embuild) crate.

### Bluetooth Support

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

### Using cargo-pio to interactively modify ESP-IDF's `sdkconfig` file

To enable Bluetooth, or do other configurations to the ESP-IDF sdkconfig you might take advantage of the cargo-pio Cargo subcommand:
* To install it, issue `cargo install cargo-pio --git https://github.com/ivmarkov/cargo-pio`
* To open the ESP-IDF interactive menuconfig system, issue `cargo pio espidf menuconfig` in the root of your **binary crate** project
* To use the generated/updated `sdkconfig` file, follow the steps described in the "Bluetooth Support" section

### More info

If you are interested how it all works under the hood, check the [build_pio.rs](build.rs)
or  script of this crate.


## Experimental feature `native`

Download all tools and build the `esp-idf` using its own tooling.

**Warning**: This is an experimental feature and subject to changes.

Currently, this build script installs all needed tools to compile the `esp-idf` as well as the `esp-idf` itself
under `<workspace-dir>/.espressif`, this is subject to change in the future.

### Requirements
- If using chips other than `esp32c3`: 
    - [Rust ESP32 compiler fork](https://github.com/esp-rs/rust)
    - [libclang of the xtensa LLVM fork](https://github.com/espressif/llvm-project/releases)
- `python >= 3.7`
- `cmake >= 3.20`
    
### Configuration
Environment variables are used to configure how the `esp-idf` is compiled.
The following environment variables are used by the build script:

- `SDK_DIR`: The path to the directory where all esp-idf tools are installed,
             defaults to `<workspace-dir>/.espressif`.
- `ESP_IDF_VERSION`:
  The version used for the `esp-idf` can be one of the following:
  - `commit:<hash>`: Uses the commit `<hash>` of the `esp-idf` repository.
                     Note that this will clone the whole `esp-idf` not just one commit.
  - `tag:<tag>`: Uses the tag `<tag>` of the `esp-idf` repository.
  - `branch:<branch>`: Uses the branch `<branch>` of the `esp-idf` repository.
  - `v<major>.<minor>` or `<major>.<minor>`: Uses the tag `v<major>.<minor>` of the `esp-idf` repository.
  - `<branch>`: Uses the branch `<branch>` of the `esp-idf` repository.

  It defaults to `v4.3`.
- `ESP_IDF_REPOSITORY`: The URL to the git repository of the `esp-idf`, defaults to <https://github.com/espressif/esp-idf.git>.
- `ESP_IDF_SDKCONFIG_DEFAULTS`: A `;`-seperated list of paths to `sdkconfig.default` files to be used as base
                                values for the `sdkconfig`.
- `ESP_IDF_SDKCONFIG`: A path (absolute or relative) to the esp-idf `sdkconfig` file.
- `ESP_IDF_EXTRA_TOOLS`: A `;`-seperated list of additional tools to install with `idf_tools.py`.
- `MCU`: The mcu name (e.g. `esp32` or `esp32c3`). If not set this will be automatically detected from the
         cargo target.