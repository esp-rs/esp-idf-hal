# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP32C3 etc.) based on the Rust target

![CI](https://github.com/esp-rs/esp-idf-sys/actions/workflows/ci.yml/badge.svg)

## Build

- To build this crate, please follow all the build requirements specified in the [ESP-IDF Rust Hello World template crate](https://github.com/esp-rs/esp-idf-template)
- The relevant Espressif toolchain, as well as the `esp-idf` itself are all automatically
  downloaded during the build by
    - with the feature `pio` (default): utilizing [platformio](https://platformio.org/) (via the [embuild](https://github.com/ivmarkov/embuild) crate) or
    - with the feature `native` (*experimental*): utilizing native `esp-idf` tooling.
- Check the [ESP-IDF Rust Hello World template crate](https://github.com/esp-rs/esp-idf-template) for a "Hello, world!" Rust template demonstrating how to use and build this crate
- Check the [demo](https://github.com/ivmarkov/rust-esp32-std-demo) crate for a more comprehensive example in terms of capabilities

## Feature `pio`
This is currently the default for installing all build tools and building the ESP-IDF framework. It uses [PlatformIO](https://platformio.org/) via the
[embuild](https://github.com/ivmarkov/embuild) crate.

The `pio` builder installs all needed tools to compile the `esp-idf` as well as the `esp-idf` itself. 
The location where the `esp-idf` source and tools are detected and installed is the following:
- **~/.platformio`**
  - This is the "standard" PlatformIO location, where all tooling, and the ESP-IDF is installed

*NOTE*: In the near future, the `pio` builder will have a flexible scheme as to where the PlatformIO tooling will be installed. This scheme will follow in spirit
the `native` builder scheme described below.

### (PIO builder only) Using cargo-pio to interactively modify ESP-IDF's `sdkconfig` file

To enable Bluetooth, or do other configurations to the ESP-IDF sdkconfig you might take advantage of the cargo-pio Cargo subcommand:
* To install it, issue `cargo install cargo-pio --git https://github.com/ivmarkov/cargo-pio`
* To open the ESP-IDF interactive menuconfig system, issue `cargo pio espidf menuconfig` in the root of your **binary crate** project
* To use the generated/updated `sdkconfig` file, follow the steps described in the "Bluetooth Support" section

## Feature `native`
This is an experimental feature for downloading all tools and building the ESP-IDF framework using the framework's "native" own tooling.
It will become the default in the near future.
It also relies on build and installation utilities available in the [embuild](https://github.com/ivmarkov/embuild) crate.

Similarly to the `pio` builder, the `native` builder also automatically installs all needed tools to compile the `esp-idf` as well as the `esp-idf` itself. 
The location where the `esp-idf` source and tools are detected and installed can be one of the following ones:
- **`<crate workspace-dir>/.embuild/espressif`**
  - This is the location used by default
- **`~/.espressif`** 
  - This is the "standard" ESP-IDF tools location
  - To enable it, set the environment variable [ESP_IDF_GLOBAL_INSTALL](...) to 1
- **`$ESP_IDF_INSTALL_DIR`**
  - This is a user-provided location
  - To enable it, simply define the [ESP_IDF_INSTALL_DIR](...) variable to point to a directory of your preference

### (Native builder only) Using cargo-idf to interactively modify ESP-IDF's `sdkconfig` file

TBD: Upcoming

## Configuration

*NOTE*: This configuration is currently honored *only* by the `native` builder.
The `pio` (default) builder has a different configuration, but it is not documented here, because in the near future the `pio` builder will also be migrating to 
the configuration supported by the `native` builder.

Environment variables are used to configure how the `esp-idf` is compiled.
The following environment variables are used by the build script:

- `ESP_IDF_INSTALL_DIR`:

    The path to the directory where all esp-idf tools are installed. If it is set to a
    relative path, it is relative to the crate workspace-dir.

    If not set, when `ESP_IDF_GLOBAL_INSTALL` is set to `1` it defaults to the global
    install dir `~/.espressif`, otherwise it defaults to the local install dir `<crate
    workspace-dir>/.embuild/espressif`.

- `ESP_IDF_GLOBAL_INSTALL`

    If set to `1`, `true`, `y` or `yes` uses the global install directory only when `ESP_IDF_INSTALL_DIR` is not specified.

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
- `ESP_IDF_SDKCONFIG_DEFAULTS`: A `;`-separated list of paths to `sdkconfig.default` files to be used as base
                                values for the `sdkconfig`.
- `ESP_IDF_SDKCONFIG`: A path (absolute or relative) to the esp-idf `sdkconfig` file.
- `MCU`: The mcu name (e.g. `esp32` or `esp32c3`). If not set this will be automatically
         detected from the cargo target.

## More info

If you are interested how it all works under the hood, check the [build.rs](build/build.rs)
build script of this crate.
