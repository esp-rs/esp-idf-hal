# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP32S3, ESP32C3 etc.) based on the Rust target.

![CI](https://github.com/esp-rs/esp-idf-sys/actions/workflows/ci.yml/badge.svg)

## Build

- To build this crate, please follow all the build requirements specified in the [ESP-IDF Rust Hello World template crate](https://github.com/esp-rs/esp-idf-template)
- The relevant Espressif toolchain, as well as the ESP-IDF framework itself are all automatically
  downloaded during the build:
    - With feature `pio` (default): utilizing [platformio](https://platformio.org/) (via the [embuild](https://github.com/ivmarkov/embuild) crate) or
    - With feature `native` (*experimental*): utilizing native `esp-idf` tooling also via the [embuild](https://github.com/ivmarkov/embuild) crate.
- Check the [ESP-IDF Rust Hello World template crate](https://github.com/esp-rs/esp-idf-template) for a "Hello, world!" Rust template demonstrating how to use and build this crate.
- Check the [demo](https://github.com/ivmarkov/rust-esp32-std-demo) crate for a more comprehensive example in terms of capabilities.

## Feature `native`
This is the default feature for downloading all tools and building the ESP-IDF framework using the framework's "native" (own) tooling.
It relies on build and installation utilities available in the [embuild](https://github.com/ivmarkov/embuild) crate.

The `native` builder installs all needed tools to compile this crate as well as the ESP-IDF framework itself. 

### (Native builder only) Using cargo-idf to interactively modify ESP-IDF's `sdkconfig` file

TBD: Upcoming

## Feature `pio`
This is a backup feature for installing all build tools and building the ESP-IDF framework. It uses [PlatformIO](https://platformio.org/) via the
[embuild](https://github.com/ivmarkov/embuild) crate.

Similarly to the `native` builder, the `pio` builder also automatically installs all needed tools (PlatformIO packages and frameworks in this case) to compile this crate as well as the ESP-IDF framework itself. 

**NOTE:** The `pio` builder is less flexible than the default `native` builder in that it can work with only **one, specific** version of ESP-IDF. At the time of writing, this is V4.3.2.

### (PIO builder only) Using cargo-pio to interactively modify ESP-IDF's `sdkconfig` file

To enable Bluetooth, or do other configurations to the ESP-IDF sdkconfig you might take advantage of the cargo-pio Cargo subcommand:
* To install it, issue `cargo install cargo-pio --git https://github.com/ivmarkov/cargo-pio`
* To open the ESP-IDF interactive menuconfig system, issue `cargo pio espidf menuconfig` in the root of your **binary crate** project
* To use the generated/updated `sdkconfig` file, follow the steps described in the "Bluetooth Support" section

## Configuration

Environment variables are used to configure how the ESP-IDF framework is compiled. 

Note that instead of / in addition to specifying those on the command line, you can also put these in a `.config/cargo.toml` file inside your crate directory 
(or a parent directory of your crate) by using the recently stabilized Cargo [configurable-env](https://doc.rust-lang.org/cargo/reference/config.html#env) feature.

The following environment variables are used by the build script:

- `ESP_IDF_SDKCONFIG_DEFAULTS`: 

    A `;`-separated list of paths to `sdkconfig.defaults` files to be used as base
    values for the `sdkconfig`. If such a path is relative, it will be relative to the
    cargo workspace directory (i.e. the directory that contains the `target` dir).
    
    If unspecified `sdkconfig.defaults` is used as default.
    
    For each defaults file in this list more specific version will also be searched and
    used. This happens with the following patterns and order (least to most specific):

    1. `<path>`
    2. `<path>.<profile>`
    3. `<path>.<mcu>`
    4. `<path>.<profile>.<mcu>`
    
    where `<profile>` is the current cargo profile used (`debug`/`release`) and `<mcu>`
    specifies the mcu for which this is currently compiled for (see the `MCU`
    configuration option below).

    Also note that a setting contained in a more specific defaults file will override the
    same setting specified in a less specific one.

- `ESP_IDF_SDKCONFIG`:     

    The base-path to the `sdkconfig` file used to [configure the
    `esp-idf`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html).
    If this is a relative path, it is relative to the cargo workspace directory.
    
    If unspecified `sdkconfig` is used as default.
    
    Similar to the `sdkconfig.defaults`-file a more specific `sdkconfig`-file will be
    selected if available. This happens with the following patterns and precedence:

    1. `<path>.<profile>.<mcu>`
    2. `<path>.<mcu>`
    3. `<path>.<profile>`
    4. `<path>`

    **Note** (*native* builder only):   
    The cargo optimization options (`debug` and `opt-level`) are used by default to
    determine the compiler optimizations of the `esp-idf`, **however** if the compiler
    optimization options are already set in the `sdkconfig` **they will be used instead.**


- `ESP_IDF_TOOLS_INSTALL_DIR`:

    The location where the ESP-IDF framework tooling is assumed to be/will be installed. 
    The framework tooling is either PlatformIO (when the `pio` builder is used), or the ESP-IDF native toolset (when the `native` builder is used).

    This variable can take one of the following values:
    - `workspace` (default) - the tooling will be installed/used in 
      `<crate-workspace-dir>/.embuild/platformio` for `pio`, and `<crate-workspace-dir>/.embuild/espressif` for the `native` builder;
    - `out` - the tooling will be installed/used inside the crate's build output directory, and will be deleted when `cargo clean` is invoked;
    - `global` - the tooling will be installed/used in its standard directory (`~/.platformio` for PlatformIO, and `~./espressif` for the native ESP-IDF toolset);
    - `custom:<dir>` -  the tooling will be installed/used in the directory specified by `<dir>`. If this directory is a relative location, it is assumed to be 
      relative to the crate's workspace dir.

      **ATTENTION**: Please be extra careful with the `custom:<dir>` setting when switching from `pio` to `native` and the other way around, because
      the builder will install the tooling in `<dir>` without using any additional `platformio` or `espressif` subdirectories, so if you are not careful, you might end up with 
      both PlatformIO, as well as the ESP-IDF native tooling intermingled together in a single folder.


    Note that both builders (`native` and `pio`) clone the ESP-IDF GIT repository *inside* the tooling directory as well. This restriction might be lifted soon for the `native` builder, whereas the user would be able to point the build to a custom ESP-IDF repository location.

- `ESP_IDF_VERSION` (*native* builder only):

  The version used for the `esp-idf` can be one of the following:
  - `commit:<hash>`: Uses the commit `<hash>` of the `esp-idf` repository.
                     Note that this will clone the whole `esp-idf` not just one commit.
  - `tag:<tag>`: Uses the tag `<tag>` of the `esp-idf` repository.
  - `branch:<branch>`: Uses the branch `<branch>` of the `esp-idf` repository.
  - `v<major>.<minor>` or `<major>.<minor>`: Uses the tag `v<major>.<minor>` of the `esp-idf` repository.
  - `<branch>`: Uses the branch `<branch>` of the `esp-idf` repository.

  It defaults to `v4.3.1`.


- `ESP_IDF_REPOSITORY` (*native* builder only): 

  The URL to the git repository of the `esp-idf`, defaults to <https://github.com/espressif/esp-idf.git>.
  
  Note that when the `pio` builder is used, it is possible to achieve something similar to `ESP_IDF_VERSION` and `ESP_IDF_REPOSITORY` by using 
  the [`platform_packages`](https://docs.platformio.org/en/latest/projectconf/section_env_platform.html#platform-packages) PlatformIO option as follows:
    - `ESP_IDF_PIO_CONF="platform_packages = framework-espidf @ <git-url> [@ <git-branch>]"`
    - The above approach however has the restriction that PlatformIO will always use the ESP-IDF build tooling from its own ESP-IDF distribution, 
      so the user-provided ESP-IDF branch may or may not compile. The current PlatformIO tooling is suitable for compiling ESP-IDF branches derived from versions 4.3.X .


- `ESP_IDF_GLOB[_XXX]_BASE` and `ESP_IDF_GLOB[_XXX]_YYY`:

  A pair of environment variable prefixes that enable copying files and directory trees that match a certain glob mask into the native C project used for building the ESP-IDF framework:
  - `ESP_IDF_GLOB[_XXX]_BASE` specifies the base directory which will be glob-ed for resources to be copied
  - `ESP_IDF_GLOB[_XXX]_BASE_YYY` specifies one or more environment variables that represent the glob masks of resources to be searched for and copied, using the directory designated by the `ESP_IDF_GLOB[_XXX]_BASE` environment variable as the root. For example, if the follwing variables are specified:
    - `ESP_IDF_GLOB_HOMEDIR_BASE=/home/someuser`
    - `ESP_IDF_GLOB_HOMEDIR_FOO=foo*`
    - `ESP_IDF_GLOB_HOMEDIR_BAR=bar*`
    ... then all files and directories matching 'foo*' or 'bar*' from the home directory of the user will be copied in theESP-IDF C project.

    Note also that `_HOMEDIR` in the above example is optional, and is just a mechanism allowing the user to specify more than base directory and its glob patterns.


- `ESP_IDF_PIO_CONF_XXX` (*pio* builder only):

  A PlatformIO setting (or multiple settings separated by a newline) that will be passed as-is to the `platformio.ini` file of the C project that compiles the ESP-IDF.
  - Check [the PlatformIO documentation](https://docs.platformio.org/en/latest/projectconf/index.html) for more information as to what settings you can pass via this variable.
  - Note also that this is not one variable - but rather - a family of variables all starting with `ESP_IDF_PIO_CONF_`. I.e., passing `ESP_IDF_PIO_CONF_1` as well as `ESP_IDF_PIO_CONF_FOO` is valid and all such variables will be honored

- `ESP_IDF_CMAKE_GENERATOR` (*native* builder only):

  The CMake generator to be used when building the ESP-IDF SDK. If not specified or set to `default`, Ninja will be used on all platforms except Linux/aarch64, where
  (for now) the Unix Makefiles generator will be used, as there are no Ninja builds for that platform provided by Espressif yet.
  Possible values for this environment variable are [the names of all command-line generators that CMake supports](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html#cmake-generators) with **spaces and hyphens removed**.

- `MCU`:

   The MCU name (i.e. `esp32`, `esp32s2`, `esp32s3` `esp32c3` and `esp32h2`). 
   
   - If not set this will be automatically detected from the cargo target.
   
   - Note that [older ESP-IDF versions might not support all MCUs from above](https://github.com/espressif/esp-idf#esp-idf-release-and-soc-compatibility).

## More info

If you are interested how it all works under the hood, check the [build.rs](build/build.rs)
build script of this crate.
