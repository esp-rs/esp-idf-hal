# Rust bindings for ESP-IDF (Espressif's IoT Development Framework)

## Background

The ESP-IDF API in Rust, with support for each ESP chip (ESP32, ESP32S2, ESP8266 etc.) based on the Rust target

## Easy Build with [cargo-pio](TBD)

**No need** to pre-configure and control the build with ANY environment variables. It all happens automatically.

Additional benefits:
* Bindings are always automatically regenerated during build
* The `sdkconfig.h` file of your main PlatformIO project is automatically picked up and used in these bindings
* The correct ESP GCC Toolchain and ESP-IDF SDK is pre-installed and used during the build by the PlatformIO CLI

**NOTE**: PlatformIO still uses an outdated ESP8266 RTOS SDK, so you cannot currently use the PlatformIO build with ESP8266 chips. The issue is tracked [here](TBD).

## Build WITHOUT [cargo-pio](TBD)

### Pregenerated Bindings

In case you can live with a default `sdkconfig.h` file and with ESP-IDF V4.1 (for ESP32* chips) / ESP8266 RTOS SDK V3.3 (for ESP8266 chips), you can use the pre-generated bindings which are committed in this repo. This saves you from having to install an [ESP-aware Clang](TBD).

### Custom Bindings Regeneration. Linking ESP-IDF

If you would like to re-generate the bindings (so as e.g. they are using your custom `sdkconfig.h` instance), you'll need:

**ESP-Aware Clang**

* **Clang** with support for the ESP-family targets on your `$PATH`
* Download precompiled binary from [here](TBD) or build it from the [Espressif fork of LLVM](https://github.com/espressif/llvm-project) sources

**ESP-IDF GCC Toolchain**

* The **ESP-IDF GCC Toolchain** corresponding to your chip on your `$PATH`.
* For e.g. ESP32, you can download it from [here](https://github.com/espressif/crosstool-NG/releases)
* For ESP8266 [here](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html).

**Environment variables**

The generator supports the following environment variables:
* `ESP_IDF_SYS_REGENERATE` - if set, the bindings will be regenerated. Otherwise, the default pre-generated bindings will be used. Note again that for bindings regeneration, you need an ESP-aware Clang, as per above!
* `ESP_IDF_SYS_LINK` - if set, the ESP-IDF / ESP8266 RTOS SDK will also be compiled to a binary and will be linked to the final ELF executable. If you are not using a [cargo-pio](TBD) driven build, you probably want to set this flag, or else you have to compile and link the ESP-IDF SDK against your Rust binary yourself, which is not easy
* `ESP_IDF_SYS_EXTRA_INCLUDES` - designates the full path to an `sdkconfig.h` header to be used during the bindings generation and the compilation of the ESP-IDF SDK itself. If not set, a default one will be used
* `IDF_PATH` - the path to your ESP-IDF SDK (for ESP32* chips) or to your ESP8266 RTOS SDK (for ESP9266 chips). If not set, the build will atuomatically download ESP-IDF from the [ESP-IDF official GIT repository](https://github.com/espressif/esp-idf.git) / from the [ESP8266 RTOS SDK official GIT repository](https://github.com/espressif/ESP8266_RTOS_SDK.git)
* `ESP_IDF_SYS_VERSION` - ESP-IDF / ESP8266 RTOS SDK version to download and use. Only relevant when `IDF_PATH` is NOT set. Equals to `v4.1` by default for ESP32* chips, and to `v3.3` for ESP8266 chips
