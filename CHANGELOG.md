# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.41.0] - 2023-05-13

* MSRV 1.66 (but MSRV 1.70 necessary if `nightly` is enabled)
* Support for new chips: esp32c2, esp32h2, esp32c6 and future proofed for esp32c5 and esp32p4
* Support for ESP IDF 5.0, 5.1 and 5.2 (master)
* Support for `embedded-hal` 1.0-alpha.10 and `embedded-hal-async` 0.2.0-alpha.2 (API breakage in i2c and spi)
* Async GPIO native API with support for `embedded-hal-async`
* PCNT driver
* Alerts and Mode support in the TWAI driver
* Task Watchdog API
* Configurable interrupt levels in all drivers (minor API breakage in `SpiDriver`)
* `EspError::from_infallible`
* Auto-reload support in the timer driver

## [0.40.1] - 2022-12-13

Fix the build when the ULP peripheral is enabled.

## [0.40] - 2022-12-09

Rebase on top of `esp-idf-sys` 0.32:
* Retire any usages of `esp-idf-sys::c_types` in favor of `core::ffi`
* Remove casts from `usize` to `u32` and back now that `esp-idf-sys` is compiled with `--size_t-is-usize` enabled

## [0.39.3, 0.39.4] - 2022-12-09

Minor doc fixes; Clippy fixes; `CriticalSection` will panic on FreeRTOS mutex lock/unlock failures.

## [0.39.1, 0.39.2] - 2022-11-21

Patch releases to fix compilation errors under no_std.

## [0.39] - 2022-11-01

Release 0.39 is a backwards-incompatible release where almost all drivers were touched in one way or another.

### Major Changes

The main themes of the 0.39 release are:
* Restore drop semantics for all drivers
* Simpler driver types (little to no generics)
* Unified naming
* Public API
* Completely revamped GPIO metaphor
* Timer driver
* Support for the `critical-section` crate by providing a `CriticalSection` implementation
* Support for the `embassy-sync` crate by providing two types of raw mutexes
* Support for the `edge-executor` crate
* Modem and Mac peripherals
* SPI driver rework

### Major changes elaboration

#### Restore drop semantics for all drivers

* The `release()` method is now retired everywhere
* Just dropping a driver will make it stop functioning 
* If peripherals need to be reused after the driver is dropped, this is achieved by passing the peripherals to the driver constructor via `&mut` references, e.g. `I2cMasterDriver::new(&mut peripherals.i2c1, &mut peripherals.pins.gpio10, &mut peripherals.pins.gpio11, &mut peripherals.pins.gpio12)`

### Simpler driver types

* All peripheral generics are removed from all drivers' types; e.g., the I2C master driver now has the following type signature: `I2cMasterDriver<'d>` 
* The lifetime - `'d` from above designates the lifetime of the peripheral (e.g. `I2c`) that is used by the driver
* In the most common case where the peripheral is just moved into the driver (and no longer accessible when the driver is dropped), `'d` = `'static`, or in other words, the type signature of the driver becomes `I2cMasterDriver<'static>` and can even be type aliased by the user as in `pub type SimpleI2cMasterDriver = I2cMasterDriver<'static>;`
* When the peripherals are passed to the driver using `&mut` references, `'d` designates the lifetime of the `&mut` reference
* Note that the constructor of each driver is still generic by the peripherals it takes, but these generics are "erased" from the type of the driver once the driver is constructed, as we only need to "remember" whether the peripheral is moved into the driver, or used via a `&mut` reference and can be reused after the driver is dropped. This semantics is captured by the `'d` lifetime

### Unified naming

* Each driver is named `<Peripheral>Driver`, where `<Peripheral>` is the name of the peripheral. E.g. the driver for the `Uart` peripheral is named `UartDriver`, the driver for the `Adc` peripheral is named `AdcDriver` and so on
* The one exception is the GPIO subsystem, where the driver is not named `GpioDriver`, but the simpler `PinDriver` instead, even if the concrete peripheral struct type might be e.g. `Gpio12`
* In case there are multiple drivers per peripheral - as in - say - a master and a slave protocol driver - only the `Slave` contains itself in its name, as in e.g. `SpiDriver` and `SpiSlaveDriver`
* NOTE: We are NOT fond of still using offensive terms like "master" and "slave". Unfortunately, the embedded industry itself has not yet agreed (to our knowledge) on a good replacement for these terms, hence they are still around

### Public API

In addition to implementing the `embedded-hal` traits where possible, all drivers now have public API. While the public API loosely follows the APIs from `embedded-hal`, it deviates where appropriate so that the native underlying ESP IDF driver is better utilized.

These public APIs mean that the user is no longer required to depend on the `embedded-hal` crate so as to consume the `esp-idf-hal` drivers.
Consuming the drivers via the `embedded-hal` traits is now only necessary when the user is targetting cross-platform portability of their application code.

### Completely revamped GPIO metaphor

In previous releases, the GPIO driver was a bit different from the others, in that each *pin peripheral itself* used to have TypeState and methods that configured the peripheral into a certain mode (as in e.g. input-only, input-output, output-only, ADC and so on) that drived the peripheral through a type-state change.

Furthermore, there was no notion of a pin "driver". The driver methods were fused directly into the peripheral, and a subset of these methods were available, depending on the type-state of the peripheral.

This scheme was asymmetric with the rest of the HAL, where peripherals are mostly opaque singleton objects, and it is the driver that exposes a user visible API (including implementation of the corresponding `embedded-hal` traits). It also suffered from less flexibility and code duplication accross the regular GPIO pins, and their degraded `Any*Pin` variants.

The new GPIO subsystem follows the driver-peripheral separation metaphor like everything else:
* Pin peripherals are static objects with no type-state, where the only allowed operation is to degrade a concrete pin instance implementing the `Pin` / `InputPin` / `OutputPin` traits
into a corresponding `Any*Pin` struct which is useful when e.g. passing an array of input or output pins accross the application code, as generics are erased by the degrading operation
* The capabilities of a pin peripheral are expressed by the concrete pin struct type (i.e. `GpioXX`) implementing a set of traits (`InputPin`, `AnalogPin`) and so on
* `PinDriver` is the universal driver for any pin peripheral (e.g. be it a non-degraded `GpioXX` struct, or a degraded `AnyIOPin`, `AnyOutputPin` or `AnyInput` pin struct)
* Providing an ISR handler is possible - via `PinDriver` - on any input pin, including on a degraded one

Finally, `PinDriver` can now work with digital pins by using the RTC digital IO API on chips that support this (ESP32 and the ESP32 S* series)

### Timer driver

The new `timer` module is exposing the hardware timer peripherals on the esp32* MCUs via the ESP-IDF timer driver. Just like the underlying ESP-IDF timer driver, a lot (but not all)
methods are actually safe to call from an ISR routine. Those which are unsafe will panic immediately if called from an ISR.

### Support for the `critical-section` crate

In addition to implementing `embedded-hal` crates, `esp-idf-hal` integrates with the wider Rust embedded ecosystem by providing out of the box critical section implementation for the `critical-section` crate.

To enable this support, you need to compile the crate with the `critical-section` feature enabled. 

Note that the provided critical section implementation is based on `esp_idf_hal::task::CriticalSection`, which itself is based on a recursive FreeRtos mutex.
This means that you can `enter()` the critical section for long periods of time, and your code can still be preempted by ISR routines or high priority FreeRtos tasks (threads), thus you should not worry about slowing down the whole RTOS reaction times by using a critical section.
This also means however, that this critical section (and its native `esp_idf_hal::task::CriticalSection` equivalent) CANNOT be used from an ISR context.

If you need a critical section with a warranty that your code will not be interrupted by an ISR or higher priority tasks (i.e. working accross ISRs and tasks), you should be using 
`esp_idf_hal::interrupt::IsrCriticalSection` instance, or the `esp_idf_hal::interrupt::free` method. Keep in mind that you should stay in this type of critical section for a very short period of time, because it **disables all interrupts**.

### Support for the `embassy-sync` crate

The HAL provides two mutex implementations that implement the `RawMutex` trait from the `embassy-sync` crate thus integrating the ESP-RS ecosystem better with the Rust embedded async ecosystem:
* `EspRawMutex` - this implementation uses a FreeRtos mutex and behaves similarly to `esp_idf_hal::task::CriticalSection`
* `IsrRawMutex` - this implementation works by disabling all interrupts and thus behaves similarly to `esp_idf_hal::interrupt::IsrCriticalSection`

To enable this support, you need to compile the crate with the `embassy-sync` feature enabled. 

While `embassy-sync` itself can use - via its `CriticalSectionRawMutex` mutex bridge - whatever a HAL is providing as a critical section to the `critical-section` crate, the above two implementations provide further benefits:
* `EspRawMutex` is almost the same as the `embassy-sync` native `CriticalSectionRawMutex` with the one major difference that ALL `CriticalSectionRawMutex` instances in fact share a **single** RTOS mutex underneath. This is a limitation which is coming from the `critical-section` crate itself. In contrast, each `EspRawMutex` instance has its own separate RTOS mutex
* `IsrRawMutex` - as the name suggests - allows `embassy-sync` to be utilized in scenarios where the application code should synchronize accross ISRs and regular RTOS task based code

### Support for the `edge-executor` crate

This is another HAL feature that integrates the ESP-RS ecosystem with the Rust embedded async ecosystem.

The [`edge-executor`](https://github.com/ivmarkov/edge-executor) crate is a simple local-only executor that is based on the popular `async-task` crate coming from the Smol async executor (which in turn powers the `async-std` executor).
Without getting into too many details, `edge-executor` is useful in embedded cotexts because 
* It is `no_std` compatible (but requires an allocator, unlike [`embassy-executor`](https://github.com/embassy-rs/embassy), so the latter might be a better fit for bare-metal use cases where ESP IDF is not utilized) 
* Has a small footprint
* Is ISR-friendly in that it can operate from an ISR context - or - which would be the natural setup when used on top of a RTOS like ESP IDF's FreeRtos - can be **awoken** directly from an ISR routine, this having a minimal latency

To enable this support, you need to compile the crate with the `edge-executor` feature enabled. 

The support for `edge-executor` in `esp-idf-hal` is in terms of providing an ISR-friendly `Monitor` trait implementation for `edge-executor` - `FreeRtosMonitor` - that can be awoken directly from an ISR, and is based on the FreeRtos task notification mechanism which is exposed in the `esp_idf_hal::task` crate.

### Modem and Mac peripherals

In previous releases, the Wifi and Ethernet drivers (part of the `esp-idf-svc` crate) did not follow the philosophy of `esp-idf-hal` in that each driver should take a corresponding peripheral (by moving or by a `&mut` reference).

This is now addressed in that the `esp-idf-hal` crate models two new peripherals:
* `modem::Modem`: models the esp32* hardware modem peripheral and implements two new marker traits: `modem::WifiModemPeripheral` and `modem::BluetoothModemPeripheral`
  * By implementing both of the above traits, the `modem::Modem` peripheral should be usable by both the Wifi driver in `esp-idf-svc`, as well as the future Bluetooth driver
  * `modem::Modem` is splittable into two other peripherals: `modem::WifiModem` (implementing only the `modem::WifiModemPeripheral` trait) and `modem::BluetoothModem` (implementing only the `modem::BluetoothModemPeripheral` trait) so that in future the simultaneous operation of the Wifi and Bluetooth drivers to be possible
* `mac::MAC`: models the EMAC hardware available on the original ESP32 chip. The Ethernet driver implementation - just like the Wifi driver implementation - is still in the `esp-idf-svc` crate which better aligns with the underlying ESP-IDF implementations, yet the new Wifi and Ethernet drivers in `esp-idf-svc` now expect their corresponding peripherals to be supplied during construction time

### SPI driver rework

The SPI driver is now split into two structures
* `SpiDriver` - represents an initialized SPI bus and manages access to the underlying SPI hardware 
* `SpiDeviceDriver` (new) - an abstraction for rach device connected to the SPI bus
  
The above split allows for the creation of more than one device per SPI bus. (Up to 6 for the esp32c* variants and 3 for all others).
When creating an `SpiDeviceDriver` instance, user is required to provide a `Borrow` to the `SpiDriver` instance (as in `SpiDriver`, `&SpiDriver`, `&mut SpiDriver`, `Rc(SpiDriver)` or `Arc(SpiDriver)`), so that the SPI bus stays initialized throughout the liftime of all devices.

A second wrapper implementation is now also provided: `SpiSoftCsDeviceDriver` It allows for more devices per SPI bus (i.e. above the 3/6 limit). This is implemented by operating the CS pin in software mode.
