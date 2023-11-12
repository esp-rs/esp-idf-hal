# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.47.3] - 2023-11-12
* BREAKING CHANGE IN A PATCH RELEASE DUE TO DISCOVERED UB: All constructors and methods in the Classic BT services, as well as in
services `EspNow`, driver `EthDriver`, `EventLoop`, `Wait`, `EspHttpServer`, `EspMqttClient`, `EspSntp`, `EspTimerService`,
driver `WifiDriver` and `EspWebSocketClient` no longer accept non-static callbacks, as these lead to UB / crash when the service/driver/subscription 
is forgotten with e.g. `core::mem::forget`. Since local borrows are a very useful feature however, these are still allowed via the newly-introduced 
and `unsafe` methods/constructors that follow the naming pattern of the safe methods/constructors, but with a `_nonstatic` suffix attached.

## [0.47.2] - 2023-11-02
* Remove dependency on `AtomicU64` which is no longer supported by the upstream `*-espidf` targets
* HTTP client: support for chunked encoding of POST requests

## [0.47.1] - 2023-10-18
* Compatibility with `embedded-svc` 0.26.1

## [0.47.0] - 2023-10-17
* MSRV raised to 1.71
* New `experimental` module - `bt` - providing Bluetooth support based on the ESP-IDF Bluedroid implementation
  * Only classic BT supported for now (on the ESP32) with the following profiles: A2DP sink, AVRC controller, HFP client, GAP
  * BLE support in the works, but not buildable yet
* TLS over TCP/IP support in the `tls` module via `EspTls` and (for async mode) `AsyncEspTls`
* PSK support for `mqtt`
* Dependencies `esp-idf-sys` and `esp-idf-hal` are now re-exported as `esp_idf_svc::sys` and `esp_idf_svc::hal`
* Upgraded to `embedded-svc` 0.26
* OTA: New method: `EspOta::finish` that allows to postpone/avoid setting the updated partition as a boot one
* Breaking change: OTA: `EspOtaUpdate` now parametric over the lifetime of a mutable reference to `EspOta` and returned by value
* Breaking change: OTA: `EspOtaUpdate::abort` and `EspOtaUpdate::complete` now take `self` instead of `&mut self`
* Breaking change: HTTP server: Scoped handlers; handlers now need to live only as long as the `EspHttpServer` instance. Therefore, `EspHttpServer` is now lifetimed: `EspHttpServer<'a>`
* Breaking change: HTTP server: `EspHttpRequest` renamed to `EspHttpRawConnection`
* Breaking change: WS client: Scoped handler; the event handler now needs to live only as long as the `EspWebSocketClient` instance. Therefore, `EspWebSocketClient` is now lifetimed: `EspWebSocketClient<'a>` 
* Breaking change: EspTimerService: scoped handler: the timer callback now only needs to live as long as the returned `EspTimer` instance. Therefore, `EspTimer` is now lifetimed: `EspTimer<'a>`
* Breaking change: EspEventLoop: scoped handler: the subscription callback now only needs to live as long as the returned `EspSubscription` instance. Therefore, `EspSubscription` is now lifetimed: `EspSubscription<'a, ...>`
* Breaking change: MQTT client: Scoped handler; the event handler now needs to live only as long as the `EspMqttClient` instance. Therefore, `EspMqttClient` is now lifetimed: `EspMqttClient<'a, ...>` 
* Breaking change: EspNow client: Scoped handlers; the event handlers now need to live only as long as the `EspNow` instance. Therefore, `EspNow` is now lifetimed: `EspNow<'a>` 
* Breaking change: Sntp client: Scoped handler; the event handler now needs to live only as long as the `EspSntp` instance. Therefore, `EspSntp` is now lifetimed: `EspSntp<'a>` 
* Breaking change (bugfix): Ping client: `EspPing::ping_details` now takes a `FnMut` callback, however the callback needs to be `Send`
* Breaking change: Removed the deprecated module `httpd` and the dependency on `anyhow`
* Breaking change: module `notify` removed as it was rarely - if ever - used, and there is a simpler `hal::task::notification` module now
* Deprecated: Using ESP-IDF 4.3 is now deprecated and all special cfg flags will be removed in the next release

## [0.46.2] - 2023-07-30

* EspMdns::query crashes when no services are found #279
* OTA: get_running_slot was wrongly returning the boot slot

## [0.46.1] - 2023-07-30

* Workaround issue 11921 in ESP IDF (new member of struct `wifi_scan_config_t`)
* Make all conversions to CString fallible rather than panic-ing
* Bugfixes in HTTPD WS support: Allow calls with a zero-length buffer
* Added docstrings for wifi module (#262)

## [0.46.0] - 2023-05-13

* MSRV 1.66
* Support for ESP IDF 5.0, 5.1 and 5.2 (master)
* Remove the `experimental` status from all formerly experimental features
* Remove the `nightly` feature flag guard from all `asyncify` modules as Rust GATs are stable now
* Async and blocking APIs for `Wifi`, `Eth` and `EspNetif` that abstract away the ESP IDF system event loop (for easier initial configuration) - API breakage in `netif`
* `Eth` SPI driver rebased on top of `esp-idf-hal`'s `SpiDeviceDriver`; it can now either borrow or own the SPI device driver (API breakage)
* `Eth` driver now supports SPI bus sharing with other SPI devices (API breakage)
* `NVS` - additional APIs that support the serde format of the ESP IDF NVS C code
* `SNTP` - new, callback API
* `log` - support for setting target level per module
* `OTA` - small API extensions
* `netif` - compilation errors when PPP & SLIP support is enabled are addressed
* HTTP client & server - various bugfixes
* `EspError::from_infallible`

## [0.45.0] - 2022-12-13

HTTP server:
* Compatibility with `embedded-svc` V0.24
* New function - `fn_handler` that addresses HRTB lifetime issues when converting a Fn closure to a `Handler` instance
* Remove `EspHttpFnTraversableChain`; it is not necessary, now that the `fn_handler` function from above does exist

## [0.44.0] - 2022-12-09

Rebase on top of `esp-idf-sys` 0.32:
* Retire any usages of `esp-idf-sys::c_types` in favor of `core::ffi`
* Remove the `cstr_core` dependency as `Cstr` and `CString` are now part of Rust core
* Remove casts from `usize` to `u32` and back now that `esp-idf-sys` is compiled with `--size_t-is-usize` enabled

## [0.43.3, 0.43.4, 0.43.5] - 2022-12-08

Patch releases:
* Eth driver:
  * SPI drivers now work with ESP IDF 5+
  * DMA channel is now configurable
* Clippy fixes

## [0.43.1, 0.43.2] - 2022-11-21

Patch releases to fix compilation errors under no_std.

## [0.43.0] - 2022-11-01

Release 0.43 is a backwards-incompatible release where almost all services were touched in one way or another.

### Major Changes

The main themes of the 0.43 release are:
* Public API
* Separate the notions of using a "nightly" compiler (which is a precondition for all async support) from "experimental" features, which might or might not be async-related
* Expose access to the wrapped ESP IDF services / drivers
* Wifi & Eth: separate layer 2 (ethernet) from layer 3 (IP)
* Http client & server: implement the new traits from `embedded-svc`
* Merge the `nvs_storage` module into `nvs`
* Support for the `embassy-time` crate by providing alarm implementation
* Support for the `embassy-time` crate by providing timer queue implementation

### Major changes elaboration

### Public API

In addition to implementing the `embedded-svc` traits where possible, all services now have public API. While the public API loosely follows the APIs from `embedded-svc`, it deviates where appropriate so that the native underlying ESP IDF service is better utilized.

These public APIs mean that the user is no longer required to depend on the `embedded-svc` crate so as to consume the `esp-idf-svc` services.
Consuming the services via the `embedded-svc` traits is now only necessary when the user is targetting cross-platform portability of their application code.

### Expose access to the wrapped ESP IDF services / drivers

All services now implement the `Handle` trait which does provide a reference to the native underlying ESP IDF service or driver. This is useful when the Rust wrapper for the service does not expose a particular functionality, as in that case, users can still call the functionality by using the raw `esp-idf-svc` bindings for the service.

### Wifi & Eth: separate layer 2 (Ethernet) from layer 3 (IP)

The Wifi and Ethernet drivers are now split into two separate structures:
* `WifiDriver` / `EthDriver` - these are layer 2 (Ethernet) drivers which follow all the conventions of the other drivers in the `esp-idf-hal` crate, including the need to own or mutably borrow the actual petihperal (the Wifi modem peripheral or the RMII / SPI peripheral from `esp-idf-hal`). They are however part of `esp-idf-svc` as this is better aligned with their native ESP IDF counterparts, which actually do use/rely on certain ESP IDF services, like the event loop and NVS storage, which - being services - are also exposed as part of `esp-idf-svc`, and not as part of `esp-idf-hal`. These drivers implement the `Wifi` and `Eth` traits respectively, which were redesigned to not have any IP-related semantics. Users are allowed to use the Layer 2 drivers directly by providing their own custom RX callbacks, thus completely bypassing the ESP IDF LwIP IP & Netif layer (i.e. with `smoltcp` or other 3rd party IP stack)
* `EspWifi` / `EspEth` - these are layer 3 (IP) + layer 2 (Ethernet) services, which - on construction - are expected to own a `WifiDriver` / `EthDriver` - either by constructing it themselves, or by the user passing the driver. These services "glue" the ESP IDF IP & Netif layer (`EspNetif`) with the ethernet layer provided by the drivers. These services *also* implement the `Wifi` / `Eth` traits, as they are wrapping Layer 2 functionality anyway. The Layer 3 functionality (configuring the network interface as well as fetching IP-related information from the network interfaces) however uses custom methods on the services' themselves and is not (yet) abstracted using `embedded-svc` traits.

Additionally, the `Wifi` and `Eth` trait implementations now provide finer grained control over the behavior of their drivers / services in that users should explicitly call `start`/`stop` to start/stop the driver, as well as `connect`/`disconnect` (for the Wifi driver in STA mode) to connect to an access point. While this makes the driver configuration a bit more involved, these changes provide the necessary flexibility for some corner use cases:
* When the Wifi driver is used together with the `EspNow` ESP NOW service, there is no need to actually `connect` the driver at all, which is now possible
* More complex connect/disconnect schemes can now be implemented by users, for roaming or for reacting in case the Wifi connection is lost

### Http client & server: implement the new traits from `embedded-svc`

Subject says it all, so to say.

### Merge the `nvs_storage` module into `nvs`

* The previous distinction of two separate modules was awkward and is thus removed
* The other notable change here is that the ESP IDF implementation actually only implements the `RawStorage` trait, which provides facilities for reading / writing blobs. It is up to the user to layer a `Storage` implementation on top of the `RawStorage` implementation, but the benefit of that is that user is in control of how their structures are serialized/deserialized into binary. To ease the layering, users may take advantage of the `StorageImpl` structure from `embedded-svc` and only provide a `Serde` trait implementation which abstracts away the concrete Rust SerDe implementation (i.e. `serde-json`, `postcard`, etc.)

### Support for the `embassy-time` crate by providing alarm implementation

`esp-idf-svc` provides an implementation of `embassy-time`'s alarm interface (the `Driver` trait), which is implemented in terms of the ESP IDF Timer service (also exposed in the `timer` module of `esp-idf-svc`). 

To use this feature, users need to enable the `embassy-time-driver` Cargo feature.

### Support for the `embassy-time` crate by providing timer queue implementation

`esp-idf-svc` does provide a custom `embassy-time` Timer Queue implementation (also implemented in terms of the ESP IDF Timer service), even though in the meantime `embassy-time` features a generic timer queue which works everywhere and can also be used. This custom timer queue does not rely on the alarms interface provided by the `embassy-time` crate (see the previous section).

The one major difference between `embassy-time`'s generic timer queue, and the one provided by `esp-idf-svc` is that the latter has a slightly lower latency in that it does support the `CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y` ESP IDF configuration parameter. When this parameter is enabled, the `esp-idf-svc` timer queue does not use the ESP IDF Timer Service dispatch task/thread and notifies the executor about due timers **directly from the ISR routine**. When this parameter is not enabled, the `esp-idf-svc` timer queue has no benefits compared to the generic timer queue in `embassy-time`.

NOTE: 
* The `esp-idf-svc` timer queue should **only** be used with async executors that are ISR safe, in that they can be awoken from an ISR. `edge-executor` is such an executor.
* `embassy-executor` is currently NOT ISR safe, because it relies - for its synchronization - on the `critical-section` crate, yet the critical section implementation that the `critical-section` crate uses on top of `esp-idf-hal` is based on a FreeRTOS mutex, and NOT on a disable-all-interupts ISR-safe implementation (that is, unless the user has configured a more elaborate setup with their own critical section implementation). On the other hand, `embassy-executor` has its own highly optimized timer queue which should probably be used anyway and is enabled by default
* All other executors can just use the generic timer queue implementation which is built-in in the `embassy-time` crate

To use this feature, users need to enable the `embassy-time-isr-queue` Cargo feature.
