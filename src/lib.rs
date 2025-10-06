#![no_std]
#![allow(async_fn_in_trait)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(clippy::unused_unit)] // enumset
#![allow(unexpected_cfgs)]
#![warn(clippy::large_futures)]
#![cfg_attr(feature = "nightly", feature(doc_cfg))]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

#[cfg(not(esp_idf_comp_driver_enabled))]
compile_error!("esp-idf-hal requires the `driver` ESP-IDF component to be enabled");

// mutually exclusive features assert
#[cfg(all(feature = "rmt-legacy", esp_idf_comp_espressif__onewire_bus_enabled))]
compile_error!("the onewire component cannot be used with the legacy rmt peripheral");

#[cfg(feature = "std")]
#[allow(unused_imports)]
#[macro_use]
extern crate std;

#[cfg(feature = "alloc")]
#[allow(unused_imports)]
#[macro_use]
extern crate alloc;

pub mod adc;
pub mod can;
pub mod cpu;
pub mod delay;
pub mod gpio;
pub mod i2c;
#[cfg_attr(
    feature = "nightly",
    doc(cfg(all(esp_idf_soc_i2s_supported, esp_idf_comp_driver_enabled)))
)]
pub mod i2s;
pub mod interrupt;
pub mod io;
pub mod ledc;
#[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
pub mod mac;
pub mod modem;
#[cfg(all(
    esp_idf_soc_rmt_supported,
    not(esp_idf_version_major = "4"),
    esp_idf_comp_espressif__onewire_bus_enabled,
))]
pub mod onewire;

#[cfg(not(esp_idf_version_at_least_6_0_0))]
#[cfg(all(any(esp32, esp32s2, esp32s3, esp32c6), feature = "pcnt-legacy"))]
#[cfg_attr(
    all(
        not(esp_idf_version_at_least_6_0_0),
        any(esp32, esp32s2, esp32s3, esp32c6),
        feature = "pcnt-legacy"
    ),
    deprecated(
        since = "0.46.0",
        note = "use the new `pcnt` api by disabling the `pcnt-legacy` feature"
    )
)]
mod pcnt_legacy;
#[cfg(all(not(feature = "pcnt-legacy"), esp_idf_soc_pcnt_supported))]
mod pcnt_new;

#[cfg_attr(feature = "nightly", doc(cfg(esp_idf_soc_pcnt_supported)))]
#[cfg(esp_idf_soc_pcnt_supported)]
/// Pulse Counter (PCNT) peripheral.
///
/// The PCNT (Pulse Counter) module is designed to count the number of rising
/// and/or falling edges of input signals. The ESP32 chip contains multiple pulse
/// counter units in the module. The number of available units and channels is
/// different for each chip. See the [Technical Reference Manual] of your chip for
/// details (or the table below).
///
/// Each unit is in effect an independent counter with multiple channels
#[cfg_attr(
    not(feature = "pcnt-legacy"),
    doc = "(see [`PcntChannel`](pcnt::PcntChannel))"
)]
/// where each channel can increment/decrement the counter on a rising/falling
/// edge. Furthermore, each channel can be configured separately.
///
/// Besides that, PCNT unit is equipped with a separate glitch filter, which is
/// helpful to remove noise from the signal.
#[cfg_attr(
    not(feature = "pcnt-legacy"),
    doc = "See [`PcntUnitDriver::set_glitch_filter`](pcnt::PcntUnitDriver::set_glitch_filter) for details."
)]
///
/// Typically, a PCNT module can be used in scenarios like:
/// - Calculate periodic signal's frequency by counting the pulse numbers within a time slice
/// - Decode quadrature signals into speed and direction
/// - Count the number of pulses that happen within a time slice (e.g. step signal of a stepper motor driver)
///
/// [Technical Reference Manual]: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#pcnt
///
/// # Driver redesign in ESP-IDF 5.0
///
/// In ESP-IDF 5.0, the [PCNT API was redesigned] to simplify and unify the usage of the
/// PCNT peripheral.
///
/// It is recommended to use the new API, but for now the old API is available through
/// the `pcnt-legacy` feature. The ESP-IDF 6.0 release will remove support for the legacy API.
///
/// [PCNT API was redesigned]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/migration-guides/release-5.x/5.0/peripherals.html#pulse-counter-driver
///
/// # Chip Capabilities
///
/// The following table is based on the ESP-IDF source code and the relevant
/// constants are not public. Therefore, this table might be inaccurate or incomplete.
///
/// |         | Units | Channels | Extra Watchpoints |
/// |---------|-------|----------|-------------------|
/// | esp32   | 8     | 2        | 2                 |
/// | esp32c6 | 4     | 2        | 2                 |
/// | esp32h4 | 4     | 2        | 2                 |
/// | esp32p4 | 4     | 2        | 2                 |
/// | esp32s2 | 4     | 2        | 2                 |
/// | esp32s3 | 4     | 2        | 2                 |
///
/// **Units**: Maximum number of [`PcntUnitDriver`](pcnt::PcntUnitDriver) that can be created
///
/// **Channels**: Maximum number of [`PcntChannel`](pcnt::PcntChannel) channels for each unit
///
/// **Extra Watchpoints**: Maximum number of watchpoints (in addition to zero point and high/low limits) that can be set for each unit.
///
/// The values seem to map to the following constants in `esp-idf-sys`:
/// - Maximum number of units: [`SOC_PCNT_UNITS_PER_GROUP`](esp_idf_sys::SOC_PCNT_UNITS_PER_GROUP)
/// - Maximum number of channels per unit: [`SOC_PCNT_CHANNELS_PER_UNIT`](esp_idf_sys::SOC_PCNT_CHANNELS_PER_UNIT)
/// - Maximum number of extra watchpoints: [`SOC_PCNT_THRES_POINT_PER_UNIT`](esp_idf_sys::SOC_PCNT_THRES_POINT_PER_UNIT)
pub mod pcnt {
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(any(esp32, esp32s2, esp32s3, esp32c6), feature = "pcnt-legacy"))]
    pub use crate::pcnt_legacy::*;

    #[cfg(all(not(feature = "pcnt-legacy"), esp_idf_soc_pcnt_supported))]
    pub use crate::pcnt_new::*;
}

pub mod peripherals;
pub mod reset;
pub mod rmt;
pub mod rom;
pub mod sd;
pub mod spi;
pub mod sys;
pub mod task;
#[cfg(all(esp_idf_soc_temp_sensor_supported, esp_idf_version_major = "5"))]
pub mod temp_sensor;
#[cfg(not(esp_idf_version_at_least_6_0_0))]
pub mod timer;
pub mod uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
    esp_idf_comp_ulp_enabled
))]
pub mod ulp;
pub mod units;
#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
pub mod usb_serial;

// This is used to create `embedded_hal` compatible error structs
// that preserve original `EspError`.
//
// Example:
// embedded_hal_error!(I2cError, embedded_hal::i2c::Error, embedded_hal::i2c::ErrorKind)
#[allow(unused_macros)]
macro_rules! embedded_hal_error {
    ($error:ident, $errortrait:ty, $kind:ty) => {
        #[derive(Debug, Copy, Clone, Eq, PartialEq)]
        pub struct $error {
            kind: $kind,
            cause: esp_idf_sys::EspError,
        }

        impl $error {
            pub fn new(kind: $kind, cause: esp_idf_sys::EspError) -> Self {
                Self { kind, cause }
            }
            pub fn other(cause: esp_idf_sys::EspError) -> Self {
                Self::new(<$kind>::Other, cause)
            }
            pub fn cause(&self) -> esp_idf_sys::EspError {
                self.cause
            }
        }
        impl From<esp_idf_sys::EspError> for $error {
            fn from(e: esp_idf_sys::EspError) -> Self {
                Self::other(e)
            }
        }

        impl $errortrait for $error {
            fn kind(&self) -> $kind {
                self.kind
            }
        }

        impl core::fmt::Display for $error {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                write!(
                    f,
                    "{} {{ kind: {}, cause: {} }}",
                    stringify!($error),
                    self.kind,
                    self.cause()
                )
            }
        }

        #[cfg(feature = "std")]
        impl std::error::Error for $error {}
    };
}

#[allow(unused_macros)]
macro_rules! impl_peripheral {
    ($name:ident) => {
        pub struct $name<'a>(::core::marker::PhantomData<&'a mut ()>);

        impl $name<'_> {
            /// Unsafely create an instance of this peripheral out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline(always)]
            pub unsafe fn steal() -> Self {
                Self(::core::marker::PhantomData)
            }

            /// Creates a new peripheral reference with a shorter lifetime.
            ///
            /// Use this method if you would like to keep working with the peripheral after
            /// you dropped the driver that consumes this.
            ///
            /// # Safety
            ///
            /// You must ensure that you are not using reborrowed peripherals in drivers which are
            /// forgotten via `core::mem::forget`.
            #[inline]
            #[allow(dead_code)]
            pub unsafe fn reborrow(&mut self) -> $name<'_> {
                Self(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $name<'_> {}
    };
}

#[allow(unused_imports)]
pub(crate) use embedded_hal_error;
#[allow(unused_imports)]
pub(crate) use impl_peripheral;
