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
#[cfg(any(esp32, esp32s2, esp32s3, esp32c6))]
pub mod pcnt;
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
