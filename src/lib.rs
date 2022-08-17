#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(target_arch = "xtensa", feature(asm_experimental_arch))]

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp_idf_comp_driver_enabled)))]
compile_error!("esp-idf-hal requires the `driver` ESP-IDF component to be enabled");

#[cfg(all(
    any(
        feature = "std",
        feature = "alloc",
        feature = "critical-section-interrupt",
        feature = "critical-section-mutex"
    ),
    feature = "riscv-ulp-hal"
))]
compile_error!("Enabling feature `riscv-ulp-hal` implies no other feature is enabled");

#[cfg(all(feature = "riscv-ulp-hal", not(esp32s2)))]
compile_error!("Feature `riscv-ulp-hal` is currently only supported on esp32s2");

#[macro_use]
pub mod riscv_ulp_hal;

pub mod adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod can;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod cpu;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod cs;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod delay;
pub mod gpio;
#[cfg(all(esp32, esp_idf_version_major = "4"))]
pub mod hall;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod i2c;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod interrupt;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod ledc;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod mac;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod modem;
pub mod peripherals;
pub mod prelude;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod rmt;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod serial;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod spi;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod task;
#[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
pub mod ulp;
pub mod units;

#[cfg(feature = "riscv-ulp-hal")]
pub use crate::riscv_ulp_hal::delay;

#[cfg(all(
    feature = "edge-executor",
    feature = "alloc",
    target_has_atomic = "ptr"
))]
pub mod executor;

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

#[allow(unused_imports)]
pub(crate) use embedded_hal_error;
