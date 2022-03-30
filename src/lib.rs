#![cfg_attr(not(feature = "std"), no_std)]
#![feature(cfg_version)]
#![feature(generic_associated_types)] // For mutex
#![cfg_attr(version("1.61"), allow(deprecated_where_clause_location))]
#![cfg_attr(not(version("1.59")), feature(asm))]
#![cfg_attr(
    all(version("1.58"), target_arch = "xtensa"),
    feature(asm_experimental_arch)
)]

#[cfg(all(feature = "std", feature = "riscv-ulp-hal"))]
compile_error!("Feature `std` is not compatible with feature `ulp`");

#[cfg(all(feature = "embedded-svc-mutex", feature = "riscv-ulp-hal"))]
compile_error!("Feature `embedded-svc-mutex` is not compatible with feature `ulp`");

#[cfg(all(feature = "riscv-ulp-hal", not(esp32s2)))]
compile_error!("Feature `ulp` is currently only supported on esp32s2");

#[macro_use]
pub mod riscv_ulp_hal;

pub mod adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod can;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod cpu;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod delay;
pub mod gpio;
#[cfg(esp32)]
pub mod hall;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod i2c;
#[cfg(all(feature = "experimental", not(feature = "riscv-ulp-hal")))]
pub mod interrupt;
pub mod ledc;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod mutex;
pub mod peripherals;
pub mod prelude;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod rmt;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod serial;
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod spi;
#[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
pub mod ulp;
pub mod units;

#[cfg(feature = "riscv-ulp-hal")]
pub use crate::riscv_ulp_hal::delay;

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

// This is used to implement an extra error trait on the `embedded_hal` compatible error struct
// Example:
// embedded_hal_error!(CanError, embedded_hal::can::Error, embedded_hal::can::ErrorKind) // For e-hal 1.0 compatibility
// embedded_hal_error_trait!(CanError, embedded_hal_0_2::can::Error, embedded_hal_0_2::can::ErrorKind) // For e-hal 0.2 compatibility
#[allow(unused_macros)]
macro_rules! embedded_hal_error_trait {
    ($error:ident, $errortrait:ty, $kind:ty) => {
        impl $errortrait for $error {
            fn kind(&self) -> $kind {
                self.kind
            }
        }
    };
}

#[allow(unused_imports)]
pub(crate) use embedded_hal_error;

#[allow(unused_imports)]
pub(crate) use embedded_hal_error_trait;
