#![feature(llvm_asm)]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(generic_associated_types)] // For mutex

#[cfg(all(feature = "std", feature = "ulp"))]
compile_error!("Feature `std` is not compatible with feature `ulp`");

#[cfg(all(feature = "embedded-svc-mutex", feature = "ulp"))]
compile_error!("Feature `embedded-svc-mutex` is not compatible with feature `ulp`");

#[cfg(all(feature = "ulp", not(esp32s2)))]
compile_error!("Feature `ulp` is currently only supported on esp32s2");

#[macro_use]
pub mod ulp;
pub mod adc;
#[cfg(not(feature = "ulp"))]
pub mod can;
#[cfg(all(feature = "experimental", not(feature = "ulp")))]
pub mod cpu;
#[cfg(not(feature = "ulp"))]
pub mod delay;
pub mod gpio;
#[cfg(esp32)]
pub mod hall;
#[cfg(not(feature = "ulp"))]
pub mod i2c;
#[cfg(all(feature = "experimental", not(feature = "ulp")))]
pub mod interrupt;
#[cfg(not(feature = "ulp"))]
pub mod mutex;
pub mod peripherals;
pub mod prelude;
#[cfg(not(feature = "ulp"))]
pub mod serial;
#[cfg(not(feature = "ulp"))]
pub mod spi;
pub mod units;

#[cfg(feature = "ulp")]
pub use crate::ulp::delay;

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
