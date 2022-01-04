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
