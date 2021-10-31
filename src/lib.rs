#![feature(llvm_asm)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(all(feature = "std", feature = "ulp"))]
compile_error!("Feature `std` is not compatible with feature `ulp`");

#[cfg(all(feature = "ulp", not(esp32s2)))]
compile_error!("Feature `ulp` is currently only supported on esp32s2");

#[macro_use]
pub mod ulp;
#[cfg(not(feature = "ulp"))]
pub mod delay;
pub mod gpio;
#[cfg(not(feature = "ulp"))]
pub mod i2c;
pub mod peripherals;
pub mod prelude;
#[cfg(not(feature = "ulp"))]
pub mod serial;
#[cfg(not(feature = "ulp"))]
pub mod spi;
pub mod units;

#[cfg(feature = "ulp")]
pub use crate::ulp::delay;
