#![feature(llvm_asm)]

#![cfg_attr(not(feature = "std"), no_std)]

cfg_if::cfg_if! {
    if #[cfg(all(feature = "std", feature = "ulp"))] {
        compile_error!("Feature \"std\" is not compatible with feature \"ulp\"");
    }
}

#[cfg(feature = "ulp")]
#[macro_use]
mod ulp;
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
