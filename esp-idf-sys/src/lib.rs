//! Raw Rust bindings for the [ESP-IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/).
//!
//! # Build Prerequisites
//!
//! Follow the [Prerequisites](https://github.com/esp-rs/esp-idf-template#prerequisites) section in the `esp-idf-template` crate.
//!
#![doc = include_str!("../BUILD-OPTIONS.md")]
#![no_std]
#![cfg_attr(
    all(not(feature = "std"), feature = "alloc_handler"),
    feature(alloc_error_handler)
)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(unexpected_cfgs)]

pub use bindings::*;
pub use error::*;

// Don't use esp_idf_soc_pcnt_supported; that's only on ESP-IDF v5.x+.
#[cfg(any(esp32, esp32s2, esp32s3, esp32c6, esp32h2))]
pub use pcnt::*;

#[doc(hidden)]
pub use build_time;
#[doc(hidden)]
pub use const_format;
#[doc(hidden)]
pub use patches::PatchesRef;

#[cfg(feature = "std")]
#[allow(unused_imports)]
#[macro_use]
extern crate std;

#[cfg(feature = "alloc")]
#[allow(unused_imports)]
#[macro_use]
extern crate alloc;

mod alloc;
mod app_desc;
mod error;
mod panic;
mod patches;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c6, esp32h2))]
mod pcnt;

mod start;

/// If any of the two constants below do not compile, you have not properly setup the rustc cfg flag `espidf_time64`:
/// When compiling against ESP-IDF V5.X or later, you need to define the following in your `.config/cargo.toml` file
/// (look for this file in the root of your binary crate):
/// ```
/// [build]
/// rustflags = "--cfg espidf_time64"
/// ```
///
/// When compiling against ESP-IDF V4.X, you need to remove the above flag
#[allow(deprecated)]
#[allow(unused)]
#[cfg(feature = "std")]
const ESP_IDF_TIME64_CHECK: ::std::os::espidf::raw::time_t = 0 as crate::time_t;
#[allow(unused)]
const ESP_IDF_TIME64_CHECK_LIBC: ::libc::time_t = 0 as crate::time_t;

/// A hack to make sure that a few patches to the ESP-IDF which are implemented in Rust
/// are linked to the final executable
///
/// Call this function once at the beginning of your main function
pub fn link_patches() -> PatchesRef {
    patches::link_patches()
}

#[allow(clippy::all)]
#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
#[allow(rustdoc::all)]
#[allow(improper_ctypes)] // TODO: For now, as 5.0 spits out tons of these
#[allow(dead_code)]
mod bindings {
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c6, esp32h2))]
    use crate::pcnt::*;

    include!(env!("EMBUILD_GENERATED_BINDINGS_FILE"));
}
