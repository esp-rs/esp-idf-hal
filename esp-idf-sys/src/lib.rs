//! Raw Rust bindings for the [ESP-IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/).
//!
//! # Build Prerequisites
//!
//! Follow the [Prerequisites](https://github.com/esp-rs/esp-idf-template#prerequisites) section in the `esp-idf-template` crate.
//!
#![doc = include_str!("../BUILD_OPTIONS.md")]
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(
    all(not(feature = "std"), feature = "alloc_handler"),
    feature(alloc_error_handler)
)]

pub use bindings::*;
pub use error::*;

#[doc(hidden)]
pub use build_time;
#[doc(hidden)]
pub use const_format;
#[doc(hidden)]
pub use patches::PatchesRef;

mod alloc;
mod app_desc;
mod error;
mod panic;
mod patches;
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
mod bindings {
    // The following is defined to remove a case where bindgen can't handle pcnt_unit_t being defined
    // in two different C namespaces (enum vs struct). The struct is opaque (used only as a pointer to an
    // opaque type via pcnt_channel_handle_t), so we use the enum definition here, taken from the v4
    // bindgen.
    #[cfg(any(esp32, esp32s2, esp32s3, esp32h2, esp32c6, esp32p4))]
    /// Selection of all available PCNT units
    #[allow(non_camel_case_types)]
    pub type pcnt_unit_t = core::ffi::c_int;

    include!(env!("EMBUILD_GENERATED_BINDINGS_FILE"));
}
