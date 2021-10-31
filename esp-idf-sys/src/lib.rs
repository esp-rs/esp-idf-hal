#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(
    all(not(feature = "std"), feature = "alloc_handler"),
    feature(alloc_error_handler)
)]

pub use bindings::*;
pub use error::*;
pub use mutex::EspMutex;

pub mod error;
pub mod mutex;

mod alloc;
mod panic;
mod start;

// (Temporary code) ESP-IDF does not (yet) have a pthread rwlock implementation, which is required by STD
// We provide a quick and very hacky implementation here
#[cfg(feature = "std")]
mod pthread_rwlock;

// (Temporary code) ESP-IDF current stable version (4.3) has atomics for ESP32S2, but not for ESP32C3
// The ESP-IDF master branch has atomics for both
#[cfg(all(esp32c3, esp_idf_version = "4.3"))]
mod atomics_esp32c3;

/// A hack to make sure that the rwlock implementation and the esp32c3 atomics are linked to the final executable
/// Call this function once e.g. in the beginning of your main function
///
/// This function will become no-op once ESP-IDF V4.4 is released
pub fn link_patches() -> (*mut c_types::c_void, *mut c_types::c_void) {
    #[cfg(feature = "std")]
    let rwlock = pthread_rwlock::link_patches();

    #[cfg(not(feature = "std"))]
    let rwlock = core::ptr::null_mut();

    #[cfg(all(esp32c3, esp_idf_version = "4.3"))]
    let atomics = atomics_esp32c3::link_patches();

    #[cfg(not(all(esp32c3, esp_idf_version = "4.3")))]
    let atomics = core::ptr::null_mut();

    (rwlock, atomics)
}

#[cfg(feature = "std")]
#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
pub mod c_types {
    pub type c_void = std::os::raw::c_void;
    pub type c_uchar = std::os::raw::c_uchar;
    pub type c_schar = std::os::raw::c_schar;
    pub type c_char = std::os::raw::c_char;
    pub type c_short = std::os::raw::c_short;
    pub type c_ushort = std::os::raw::c_ushort;
    pub type c_int = std::os::raw::c_int;
    pub type c_uint = std::os::raw::c_uint;
    pub type c_long = std::os::raw::c_long;
    pub type c_ulong = std::os::raw::c_ulong;
    pub type c_longlong = std::os::raw::c_longlong;
    pub type c_ulonglong = std::os::raw::c_ulonglong;
}

#[cfg(not(feature = "std"))]
#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
pub mod c_types {
    pub type c_void = core::ffi::c_void;
    pub type c_uchar = u8;
    pub type c_schar = i8;
    pub type c_char = i8;
    pub type c_short = i16;
    pub type c_ushort = u16;
    pub type c_int = i32;
    pub type c_uint = u32;
    pub type c_long = i32;
    pub type c_ulong = u32;
    pub type c_longlong = i64;
    pub type c_ulonglong = u64;
}

#[allow(clippy::all)]
#[allow(non_upper_case_globals)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
mod bindings {
    use super::c_types;

    include!(env!("EMBUILD_GENERATED_BINDINGS_FILE"));
}
