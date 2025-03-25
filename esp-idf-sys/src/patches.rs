// (Temporary code) ESP-IDF does not (yet) have a pthread rwlock implementation, which is required by STD
// We provide a quick and very hacky implementation here
mod atexit;
#[cfg(feature = "std")]
mod lstat;

#[allow(dead_code)]
pub struct PatchesRef(*mut core::ffi::c_void, *mut core::ffi::c_void);

/// A hack to make sure that certain symbols are linked to the final executable
/// Call this function once e.g. in the beginning of your main function
pub fn link_patches() -> PatchesRef {
    #[cfg(esp_idf_version_major = "4")]
    notify_about_deprecation();

    #[cfg(feature = "std")]
    let lstat = lstat::link_patches();
    #[cfg(not(feature = "std"))]
    let lstat = core::ptr::null_mut();

    let atexit = atexit::link_patches();

    PatchesRef(lstat, atexit)
}

#[cfg(esp_idf_version_major = "4")]
#[deprecated(
    since = "0.35.0",
    note = "\nESP-IDF version 4 is now considered deprecated.\nPlease upgrade your project to ESP-IDF 5 or later, because future releases will remove support for ESP IDF 4."
)]
const fn notify_about_deprecation() {}
