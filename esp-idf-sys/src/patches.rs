// (Temporary code) ESP-IDF does not (yet) have a pthread rwlock implementation, which is required by STD
// We provide a quick and very hacky implementation here
#[cfg(all(feature = "std", esp_idf_version = "4.3"))]
mod pthread_rwlock;

#[cfg(feature = "std")]
mod lstat;

pub struct PatchesRef(*mut core::ffi::c_void, *mut core::ffi::c_void);

/// A hack to make sure that the rwlock implementation is linked to the final executable
/// Call this function once e.g. in the beginning of your main function
pub fn link_patches() -> PatchesRef {
    #[cfg(all(feature = "std", esp_idf_version = "4.3"))]
    let rwlock = pthread_rwlock::link_patches();

    #[cfg(any(
        not(feature = "std"),
        not(all(feature = "std", esp_idf_version = "4.3"))
    ))]
    let rwlock = core::ptr::null_mut();

    #[cfg(feature = "std")]
    let lstat = lstat::link_patches();
    #[cfg(not(feature = "std"))]
    let lstat = core::ptr::null_mut();

    PatchesRef(rwlock, lstat)
}
