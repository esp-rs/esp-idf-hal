// (Temporary code) ESP-IDF does not (yet) have a pthread rwlock implementation, which is required by STD
// We provide a quick and very hacky implementation here
#[cfg(all(feature = "std", esp_idf_version = "4.3"))]
mod pthread_rwlock;

// (Temporary code) ESP-IDF current stable version (4.3) has atomics for ESP32S2, but not for ESP32C3
// The ESP-IDF master branch has atomics for both
#[cfg(all(esp32c3, esp_idf_version = "4.3"))]
mod atomics_esp32c3;

pub struct PatchesRef(*mut crate::c_types::c_void, *mut crate::c_types::c_void);

/// A hack to make sure that the rwlock implementation and the esp32c3 atomics are linked to the final executable
/// Call this function once e.g. in the beginning of your main function
///
/// This function will become no-op once ESP-IDF V4.4 is released
pub fn link_patches() -> PatchesRef {
    #[cfg(all(feature = "std", esp_idf_version = "4.3"))]
    let rwlock = pthread_rwlock::link_patches();

    #[cfg(any(
        not(feature = "std"),
        not(all(feature = "std", esp_idf_version = "4.3"))
    ))]
    let rwlock = core::ptr::null_mut();

    #[cfg(all(esp32c3, esp_idf_version = "4.3"))]
    let atomics = atomics_esp32c3::link_patches();

    #[cfg(not(all(esp32c3, esp_idf_version = "4.3")))]
    let atomics = core::ptr::null_mut();

    PatchesRef(rwlock, atomics)
}
