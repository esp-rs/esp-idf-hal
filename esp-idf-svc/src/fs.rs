#[cfg(all(feature = "alloc", esp_idf_comp_fatfs_enabled))]
pub mod fatfs;
#[cfg(all(feature = "alloc", esp_idf_comp_spiffs_enabled))]
pub mod spiffs;
