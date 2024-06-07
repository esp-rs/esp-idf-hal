use crate::sys::*;

pub struct Configuration {
    pub max_files: u32,
    pub format_if_mount_failed: bool,
    pub allocation_unit_size: usize,
    #[cfg(not(esp_idf_version_major = "4",))] // For ESP-IDF v5.0 and later
    pub disk_status_check_enable: bool,
    #[cfg(not(any(
        esp_idf_version_major = "4",
        all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
        all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
        all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
    )))] // ESP-IDF 5.3 and later
    pub use_one_fat: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self::new()
    }
}

impl Configuration {
    pub const fn new() -> Self {
        Self {
            max_files: 4,
            format_if_mount_failed: false,
            allocation_unit_size: 16 * 1024,
            #[cfg(not(
                esp_idf_version_major = "4",
            ))] // For ESP-IDF v5.0 and later
            disk_status_check_enable: false,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))] // ESP-IDF 5.3 and later
            use_one_fat: false,
        }
    }
}

impl From<Configuration> for esp_vfs_fat_mount_config_t {
    fn from(config: Configuration) -> Self {
        esp_vfs_fat_mount_config_t {
            max_files: config.max_files as i32,
            format_if_mount_failed: config.format_if_mount_failed,
            allocation_unit_size: config.allocation_unit_size,
            #[cfg(not(
                esp_idf_version_major = "4",
            ))] // For ESP-IDF v5.0 and later
            disk_status_check_enable: config.disk_status_check_enable,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))] // ESP-IDF 5.3 and later
            use_one_fat: config.use_one_fat,
        }
    }
}
