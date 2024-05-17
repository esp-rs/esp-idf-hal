#[cfg(esp_idf_comp_fatfs_enabled)]
pub mod fat;
#[cfg(esp_idf_comp_fatfs_enabled)]
pub use fat::*;

#[cfg(esp_idf_comp_fatfs_enabled)]
pub mod config;
#[cfg(esp_idf_comp_fatfs_enabled)]
pub use config::Configuration as FatConfiguration;
