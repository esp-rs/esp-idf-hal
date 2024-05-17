#[cfg(not(any(
    esp_idf_version_major = "4",
    all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
    all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
)))] // For ESP-IDF v5.2 and later
use crate::sys::*;

#[cfg(not(any(
    esp_idf_version_major = "4",
    all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
    all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
)))] // For ESP-IDF v5.2 and later
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum DelayPhase {
    Phase0 = sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0 as isize,
    Phase1 = sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_1 as isize,
    Phase2 = sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_2 as isize,
    Phase3 = sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_3 as isize,
}

#[cfg(not(any(
    esp_idf_version_major = "4",
    all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
    all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
)))] // For ESP-IDF v4.x, v5.0, and v5.1
impl From<sdmmc_delay_phase_t> for DelayPhase {
    fn from(phase: sdmmc_delay_phase_t) -> Self {
        #[allow(non_upper_case_globals)]
        match phase {
            sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0 => Self::Phase0,
            sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_1 => Self::Phase1,
            sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_2 => Self::Phase2,
            sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_3 => Self::Phase3,
            _ => panic!("Invalid delay phase"),
        }
    }
}

pub struct Configuration {
    pub command_timeout_ms: u32,
    pub io_voltage: f32,
    pub high_speed: bool,
    #[cfg(not(any(
        esp_idf_version_major = "4",
        all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
        all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
    )))] // For ESP-IDF v5.2 and later
    pub input_delay_phase: DelayPhase,
}

impl Default for Configuration {
    fn default() -> Self {
        Self::new()
    }
}

impl Configuration {
    pub const fn new() -> Self {
        Self {
            command_timeout_ms: 0,
            io_voltage: 3.3,
            high_speed: false,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
            )))] // For ESP-IDF v5.2 and later
            input_delay_phase: DelayPhase::Phase0,
        }
    }
}
