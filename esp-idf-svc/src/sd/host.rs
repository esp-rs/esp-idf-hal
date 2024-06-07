use crate::sys::*;

#[cfg(esp_idf_soc_sdmmc_host_supported)]
use super::mmc::SlotConfiguration;

use super::{config::Configuration, spi::SpiDevice};

pub enum SdDevice<'d> {
    Spi(SpiDevice<'d>),
    #[cfg(esp_idf_soc_sdmmc_host_supported)]
    Mmc(SlotConfiguration<'d>),
}

pub struct SdHost<'d> {
    device: SdDevice<'d>,
    host: sdmmc_host_t,
}

const _HOST_FLAG_SPI: u32 = 1 << 3;
const _HOST_FLAG_DEINIT_ARG: u32 = 1 << 5;
const _DEFAULT_FREQUENCY: i32 = 20000;
const _HIGH_SPEED_FREQUENCY: i32 = 40000;

const _DEFAULT_IO_VOLTAGE: f32 = 3.3;
const _SDMMC_HOST_FLAG_1BIT: u32 = 1 << 0;
const _SDMMC_HOST_FLAG_4BIT: u32 = 1 << 1;
const _SDMMC_HOST_FLAG_8BIT: u32 = 1 << 2;
const _SDMMC_HOST_FLAG_DDR: u32 = 1 << 3;

impl<'d> SdHost<'d> {
    fn set_from_configuration(mut self, configuration: &Configuration) -> Self {
        self.host.command_timeout_ms = configuration.command_timeout_ms as i32;
        self.host.io_voltage = configuration.io_voltage;
        self.host.max_freq_khz = if configuration.high_speed {
            _HIGH_SPEED_FREQUENCY
        } else {
            _DEFAULT_FREQUENCY
        };

        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
        )))] // For ESP-IDF v5.2 and later
        {
            self.host.input_delay_phase = configuration.input_delay_phase as sdmmc_delay_phase_t;
        }

        self
    }

    pub fn new_with_spi(configuration: &Configuration, device: SpiDevice<'d>) -> Self {
        let host = sdmmc_host_t {
            flags: _HOST_FLAG_SPI | _HOST_FLAG_DEINIT_ARG,
            slot: device.get_host() as i32,
            max_freq_khz: _DEFAULT_FREQUENCY,
            io_voltage: 3.3,
            init: Some(sdspi_host_init),
            set_bus_width: None,
            get_bus_width: None,
            set_bus_ddr_mode: None,
            set_card_clk: Some(sdspi_host_set_card_clk),
            set_cclk_always_on: None,
            do_transaction: Some(sdspi_host_do_transaction),
            __bindgen_anon_1: sdmmc_host_t__bindgen_ty_1 {
                deinit_p: Some(sdspi_host_remove_device),
            },
            io_int_enable: Some(sdspi_host_io_int_enable),
            io_int_wait: Some(sdspi_host_io_int_wait),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            )))]    // For ESP-IDF v5.1 and later
            get_real_freq: Some(sdspi_host_get_real_freq),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
            )))]    // For ESP-IDF v5.2 and later
            input_delay_phase: sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
            )))]   // For ESP-IDF v5.2 and later
            set_input_delay: None,
            command_timeout_ms: 0,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            dma_aligned_buffer: core::ptr::null_mut(),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            get_dma_info: None,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            pwr_ctrl_handle: core::ptr::null_mut() as _,
        };

        Self {
            device: SdDevice::Spi(device),
            host,
        }
        .set_from_configuration(configuration)
    }

    /// Create a new SD/MMC host with the default configuration.
    /// This host will use the MMC slot 1, with 4-bit mode enabled, and max frequency set to 20MHz
    #[cfg(esp_idf_soc_sdmmc_host_supported)]
    pub fn new_with_mmc(
        configuration: &Configuration,
        mmc_configuration: SlotConfiguration<'d>,
    ) -> Self {
        let host = sdmmc_host_t {
            flags: _SDMMC_HOST_FLAG_8BIT
                | _SDMMC_HOST_FLAG_4BIT
                | _SDMMC_HOST_FLAG_1BIT
                | _SDMMC_HOST_FLAG_DDR,
            slot: mmc_configuration.get_slot() as i32,
            max_freq_khz: _DEFAULT_FREQUENCY,
            io_voltage: _DEFAULT_IO_VOLTAGE,
            init: Some(sdmmc_host_init),
            set_bus_width: Some(sdmmc_host_set_bus_width),
            get_bus_width: Some(sdmmc_host_get_slot_width),
            set_bus_ddr_mode: Some(sdmmc_host_set_bus_ddr_mode),
            set_card_clk: Some(sdmmc_host_set_card_clk),
            set_cclk_always_on: Some(sdmmc_host_set_cclk_always_on),
            do_transaction: Some(sdmmc_host_do_transaction),
            __bindgen_anon_1: sdmmc_host_t__bindgen_ty_1 {
                deinit: Some(sdmmc_host_deinit),
            },
            io_int_enable: Some(sdmmc_host_io_int_enable),
            io_int_wait: Some(sdmmc_host_io_int_wait),
            get_real_freq: Some(sdmmc_host_get_real_freq),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
             )))] // For ESP-IDF v5.2 and later            
            input_delay_phase: sdmmc_delay_phase_t_SDMMC_DELAY_PHASE_0,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
             )))] // For ESP-IDF v5.2 and later            
            set_input_delay: Some(sdmmc_host_set_input_delay),
            command_timeout_ms: 0,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            dma_aligned_buffer: core::ptr::null_mut(),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            get_dma_info: None,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "2"),
            )))]   // For ESP-IDF v5.3 and later
            pwr_ctrl_handle: core::ptr::null_mut() as _,
        };

        Self {
            device: SdDevice::Mmc(mmc_configuration),
            host,
        }
        .set_from_configuration(configuration)
    }

    pub(crate) fn get_inner_handle(&self) -> &sdmmc_host_t {
        &self.host
    }

    pub(crate) fn get_device(&self) -> &SdDevice {
        &self.device
    }
}
