use crate::sys::*;

use core::{marker::PhantomData, ops::Deref};

use esp_idf_hal::{
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
    spi::SpiDriver,
};

pub struct SpiDevice<'d> {
    configuration: sdspi_device_config_t,
    driver: SpiDriver<'d>,
    _p: PhantomData<&'d mut ()>,
}

impl Drop for SpiDevice<'_> {
    fn drop(&mut self) {
        let result = unsafe { spi_bus_free(self.get_host()) };

        if result != ESP_OK {
            panic!("Failed to remove SPI device");
        }
    }
}

impl<'d> SpiDevice<'d> {
    pub fn new(
        driver: SpiDriver<'d>,
        cs: impl Peripheral<P = impl OutputPin> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        int: Option<impl Peripheral<P = impl InputPin> + 'd>,
        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
        )))] // For ESP-IDF v5.2 and later
        wp_polarity: Option<bool>,
    ) -> Self {
        let configuration = sdspi_device_config_t {
            host_id: driver.host(),
            gpio_cs: cs.into_ref().deref().pin(),
            gpio_cd: cd.map(|cd| cd.into_ref().deref().pin()).unwrap_or(-1),
            gpio_wp: wp.map(|wp| wp.into_ref().deref().pin()).unwrap_or(-1),
            gpio_int: int.map(|int| int.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
            )))] // For ESP-IDF v5.2 and later
            gpio_wp_polarity: wp_polarity.unwrap_or(false), // Active when low
        };

        Self {
            configuration,
            driver,
            _p: PhantomData,
        }
    }

    pub fn get_host(&self) -> spi_host_device_t {
        self.driver.host()
    }

    pub fn set_clock(&mut self, clock: u32) -> Result<(), EspError> {
        esp!(unsafe { sdspi_host_set_card_clk(self.get_host() as i32, clock) })
    }

    pub fn get_clock(&mut self) -> Result<u32, EspError> {
        unimplemented!() // ! : Function not found in `esp-idf-sys`
    }

    pub fn enable_interrupt(&mut self) -> Result<(), EspError> {
        esp!(unsafe { sdspi_host_io_int_enable(self.get_host() as i32) })
    }

    pub fn wait_interrupt(&mut self, timeout: u32) -> Result<(), EspError> {
        esp!(unsafe { sdspi_host_io_int_wait(self.get_host() as i32, timeout) })
    }

    pub(crate) fn get_device_configuration(&self) -> &sdspi_device_config_t {
        &self.configuration
    }
}
