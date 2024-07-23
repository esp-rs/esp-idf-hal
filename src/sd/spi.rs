use core::borrow::Borrow;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::gpio::{InputPin, OutputPin};
use crate::peripheral::Peripheral;
use crate::spi::SpiDriver;
use crate::sys::*;

static USED: AtomicU8 = AtomicU8::new(0);
static USED_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

/// SPI Host driver for SD cards supporting the SPI protocol.
pub struct SdSpiHostDriver<'d, T> {
    _spi_driver: T,
    handle: sdspi_dev_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d, T> SdSpiHostDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>>,
{
    /// Create a new SPI host driver for SD cards
    ///
    /// # Arguments
    /// - spi_driver: SPI peripheral driver
    /// - cs: Chip Select pin (optional)
    /// - cd: Card Detect pin (optional)
    /// - wp: Write Protect pin (optional)
    /// - int: Interrupt pin (optional)
    /// - wp_active_high: Write Protect active when high (optional, default = `false`)
    pub fn new(
        spi_driver: T,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        int: Option<impl Peripheral<P = impl InputPin> + 'd>,
        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
        )))] // For ESP-IDF v5.2 and later
        wp_active_high: Option<bool>,
    ) -> Result<Self, EspError>
    where
        T: Borrow<SpiDriver<'d>>,
    {
        let dev_config = sdspi_device_config_t {
            host_id: spi_driver.borrow().host(),
            gpio_cs: cs.map(|cd| cd.into_ref().deref().pin()).unwrap_or(-1),
            gpio_cd: cd.map(|cd| cd.into_ref().deref().pin()).unwrap_or(-1),
            gpio_wp: wp.map(|wp| wp.into_ref().deref().pin()).unwrap_or(-1),
            gpio_int: int.map(|int| int.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
            )))] // For ESP-IDF v5.2 and later
            gpio_wp_polarity: wp_active_high.unwrap_or(false), // `false` = active when low
        };

        {
            let _cs = USED_CS.enter();

            if USED.load(Ordering::SeqCst) == 0 {
                esp!(unsafe { sdspi_host_init() })?;

                USED.fetch_add(1, Ordering::SeqCst);
            }
        }

        let mut handle = 0;

        esp!(unsafe { sdspi_host_init_device(&dev_config, &mut handle) })?;

        Ok(Self {
            _spi_driver: spi_driver,
            handle,
            _p: PhantomData,
        })
    }

    pub fn handle(&self) -> sdspi_dev_handle_t {
        self.handle
    }
}

impl<'d, T> Drop for SdSpiHostDriver<'d, T> {
    fn drop(&mut self) {
        esp!(unsafe { sdspi_host_remove_device(self.handle) }).unwrap();

        {
            let _cs = USED_CS.enter();

            if USED.fetch_sub(1, Ordering::SeqCst) == 1 {
                esp!(unsafe { sdspi_host_deinit() }).unwrap();
            }
        }
    }
}
