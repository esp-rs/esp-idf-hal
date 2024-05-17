#[cfg(esp32)]
fn main() -> anyhow::Result<()> {
    use esp_idf_svc::{
        fs::{Fat, FatConfiguration},
        hal::{
            gpio,
            prelude::*,
            spi::{config::DriverConfig, Dma, SpiDriver},
        },
        log::EspLogger,
        sd::{host::SdHost, spi::SpiDevice, SdConfiguration},
    };
    use std::{fs::File, io::Write};

    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;

    let spi_driver = SpiDriver::new(
        peripherals.spi3,
        pins.gpio23,
        pins.gpio19,
        Some(pins.gpio18),
        &DriverConfig::default().dma(Dma::Auto(0)),
    )?;

    let spi_device = SpiDevice::new(
        spi_driver,
        pins.gpio13,
        Option::<gpio::AnyInputPin>::None,
        Option::<gpio::AnyInputPin>::None,
        Option::<gpio::AnyInputPin>::None,
        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(esp_idf_version_major = "5", esp_idf_version_minor = "1"),
        )))] // For ESP-IDF v5.2 and later
        Option::<bool>::None,
    );

    let host_config = SdConfiguration::new();

    let host = SdHost::new_with_spi(&host_config, spi_device);

    let fat_configuration = FatConfiguration::new();

    let _fat = Fat::mount(fat_configuration, host, "/")?;

    let mut file = File::create("/test.txt")?;

    file.write_all(b"Hello, world!")?;

    Ok(())
}

#[cfg(not(esp32))]
fn main() {
    use esp_idf_svc::{self as _};

    panic!("This example is configured for esp32, please adjust pins to your module");
}
