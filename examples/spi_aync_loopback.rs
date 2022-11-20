//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK   GPIO6
//! SDI    GPIO2
//! SDO    GPIO7
//! CS_1   GPIO10
//! CS_2   GPIO3
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! This example transfers data via SPI.
//! Connect SDI and SDO pins to see the outgoing data is read as incoming data.

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::{config, Dma, SpiDriver, SPI2};
use esp_idf_hal::spi_async::SpiBusDriver;

fn main() -> anyhow::Result<()> {
    // TODO: Run a_main()
    Ok(())
}

async fn _main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio6;
    let serial_in = peripherals.pins.gpio2; // SDI
    let serial_out = peripherals.pins.gpio7; // SDO

    println!("Starting SPI loopback test");

    let mut driver = SpiDriver::new::<SPI2>(spi, sclk, serial_out, Some(serial_in), Dma::Disabled)?;

    let config = config::Config::new().baudrate(26.MHz().into());
    let mut bus = SpiBusDriver::new(&mut driver, &config)?;

    let mut read = [0u8; 4];
    let write = [0xde, 0xad, 0xbe, 0xef];

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        FreeRtos::delay_ms(500);
        bus.transfer(&mut read, &write).await?;
        println!("Bus: Wrote {:x?}, read {:x?}", write, read);
    }
}
