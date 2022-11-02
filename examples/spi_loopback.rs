//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK   GPIO6
//! SDI    GPIO2
//! SDO    GPIO7
//! CS     GPIO10
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! This example transfers data via SPI.
//! Connect SDI and SDO pins to see the outgoing data is read as incoming data.

use embedded_hal::spi::SpiDevice;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio6;
    let serial_in = peripherals.pins.gpio2; // SDI
    let serial_out = peripherals.pins.gpio7; // SDO
    let cs = peripherals.pins.gpio10;

    println!("Starting SPI loopback test");
    let config = config::Config::new().baudrate(26.MHz().into());
    let mut spi =
        SpiMasterDriver::new::<SPI2>(spi, sclk, serial_out, Some(serial_in), Some(cs), &config)?;

    let mut read = [0u8; 4];
    let write = [0xde, 0xad, 0xbe, 0xef];

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        FreeRtos::delay_ms(500);
        spi.transfer(&mut read, &write)?;
        println!("Wrote {:x?}, read {:x?}", write, read);
    }
}
