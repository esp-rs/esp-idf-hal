//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO6
//! MISO    GPIO2
//! MOSI    GPIO7
//! CS      GPIO10
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming data.

use std::thread;
use std::time::Duration;

use embedded_hal::spi::blocking::SpiDevice;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio6;
    let miso = peripherals.pins.gpio2;
    let mosi = peripherals.pins.gpio7;
    let cs = peripherals.pins.gpio10;

    println!("Starting SPI loopback test");
    let config = <spi::config::Config as Default>::default().baudrate(26.MHz().into());
    let mut spi = spi::Master::<spi::SPI2, _, _, _, _>::new(
        spi,
        spi::Pins {
            sclk,
            sdo: miso,
            sdi: Some(mosi),
            cs: Some(cs),
        },
        config,
    )?;

    let mut read = [0u8; 4];
    let write = [0xde, 0xad, 0xbe, 0xef];

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(500));
        spi.transfer(&mut read, &write)?;
        println!("Wrote {:x?}, read {:x?}", write, read);
    }
}
