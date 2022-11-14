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
use esp_idf_hal::spi::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio6;
    let serial_in = peripherals.pins.gpio2; // SDI
    let serial_out = peripherals.pins.gpio7; // SDO
    let cs_1 = peripherals.pins.gpio10;
    let cs_2 = peripherals.pins.gpio3;

    println!("Starting SPI loopback test");

    let driver = SpiDriver::new::<SPI2>(spi, sclk, serial_out, Some(serial_in), Dma::Disabled)?;

    let config_1 = config::Config::new().baudrate(26.MHz().into());
    let mut device_1 = SpiDeviceDriver::new(&driver, Some(cs_1), &config_1)?;

    let config_2 = config::Config::new().baudrate(13.MHz().into());
    let mut device_2 = SpiDeviceDriver::new(&driver, Some(cs_2), &config_2)?;

    let mut read = [0u8; 4];
    let write = [0xde, 0xad, 0xbe, 0xef];

    let mut in_place_buf = [0xde, 0xad, 0xbe, 0xef];

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        FreeRtos::delay_ms(500);
        device_1.transfer(&mut read, &write)?;
        println!("Device 1: Wrote {:x?}, read {:x?}", write, read);

        println!("Device 2: To write {:x?} ... ", in_place_buf);
        device_2.transaction(|bus| bus.transfer_in_place(&mut in_place_buf))?;
        println!("... read {:x?}", in_place_buf);
    }
}
