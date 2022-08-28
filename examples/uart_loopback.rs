//! UART loopback test
//!
//! Folowing pins are used:
//! TX    GPIO5
//! RX    GPIO6
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! This example transfers data via UART.
//! Connect TX and RX pins to see the outgoing data is read as incoming data.

use std::thread;
use std::time::Duration;

use embedded_hal::serial::nb::{Read, Write};

use esp_idf_hal::gpio;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let tx = peripherals.pins.gpio5;
    let rx = peripherals.pins.gpio6;

    println!("Starting UART loopback test");
    let config = config::Config::new().baudrate(Hertz(115_200));
    let mut uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
    .unwrap();

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(500));
        nb::block!(uart.write(0xaa))?;

        // note: this will block - if you don't connect RX and TX you will see the watchdog kick in
        let byte = nb::block!(uart.read())?;
        println!("Written 0xaa, read 0x{:02x}", byte);
    }
}
