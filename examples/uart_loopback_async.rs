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

use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::task::executor::*;
use esp_idf_hal::uart::*;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let tx = peripherals.pins.gpio5;
    let rx = peripherals.pins.gpio6;

    println!("Starting UART loopback test");
    let config = config::Config::new().baudrate(Hertz(115_200));
    let uart = AsyncUartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
    .unwrap();

    EspBlocker::new().block_on(async {
        loop {
            uart.write(&[0xaa]).await?;

            let mut buf = [0_u8; 1];
            uart.read(&mut buf).await?;

            println!("Written 0xaa, read 0x{:02x}", buf[0]);
        }
    })
}
