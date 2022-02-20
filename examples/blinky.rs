//! Blinks an LED
//!
//! This assumes that a LED is connected to GPIO4.
//! Depending on your target and the board you are using you should change the pin.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.
//!

use std::thread;
use std::time::Duration;

use embedded_hal::digital::blocking::OutputPin;
use esp_idf_hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut led = peripherals.pins.gpio4.into_output()?;

    loop {
        led.set_high()?;
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(1000));

        led.set_low()?;
        thread::sleep(Duration::from_millis(1000));
    }
}
