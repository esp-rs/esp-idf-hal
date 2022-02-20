//! Turn an LED on/off depending on the state of a button
//!
//! This assumes that a LED is connected to GPIO4.
//! Additionally this assumes a button connected to GPIO9.
//! On an ESP32C3 development board this is the BOOT button.
//!
//! Depending on your target and the board you are using you should change the pins.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.

use std::thread;
use std::time::Duration;

use embedded_hal::digital::blocking::InputPin;
use embedded_hal::digital::blocking::OutputPin;
use esp_idf_hal::gpio::Pull;
use esp_idf_hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut led = peripherals.pins.gpio4.into_output()?;
    let mut button = peripherals.pins.gpio9.into_input()?;
    button.set_pull_down()?;

    loop {
        // we are using thread::sleep here to make sure the watchdog isn't triggered
        thread::sleep(Duration::from_millis(10));

        if button.is_high()? {
            led.set_low()?;
        } else {
            led.set_high()?;
        }
    }
}
