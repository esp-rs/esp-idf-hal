//! ADC example, reading a value form a pin and printing it on the terminal
//!

use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use esp_idf_hal::adc::config::Config;
use esp_idf_hal::adc::*;
use esp_idf_hal::peripherals::Peripherals;

fn main() -> anyhow::Result<()> {
    let peripherals = Peripherals::take().unwrap();

    #[cfg(not(esp32))]
    let mut adc = AdcDriver::new(peripherals.adc1, &Config::new().calibration(true))?;

    #[cfg(esp32)]
    let mut adc = AdcDriver::new(peripherals.adc2, &Config::new().calibration(true))?;

    // configuring pin to analog read, you can regulate the adc input voltage range depending on your need
    // for this example we use the attenuation of 11db which sets the input voltage range to around 0-3.6V
    #[cfg(not(esp32))]
    let mut adc_pin: esp_idf_hal::adc::AdcChannelDriver<{ attenuation::DB_11 }, _> =
        AdcChannelDriver::new(peripherals.pins.gpio4)?;

    #[cfg(esp32)]
    let mut adc_pin: esp_idf_hal::adc::AdcChannelDriver<{ attenuation::DB_11 }, _> =
        AdcChannelDriver::new(peripherals.pins.gpio12)?;

    loop {
        // you can change the sleep duration depending on how often you want to sample
        thread::sleep(Duration::from_millis(10));
        println!("ADC value: {}", adc.read(&mut adc_pin).unwrap());
    }
}
