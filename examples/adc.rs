//! ADC example 
//!

use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use std::thread;
use std::time::Duration;

use embedded_hal_0_2::adc::OneShot;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::adc::*;
use esp_idf_hal::adc::config::Config;

fn main() -> anyhow::Result<()> {

    let peripherals = Peripherals::take().unwrap();

    let mut adc1 = PoweredAdc::new(
        peripherals.adc1,
        Config::new().calibration(true),
    )?;

    // configuring pin0 to analog read, you can regulate the adc input voltage range depending on your need
    // for this example we use the attenuation of 11db which sets the input voltage range to around 0-3.6V
    let mut adc_pin = peripherals.pins.gpio0.into_analog_atten_11db()?;

    loop {
        // you can change the sleep duration depending on how often you want to sample
        thread::sleep(Duration::from_millis(10));
        println!("ADC value: {}",adc1.read(&mut adc_pin).unwrap());
    }
}
