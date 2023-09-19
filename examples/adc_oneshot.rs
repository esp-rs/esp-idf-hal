//! ADC oneshot example, reading a value form a pin and printing it on the terminal
//! requires ESP-IDF v5.0 or newer

use std::thread;
use std::time::Duration;

#[cfg(not(esp_idf_version_major = "4"))]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::adc::attenuation::DB_11;
    use esp_idf_hal::adc::oneshot::config::AdcChannelConfig;
    use esp_idf_hal::adc::oneshot::*;
    use esp_idf_hal::peripherals::Peripherals;

    let peripherals = Peripherals::take().unwrap();

    #[cfg(not(esp32))]
    let adc = AdcDriver::new(peripherals.adc1)?;

    #[cfg(esp32)]
    let adc = AdcDriver::new(peripherals.adc2)?;

    // configuring pin to analog read, you can regulate the adc input voltage range depending on your need
    // for this example we use the attenuation of 11db which sets the input voltage range to around 0-3.6V
    let config = AdcChannelConfig {
        attenuation: DB_11,
        calibration: true,
        ..Default::default()
    };

    #[cfg(not(esp32))]
    let mut adc_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio2, &config)?;

    #[cfg(esp32)]
    let mut adc_pin = AdcChannelDriver::new(&adc, peripherals.pins.gpio12, &config)?;

    loop {
        // you can change the sleep duration depending on how often you want to sample
        thread::sleep(Duration::from_millis(100));
        println!("ADC value: {}", adc.read(&mut adc_pin).unwrap());
    }
}

#[cfg(esp_idf_version_major = "4")]
fn main() -> anyhow::Result<()> {
    println!("This example requires ESP-IDF v5.0 or newer");

    loop {
        thread::sleep(Duration::from_millis(1000));
    }
}
