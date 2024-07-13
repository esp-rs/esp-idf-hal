//! Example demonstrating the use of the onewire component.
//!
//! Connection sketch, this example uses gpio 16, but any pin capable of
//! input AND output is suitable.
//!
//! ┌──────────────────────────┐
//! │                      3.3V├───────┬─────────────┬──────────────────────┐
//! │                          │      ┌┴┐            │VDD                   │VDD
//! │          ESP Board       │  4.7k│ │     ┌──────┴──────┐        ┌──────┴──────┐
//! │                          │      └┬┘   DQ│             │      DQ│             │
//! │          ONEWIRE_GPIO_PIN├───────┴──┬───┤   DS18B20   │    ┌───┤   DS18B20   │   ......
//! │                          │          └───│-------------│────┴───│-------------│──
//! │                          │              └──────┬──────┘        └──────┬──────┘
//! │                          │                     │GND                   │GND
//! │                       GND├─────────────────────┴──────────────────────┘
//! └──────────────────────────┘

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::onewire::{DeviceSearch, OneWireBusDriver};
use esp_idf_hal::peripherals::Peripherals;

#[cfg(not(any(feature = "rmt-legacy", esp_idf_version_major = "4")))]
fn main() -> anyhow::Result<()> {
    println!("Starting APP!");

    let peripherals = Peripherals::take()?;

    let onewire_gpio_pin = peripherals.pins.gpio16;

    let mut rmt_onewire: OneWireBusDriver = OneWireBusDriver::new(onewire_gpio_pin)?;
    let search: DeviceSearch = rmt_onewire.search()?;

    for device in search {
        println!("Found Device: {}", device.address());
    }
    loop {
        FreeRtos::delay_ms(3000);
    }
}

#[cfg(any(feature = "rmt-legacy", esp_idf_version_major = "4"))]
fn main() -> anyhow::Result<()> {
    println!("This example requires feature `rmt-legacy` disabled or using ESP-IDF > v4.4.X");

    loop {
        thread::sleep(Duration::from_millis(1000));
    }
}
