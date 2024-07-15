//! RMT Onewire Example
//!
//! Example demonstrating the use of the onewire component.
//!
//! In order to use this example, an overidden `Cargo.toml` must be defined with the following definitions:
//! ```
//! [[package.metadata.esp-idf-sys.extra_components]]
//! remote_component = { name = "onewire_bus", version = "^1.0.2" }
//!
//!
//! [patch.crates-io]
//! esp-idf-sys = { git = "https://github.com/esp-rs/esp-idf-sys", rev = "2728b85" }
//!
//! ```
//!
//! The example can then be run with
//! `MCU=<target> cargo run --example rmt_onewire --manifest-path /path/to/other/Cargo.toml`
//!
//! Below is a connection sketch, the signal pin must be externally pulled-up
//! with a 4.7kOhm resistor.
//! This example uses gpio 16, but any pin capable of
//! input AND output is suitable.
//!
//! If the example is successful, it should print the address of each
//! onewire device attached to the bus.
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
//!
//!
//! This example demonstrates:
//! * A RMT device in both TX and RX mode.
//! * Usage of the onewire bus driver interface.
//! * How to iterate through a device search to discover devices on the bus.

use esp_idf_hal::delay::FreeRtos;
#[cfg(not(any(feature = "rmt-legacy", esp_idf_version_major = "4")))]
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
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
