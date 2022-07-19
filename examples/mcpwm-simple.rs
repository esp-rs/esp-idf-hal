use embedded_hal::delay::blocking::DelayUs;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::mcpwm::{Mcpwm, Operator, OperatorConfig};
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::units::FromValueType;

#[cfg(any(esp32, esp32s3))]
fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    println!("Configuring MCPWM");

    let peripherals = Peripherals::take().unwrap();
    let config = OperatorConfig::default().frequency(25.kHz().into());
    let mcpwm = Mcpwm::new(peripherals.mcpwm0.mcpwm)?;
    let mut operator = Operator::new(
        peripherals.mcpwm0.operator0,
        &mcpwm,
        &config,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
    )?;

    println!("Starting duty-cycle loop");

    for &duty in [0.0, 20.0, 40.0, 60.0, 80.0, 100.0].iter().cycle() {
        println!("Duty {}%", duty);
        operator.set_duty_a(duty)?;
        operator.set_duty_b(100.0 - duty)?;
        FreeRtos.delay_ms(2000)?;
    }

    loop {
        FreeRtos.delay_ms(1000)?;
    }
}

#[cfg(not(any(esp32, esp32s3)))]
fn main() {}
