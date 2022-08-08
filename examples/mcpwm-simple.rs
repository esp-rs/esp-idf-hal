/// # x = 10
///
///               .                               .
///               .                               .
///               .------.                        .------.
///               |      |                        |      |
/// pin4          |      |                        |      |
///               |      |                        |      |
///           -----      --------------------------      --------------------------
///               .                               .
///               .                               .
///               .------------------------.      .------------------------.
///               |                        |      |                        |
/// pin5          |                        |      |                        |
///               |                        |      |                        |
///           -----                        --------                        --------
///               .                               .
///
///
/// # x = 50
///               .                               .
///               .                               .
///               .---------------.               .---------------.
///               |               |               |               |
/// pin4          |               |               |               |
///               |               |               |               |
///           -----               -----------------               -----------------
///               .                               .
///               .                               .
///               .---------------.               .---------------.
///               |               |               |               |
/// pin5          |               |               |               |
///               |               |               |               |
///           -----               -----------------               -----------------
///               .                               .
///
///
/// # x = 90
///               .                               .
///               .                               .
///               .------------------------.      .------------------------.
///               |                        |      |                        |
/// pin4          |                        |      |                        |
///               |                        |      |                        |
///           -----                        --------                        --------
///               .                               .
///               .                               .
///               .------.                        .------.
///               |      |                        |      |
/// pin5          |      |                        |      |
///               |      |                        |      |
///           -----      --------------------------      --------------------------
///               .                               .

#[cfg(any(esp32, esp32s3))]
fn main() -> anyhow::Result<()> {
    use embedded_hal::delay::blocking::DelayUs;

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::mcpwm::{Mcpwm, Operator, OperatorConfig};
    use esp_idf_hal::prelude::Peripherals;
    use esp_idf_hal::units::FromValueType;

    esp_idf_sys::link_patches();

    println!("Configuring MCPWM");

    let peripherals = Peripherals::take().unwrap();
    let config = OperatorConfig::default().frequency(25.kHz());
    let mcpwm = Mcpwm::new(peripherals.mcpwm0.mcpwm)?;
    let mut operator = Operator::new(
        peripherals.mcpwm0.operator0,
        &mcpwm,
        &config,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
    )?;

    println!("Starting duty-cycle loop");

    for x in (0..10000u16).cycle() {
        let duty = f32::from(x) * 0.01;

        if x % 100 == 0 {
            println!("Duty {}%", x / 100);
        }

        operator.set_duty_a(duty)?;
        operator.set_duty_b(100.0 - duty)?;
        FreeRtos.delay_ms(10)?;
    }

    unreachable!()
}

#[cfg(not(any(esp32, esp32s3)))]
fn main() {
    esp_idf_sys::link_patches();
}
