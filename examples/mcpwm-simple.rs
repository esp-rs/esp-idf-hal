/// Simple example showing how MCPWM may be used to generate two synchronized pwm signals with varying duty.
/// The duty on the pin4 will increase from 0% all the way up to 100% and repeat. At the same time the duty
/// on pin 5 will go from 100% down to 0%
///
/// # duty = 10
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
/// # duty = 50
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
/// # duty = 90
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
    let timer_config = TimerConfig::default().frequency(25.kHz());
    let operator_config = OperatorConfig::default();
    let timer = Mcpwm::new(peripherals.mcpwm0.timer, timer_config)?;

    let timer = timer.into_connection()
        .attatch_operator0(
            peripherals.mcpwm0.operator0,
            operator_config,
            peripherals.pins.gpio4,
            peripherals.pins.gpio5,
        )?;

    let (timer, operator, _, _) = timer.split();

    println!("Starting duty-cycle loop");

    for duty in (0..timer.get_top_value()).cycle() {
        if duty % 100 == 0 {
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
