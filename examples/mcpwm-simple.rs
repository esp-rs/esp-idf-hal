/// Simple example showing how MCPWM may be used to generate two synchronized pwm signals with varying duty.
/// The duty on the pin4 will increase from 0% all the way up to 100% and repeat. At the same time the duty
/// on pin 5 will go from 100% down to 0%
///
/// # duty = 10%
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
/// # duty = 50%
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
/// # duty = 90%
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

#[cfg(all(any(esp32, esp32s3), esp_idf_version_major = "5"))]
fn main() -> anyhow::Result<()> {
    use embedded_hal::delay::DelayUs;

    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::mcpwm::{OperatorConfig, TimerConfig, TimerDriver};
    use esp_idf_hal::prelude::Peripherals;

    esp_idf_sys::link_patches();

    println!("Configuring MCPWM");

    let peripherals = Peripherals::take().unwrap();
    let timer_config = TimerConfig::default().period_ticks(8_000); // 10kHz
    let operator_config = OperatorConfig::default(peripherals.pins.gpio4, peripherals.pins.gpio5);
    let timer =
        TimerDriver::new(peripherals.mcpwm0.timer0, timer_config).expect("Failed to set up timer");

    let mut timer = timer
        .into_connection()
        .attach_operator0(peripherals.mcpwm0.operator0, operator_config)
        .expect("Failed to set up operator and timer connection");

    // Borrow references to the contained timer and operator
    let (timer, operator, _, _) = timer.split();

    println!("Starting duty-cycle loop");

    let period_ticks = timer.get_period_peak();

    // TODO: Will this work as expected in UP_DOWN counter mode?
    for duty in (0..period_ticks).step_by(10).cycle() {
        if duty % 100 == 0 {
            println!(
                "cmp: {}, duty {}%",
                duty,
                100 * u32::from(duty) / u32::from(period_ticks)
            );
        }

        operator.set_compare_value_x(duty)?; // In this configuration this controls the duty on pin4
        operator.set_compare_value_y(period_ticks - duty)?; // and this controls pin5
        FreeRtos.delay_ms(10)?;
    }

    unreachable!()
}

#[cfg(not(all(any(esp32, esp32s3), esp_idf_version_major = "5")))]
fn main() {
    esp_idf_sys::link_patches();

    println!("Sorry MCPWM is only supported on ESP32 and ESP32-S3");
}
