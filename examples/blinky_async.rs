//! Blinks an LED
//!
//! This assumes that a LED is connected to GPIO4.
//! Depending on your target and the board you are using you should change the pin.
//! If your board doesn't have on-board LEDs don't forget to add an appropriate resistor.
//!

use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task::*;
use esp_idf_hal::timer::*;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;

    let mut led = PinDriver::output(peripherals.pins.gpio4)?;
    let mut timer = TimerDriver::new(peripherals.timer00, &TimerConfig::new())?;

    block_on(async {
        loop {
            led.set_high()?;

            timer.delay(timer.tick_hz()).await?;

            led.set_low()?;

            timer.delay(timer.tick_hz()).await?;
        }
    })
}
