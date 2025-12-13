//! Tests deep sleep
//!
//! Enables multiple deep sleep wakeup sources and then enter deep sleep.
//! There is no loop here, since the program will not continue after deep sleep,
//! it always starts from the beginning after a deep sleep wake-up.
//! The program starts by printing reset and wakeup reason, since the deep
//! sleep effectively ends the program, this is how we get information about
//! the previous run.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use core::time::Duration;

#[cfg(any(esp32c2, esp32c3))]
use esp_idf_hal::gpio::Level;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::gpio::{PinDriver, Pull};
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::reset::{ResetReason, WakeupReason};
use esp_idf_hal::sleep::*;

fn print_wakeup_result() {
    let reset_reason = ResetReason::get();
    let wakeup_reason = WakeupReason::get();
    println!(
        "reset after {:?} wakeup due to {:?}",
        reset_reason, wakeup_reason
    );
}

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    print_wakeup_result();

    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    let peripherals = Peripherals::take().unwrap();

    // 1. Initialize Deep Sleep Driver (resets previous config)
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    let mut sleep = DeepSleep::new()?;

    // 2. Configure Timer Wakeup (All chips)
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    {
        sleep = sleep.wakeup_on_timer(Duration::from_secs(5))?;
    }

    // 3. Configure RTC Wakeup (ESP32/S2/S3)
    // Uses the chaining mechanism to wake up on ANY HIGH from a set of pins.
    #[cfg(any(esp32, esp32s2, esp32s3))]
    {
        // Define pins based on chip variant
        #[cfg(esp32)]
        let (pin0, pin1) = (peripherals.pins.gpio26, peripherals.pins.gpio27);
        #[cfg(any(esp32s2, esp32s3))]
        let (pin0, pin1) = (peripherals.pins.gpio1, peripherals.pins.gpio2);

        let rtc0 = PinDriver::rtc_input(pin0, Pull::Down)?;
        let rtc1 = PinDriver::rtc_input(pin1, Pull::Down)?;

        // Chain the pins together using the new API trait
        let pins = rtc0.chain(&rtc1);

        // Apply configuration immediately
        sleep = sleep.wakeup_on_rtc(&pins, RtcWakeLevel::AnyHigh)?;
    }

    // 4. Configure GPIO Wakeup (C2/C3 only)
    // C-series chips configure Deep Sleep GPIOs individually.
    #[cfg(any(esp32c2, esp32c3))]
    {
        let gpio0 = PinDriver::input(peripherals.pins.gpio0, Pull::Down)?;
        let gpio1 = PinDriver::input(peripherals.pins.gpio1, Pull::Down)?;

        sleep = sleep
            .wakeup_on_gpio(&gpio0, Level::High)?
            .wakeup_on_gpio(&gpio1, Level::Low)?;
    }

    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    println!("Entering deep sleep...");

    // 5. Enter Sleep
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
    sleep.enter();

    #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3)))]
    Ok(())
}