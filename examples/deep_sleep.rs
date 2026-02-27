//! Tests deep sleep
//!
//! Enables multiple deep sleep wakeup sources and then enter deep sleep.
//! There is no loop here, since the program will not continue after deep sleep,
//! it always starts from the beginning after a deep sleep wake-up.
//! For ESP32c3, only timer wakeup is supported.
//! The program starts by printing reset and wakeup reason, since the deep
//! sleep effectively ends the program, this is how we get information about
//! the previous run.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

use core::time::Duration;

use esp_idf_hal::gpio::{Level, PinDriver, Pull};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::reset::{ResetReason, WakeupReason};
use esp_idf_hal::sleep::*;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let reset_reason = ResetReason::get();
    let wakeup_reason = WakeupReason::get();
    println!("Reset after {reset_reason:?}; wakeup due to {wakeup_reason:?}");

    let peripherals = Peripherals::take()?;

    // Sample GPIO pins
    // GPIO pins can only be used in deep sleep on the esp32c2 and esp32c3 where there are no RTC pins
    //
    // Note also, that ONLY pins connected to the VDD3P3_RTC power domain can be used for deep-sleep wakeup,
    // which is not enforced in the deep-sleep API.

    #[cfg(any(esp32c2, esp32c3))]
    let gpio0 = PinDriver::input(peripherals.pins.gpio5, Pull::Down)?;
    #[cfg(any(esp32c2, esp32c3))]
    let gpio1 = PinDriver::input(peripherals.pins.gpio6, Pull::Down)?;

    // Sample RTC pins
    // No RTC on esp32c2 and esp32c3

    #[cfg(not(any(esp32c2, esp32c3)))]
    let rtc0 = PinDriver::rtc_input(peripherals.pins.gpio2, Pull::Down)?;
    #[cfg(not(any(esp32c2, esp32c3)))]
    let rtc1 = PinDriver::rtc_input(peripherals.pins.gpio4, Pull::Down)?;

    // Assemble the wakeup sources now

    let wakeups: &[&dyn DeepSleepWakeup] = &[
        // Add timer wakeup
        &TimerWakeup::new(Duration::from_secs(5)),
        #[cfg(any(esp32c2, esp32c3))]
        &PinsWakeup::<_, Gpio>::new([(&gpio0, Level::High), (&gpio1, Level::High)]),
        #[cfg(not(any(esp32c2, esp32c3)))]
        &PinsWakeup::<_, Rtc>::new([(&rtc0, Level::High), (&rtc1, Level::High)]),
    ];

    // println!("Deep sleep with wakeup: {wakeup:?}");

    let e = deep_sleep(wakeups);

    println!("Failed to enter deep sleep: {e:?}");

    Ok(())
}
