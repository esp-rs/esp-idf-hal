//! Tests deep sleep
//!
//! Enables multiple deep sleep wakeup sources and then enter deep sleep.
//! There is no loop here, since the program will not continue after deep sleep.
//! For ESP32c3, only timer wakeup is supported.
//! The program starts by printing reset and wakeup reason, since the deep
//! sleep effectively ends the program, this is how we get information about
//! the previous run.

use core::time::Duration;
#[cfg(any(esp32, esp32s2, esp32s3))]
use esp_idf_hal::gpio::PinDriver;
#[cfg(any(esp32, esp32s2, esp32s3))]
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
    esp_idf_sys::link_patches();

    print_wakeup_result();

    #[cfg(any(esp32, esp32s2, esp32s3))]
    let peripherals = Peripherals::take().unwrap();

    // RTC wakeup definitions
    #[cfg(esp32)]
    let rtc0 = PinDriver::rtc_input(peripherals.pins.gpio26)?;
    #[cfg(esp32)]
    let rtc1 = PinDriver::rtc_input(peripherals.pins.gpio27)?;

    #[cfg(any(esp32s2, esp32s3))]
    let rtc0 = PinDriver::rtc_input(peripherals.pins.gpio1)?;
    #[cfg(any(esp32s2, esp32s3))]
    let rtc1 = PinDriver::rtc_input(peripherals.pins.gpio2)?;

    #[cfg(any(esp32, esp32s2, esp32s3))]
    let rtc_pin0 = RtcWakeupPin { pindriver: &rtc0 };
    #[cfg(any(esp32, esp32s2, esp32s3))]
    let rtc_pin1 = RtcWakeupPin { pindriver: &rtc1 };
    #[cfg(any(esp32, esp32s2, esp32s3))]
    let rtc_wakeup = Some(RtcWakeup {
        pins: EmptyRtcWakeupPins::chain(rtc_pin0).chain(rtc_pin1),
        wake_level: RtcWakeLevel::AnyHigh,
    });

    #[cfg(any(esp32, esp32s2, esp32s3))]
    let dsleep = make_deep_sleep_rtc_pins(
        Some(TimerWakeup::new(Duration::from_secs(5))),
        rtc_wakeup,
        None,
        None,
    );

    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    let dsleep = make_deep_sleep_no_pins(Some(TimerWakeup::new(Duration::from_secs(5))));

    println!("Deep sleep with: {:?}", dsleep);
    dsleep.prepare()?;
    dsleep.sleep();
}
