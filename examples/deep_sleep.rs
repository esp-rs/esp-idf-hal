//! Tests deep sleep
//!
//! Enables multiple deep sleep wakeup sources and then enter deep sleep.
//! There is no loop here, since the program will not continue after deep sleep.
//! For ESP32c3, only timer wakeup is supported.
//! The program starts by printing reset and wakeup reason, since the deep
//! sleep effectively ends the program, this is how we get information about
//! the previous run.

use core::time::Duration;
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
    let rtc_pins = [
        RtcWakeupPin::new(peripherals.pins.gpio4.into(), false, true),
        RtcWakeupPin::new(peripherals.pins.gpio27.into(), false, true),
    ];
    #[cfg(any(esp32s2, esp32s3))]
    let rtc_pins = [
        RtcWakeupPin::new(peripherals.pins.gpio1.into(), false, true),
        RtcWakeupPin::new(peripherals.pins.gpio2.into(), false, true),
    ];
    #[cfg(any(esp32, esp32s2, esp32s3))]
    let rtc_wakeup = Some(RtcWakeup::new(&rtc_pins, RtcWakeLevel::AnyHigh));

    let dsleep = DeepSleep {
        timer: Some(TimerWakeup::new(Duration::from_secs(5))),
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rtc: rtc_wakeup,
        ..Default::default()
    };

    println!("Deep sleep with: {:?}", dsleep);
    dsleep.prepare()?;
    dsleep.sleep();
}
