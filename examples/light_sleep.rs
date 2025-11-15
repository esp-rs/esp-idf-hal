//! Light sleep example
//!
//! Enable multiple light sleep wakeup sources and do sleep in a loop.
//! Print wakeup reason and sleep time on wakeup.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

use core::time::Duration;

use std::thread;
use std::time::Instant;

use esp_idf_hal::gpio::{AnyIOPin, Level, PinDriver, Pull};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::sleep::*;
use esp_idf_hal::uart::config::Config;
use esp_idf_hal::uart::UartDriver;
use esp_idf_hal::units::Hertz;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let peripherals = Peripherals::take()?;

    // Sample GPIO pins

    let gpio0 = PinDriver::input(peripherals.pins.gpio5, Pull::Down)?;
    let gpio1 = PinDriver::input(peripherals.pins.gpio6, Pull::Down)?;

    // Sample RTC pins

    #[cfg(not(any(esp32c2, esp32c3)))]
    let rtc0 = PinDriver::rtc_input(peripherals.pins.gpio2, Pull::Down)?;
    #[cfg(not(any(esp32c2, esp32c3)))]
    let rtc1 = PinDriver::rtc_input(peripherals.pins.gpio4, Pull::Down)?;

    // Sample UART driver

    let uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio7,
        peripherals.pins.gpio8,
        None::<AnyIOPin>,
        None::<AnyIOPin>,
        &Config::new().baudrate(Hertz(115_200)),
    )?;

    // Assemble the wakeup sources now

    let wakeup = Empty
        // Add GPIO wakeup
        .chain(PinsWakeup::<_, Gpio>::new(
            Empty
                .chain((&gpio0, Level::High))
                .chain((&gpio1, Level::High)),
        ))
        // Add UART wakeup
        .chain(UartWakeup::new(&uart, 3))
        // Add timer wakeup
        .chain(TimerWakeup::new(Duration::from_secs(5)));

    #[cfg(not(esp32c3))]
    let wakeup = wakeup
        // Add RTC wakeup
        .chain(PinsWakeup::<_, Rtc>::new(
            Empty
                .chain((&rtc0, Level::High))
                .chain((&rtc1, Level::High)),
        ));

    loop {
        println!("Light sleep with wakeup: {wakeup:?}");

        // short sleep to flush stdout
        thread::sleep(Duration::from_millis(60));

        let time_before = Instant::now();

        match light_sleep(&wakeup) {
            Ok(_) => {
                let time_after = Instant::now();

                let wakeup_reason = WakeupReason::get();
                println!(
                    "wake up from light sleep due to {wakeup_reason:?} which lasted for {:?}",
                    time_after - time_before
                );
            }
            Err(e) => {
                println!("failed to do sleep: {e:?}");
                thread::sleep(Duration::from_secs(1));
            }
        }

        println!("---");
    }
}
