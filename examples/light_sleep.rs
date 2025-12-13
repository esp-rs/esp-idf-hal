//! Tests light sleep
//!
//! Enables multiple light sleep wakeup sources and do sleeps in a loop.
//! Prints wakeup reason and sleep time on wakeup.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

use core::time::Duration;
use std::thread;
use std::time::Instant;

#[cfg(esp32)]
use esp_idf_hal::gpio::AnyIOPin;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::gpio::{PinDriver, Pull};
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::sleep::*;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::uart::config::Config;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::uart::UartDriver;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::units::Hertz;

#[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
use esp_idf_hal::gpio::Level;

fn print_wakeup_result(time_before: Instant) {
    let time_after = Instant::now();
    let wakeup_reason = WakeupReason::get();
    println!(
        "wake up from light sleep due to {:?} which lasted for {:?}",
        wakeup_reason,
        time_after - time_before
    );
}

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    let builder = std::thread::Builder::new().stack_size(10 * 1024);
    let th = builder.spawn(move || -> anyhow::Result<()> {
        #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
        let peripherals = Peripherals::take().unwrap();

        #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
        let mut sleep = LightSleep::new()?;

        #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
        {
            sleep = sleep.wakeup_on_timer(Duration::from_secs(5))?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        {
            #[cfg(esp32)]
            let (p_rtc0, p_rtc1) = (peripherals.pins.gpio26, peripherals.pins.gpio27);
            #[cfg(any(esp32s2, esp32s3))]
            let (p_rtc0, p_rtc1) = (peripherals.pins.gpio1, peripherals.pins.gpio2);

            let rtc0 = PinDriver::rtc_input(p_rtc0, Pull::Down)?;
            let rtc1 = PinDriver::rtc_input(p_rtc1, Pull::Down)?;

            let rtc_pins = rtc0.chain(&rtc1);
            sleep = sleep.wakeup_on_rtc(&rtc_pins, RtcWakeLevel::AnyHigh)?;
        }

        #[cfg(esp32)]
        let (p_gpio0, p_gpio1) = (peripherals.pins.gpio12, peripherals.pins.gpio14);
        #[cfg(any(esp32s2, esp32s3))]
        let (p_gpio0, p_gpio1) = (peripherals.pins.gpio37, peripherals.pins.gpio38);
        #[cfg(any(esp32c3, esp32c2))]
        let (p_gpio0, p_gpio1) = (peripherals.pins.gpio0, peripherals.pins.gpio1);

        #[cfg(any(esp32, esp32c3, esp32c2, esp32s2, esp32s3))]
        {
            let gpio0 = PinDriver::input(p_gpio0, Pull::Down)?;
            let gpio1 = PinDriver::input(p_gpio1, Pull::Down)?;

            sleep = sleep.wakeup_on_gpio(&gpio0, Level::High)?;
            sleep = sleep.wakeup_on_gpio(&gpio1, Level::High)?;
        }

        // 5. Configure UART Wakeup
        #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
        let config = Config::new().baudrate(Hertz(115_200));

        #[cfg(esp32)]
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio4,
            peripherals.pins.gpio3,
            None::<AnyIOPin>,
            None::<AnyIOPin>,
            &config,
        )?;
        #[cfg(any(esp32s2, esp32s3))]
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio43,
            peripherals.pins.gpio44,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
            &config,
        )?;
        #[cfg(esp32c3)]
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio21,
            peripherals.pins.gpio20,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
            &config,
        )?;

        #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
        {
            sleep = sleep.wakeup_on_uart(&uart, 3)?;
        }

        loop {
            println!("Entering light sleep...");
            thread::sleep(Duration::from_millis(60));

            let time_before = Instant::now();

            #[cfg(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3))]
            let err = sleep.enter();

            #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c2, esp32c3)))]
            let err: Result<(), anyhow::Error> = Ok(());

            match err {
                Ok(_) => print_wakeup_result(time_before),
                Err(e) => {
                    println!("failed to do sleep: {:?}", e);
                    thread::sleep(Duration::from_secs(1));
                }
            }
            println!("---");
        }
    })?;

    let _err = th.join();
    Ok(())
}
