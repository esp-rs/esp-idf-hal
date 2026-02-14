//! Tests light sleep
//!
//! Enables multiple light sleep wakeup sources and sleeps in a loop.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

use core::time::Duration;
use std::thread;
use std::time::Instant;

use esp_idf_hal::gpio::{Level, PinDriver, Pull};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::sleep::LightSleep;

#[cfg(not(esp32c2))]
use esp_idf_hal::gpio::AnyIOPin;
#[cfg(not(esp32c2))]
use esp_idf_hal::uart::{config::Config, UartDriver};
#[cfg(not(esp32c2))]
use esp_idf_hal::units::Hertz;

#[cfg(not(any(esp32c2, esp32c3)))]
use esp_idf_hal::sleep::RtcWakeLevel;

fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();
    let peripherals = Peripherals::take()?;

    #[cfg(esp32)]
    let (gpio_pin0, gpio_pin1) = (peripherals.pins.gpio12, peripherals.pins.gpio14);
    #[cfg(any(esp32s2, esp32s3))]
    let (gpio_pin0, gpio_pin1) = (peripherals.pins.gpio37, peripherals.pins.gpio38);
    #[cfg(any(esp32c2, esp32c3))]
    let (gpio_pin0, gpio_pin1) = (peripherals.pins.gpio0, peripherals.pins.gpio1);

    let gpio0 = PinDriver::input(gpio_pin0, Pull::Down)?;
    let gpio1 = PinDriver::input(gpio_pin1, Pull::Down)?;

    #[cfg(not(any(esp32c2, esp32c3)))]
    let (rtc0, rtc1) = {
        #[cfg(esp32)]
        let (p0, p1) = (peripherals.pins.gpio26, peripherals.pins.gpio27);
        #[cfg(not(esp32))]
        let (p0, p1) = (peripherals.pins.gpio1, peripherals.pins.gpio2);
        (
            PinDriver::rtc_input(p0, Pull::Down)?,
            PinDriver::rtc_input(p1, Pull::Down)?,
        )
    };

    #[cfg(not(esp32c2))]
    let uart = {
        let config = Config::new().baudrate(Hertz(115_200));

        #[cfg(esp32)]
        let (tx, rx) = (peripherals.pins.gpio4, peripherals.pins.gpio3);
        #[cfg(any(esp32s2, esp32s3))]
        let (tx, rx) = (peripherals.pins.gpio43, peripherals.pins.gpio44);
        #[cfg(esp32c3)]
        let (tx, rx) = (peripherals.pins.gpio21, peripherals.pins.gpio20);

        UartDriver::new(
            peripherals.uart0,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &config,
        )?
    };

    let mut sleep = LightSleep::new()?;

    sleep = sleep
        .wakeup_on_timer(Duration::from_secs(5))?
        .wakeup_on_gpio(&gpio0, Level::High)?
        .wakeup_on_gpio(&gpio1, Level::High)?;

    #[cfg(not(any(esp32c2, esp32c3)))]
    {
        let pins = rtc0.chain(&rtc1);
        sleep = sleep.wakeup_on_rtc(&pins, RtcWakeLevel::AnyHigh)?;
    }

    #[cfg(not(esp32c2))]
    {
        sleep = sleep.wakeup_on_uart(&uart, 3)?;
    }

    loop {
        thread::sleep(Duration::from_millis(60));
        println!("Entering light sleep...");

        let start = Instant::now();
        sleep.enter()?;

        println!(
            "Wakeup due to {:?} lasting {:?}",
            WakeupReason::get(),
            start.elapsed()
        );
    }
}
