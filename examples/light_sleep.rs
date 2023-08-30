//! Tests light sleep
//!
//! Enables multiple light sleep wakeup sources and do sleeps in a loop.
//! Prints wakeup reason and sleep time on wakeup.

use core::time::Duration;
use esp_idf_hal::gpio::{self, AnyInputPin, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::sleep::*;
use esp_idf_hal::uart::config::Config;
use esp_idf_hal::uart::UartDriver;
use std::thread;
use std::time::Instant;

use crate::gpio::Level;

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
    esp_idf_sys::link_patches();

    // run in a thread with increased stack size to prevent overflow
    let builder = std::thread::Builder::new().stack_size(8 * 1024);
    let th = builder.spawn(move || -> anyhow::Result<()> {
        let peripherals = Peripherals::take().unwrap();

        // RTC wakeup definitions
        #[cfg(esp32)]
        let rtc_pins = [
            RtcWakeupPin::new(peripherals.pins.gpio26.into(), false, true),
            RtcWakeupPin::new(peripherals.pins.gpio27.into(), false, true),
        ];
        #[cfg(any(esp32s2, esp32s3))]
        let rtc_pins = [
            RtcWakeupPin::new(peripherals.pins.gpio1.into(), false, true),
            RtcWakeupPin::new(peripherals.pins.gpio2.into(), false, true),
        ];
        #[cfg(any(esp32, esp32s2, esp32s3))]
        let rtc_wakeup = Some(RtcWakeup::new(&rtc_pins, RtcWakeLevel::AnyHigh));

        // GPIO wakeup definitions
        #[cfg(esp32)]
        let gpio_pin1 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio16))?;
        #[cfg(esp32)]
        let gpio_pin2 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio17))?;

        #[cfg(any(esp32s2, esp32s3))]
        let gpio_pin1 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio37))?;
        #[cfg(any(esp32s2, esp32s3))]
        let gpio_pin2 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio38))?;

        #[cfg(esp32c3)]
        let gpio_pin1 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio6))?;
        #[cfg(esp32c3)]
        let gpio_pin2 = PinDriver::input(AnyInputPin::from(peripherals.pins.gpio7))?;

        let gpio_pins = [
            GpioWakeupPin::new(gpio_pin1, Level::High)?,
            GpioWakeupPin::new(gpio_pin2, Level::High)?,
        ];
        #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
        let gpio_wakeup = Some(GpioWakeup::new(&gpio_pins));
        #[cfg(not(any(esp32, esp32c3, esp32s2, esp32s3)))]
        let gpio_wakeup: Option<GpioWakeup> = None;

        // UART definitions
        let config = Config::new().baudrate(Hertz(115_200));
        #[cfg(any(esp32))]
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio4,
            peripherals.pins.gpio3,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
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
        let uart_wakeup = Some(UartWakeup::new(&uart, 3));
        #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c3)))]
        let uart_wakeup: Option<UartWakeup> = None;

        let lsleep = LightSleep {
            timer: Some(TimerWakeup::new(Duration::from_secs(5))),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            rtc: rtc_wakeup,
            gpio: gpio_wakeup,
            uart: uart_wakeup,
            ..Default::default()
        };

        loop {
            println!("Light sleep with: {:?}", lsleep);
            // short sleep to flush stdout
            thread::sleep(Duration::from_millis(60));

            let time_before = Instant::now();
            let err = lsleep.sleep();
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
