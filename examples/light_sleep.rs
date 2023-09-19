//! Tests light sleep
//!
//! Enables multiple light sleep wakeup sources and do sleeps in a loop.
//! Prints wakeup reason and sleep time on wakeup.

use core::time::Duration;
#[cfg(any(esp32, esp32s2, esp32s3))]
use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::gpio::{self, PinDriver};
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
    esp_idf_hal::sys::link_patches();

    // run in a thread with increased stack size to prevent overflow
    let builder = std::thread::Builder::new().stack_size(10 * 1024);
    let th = builder.spawn(move || -> anyhow::Result<()> {
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

        // GPIO wakeup definitions
        #[cfg(esp32)]
        let gpio0 = PinDriver::input(peripherals.pins.gpio12)?;
        #[cfg(esp32)]
        let gpio1 = PinDriver::input(peripherals.pins.gpio14)?;

        #[cfg(any(esp32s2, esp32s3))]
        let gpio0 = PinDriver::input(peripherals.pins.gpio37)?;
        #[cfg(any(esp32s2, esp32s3))]
        let gpio1 = PinDriver::input(peripherals.pins.gpio38)?;

        #[cfg(esp32c3)]
        let gpio0 = PinDriver::input(peripherals.pins.gpio6)?;
        #[cfg(esp32c3)]
        let gpio1 = PinDriver::input(peripherals.pins.gpio7)?;

        #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
        let gpio_pin0 = GpioWakeupPin {
            pindriver: &gpio0,
            wake_level: Level::High,
        };
        #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
        let gpio_pin1 = GpioWakeupPin {
            pindriver: &gpio1,
            wake_level: Level::High,
        };
        #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
        let gpio_wakeup = Some(GpioWakeup {
            pins: EmptyGpioWakeupPins::chain(gpio_pin0).chain(gpio_pin1),
        });
        #[cfg(not(any(esp32, esp32c3, esp32s2, esp32s3)))]
        let gpio_wakeup = None::<GpioWakeup>;

        // UART definitions
        let config = Config::new().baudrate(Hertz(115_200));
        #[cfg(any(esp32))]
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
        let uart_wakeup = Some(UartWakeup::new(&uart, 3));
        #[cfg(not(any(esp32, esp32s2, esp32s3, esp32c3)))]
        let uart_wakeup: Option<UartWakeup> = None;

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let lsleep = make_light_sleep_rtc_gpio_pins(
            Some(TimerWakeup::new(Duration::from_secs(5))),
            rtc_wakeup,
            gpio_wakeup,
            uart_wakeup,
            None,
            None,
        );

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        let lsleep = make_light_sleep_gpio_pins(
            Some(TimerWakeup::new(Duration::from_secs(5))),
            gpio_wakeup,
            uart_wakeup,
        );

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
