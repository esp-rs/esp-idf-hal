use std::{sync::Arc, time::Duration};

use esp_idf_hal::ledc::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;

use esp_idf_sys::EspError;

const CYCLES: usize = 3;

fn cycle_duty<C, H, T, P>(
    mut pwm: Channel<C, H, T, P>,
    times: usize,
    log_prefix: &str,
    sleep: Duration,
) -> anyhow::Result<()>
where
    C: HwChannel,
    H: HwTimer,
    T: Borrow<Timer<H>>,
    P: OutputPin,
{
    let max_duty = pwm.get_max_duty()?;

    for cycle in 0..times {
        println!("{} cycle: {}", log_prefix, cycle);

        for numerator in [0, 1, 2, 3, 4, 5].iter() {
            println!("{} duty: {}/5", log_prefix, numerator);
            pwm.set_duty(max_duty * numerator / 5)?;
            std::thread::sleep(sleep);
        }
    }

    Ok(())
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    println!("Setting up PWM output channels");

    let mut peripherals = Peripherals::take().unwrap();
    let config = TimerConfig::default().frequency(25.kHz().into());
    let timer = Arc::new(Timer::new(peripherals.ledc.timer0, &config)?);
    let timer0 = timer.clone();
    let timer1 = timer.clone();
    let channel0 = Channel::new(peripherals.ledc.channel0, timer0, peripherals.pins.gpio4)?;
    let channel1 = Channel::new(peripherals.ledc.channel1, timer1, peripherals.pins.gpio5)?;

    println!("Spawning PWM threads");

    let thread0 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || cycle_duty(channel0, CYCLES, "PWM 0", Duration::from_millis(1000)))?;
    let thread1 = std::thread::Builder::new()
        .stack_size(7000)
        .spawn(move || cycle_duty(channel1, CYCLES, "PWM 1", Duration::from_millis(1750)))?;

    println!("Waiting for PWM threads");

    thread0.join().unwrap()?;
    thread1.join().unwrap()?;

    println!("Joined PWM threads");

    if let Ok(timer) = Arc::try_unwrap(timer) {
        println!("Unwrapped timer");
        if let Ok(hw_timer) = timer.release() {
            println!("Recovered HW timer");
            peripherals.ledc.timer0 = hw_timer;
        }
    }

    println!("Done");

    loop {
        // Don't let the idle task starve and trigger warnings from the watchdog.
        std::thread::sleep(Duration::from_millis(100));
    }
}
