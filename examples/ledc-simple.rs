use embedded_hal::delay::blocking::DelayUs;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::ledc::{config::TimerConfig, Channel, Timer};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    println!("Configuring output channel");

    let peripherals = Peripherals::take().unwrap();
    let config = TimerConfig::default().frequency(25.kHz().into());
    let timer = Timer::new(peripherals.ledc.timer0, &config)?;
    let mut channel = Channel::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio4)?;

    println!("Starting duty-cycle loop");

    let max_duty = channel.get_max_duty()?;
    for numerator in [0, 1, 2, 3, 4, 5].iter().cycle() {
        println!("Duty {}/5", numerator);
        channel.set_duty(max_duty * numerator / 5)?;
        FreeRtos.delay_ms(2000)?;
    }

    loop {
        FreeRtos.delay_ms(1000)?;
    }
}
