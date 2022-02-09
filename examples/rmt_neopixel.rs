// TODO: Write some notes

use core::time::Duration;
use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::PinState;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::Gpio18;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::WriterConfig;
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{Pulse, StackWriterData, VecData, Writer};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio18<Output> = peripherals.pins.gpio18.into_output().unwrap();
    let config = WriterConfig::new().clock_divider(1);
    let mut writer = Writer::new(led, Channel0, &config).unwrap();

    let rgbs = [0xff0000, 0xffff00, 0x00ffff, 0x00ff00, 0xa000ff];
    loop {
        for rgb in rgbs {
            neopixel(&mut writer, rgb)?;
            Ets.delay_ms(1000).unwrap();
        }
    }
}

fn neopixel(writer: &mut Writer, rgb: u32) -> anyhow::Result<()> {
    let ticks_hz = writer.counter_clock()?;
    let t0h = Pulse::new_with_duration(ticks_hz, PinState::High, Duration::from_nanos(350))?;
    let t0l = Pulse::new_with_duration(ticks_hz, PinState::Low, Duration::from_nanos(800))?;
    let t1h = Pulse::new_with_duration(ticks_hz, PinState::High, Duration::from_nanos(700))?;
    let t1l = Pulse::new_with_duration(ticks_hz, PinState::Low, Duration::from_nanos(600))?;

    let mut data = VecData::new();
    for i in 0..24 {
        let bit = 2_u32.pow(i) & rgb != 0;
        let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
        data.add([high_pulse, low_pulse])?;
    }
    Ok(writer.start(data)?)
}
