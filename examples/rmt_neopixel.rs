use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::PinState;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::Gpio18;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{Pulse, PulseDuration, Writer, WriterConfig};
use std::time::SystemTime;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    // esp_idf_svc::log::EspLogger::initialize_default();

    println!("???????????");
    return Ok(());

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio18<Output> = peripherals.pins.gpio18.into_output().unwrap();
    let config = WriterConfig::new(&led, Channel0).clock_divider(1);
    let mut writer = Writer::new(config).unwrap();
    let start = SystemTime::now();

    loop {
        let r = band(&start, 1f32);
        let c = r * 0xff00;
        neopixel(&mut writer, c);
        Ets.delay_ms(10).unwrap();
    }
}

fn band(start: &SystemTime, speed: f32) -> u32 {
    let b = SystemTime::now()
        .duration_since(*start)
        .unwrap()
        .as_secs_f32()
        * speed;
    (b * ((b.cos() + 1f32) / 2f32)) as u32
}

fn neopixel(writer: &mut Writer, rgb: u32) {
    const T0H: u32 = 350;
    const T1H: u32 = 700;
    const T0L: u32 = 800;
    const T1L: u32 = 600;

    writer.clear().unwrap();

    for i in 0..24 {
        let bit = 2_u32.pow(i) & rgb != 0;
        let (high, low) = if bit { (T1H, T1L) } else { (T0H, T0L) };

        writer
            .add([
                Pulse::new(PinState::High, PulseDuration::Nanos(high)),
                Pulse::new(PinState::Low, PulseDuration::Nanos(low)),
            ])
            .unwrap();
    }

    writer.start().unwrap();
}
