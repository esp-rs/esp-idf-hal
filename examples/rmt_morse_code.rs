//! RMT Transmit Example -- Morse Code
//!
//! Example loosely based off:
//! https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code

use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::blocking::InputPin;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{Gpio16, Gpio17, GpioPin, Input, Output, Pin};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::{CarrierConfig, DutyPercent, Loop, WriterConfig};
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, VecData, Writer, CHANNEL0};
use esp_idf_hal::units::FromValueType;
use log::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio17<Output> = peripherals.pins.gpio17.into_output()?;
    let stop: Gpio16<Input> = peripherals.pins.gpio16.into_input()?;
    let channel = peripherals.rmt.channel0;

    let carrier = CarrierConfig::new()
        .duty_percent(DutyPercent::new(50)?)
        .frequency(611.Hz());
    let mut config = WriterConfig::new()
        .carrier(Some(carrier))
        .looping(Loop::Forever)
        .clock_divider(255);

    let writer = send_morse_code(&config, led, channel, "IS ANYONE THERE  ")?;

    info!("Keep sending until pin {} is set high.", stop.pin());
    while stop.is_low()? {
        Ets.delay_ms(100)?;
    }

    // Release pin and channel so we can use them again.
    let (led, channel) = writer.release()?;

    // Now send a single message and stop.
    config.looping = Loop::None;
    let writer = send_morse_code(&config, led, channel, "HELLO AND BYE")?;

    // TODO: writer.wait()?;

    Ok(())
}

fn send_morse_code(
    config: &WriterConfig,
    led: Gpio17<Output>,
    channel: CHANNEL0,
    message: &str,
) -> anyhow::Result<Writer<Gpio17<Output>, CHANNEL0>> {
    info!("Sending morse code to pin {}.", led.pin());

    let mut data = VecData::new();
    data.add(str_pulses(message))?;

    let writer = Writer::new(led, channel, &config)?;
    writer.start(data)?;

    // Return writer so we can release the pin and channel later.
    Ok(writer)
}

fn high() -> Pulse {
    Pulse::new(PinState::High, PulseTicks::max())
}

fn low() -> Pulse {
    Pulse::new(PinState::Low, PulseTicks::max())
}

enum Code {
    Dot,
    Dash,
    WordGap,
}

impl Code {
    pub fn push_pulse(&self, pulses: &mut Vec<Pulse>) {
        match &self {
            Code::Dot => pulses.extend_from_slice(&[high(), low()]),
            Code::Dash => pulses.extend_from_slice(&[high(), high(), high(), low()]),
            Code::WordGap => pulses.extend_from_slice(&[low(), low(), low(), low(), low(), low()]),
        }
    }
}

const CODES: &[(char, &[Code])] = &[
    (' ', &[Code::WordGap]),
    ('A', &[Code::Dot, Code::Dash]),
    ('B', &[Code::Dash, Code::Dot, Code::Dot, Code::Dot]),
    ('C', &[Code::Dash, Code::Dot, Code::Dash, Code::Dot]),
];

fn find_codes(c: &char) -> &'static [Code] {
    for (found, codes) in CODES.iter() {
        if found == c {
            return codes;
        }
    }
    &[]
}

fn str_pulses(s: &str) -> Vec<Pulse> {
    let mut pulses = vec![];
    for c in s.chars() {
        for code in find_codes(&c) {
            code.push_pulse(&mut pulses);
        }
        pulses.push(low());
        pulses.push(low());
    }
    pulses
}
