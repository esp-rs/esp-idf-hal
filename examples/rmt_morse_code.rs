//! RMT Transmit Example -- Morse Code
//!
//! Example loosely based off:
//! https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code

use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::PinState;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::Gpio17;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::{CarrierConfig, WriterConfig};
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{Pulse, PulseTicks, VecData, Writer};
use log::*;
use Code::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio17<Output> = peripherals.pins.gpio17.into_output()?;

    let carrier = CarrierConfig::new().duty_percent(50).frequency_hz(611);
    let config = WriterConfig::new()
        .carrier(Some(carrier))
        .clock_divider(255);

    let mut writer = Writer::new(led, Channel0, &config)?;
    let pulses = str_pulses("ABC CBA");
    let mut data = VecData::new();
    data.add(pulses)?;

    // loop {
    writer.start(data)?;
    // Ets.delay_ms(3000)?;
    // }
    Ok(())
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
            Dot => pulses.extend_from_slice(&[high(), low()]),
            Dash => pulses.extend_from_slice(&[high(), high(), high(), low()]),
            WordGap => pulses.extend_from_slice(&[low(), low(), low(), low(), low(), low()]),
        }
    }
}

const CODES: &[(char, &[Code])] = &[
    (' ', &[WordGap]),
    ('A', &[Dot, Dash]),
    ('B', &[Dash, Dot, Dot, Dot]),
    ('C', &[Dash, Dot, Dash, Dot]),
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
