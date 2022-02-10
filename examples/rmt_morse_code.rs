//! RMT Transmit Example -- Morse Code
//!
//! Example loosely based off:
//! https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code

use esp_idf_hal::gpio::Gpio17;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::{CarrierConfig, DutyPercent, WriterConfig};
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, VecData, Writer};
use esp_idf_hal::units::FromValueType;
use log::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio17<Output> = peripherals.pins.gpio17.into_output()?;

    let carrier = CarrierConfig::new()
        .duty_percent(DutyPercent::new(50)?)
        .frequency(611.Hz());
    let config = WriterConfig::new()
        .carrier(Some(carrier))
        .clock_divider(255);

    let writer = Writer::new(led, Channel0, &config)?;
    let pulses = str_pulses("ABC CBA");

    let mut data = VecData::new();
    data.add(pulses)?;

    info!("Starting signal...");
    writer.start_blocking(&data)?;
    info!("Done!");

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
