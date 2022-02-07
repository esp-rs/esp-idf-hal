/// RMT Transmit Example -- Morse Code
///
/// Example loosely based off:
/// https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code
///
use crate::Code::*;
use embedded_hal::digital::PinState;
use esp_idf_hal::gpio::Gpio17;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{Pulse, PulseTicks, Writer, WriterConfig};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    println!("test");

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio17<Output> = peripherals.pins.gpio17.into_output()?;

    let config = WriterConfig::new(&led, Channel0)
        .loop_enabled(true)
        .carrier_duty_percent(50)
        .carrier_freq_hz(611)
        .clock_divider(255);
    let mut writer = Writer::new(config)?;

    let pulses = str_pulses("ABC CBA");
    writer.add(pulses)?;

    loop {
        writer.start()?;
    }
}

enum Code {
    Dot,
    Dash,
    WordGap,
}

impl Code {
    pub fn push_pulse(&self, pulses: &mut Vec<Pulse>) {
        let high = Pulse::new(PinState::High, PulseTicks::max());
        let low = Pulse::new(PinState::Low, PulseTicks::max());

        match &self {
            Dot => pulses.extend_from_slice(&[high, low]),
            Dash => pulses.extend_from_slice(&[high, high, high, low]),
            WordGap => pulses.extend_from_slice(&[low, low, low, low, low, low]),
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
    }
    pulses
}
