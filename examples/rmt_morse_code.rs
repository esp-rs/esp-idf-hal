use crate::Code::*;
/// RMT Transmit Example -- Morse Code
///
/// Example loosely based off:
/// https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code
///
use embedded_hal::digital::PinState;
use esp_idf_hal::gpio::Gpio18;
use esp_idf_hal::gpio::Output;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::Channel::Channel0;
use esp_idf_hal::rmt::{Pulse, PulseDuration, Writer, WriterConfig};

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio18<Output> = peripherals.pins.gpio18.into_output()?;

    let config = WriterConfig::new(&led, Channel0);
    let mut writer = Writer::new(config)?;

    let pulses = str_pulses("ABC CBA");
    writer.add(pulses);

    loop {
        writer.start();
    }
}

enum Code {
    Dot,
    Dash,
    LetterGap,
    WordGap,
}

impl Into<Vec<Pulse>> for Code {
    fn into(self) -> Vec<Pulse> {
        let mut v = vec![];
        match self {
            Dot => vec![Pulse::new(PinState::High, PulseDuration::Tick(0xffff))],
            Dash => vec![],
            LetterGap => vec![],
            WordGap => vec![],
        }
        v
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
        for pulse in find_codes(&c).iter().map(|code| code.into()) {
            pulses.push(pulse);
        }
    }

    pulses
}
