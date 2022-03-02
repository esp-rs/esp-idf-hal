//! RMT Transmit Example -- Morse Code
//!
//! To try this out, connect a piezo buzzer via a resistor into pin 17. It should repeat "HELLO"
//! in morse code.
//!
//! Set pin 16 to low to stop the signal and make the buzzer sound "GOODBYE" in morse code.
//!
//! Example loosely based off which has a circuit you can follow.
//! https://github.com/espressif/esp-idf/tree/master/examples/peripherals/rmt/morse_code
//!
//! This example demonstrates:
//! * A carrier signal.
//! * Looping.
//! * Background sending.
//! * Stopping/releasing a [`Pin`] and [`Channel`], to be used again.
//!
use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::blocking::InputPin;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::{Gpio16, Gpio17, Input, Output, Pin};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::{CarrierConfig, DutyPercent, Loop, TransmitConfig};
use esp_idf_hal::rmt::VariableLengthSignal; // This example requires `alloc` feature.
use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, Transmit, CHANNEL0};
use esp_idf_hal::units::FromValueType;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let led: Gpio17<Output> = peripherals.pins.gpio17.into_output()?;
    let stop: Gpio16<Input> = peripherals.pins.gpio16.into_input()?;
    let channel = peripherals.rmt.channel0;

    let carrier = CarrierConfig::new()
        .duty_percent(DutyPercent::new(50)?)
        .frequency(611.Hz());
    let mut config = TransmitConfig::new()
        .carrier(Some(carrier))
        .looping(Loop::Count(100))
        .clock_divider(255);

    let tx = send_morse_code(&config, led, channel, "HELLO ")?;

    println!("Keep sending until pin {} is set low.", stop.pin());
    while stop.is_high()? {
        Ets.delay_ms(100)?;
    }
    println!("Pin {} is set to low--stopping message.", stop.pin());

    // Release pin and channel so we can use them again.
    let (led, channel) = tx.release()?;

    // Wait so the messages don't get garbled.
    Ets.delay_ms(3000)?;

    // Now send a single message and stop.
    println!("Saying GOODBYE!");
    config.looping = Loop::None;
    send_morse_code(&config, led, channel, "GOODBYE")?;

    Ok(())
}

fn send_morse_code(
    config: &TransmitConfig,
    led: Gpio17<Output>,
    channel: CHANNEL0,
    message: &str,
) -> anyhow::Result<Transmit<Gpio17<Output>, CHANNEL0>> {
    println!("Sending morse message '{}' to pin {}.", message, led.pin());

    let mut signal = VariableLengthSignal::new();
    let pulses = str_pulses(message);
    // We've been collecting `Pulse` but `VariableLengthSignal` needs `&Pulse`:
    let pulses: Vec<&Pulse> = pulses.iter().collect();
    signal.push(pulses)?;

    let mut tx = Transmit::new(led, channel, config)?;
    tx.start(signal)?;

    // Return `tx` so we can release the pin and channel later.
    Ok(tx)
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

        // Create a gap after each symbol.
        pulses.push(low());
        pulses.push(low());
    }
    pulses
}

const CODES: &[(char, &[Code])] = &[
    (' ', &[Code::WordGap]),
    ('A', &[Code::Dot, Code::Dash]),
    ('B', &[Code::Dash, Code::Dot, Code::Dot, Code::Dot]),
    ('C', &[Code::Dash, Code::Dot, Code::Dash, Code::Dot]),
    ('D', &[Code::Dash, Code::Dot, Code::Dot]),
    ('E', &[Code::Dot]),
    ('F', &[Code::Dot, Code::Dot, Code::Dash, Code::Dot]),
    ('G', &[Code::Dash, Code::Dash, Code::Dot]),
    ('H', &[Code::Dot, Code::Dot, Code::Dot, Code::Dot]),
    ('I', &[Code::Dot, Code::Dot]),
    ('J', &[Code::Dot, Code::Dash, Code::Dash, Code::Dash]),
    ('K', &[Code::Dash, Code::Dot, Code::Dash]),
    ('L', &[Code::Dot, Code::Dash, Code::Dot, Code::Dot]),
    ('M', &[Code::Dash, Code::Dash]),
    ('N', &[Code::Dash, Code::Dot]),
    ('O', &[Code::Dash, Code::Dash, Code::Dash]),
    ('P', &[Code::Dot, Code::Dash, Code::Dash, Code::Dot]),
    ('Q', &[Code::Dash, Code::Dash, Code::Dot, Code::Dash]),
    ('R', &[Code::Dot, Code::Dash, Code::Dot]),
    ('S', &[Code::Dot, Code::Dot, Code::Dot]),
    ('T', &[Code::Dash]),
    ('U', &[Code::Dot, Code::Dot, Code::Dash]),
    ('V', &[Code::Dot, Code::Dot, Code::Dot, Code::Dash]),
    ('W', &[Code::Dot, Code::Dash, Code::Dash]),
    ('X', &[Code::Dash, Code::Dot, Code::Dot, Code::Dash]),
    ('Y', &[Code::Dash, Code::Dot, Code::Dash, Code::Dash]),
    ('Z', &[Code::Dash, Code::Dash, Code::Dot, Code::Dot]),
];
