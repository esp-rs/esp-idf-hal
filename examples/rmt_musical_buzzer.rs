//! Play a song using a piezo buzzer.
//!
//! Should play "Ode to Joy" on pin 17.
//!
//! Based off the ESP-IDF rmt musical buzzer example:
//! https://github.com/espressif/esp-idf/blob/b092fa073047c957545a0ae9504f04972a8c6d74/examples/peripherals/rmt/musical_buzzer/main/musical_buzzer_example_main.c
use core::time::Duration;
use embedded_hal::delay::blocking::DelayUs;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::config::{Loop, TransmitConfig};
use esp_idf_hal::rmt::{FixedLengthSignal, HwChannel, PinState, Pulse, PulseTicks, Transmit};
use notes::*;

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let led = peripherals.pins.gpio17.into_output()?;
    let channel = peripherals.rmt.channel0;
    let config = TransmitConfig::new().looping(Loop::Count(1024));
    let mut tx = Transmit::new(led, channel, &config)?;

    loop {
        play_song(&mut tx, ODE_TO_JOY)?;
        Ets.delay_ms(3000)?;
    }
}

pub fn play_song<P: OutputPin, C: HwChannel>(
    tx: &mut Transmit<P, C>,
    song: &[NoteValue],
) -> anyhow::Result<()> {
    for note_value in song {
        play_note(tx, note_value.note.0, note_value.duration)?;
    }
    Ok(())
}

pub fn play_note<P: OutputPin, C: HwChannel>(
    tx: &mut Transmit<P, C>,
    pitch: u16,
    duration: Duration,
) -> anyhow::Result<()> {
    // Calculate the frequency for a piezo buzzer.
    let ticks_hz = tx.counter_clock()?;
    let tick_count = (ticks_hz.0 as u128 / pitch as u128 / 2_u128) as u16;
    let ticks = PulseTicks::new(tick_count)?;

    // Add high and low pulses for the tick duration.
    let on = Pulse::new(PinState::High, ticks);
    let off = Pulse::new(PinState::Low, ticks);
    let mut signal = FixedLengthSignal::<1>::new();
    signal.set(0, &(on, off))?;

    // Play the note for the 80% of the duration.
    tx.start(signal)?;
    Ets.delay_ms((80 * duration.as_millis() / 100) as u32)?;

    // Small pause between notes, 20% of the specified duration.
    tx.stop()?;
    Ets.delay_ms((20 * duration.as_millis() / 100) as u32)?;

    Ok(())
}

#[derive(Debug)]
pub struct Note(u16);

pub mod notes {
    use crate::Note;

    #[allow(dead_code)]
    pub const A4: Note = Note(440);
    pub const AS4: Note = Note(466);
    pub const B4: Note = Note(494);
    pub const C5: Note = Note(523);
    pub const CS5: Note = Note(554);
    pub const D5: Note = Note(587);
    pub const DS5: Note = Note(622);
    pub const E5: Note = Note(659);
    pub const F5: Note = Note(698);
    pub const FS5: Note = Note(740);
    pub const G5: Note = Note(784);
    pub const GS5: Note = Note(831);
    pub const A5: Note = Note(880);
}

#[derive(Debug)]
pub struct NoteValue {
    note: Note,
    duration: Duration,
}

macro_rules! n {
    ($n: expr, $duration: expr) => {
        NoteValue {
            note: $n,
            duration: Duration::from_millis($duration),
        }
    };
}

const ODE_TO_JOY: &[NoteValue] = &[
    n!(FS5, 400),
    n!(FS5, 600),
    n!(G5, 400),
    n!(A5, 400),
    n!(A5, 400),
    n!(G5, 400),
    n!(FS5, 400),
    n!(E5, 400),
    n!(D5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(FS5, 400),
    n!(FS5, 400),
    n!(FS5, 200),
    n!(E5, 200),
    n!(E5, 800),
    n!(FS5, 400),
    n!(FS5, 600),
    n!(G5, 400),
    n!(A5, 400),
    n!(A5, 400),
    n!(G5, 400),
    n!(FS5, 400),
    n!(E5, 400),
    n!(D5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(FS5, 400),
    n!(E5, 400),
    n!(E5, 200),
    n!(D5, 200),
    n!(D5, 800),
    n!(E5, 400),
    n!(E5, 400),
    n!(FS5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(FS5, 200),
    n!(G5, 200),
    n!(FS5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(FS5, 200),
    n!(G5, 200),
    n!(FS5, 400),
    n!(E5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(A4, 400),
    n!(A4, 400),
    n!(FS5, 400),
    n!(FS5, 600),
    n!(G5, 400),
    n!(A5, 400),
    n!(A5, 400),
    n!(G5, 400),
    n!(FS5, 400),
    n!(E5, 400),
    n!(D5, 400),
    n!(D5, 400),
    n!(E5, 400),
    n!(FS5, 400),
    n!(E5, 400),
    n!(E5, 200),
    n!(D5, 200),
    n!(D5, 800),
];
