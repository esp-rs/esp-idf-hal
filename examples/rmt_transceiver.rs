//! A simple example that sets up a RMT transmiiter and an RMT receiver
//!
//! GPIO pin 36 is input and GPIO pin 25 is output
//!
//!    TYPICAL OUTPUT
//! E (1378) mcpwm_test: Tx Loop
//! W (1378) mcpwm_test: Rx Loop
//! W (1378) mcpwm_test: level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! W (1378) mcpwm_test: level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! W (1388) mcpwm_test: level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(410)
//! W (1398) mcpwm_test: level0 = High   dur0 = PulseTicks(410)   level1 = Low   dur1 = PulseTicks(210)
//! W (1408) mcpwm_test: level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(0)
//! W (1918) mcpwm_test: Rx Loop
//! W (1918) mcpwm_test: level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! W (1918) mcpwm_test: level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! W (1928) mcpwm_test: level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(410)
//! W (1938) mcpwm_test: level0 = High   dur0 = PulseTicks(410)   level1 = Low   dur1 = PulseTicks(210)
//! W (1948) mcpwm_test: level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(0)
//! E (2378) mcpwm_test: Tx Loop

use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use log::*;

use esp_idf_hal::peripherals::Peripherals;
use std::thread;
use std::time::Duration;

use esp_idf_hal::rmt::{
    config::ReceiveConfig, config::TransmitConfig, FixedLengthSignal, Peripheral, PinState, Pulse,
    PulseTicks, Receive, Transmit, CHANNEL0, CHANNEL2,
};

fn main() -> anyhow::Result<()> {
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting APP!");

    let peripherals = Peripherals::take().unwrap();

    /*
     *********************** SET UP RMT RECEIVER ******************************
     */
    let input_pin = peripherals.pins.gpio36.into_input()?;
    let rx_rmt_channel: CHANNEL2 = unsafe { Peripheral::new().channel2 };
    let rx_config = ReceiveConfig::new().idle_threshold(700u16);
    let mut rx = Receive::new(input_pin, rx_rmt_channel, &rx_config, 1000)?;
    let _rx_start = rx.start().unwrap();

    let _receive_task = thread::spawn(move || loop {
        warn!("Rx Loop");

        // See sdkconfig.defaults to determine the tick time value ( default is one tick = 10 milliseconds)
        // Set ticks_to_wait to 0 for non-blocking
        let ticks_to_wait = 0;
        let length = rx.get_rmt_items(ticks_to_wait).unwrap();

        if length != 0 {
            for n in 0..length / 4 {
                warn!(
                    "level0 = {:?}   dur0 = {:?}   level1 = {:?}   dur1 = {:?}",
                    rx.pulse_pair_vec[n as usize].level0,
                    rx.pulse_pair_vec[n as usize].duration0,
                    rx.pulse_pair_vec[n as usize].level1,
                    rx.pulse_pair_vec[n as usize].duration1
                );
            }
        }

        thread::sleep(Duration::from_millis(500));
    });

    /*
     *********************** SET UP RMT TRANSMITTER ******************************
     */
    let output_pin = peripherals.pins.gpio25.into_output()?;
    let tx_rmt_channel: CHANNEL0 = unsafe { Peripheral::new().channel0 };

    // Prepare the tx_config
    // The default uses one memory block or 64 signals and clock divider set to 80 (1us tick)
    let tx_config = TransmitConfig::new();
    let mut tx = Transmit::new(output_pin, tx_rmt_channel, &tx_config)?;

    // Prepare signal pulse signal to be sent.
    let one_low = Pulse::new(PinState::Low, PulseTicks::new(410)?);
    let one_high = Pulse::new(PinState::High, PulseTicks::new(210)?);
    let zero_low = Pulse::new(PinState::Low, PulseTicks::new(210)?);
    let zero_high = Pulse::new(PinState::High, PulseTicks::new(410)?);
    let sync_low = Pulse::new(PinState::Low, PulseTicks::new(620)?);
    let sync_high = Pulse::new(PinState::High, PulseTicks::new(620)?);

    let _transmit_task = thread::spawn(move || loop {
        error!("Tx Loop");

        // Create a sequence
        let mut signal = FixedLengthSignal::<5>::new();
        signal.set(0, &(sync_high, sync_low)).unwrap();
        signal.set(1, &(sync_high, sync_low)).unwrap();
        signal.set(2, &(one_high, one_low)).unwrap();
        signal.set(3, &(zero_high, zero_low)).unwrap();
        signal.set(4, &(one_high, one_low)).unwrap();

        // Transmit the signal (send sequence)
        tx.start(signal).unwrap();

        thread::sleep(Duration::from_secs(1));
    });

    loop {
        thread::sleep(Duration::from_secs(3));
    }
}
