//! A simple example that sets up a RMT transmiiter and an RMT receiver
//!
//! GPIO pin 2 is input and GPIO pin 4 is output
//!
//!    TYPICAL OUTPUT
//! Tx Loop
//! Rx Loop
//! level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(410)
//! level0 = High   dur0 = PulseTicks(410)   level1 = Low   dur1 = PulseTicks(210)
//! level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(0)
//! Rx Loop
//! level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! level0 = High   dur0 = PulseTicks(620)   level1 = Low   dur1 = PulseTicks(620)
//! level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(410)
//! level0 = High   dur0 = PulseTicks(410)   level1 = Low   dur1 = PulseTicks(210)
//! level0 = High   dur0 = PulseTicks(210)   level1 = Low   dur1 = PulseTicks(0)
//! Tx Loop

use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::rmt::{
    FixedLengthSignal, PinState, Pulse, PulseTicks, RmtReceiveConfig, RmtTransmitConfig,
    RxRmtDriver, TxRmtDriver, CHANNEL0, CHANNEL2,
};

fn main() -> anyhow::Result<()> {
    println!("Starting APP!");

    let peripherals = Peripherals::take().unwrap();

    /*
     *********************** SET UP RMT RECEIVER ******************************
     */
    let input_pin = peripherals.pins.gpio2;
    let rx_rmt_channel: CHANNEL2 = peripherals.rmt.channel2;
    let rx_config = RmtReceiveConfig::new().idle_threshold(700u16);
    let mut rx = RxRmtDriver::new(rx_rmt_channel, input_pin, &rx_config, 1000)?;
    let _rx_start = rx.start().unwrap();

    let _ = std::thread::spawn(move || loop {
        println!("Rx Loop");

        // See sdkconfig.defaults to determine the tick time value ( default is one tick = 10 milliseconds)
        // Set ticks_to_wait to 0 for non-blocking
        let ticks_to_wait = 0;
        let length = rx.get_rmt_items(ticks_to_wait).unwrap();

        if length != 0 {
            for n in 0..length / 4 {
                println!(
                    "level0 = {:?}   dur0 = {:?}   level1 = {:?}   dur1 = {:?}",
                    rx.pulse_pair_vec[n as usize].level0,
                    rx.pulse_pair_vec[n as usize].duration0,
                    rx.pulse_pair_vec[n as usize].level1,
                    rx.pulse_pair_vec[n as usize].duration1
                );
            }
        }

        FreeRtos::delay_ms(500);
    });

    /*
     *********************** SET UP RMT TRANSMITTER ******************************
     */
    let output_pin = peripherals.pins.gpio4;
    let tx_rmt_channel: CHANNEL0 = peripherals.rmt.channel0;

    // Prepare the tx_config
    // The default uses one memory block or 64 signals and clock divider set to 80 (1us tick)
    let tx_config = RmtTransmitConfig::new();
    let mut tx = TxRmtDriver::new(tx_rmt_channel, output_pin, &tx_config)?;

    // Prepare signal pulse signal to be sent.
    let one_low = Pulse::new(PinState::Low, PulseTicks::new(410)?);
    let one_high = Pulse::new(PinState::High, PulseTicks::new(210)?);
    let zero_low = Pulse::new(PinState::Low, PulseTicks::new(210)?);
    let zero_high = Pulse::new(PinState::High, PulseTicks::new(410)?);
    let sync_low = Pulse::new(PinState::Low, PulseTicks::new(620)?);
    let sync_high = Pulse::new(PinState::High, PulseTicks::new(620)?);

    let _transmit_task = thread::spawn(move || loop {
        println!("Tx Loop");

        // Create a sequence
        let mut signal = FixedLengthSignal::<5>::new();
        signal.set(0, &(sync_high, sync_low)).unwrap();
        signal.set(1, &(sync_high, sync_low)).unwrap();
        signal.set(2, &(one_high, one_low)).unwrap();
        signal.set(3, &(zero_high, zero_low)).unwrap();
        signal.set(4, &(one_high, one_low)).unwrap();

        // Transmit the signal (send sequence)
        tx.start(signal).unwrap();

        FreeRtos::delay_ms(1000);
    });

    loop {
        FreeRtos::delay_ms(3000);
    }
}
