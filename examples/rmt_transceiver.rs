//! A simple example that sets up a RMT transmitter and an RMT receiver
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

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(any(feature = "rmt-legacy", esp_idf_version_major = "4"))]
fn main() -> anyhow::Result<()> {
    example::main()
}

#[cfg(not(any(feature = "rmt-legacy", esp_idf_version_major = "4")))]
fn main() -> anyhow::Result<()> {
    println!("This example requires feature `rmt-legacy` enabled or using ESP-IDF v4.4.X");

    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(any(feature = "rmt-legacy", esp_idf_version_major = "4"))]
mod example {
    use esp_idf_hal::{
        delay::FreeRtos,
        prelude::Peripherals,
        rmt::{
            FixedLengthSignal, PinState, Pulse, PulseTicks, Receive, RmtReceiveConfig,
            RmtTransmitConfig, RxRmtDriver, TxRmtDriver,
        },
    };

    pub fn main() -> anyhow::Result<()> {
        println!("Starting APP!");

        let peripherals = Peripherals::take()?;

        /*
         *********************** SET UP RMT RECEIVER ******************************
         */
        let mut rx = RxRmtDriver::new(
            peripherals.rmt.channel2,
            peripherals.pins.gpio2,
            &RmtReceiveConfig::new().idle_threshold(700u16),
            250,
        )?;

        rx.start()?;

        let _ = std::thread::Builder::new()
            .stack_size(10000)
            .spawn(move || loop {
                println!("Rx Loop");

                let mut pulses = [(Pulse::zero(), Pulse::zero()); 250];

                // See sdkconfig.defaults to determine the tick time value ( default is one tick = 10 milliseconds)
                // Set ticks_to_wait to 0 for non-blocking
                let receive = rx.receive(&mut pulses, 0).unwrap();

                if let Receive::Read(length) = receive {
                    let pulses = &pulses[..length];

                    for (pulse0, pulse1) in pulses {
                        println!("0={pulse0:?}, 1={pulse1:?}");
                    }
                }

                FreeRtos::delay_ms(500);
            });

        /*
         *********************** SET UP RMT TRANSMITTER ******************************
         */

        // Prepare the tx_config
        // The default uses one memory block or 64 signals and clock divider set to 80 (1us tick)
        let mut tx = TxRmtDriver::new(
            peripherals.rmt.channel0,
            peripherals.pins.gpio4,
            &RmtTransmitConfig::new(),
        )?;

        // Prepare signal pulse signal to be sent.
        let one_low = Pulse::new(PinState::Low, PulseTicks::new(410)?);
        let one_high = Pulse::new(PinState::High, PulseTicks::new(210)?);
        let zero_low = Pulse::new(PinState::Low, PulseTicks::new(210)?);
        let zero_high = Pulse::new(PinState::High, PulseTicks::new(410)?);
        let sync_low = Pulse::new(PinState::Low, PulseTicks::new(620)?);
        let sync_high = Pulse::new(PinState::High, PulseTicks::new(620)?);

        let _ = std::thread::spawn(move || loop {
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
}
