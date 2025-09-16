// TODO: explain how this example works and what it demonstrates?

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(any(
    feature = "rmt-legacy",
    not(esp_idf_version_at_least_5_1_2),
    not(esp_idf_soc_rmt_supported),
))]
fn main() -> anyhow::Result<()> {
    println!("This example requires feature `rmt-legacy` disabled, using ESP-IDF >= v5.1.2, or is not supported on this MCU");

    loop {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}

#[cfg(all(
    esp_idf_soc_rmt_supported,
    esp_idf_version_at_least_5_1_2,
    not(feature = "rmt-legacy")
))]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::rmt::config::TxChannelConfig;
    use esp_idf_hal::rmt::encoder::{DelegatorState, NotEnoughSpace, SimpleEncoder, SymbolWriter};
    use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, Symbol, TxChannel};
    use esp_idf_hal::units::Hertz;
    use std::time::Duration;

    const RMT_LED_STRIP_RESOLUTION_HZ: Hertz = Hertz(10_000_000); // 10MHz resolution, 1 tick = 0.1us

    println!("Starting APP!");

    let peripherals = Peripherals::take()?;

    let channel = TxChannel::new(
        peripherals.pins.gpio19, // TODO: use builtin LED pin
        &TxChannelConfig {
            clock_source: Default::default(),
            ..Default::default()
        },
    )?;

    fn encode_led_data(
        data: &[u8],
        writer: &mut SymbolWriter<'_>,
        _arg: &mut (),
    ) -> Result<(), NotEnoughSpace> {
        // TODO: make these constants
        // TODO: construct PulseTicks with a duration?
        let ws2812_zero: Symbol = Symbol::new(
            Pulse::new(
                PinState::High,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_nanos(300),
                )
                .unwrap(),
            ),
            Pulse::new(
                PinState::Low,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_nanos(900),
                )
                .unwrap(),
            ),
        );
        let ws2812_one: Symbol = Symbol::new(
            Pulse::new(
                PinState::High,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_nanos(900),
                )
                .unwrap(),
            ),
            Pulse::new(
                PinState::Low,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_nanos(300),
                )
                .unwrap(),
            ),
        );
        let ws2812_reset: Symbol = Symbol::new(
            Pulse::new(
                PinState::Low,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_micros(25),
                )
                .unwrap(),
            ),
            Pulse::new(
                PinState::Low,
                PulseTicks::new_with_duration(
                    RMT_LED_STRIP_RESOLUTION_HZ,
                    &Duration::from_micros(25),
                )
                .unwrap(),
            ),
        );

        // We need a minimum of 8 symbol spaces to encode a byte. We only
        // need one to encode a reset, but it's simpler to simply demand that
        // there are 8 symbol spaces free to write anything.
        if writer.remaining() < 8 * data.len() + 1 {
            return Err(NotEnoughSpace);
        }

        // TODO: it is impossible to indicate a partial write?
        //       e.g. not finished with the entire transmission, because I ran out of space

        for &byte in data {
            let mut bitmask = 0x80;
            while bitmask != 0 {
                if (byte & bitmask) != 0 {
                    writer.write_all(&[ws2812_one])?;
                } else {
                    writer.write_all(&[ws2812_zero])?;
                }

                bitmask >>= 1;
            }
        }

        writer.write_all(&[ws2812_reset])?;

        Ok(())
    }

    let mut state = DelegatorState {
        callback: encode_led_data,
        arg: &mut (),
    };
    let mut encoder = SimpleEncoder::with_config(&mut state, &Default::default())?;

    Ok(())
}
