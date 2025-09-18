// TODO: explain how this example works and what it demonstrates?

// This is a port of https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt/led_strip_simple_encoder/main/led_strip_example_main.c

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(any(
    feature = "rmt-legacy",
    not(esp_idf_version_at_least_5_3_0),
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
    esp_idf_version_at_least_5_3_0,
    not(feature = "rmt-legacy")
))]
fn main() -> anyhow::Result<()> {
    use esp_idf_hal::delay::Ets;
    use esp_idf_hal::peripherals::Peripherals;
    use esp_idf_hal::rmt::config::TxChannelConfig;
    use esp_idf_hal::rmt::encoder::{EncoderCallback, NotEnoughSpace, SimpleEncoder, SymbolBuffer};
    use esp_idf_hal::rmt::{PinState, Pulse, PulseTicks, RmtChannel, Symbol, TxChannel};
    use esp_idf_hal::units::Hertz;

    use std::f64::consts::PI;
    use std::ops::AddAssign;
    use std::time::Duration;

    const RMT_LED_STRIP_RESOLUTION_HZ: Hertz = Hertz(10_000_000); // 10MHz resolution, 1 tick = 0.1us

    println!("Starting APP!");

    let peripherals = Peripherals::take()?;

    fn numbered_array<T, const N: usize>(start: T, step: T) -> [T; N]
    where
        T: AddAssign + Copy,
    {
        let mut current = start;
        [(); N].map(|_| {
            let value = current;
            current += step;
            value
        })
    }

    pub struct LedEncoder {
        input_position: usize,
    }

    impl LedEncoder {
        pub fn new() -> Self {
            Self { input_position: 0 }
        }
    }

    #[derive(Debug, Clone, Copy)]
    #[repr(C)]
    pub struct Color {
        pub red: u8,
        pub green: u8,
        pub blue: u8,
    }

    fn byte_to_symbols(byte: u8) -> [Symbol; 8] {
        // TODO: make these constants
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

        numbered_array(0, 1)
            .map(|i| 0x80 >> i)
            .map(|bitmask| (byte & bitmask) != 0)
            .map(|is_one| if is_one { ws2812_one } else { ws2812_zero })
    }

    impl Color {
        pub fn write_symbols(&self, buffer: &mut [Symbol]) {
            buffer[0..8].copy_from_slice(&byte_to_symbols(self.green));
            buffer[8..16].copy_from_slice(&byte_to_symbols(self.red));
            buffer[16..24].copy_from_slice(&byte_to_symbols(self.blue));
        }
    }

    impl EncoderCallback for LedEncoder {
        type Item = Color;

        fn encode(
            &mut self,
            input_data: &[Self::Item],
            buffer: &mut SymbolBuffer<'_>,
        ) -> Result<(), NotEnoughSpace> {
            let ws2812_reset: Symbol = Symbol::new(
                Pulse::new(
                    PinState::Low,
                    PulseTicks::new_with_duration(
                        RMT_LED_STRIP_RESOLUTION_HZ,
                        &Duration::from_micros(150),
                    )
                    .unwrap(),
                ),
                Pulse::new(
                    PinState::Low,
                    PulseTicks::new_with_duration(
                        RMT_LED_STRIP_RESOLUTION_HZ,
                        &Duration::from_micros(150),
                    )
                    .unwrap(),
                ),
            );

            if buffer.position() == 0 {
                self.input_position = 0;
            }

            for &next_color in &input_data[self.input_position..] {
                let mut symbols = [ws2812_reset; 25];
                next_color.write_symbols(&mut symbols[1..]);
                buffer.write_all(&symbols)?;
                self.input_position += 1;
            }

            // finished encoding all input data
            Ok(())
        }
    }

    let mut channel = TxChannel::new(
        peripherals.pins.gpio21, // TODO: use builtin LED pin
        &TxChannelConfig {
            clock_source: Default::default(),
            memory_block_symbols: 64, // increasing might reduce flickering
            resolution: RMT_LED_STRIP_RESOLUTION_HZ,
            transaction_queue_depth: 4, // The number of transactions that can be pending in the background
            ..Default::default()
        },
    )?;

    channel.enable()?;

    let mut encoder = SimpleEncoder::with_config(LedEncoder::new(), &Default::default())?;

    const NUMBER_OF_LEDS: usize = 1;
    const ANGLE_INC_FRAME: f64 = 0.02;
    const ANGLE_INC_LED: f64 = 0.3;
    const FRAME_DURATION_MS: u32 = 20;

    let mut offset = 0.0;
    loop {
        let mut signal = [Color {
            red: 0,
            green: 0,
            blue: 0,
        }; NUMBER_OF_LEDS];
        for (i, color) in signal.iter_mut().enumerate() {
            let angle = offset + (i as f64 * ANGLE_INC_LED);
            let color_offset = (PI * 2.0) / 3.0;

            *color = Color {
                red: ((angle + color_offset * 1.0).sin() * 127.0 + 128.0) as u8,
                green: ((angle + color_offset * 0.0).sin() * 127.0 + 128.0) as u8,
                blue: ((angle + color_offset * 2.0).sin() * 117.0 + 128.0) as u8,
            };
        }

        channel.send_and_wait(&mut encoder, &signal, &Default::default())?;

        Ets::delay_ms(FRAME_DURATION_MS);

        // Increase offset to shift pattern
        offset += ANGLE_INC_FRAME;
        if offset > 2.0 * PI {
            offset -= 2.0 * PI;
        }
    }
}
