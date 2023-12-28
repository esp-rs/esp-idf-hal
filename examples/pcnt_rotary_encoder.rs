//! PCNT decoding a rotary encoder
//!
//! To try this out, connect a rotary encoder to pins 5 and 6, the common should be grounded
//!
//! Note that PCNT only track a singed 16bit value.  We use interrupts to detect a LOW and HIGH
//! threshold and track how much that accounts for and provide an i32 value result
//!

#[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
fn main() -> anyhow::Result<()> {
    use anyhow::Context;
    use encoder::Encoder;
    use esp_idf_hal::delay::FreeRtos;
    use esp_idf_hal::prelude::*;

    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_hal::sys::link_patches();

    println!("setup pins");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    let mut pin_a = peripherals.pins.gpio5;
    let mut pin_b = peripherals.pins.gpio6;
    println!("setup encoder");
    let encoder = Encoder::new(peripherals.pcnt0, &mut pin_a, &mut pin_b)?;

    let mut last_value = 0i32;
    loop {
        let value = encoder.get_value()?;
        if value != last_value {
            println!("value: {value}");
            last_value = value;
        }
        FreeRtos::delay_ms(100u32);
    }
}

#[cfg(not(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3))))]
fn main() {
    use esp_idf_hal::delay::FreeRtos;
    println!("pcnt peripheral not supported on this device!");
    loop {
        FreeRtos::delay_ms(100u32);
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
// esp-idf encoder implementation using v4 pcnt api
mod encoder {
    use std::cmp::min;
    use std::sync::atomic::AtomicI32;
    use std::sync::atomic::Ordering;
    use std::sync::Arc;

    use esp_idf_hal::gpio::AnyInputPin;
    use esp_idf_hal::gpio::InputPin;
    use esp_idf_hal::pcnt::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_sys::EspError;

    const LOW_LIMIT: i16 = -100;
    const HIGH_LIMIT: i16 = 100;

    pub struct Encoder<'d> {
        unit: PcntDriver<'d>,
        approx_value: Arc<AtomicI32>,
    }

    impl<'d> Encoder<'d> {
        pub fn new<PCNT: Pcnt>(
            pcnt: impl Peripheral<P = PCNT> + 'd,
            pin_a: impl Peripheral<P = impl InputPin> + 'd,
            pin_b: impl Peripheral<P = impl InputPin> + 'd,
        ) -> Result<Self, EspError> {
            let mut unit = PcntDriver::new(
                pcnt,
                Some(pin_a),
                Some(pin_b),
                Option::<AnyInputPin>::None,
                Option::<AnyInputPin>::None,
            )?;
            unit.channel_config(
                PcntChannel::Channel0,
                PinIndex::Pin0,
                PinIndex::Pin1,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Decrement,
                    neg_mode: PcntCountMode::Increment,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;
            unit.channel_config(
                PcntChannel::Channel1,
                PinIndex::Pin1,
                PinIndex::Pin0,
                &PcntChannelConfig {
                    lctrl_mode: PcntControlMode::Reverse,
                    hctrl_mode: PcntControlMode::Keep,
                    pos_mode: PcntCountMode::Increment,
                    neg_mode: PcntCountMode::Decrement,
                    counter_h_lim: HIGH_LIMIT,
                    counter_l_lim: LOW_LIMIT,
                },
            )?;

            unit.set_filter_value(min(10 * 80, 1023))?;
            unit.filter_enable()?;

            let approx_value = Arc::new(AtomicI32::new(0));
            // unsafe interrupt code to catch the upper and lower limits from the encoder
            // and track the overflow in `value: Arc<AtomicI32>` - I plan to use this for
            // a wheeled robot's odomerty
            unsafe {
                let approx_value = approx_value.clone();
                unit.subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                    }
                })?;
            }
            unit.event_enable(PcntEvent::HighLimit)?;
            unit.event_enable(PcntEvent::LowLimit)?;
            unit.counter_pause()?;
            unit.counter_clear()?;
            unit.counter_resume()?;

            Ok(Self { unit, approx_value })
        }

        pub fn get_value(&self) -> Result<i32, EspError> {
            let value =
                self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i32;
            Ok(value)
        }
    }
}
