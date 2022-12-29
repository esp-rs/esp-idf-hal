//! PCNT decoding a rotary encoder
//!
//! To try this out, connect a rotary encoder to pins 5 and 6, the common should be grounded
//!
//! Note that PCNT only track a singed 16bit value.  We use interrupts to detect a LOW and HIGH
//! threshold and track how much that accounts for and provide an i64 value result
//!

use anyhow;
use anyhow::Context;
use log::*;

use esp_idf_hal::delay::FreeRtos; 
use esp_idf_hal::prelude::*;

use encoder::Encoder;

fn main() -> anyhow::Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("setup pins");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    let mut pin_a = peripherals.pins.gpio5;
    let mut pin_b = peripherals.pins.gpio6;
    info!("setup encoder");
    #[cfg(any(feature = "pcnt", esp_idf_version_major = "4"))]
    let encoder = Encoder::new(peripherals.pcnt0, &mut pin_a, &mut pin_b)?;
    #[cfg(not(any(feature = "pcnt", esp_idf_version_major = "4")))]
    let encoder = Encoder::new(&mut pin_a, &mut pin_b)?;

    let mut last_value = 0i64;
    loop {
        let value = encoder.get_value()?;
        if value != last_value {
            info!("value: {value}");
            last_value = value;
        }
        FreeRtos::delay_ms(100u32);
    }
}


// esp-idf encoder implementation using v4 pcnt api
#[cfg(any(feature = "pcnt", esp_idf_version_major = "4"))]
mod encoder {
    use std::cmp::min;
    use std::sync::Arc;
    use std::sync::atomic::AtomicI64;
    use std::sync::atomic::Ordering;

    use esp_idf_hal::gpio::InputPin;
    use esp_idf_hal::pcnt::*;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_sys::EspError;

    const LOW_LIMIT: i16 = -100;
    const HIGH_LIMIT: i16 = 100;

    pub struct Encoder<'d> {
        unit: PcntDriver<'d>,
        approx_value: Arc<AtomicI64>,
    }

    impl<'d> Encoder<'d> {
        pub fn new<'a, PCNT: Pcnt>(
            pcnt: impl Peripheral<P = PCNT> + 'd,
            mut pin_a: impl Peripheral<P = impl InputPin>,
            mut pin_b: impl Peripheral<P = impl InputPin>
        ) -> Result<Self, EspError> {
            let mut unit = PcntDriver::new(pcnt)?;
            unit.channel_config(PcntChannel::Channel0, Some(&mut pin_a), Some(&mut pin_b), &mut PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            })?;
            unit.channel_config(PcntChannel::Channel1, Some(&mut pin_b), Some(&mut pin_a), &mut PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            })?;

            unit.set_filter_value(min(10 * 80, 1023))?;
            unit.filter_enable()?;

            let approx_value = Arc::new(AtomicI64::new(0));
            // unsafe interrupt code to catch the upper and lower limits from the encoder
            // and track the overflow in `value: Arc<AtomicI64>` - I plan to use this for
            // a wheeled robot's odomerty
            unsafe {
                let approx_value = approx_value.clone();
                unit.subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        approx_value.fetch_add(HIGH_LIMIT as i64, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        approx_value.fetch_add(LOW_LIMIT as i64, Ordering::SeqCst);
                    }
                })?;
            }
            unit.event_enable(PcntEvent::HighLimit)?;
            unit.event_enable(PcntEvent::LowLimit)?;
            unit.counter_pause()?;
            unit.counter_clear()?;
            unit.counter_resume()?;

            Ok(Self {
                unit,
                approx_value,
            })
        }

        pub fn get_value(&self) -> Result<i64, EspError> {
            let value = self.approx_value.load(Ordering::Relaxed) + self.unit.get_counter_value()? as i64;    
            Ok(value)
        }
    }
}

// esp-idf v5 encoder implementation using pulse_cnt api
#[cfg(not(any(feature = "pcnt", esp_idf_version_major = "4")))]
mod encoder {
    use std::sync::Arc;
    use std::sync::atomic::AtomicI64;
    use std::sync::atomic::Ordering;

    use esp_idf_hal::gpio::InputPin;
    use esp_idf_hal::peripheral::Peripheral;
    use esp_idf_hal::pulse_cnt::*;
    use esp_idf_sys::EspError;

    const LOW_LIMIT: i32 = -100;
    const HIGH_LIMIT: i32 = 100;

    pub struct Encoder {
        unit: PcntUnit,
        _channels: [PcntChannel; 2], // we don't use but don't want to drop
        approx_value: Arc<AtomicI64>,
    }

    impl Encoder {
        pub fn new(mut pin_a: impl Peripheral<P = impl InputPin>, mut pin_b: impl Peripheral<P = impl InputPin>) -> Result<Self, EspError> {
            let mut unit = PcntUnit::new(&PcntUnitConfig {
                low_limit: LOW_LIMIT,
                high_limit: HIGH_LIMIT,
                ..Default::default()
            })?;
            let channel0 = unit.channel(Some(&mut pin_a), Some(&mut pin_b), PcntChanFlags::default())?;
            channel0.set_level_action(PcntLevelAction::Keep, PcntLevelAction::Inverse)?;
            channel0.set_edge_action(PcntEdgeAction::Decrease, PcntEdgeAction::Increase)?;
            let channel1 = unit.channel(Some(&mut pin_b), Some(&mut pin_a), PcntChanFlags::default())?;
            channel1.set_level_action(PcntLevelAction::Keep, PcntLevelAction::Inverse)?;
            channel1.set_edge_action(PcntEdgeAction::Increase, PcntEdgeAction::Decrease)?;

            unit.add_watch_point(LOW_LIMIT)?;
            unit.add_watch_point(HIGH_LIMIT)?;

            let approx_value = Arc::new(AtomicI64::new(0));
            {
                let approx_value = approx_value.clone();
                unit.subscribe(move |event| {
                    approx_value.fetch_add(event.watch_point_value.into(), Ordering::SeqCst);
                    false // no high priority task woken
                })?;
            }

            unit.enable()?;
            unit.start()?;
        
            Ok(Self {
                unit,
                _channels: [channel0, channel1],
                approx_value,
            })
        }

        pub fn get_value(&self) -> Result<i64, EspError> {
            Ok(self.approx_value.load(Ordering::SeqCst) + self.unit.get_count()? as i64)
        }
    }
}