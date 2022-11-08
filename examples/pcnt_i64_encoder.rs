//! PCNT decoding a rotery encoder
//!
//! To try this out, connect a rotery encoder to pins 5 and 6, the common should be grounded
//!
//! Note that PCNT only track a singed 16bit value.  We use interrupts to detect a LOW and HIGH
//! threshold and track how much that accounts for and provide an i64 value result
//!
use std::{sync::{atomic::{Ordering, AtomicI64}, Arc}, cmp::min};

use anyhow;
use anyhow::Context;
use embedded_hal_0_2::blocking::delay::DelayMs;
use esp_idf_hal::delay::FreeRtos as delay;
use esp_idf_hal::prelude::*;
use esp_idf_hal::gpio::Pull;
use tracing::{info, Level};
use tracing_subscriber::FmtSubscriber;
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_hal::pcnt::*;

fn main() -> anyhow::Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    let subscriber = FmtSubscriber::builder()
    // all spans/events with a level higher than TRACE (e.g, debug, info, warn, etc.)
    // will be written to stdout.
    .with_max_level(Level::TRACE)
    // completes the builder.
    .finish();

    tracing::subscriber::set_global_default(subscriber)
        .expect("setting default subscriber failed");

    info!("setup pins");
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;
    let m1_enc1_pin = peripherals.pins.gpio5;
    let m1_enc2_pin = peripherals.pins.gpio6;
    let m1_enc1_pin = PcntPin::new(m1_enc1_pin, Pull::Up)?;
    let m1_enc2_pin = PcntPin::new(m1_enc2_pin, Pull::Up)?;
    info!("creating pcnt unit 0");
    let mut pcnt = Pcnt::new()?;
    info!("configure pcnt chanel 0");
    const POS_LIMIT: i16 = 100;
    const NEG_LIMIT: i16 = -100;
    let mut config = PcntConfig {
        pulse_pin: Some(&m1_enc1_pin),
        ctrl_pin: Some(&m1_enc2_pin),
        lctrl_mode: PcntControlMode::Reverse,
        hctrl_mode: PcntControlMode::Keep,
        pos_mode: PcntCountMode::Decrement,
        neg_mode: PcntCountMode::Increment,
        counter_h_lim: POS_LIMIT,
        counter_l_lim: NEG_LIMIT,
        channel: PcntChannel::Channel0,
        _p: std::marker::PhantomData,
    };
    pcnt.config(&mut config).context("configuring CHANNEL0")?;

    info!("configure pcnt chanel 1");
    config.channel = PcntChannel::Channel1;
    config.pulse_pin = Some(&m1_enc2_pin);
    config.ctrl_pin = Some(&m1_enc1_pin);
    config.pos_mode = PcntCountMode::Increment;
    config.neg_mode = PcntCountMode::Decrement;
    pcnt.config(&mut config).context("configuring CHANNEL1")?;
    pcnt.set_filter_value(min(10*80, 1023))?;
    pcnt.filter_enable()?;
    let value = Arc::new(AtomicI64::new(0));
    // unsafe interrupt code to catch the upper and lower limits from the encoder
    // and track the overflow in `value: Arc<AtomicI64>` - I plan to use this for 
    // a wheeled robot's odomerty
    unsafe {
        let value = value.clone();
        pcnt.subscribe(move |status| {
            let status = PcntEventType::from_bits_retain(status);
            if status.contains(PcntEventType::H_LIM) {
                value.fetch_add(POS_LIMIT as i64, Ordering::SeqCst);
            }
            if status.contains(PcntEventType::L_LIM) {
                value.fetch_add(NEG_LIMIT as i64, Ordering::SeqCst);
            }
        })?;
    }
    pcnt.event_enable(PcntEventType::H_LIM)?;
    pcnt.event_enable(PcntEventType::L_LIM)?;
    info!("starting pcnt counter");
    pcnt.counter_pause()?;
    pcnt.counter_clear()?;
    pcnt.counter_resume()?;

    let mut last_value = 0i64;
    loop {
        let pcnt_value = pcnt.get_counter_value()? as i64;
        let acc_value = value.load(Ordering::SeqCst);
        let value = acc_value + pcnt_value;
        if value != last_value {
            info!("value: {value} pct={pcnt_value} acc={acc_value}");
            last_value = value;
        }
        delay.delay_ms(100u32);
    }
}
