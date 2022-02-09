//! Remote Control (RMT) module driver.
//!
//! The RMT (Remote Control) module driver can be used to send and receive infrared remote control
//! signals. Due to flexibility of RMT module, the driver can also be used to generate or receive
//! many other types of signals.
//!
//! This module is a wrapper around the [IDF RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
//! implementation. It is recommended to read before using this module.
//!
//! This is an initial implementation supporting:
//!  * Transmission, currently only blocking.
//!
//! Not supported:
//! * Receiving.
//! * No channel locking. (To prevent users from accidentally reusing a channel.)
//! * No buffer protection while transmitting.
//! * Change of config after initialisation.
//!
//! # Example Usage
//! ```
//! use esp_idf_hal::gpio::Output;
//! use esp_idf_hal::rmt::Channel::Channel0;
//!
//! let cfg = config:WriterConfig::new().clock_divider(1);
//!
//! let peripherals = Peripherals::take().unwrap();
//! let led: Gpio18<Output> = peripherals.pins.gpio18.into_output().unwrap();
//! let mut writer = Writer::new(led, cfg).unwrap();
//!
//! writer.push_pulse(Pulse::new(PinState::High, PulseTicks::max()));
//! writer.push_pulse(Pulse::new(PinState::Low, PulseTicks::max()));
//!
//! writer.start().unwrap();
//!
//!```
//!
//! See the `examples/` folder of this repository for more.

// TODO: Do we need to prevent users from creating two drivers on the same channel?
// TODO: Should probably prevent writing to buffer while rmt is transmitting:
//       "We must make sure the item data will not be damaged when the driver is still sending items in driver interrupt."
use crate::gpio::OutputPin;
use crate::rmt::config::WriterConfig;
use embedded_hal::digital::PinState;
use esp_idf_sys::{
    esp, esp_err_t, rmt_channel_t_RMT_CHANNEL_0, rmt_channel_t_RMT_CHANNEL_1,
    rmt_channel_t_RMT_CHANNEL_2, rmt_channel_t_RMT_CHANNEL_3, rmt_config, rmt_config_t,
    rmt_config_t__bindgen_ty_1, rmt_driver_install, rmt_get_counter_clock, rmt_item32_t,
    rmt_item32_t__bindgen_ty_1, rmt_item32_t__bindgen_ty_1__bindgen_ty_1, rmt_mode_t_RMT_MODE_RX,
    rmt_mode_t_RMT_MODE_TX, rmt_tx_config_t, rmt_write_items, EspError, ERANGE,
    ESP_ERR_INVALID_ARG, RMT_CHANNEL_FLAGS_AWARE_DFS,
};
use std::mem::ManuallyDrop;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Channel {
    // TODO: Work out the different number of channels per chip.
    Channel0,
    Channel1,
    Channel2,
    Channel3,
}

impl Into<u32> for Channel {
    fn into(self) -> u32 {
        match self {
            Channel::Channel0 => rmt_channel_t_RMT_CHANNEL_0,
            Channel::Channel1 => rmt_channel_t_RMT_CHANNEL_1,
            Channel::Channel2 => rmt_channel_t_RMT_CHANNEL_2,
            Channel::Channel3 => rmt_channel_t_RMT_CHANNEL_3,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Mode {
    Tx,
    Rx,
}

impl Into<u32> for Mode {
    fn into(self) -> u32 {
        match self {
            Mode::Tx => rmt_mode_t_RMT_MODE_TX,
            Mode::Rx => rmt_mode_t_RMT_MODE_RX,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pulse {
    ticks: PulseTicks,
    pin_state: PinState,
}

impl Pulse {
    pub fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
        Pulse { pin_state, ticks }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767;

    /// PulseTick needs to be unsigned 15 bits: 0-32767 inclusive.
    ///
    /// It will return `None` if it's out of range.
    pub fn new(v: u16) -> Result<Self, EspError> {
        if v > 32767 {
            Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
        } else {
            Ok(Self(v))
        }
    }

    /// Use the maximum value of 32767.
    pub fn max() -> Self {
        Self(Self::MAX)
    }
}

pub mod config {
    use embedded_hal::digital::PinState;
    use esp_idf_sys::EspError;

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct CarrierConfig {
        // TODO: Use units::Frequency.
        pub frequency_hz: u32,
        pub carrier_level: PinState,
        pub idle_level: PinState,
        // TODO: Use a Percentage type to restrict range to 0-100.
        pub duty_percent: u8,
    }

    impl Default for CarrierConfig {
        // Defaults from https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101
        fn default() -> Self {
            Self {
                frequency_hz: 38000,
                carrier_level: PinState::High,
                idle_level: PinState::Low,
                duty_percent: 33,
            }
        }
    }

    impl CarrierConfig {
        pub fn new() -> Self {
            Default::default()
        }

        pub fn frequency_hz(mut self, f: u32) -> Self {
            self.frequency_hz = f;
            self
        }

        pub fn carrier_level(mut self, state: PinState) -> Self {
            self.carrier_level = state;
            self
        }

        pub fn idle_level(mut self, state: PinState) -> Self {
            self.idle_level = state;
            self
        }

        pub fn duty_percent(mut self, duty: u8) -> Self {
            self.duty_percent = duty;
            self
        }
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum Loop {
        None,
        Count(u32),
        Infinite,
    }

    pub struct WriterConfig {
        pub clock_divider: u8,
        pub mem_block_num: u8,
        pub carrier: Option<CarrierConfig>,
        // TODO: `loop` is taken. Maybe can change to repeat even though it doesn't match the IDF.
        pub looping: Loop,
        /// Enable and set the signal level on the output if idle.
        pub idle: Option<PinState>,
        pub aware_dfs: bool,
    }

    impl Default for WriterConfig {
        // Defaults from https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101
        fn default() -> Self {
            Self {
                aware_dfs: false,
                mem_block_num: 1,
                clock_divider: 80,
                looping: Loop::None,
                carrier: None,
                idle: Some(PinState::Low),
            }
        }
    }

    impl WriterConfig {
        pub fn new() -> Self {
            Self::default()
        }

        /// Channel can work during APB clock scaling.
        ///
        /// When set, RMT channel will take REF_TICK or XTAL as source clock. The benefit is, RMT
        /// channel can continue work even when APB clock is changing.
        pub fn aware_dfs(mut self, enable: bool) -> Self {
            self.aware_dfs = enable;
            self
        }

        pub fn mem_block_num(mut self, mem_block_num: u8) -> Self {
            self.mem_block_num = mem_block_num;
            self
        }

        pub fn clock_divider(mut self, divider: u8) -> Self {
            self.clock_divider = divider;
            self
        }

        pub fn looping(mut self, looping: Loop) -> Self {
            self.looping = looping;
            self
        }

        pub fn carrier(mut self, carrier: Option<CarrierConfig>) -> Self {
            self.carrier = carrier;
            self
        }

        pub fn idle(mut self, idle: Option<PinState>) -> Self {
            self.idle = idle;
            self
        }
    }
}

pub struct Writer {
    channel: Channel,

    // This must be ManuallyDrop to ensure that it isn't automatically dropped before the driver is
    // done using the items.
    // TODO: Manually drop this!
    items: ManuallyDrop<Vec<rmt_item32_t>>,

    // An item that has had only its first half populated.
    half_inserted: Option<rmt_item32_t>,

    ticks_hz: Option<u32>,
}

impl Writer {
    pub fn new<P>(pin: P, channel: Channel, config: &WriterConfig) -> Result<Writer, EspError>
    where
        P: OutputPin,
    {
        let mut flags = 0;
        if config.aware_dfs {
            flags |= RMT_CHANNEL_FLAGS_AWARE_DFS;
        }

        let carrier_en = config.carrier.is_some();
        let carrier = config.carrier.unwrap_or_default();

        use config::Loop;
        let loop_en = config.looping != Loop::None;
        let loop_count = match config.looping {
            Loop::None => 0,
            Loop::Count(c) => c,
            Loop::Infinite => 0,
        };

        let sys_config = rmt_config_t {
            rmt_mode: rmt_mode_t_RMT_MODE_TX,
            channel: channel as u32,
            gpio_num: pin.pin(),
            clk_div: config.clock_divider,
            mem_block_num: config.mem_block_num,
            flags,
            __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                tx_config: rmt_tx_config_t {
                    carrier_en,
                    carrier_freq_hz: carrier.frequency_hz,
                    carrier_level: carrier.carrier_level as u32,
                    carrier_duty_percent: carrier.duty_percent,
                    idle_output_en: config.idle.is_some(),
                    idle_level: config.idle.map(|i| i as u32).unwrap_or(0),
                    loop_en,
                    loop_count,
                },
            },
        };

        unsafe {
            esp!(rmt_config(&sys_config))?;
            esp!(rmt_driver_install(channel as u32, 0, 0))?;
        }

        Ok(Self {
            channel,
            items: Default::default(),
            half_inserted: None,
            ticks_hz: None,
        })
    }

    pub fn counter_clock(&self) -> Result<u32, EspError> {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(self.channel as u32, &mut ticks_hz) })?;
        Ok(ticks_hz)
    }

    /// Creates a Pulse based on Nanoseconds.
    ///
    /// This function exists on `Writer` because of the internal call to `rmt_get_counter_clock`
    /// requires a channel which is managed by `Writer`.
    pub fn pulse_ns(&mut self, pin_state: PinState, ns: u32) -> Result<Pulse, EspError> {
        let ticks_hz = match &self.ticks_hz {
            None => {
                self.ticks_hz = Some(self.counter_clock()?);
                self.ticks_hz.as_ref().unwrap()
            }
            Some(t) => t,
        };
        let ticks = (*ticks_hz as u64 * ns as u64 / 1_000_000_000) as u16;
        let ticks = PulseTicks::new(ticks)?;
        Ok(Pulse::new(pin_state, ticks))
    }

    pub fn add<I>(&mut self, pulses: I) -> Result<(), EspError>
    where
        I: IntoIterator<Item = Pulse>,
    {
        for pulse in pulses {
            if let Some(item) = self.half_inserted.as_mut() {
                // SAFETY: We have retrieved this item which is previously populated with the same
                // union field.
                let inner = unsafe { &mut item.__bindgen_anon_1.__bindgen_anon_1 };

                inner.set_level1(pulse.pin_state as u32);
                inner.set_duration1(pulse.ticks.0 as u32);

                self.flush_half_inserted();
            } else {
                let mut inner = rmt_item32_t__bindgen_ty_1__bindgen_ty_1::default();
                inner.set_level0(pulse.pin_state as u32);
                inner.set_duration0(pulse.ticks.0 as u32);
                let item = esp_idf_sys::rmt_item32_t {
                    __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                        __bindgen_anon_1: inner,
                    },
                };
                self.half_inserted = Some(item);
            };
        }

        Ok(())
    }

    fn flush_half_inserted(&mut self) {
        if let Some(item) = self.half_inserted {
            self.items.push(item);
            self.half_inserted = None;
        }
    }

    pub fn clear(&mut self) {
        self.items.clear();
        self.half_inserted = None;
    }

    /// Start sending the pulses.
    pub fn start(&mut self) -> Result<(), EspError> {
        self.flush_half_inserted();

        esp!(unsafe {
            rmt_write_items(
                self.channel as u32,
                self.items.as_ptr(),
                self.items.len() as i32,
                true, // TODO: Blocking.
            )
        })
    }

    pub fn stop(&self) -> Result<(), EspError> {
        todo!()
    }
}

impl Drop for Writer {
    fn drop(&mut self) {
        // We're not able to return errors, so log a warning if we can't stop.
        if let Err(err) = self.stop() {
            println!("Failed to stop rmt::Writer during Drop: {:?}", err);
        }

        todo!("Ensure we have stopped before dropping items.");
    }
}
