//! Remote Control (RMT) module driver.
//!
//! The RMT (Remote Control) module driver can be used to send and receive infrared remote control
//! signals. Due to flexibility of RMT module, the driver can also be used to generate or receive
//! many other types of signals.
//!
//! This module is an abstraction around the [IDF RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
//! implementation. It is recommended to read before using this module.
//!
//! This is an initial implementation supporting:
//!  * Transmission, currently only blocking.
//!
//! Not supported:
//! * Receiving.
//! * Change of config after initialisation.
//!
//! # Loading pulses
//! TODO: Mention VecData vs StackPairedData.
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

use crate::gpio::OutputPin;
use crate::units::Hertz;
use chip::HwChannel;
use config::WriterConfig;
use core::convert::TryFrom;
use core::time::Duration;
use esp_idf_sys::*;

pub use chip::*;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinState {
    Low,
    High,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pulse {
    pub ticks: PulseTicks,
    pub pin_state: PinState,
}

impl Pulse {
    pub fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
        Pulse { pin_state, ticks }
    }

    /// Create a `Pulse` using a `Duration`.
    ///
    /// We need to know the ticks frequency (`ticks_hz`), which depends on the prepared channel
    /// within a `Writer`. To get the frequency for the `ticks_hz` argument, use
    /// `Writer::counter_clock()`. For example:
    /// ```
    /// # use esp_idf_sys::EspError;
    /// # use esp_idf_hal::gpio::Output;
    /// # use esp_idf_hal::rmt::Channel::Channel0;
    /// # fn example() -> Result<(), EspError> {
    /// # let peripherals = Peripherals::take()?;
    /// # let led: Gpio18<Output> = peripherals.pins.gpio18.into_output()?;
    /// let mut writer = Writer::new(led, cfg)?;
    /// let ticks_hz = writer.counter_clock()?;
    /// let pulse = Pulse::new_with_duration(ticks_hz, PinState::High, Duration::from_nanos(500))?;
    /// # }
    /// ```
    pub fn new_with_duration(
        ticks_hz: Hertz,
        pin_state: PinState,
        duration: Duration,
    ) -> Result<Self, EspError> {
        let ticks = PulseTicks::new_with_duration(ticks_hz, duration)?;
        Ok(Self::new(pin_state, ticks))
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767;

    /// PulseTick needs to be unsigned 15 bits: 0-32767 inclusive.
    pub fn new(v: u16) -> Result<Self, EspError> {
        if v > Self::MAX {
            Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
        } else {
            Ok(Self(v))
        }
    }

    /// Use the maximum value of 32767.
    pub fn max() -> Self {
        Self(Self::MAX)
    }

    /// Convert a `Duration` into `PulseTicks`.
    ///
    /// See `Pulse::new_with_duration()` for details.
    pub fn new_with_duration(ticks_hz: Hertz, duration: Duration) -> Result<Self, EspError> {
        let ticks = duration
            .as_nanos()
            .checked_mul(u32::from(ticks_hz) as u128)
            .ok_or(EspError::from(EOVERFLOW as i32).unwrap())?
            / 1_000_000_000;
        let ticks = u16::try_from(ticks).map_err(|_| EspError::from(EOVERFLOW as i32).unwrap())?;

        Self::new(ticks)
    }
}

pub mod config {
    use super::PinState;
    use crate::units::{FromValueType, Hertz};
    use esp_idf_sys::{EspError, ESP_ERR_INVALID_ARG};

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct DutyPercent(pub(super) u8);

    impl DutyPercent {
        pub fn new(v: u8) -> Result<Self, EspError> {
            if v > 100 {
                Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
            } else {
                Ok(Self(v))
            }
        }
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct CarrierConfig {
        // TODO: Use units::Frequency.
        pub frequency: Hertz,
        pub carrier_level: PinState,
        pub idle_level: PinState,
        pub duty_percent: DutyPercent,
    }

    impl Default for CarrierConfig {
        // Defaults from https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101
        fn default() -> Self {
            Self {
                frequency: 38.kHz().into(),
                carrier_level: PinState::High,
                idle_level: PinState::Low,
                duty_percent: DutyPercent(33),
            }
        }
    }

    impl CarrierConfig {
        pub fn new() -> Self {
            Default::default()
        }

        pub fn frequency(mut self, hz: Hertz) -> Self {
            self.frequency = hz;
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

        pub fn duty_percent(mut self, duty: DutyPercent) -> Self {
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

pub struct Writer<P: OutputPin, C: HwChannel> {
    pin: P,
    channel: C,
}

impl<P: OutputPin, C: HwChannel> Writer<P, C> {
    pub fn new(pin: P, channel: C, config: &WriterConfig) -> Result<Self, EspError> {
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
            channel: C::channel(),
            gpio_num: pin.pin(),
            clk_div: config.clock_divider,
            mem_block_num: config.mem_block_num,
            flags,
            __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                tx_config: rmt_tx_config_t {
                    carrier_en,
                    carrier_freq_hz: carrier.frequency.into(),
                    carrier_level: carrier.carrier_level as u32,
                    carrier_duty_percent: carrier.duty_percent.0,
                    idle_output_en: config.idle.is_some(),
                    idle_level: config.idle.map(|i| i as u32).unwrap_or(0),
                    loop_en,
                    loop_count,
                },
            },
        };

        unsafe {
            esp!(rmt_config(&sys_config))?;
            esp!(rmt_driver_install(C::channel(), 0, 0))?;
        }

        Ok(Self { pin, channel })
    }

    pub fn counter_clock(&self) -> Result<Hertz, EspError> {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(C::channel(), &mut ticks_hz) })?;
        Ok(ticks_hz.into())
    }

    /// Start sending the pulses returning immediately. Non blocking.
    ///
    /// `data` is captured for safety so that the user can't change the data while transmitting.
    pub fn start<D>(&self, data: D) -> Result<(), EspError>
    where
        D: Data,
    {
        self.write_items(&data, false)
    }

    pub fn start_blocking<D>(&self, data: &D) -> Result<(), EspError>
    where
        D: Data,
    {
        self.write_items(data, true)
    }

    fn write_items<D>(&self, data: &D, block: bool) -> Result<(), EspError>
    where
        D: Data,
    {
        let items = data.as_slice();
        esp!(unsafe { rmt_write_items(C::channel(), items.as_ptr(), items.len() as i32, block,) })
    }

    // TODO: Need to specify ticks.
    // pub fn wait(&self) -> Result<(), EspError> {
    //     esp!(unsafe { rmt_wait_tx_done(C::channel()) })
    // }

    pub fn stop(&self) -> Result<(), EspError> {
        esp!(unsafe { rmt_tx_stop(C::channel()) })
    }

    pub fn release(self) -> Result<(P, C), EspError> {
        self.stop()?;
        Ok((self.pin, self.channel))
    }
}

/// Data storage for writer in the format for the RMT driver.
pub trait Data {
    fn as_slice(&self) -> &[rmt_item32_t];
}

/// Stack based storage for RMT pulse data.
///
/// Use this if you know the length of the pulses ahead of time.
///
/// Internally RMT uses pairs of pulses as part of its data structure, so keep things simple
/// in this implementation, you need to `set` a pair of `Pulse`s for each index.
#[derive(Clone)]
pub struct StackPairedData<const N: usize>([rmt_item32_t; N]);

impl<const N: usize> StackPairedData<N> {
    pub fn new() -> Self {
        Self(
            [rmt_item32_t {
                __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                    // Quick way to set all 32 bits to zero, instead of using `__bindgen_anon_1`.
                    val: 0,
                },
            }; N],
        )
    }

    pub fn set(&mut self, index: usize, pair: &(Pulse, Pulse)) -> Result<(), EspError> {
        let item = self
            .0
            .get_mut(index)
            .ok_or(EspError::from(ERANGE as i32).unwrap())?;

        // SAFETY: We're overriding all 32 bits, so it doesn't matter what was here before.
        let inner = unsafe { &mut item.__bindgen_anon_1.__bindgen_anon_1 };
        inner.set_level0(pair.0.pin_state as u32);
        inner.set_duration0(pair.0.ticks.0 as u32);
        inner.set_level1(pair.1.pin_state as u32);
        inner.set_duration1(pair.1.ticks.0 as u32);

        Ok(())
    }
}

impl<const N: usize> Data for StackPairedData<N> {
    fn as_slice(&self) -> &[rmt_item32_t] {
        &self.0
    }
}

// TODO: impl<const N: usize> From<&[Pulse; N]> for StackWriterData<{ (N + 1) / 2 }> {
// Implementing this caused the compiler to crash!

/// `Vec` based storage for RMT pulse data.
///
/// Use this for when you don't know the final size of your data.
#[derive(Clone)]
pub struct VecData {
    items: Vec<rmt_item32_t>,

    // Items contain two pulses. Track if we're adding a new pulse to the first one (true) or if
    // we're changing the second one (false).
    next_item_is_new: bool,
}

impl VecData {
    pub fn new() -> Self {
        Self {
            items: vec![],
            next_item_is_new: true,
        }
    }

    pub fn add<I>(&mut self, pulses: I) -> Result<(), EspError>
    where
        I: IntoIterator<Item = Pulse>,
    {
        for pulse in pulses {
            if self.next_item_is_new {
                let mut inner_item = rmt_item32_t__bindgen_ty_1__bindgen_ty_1::default();
                inner_item.set_level0(pulse.pin_state as u32);
                inner_item.set_duration0(pulse.ticks.0 as u32);
                let item = rmt_item32_t {
                    __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
                        __bindgen_anon_1: inner_item,
                    },
                };
                self.items.push(item);
            } else {
                // There should be at least one item in the vec.
                let len = self.items.len();
                let item = self.items.get_mut(len - 1).unwrap();

                // SAFETY: This item was previously populated with the same union field.
                let inner = unsafe { &mut item.__bindgen_anon_1.__bindgen_anon_1 };

                inner.set_level1(pulse.pin_state as u32);
                inner.set_duration1(pulse.ticks.0 as u32);
            }

            self.next_item_is_new = !self.next_item_is_new;
        }

        Ok(())
    }

    pub fn clear(&mut self) {
        self.next_item_is_new = true;
        self.items.clear();
    }
}

impl Data for VecData {
    fn as_slice(&self) -> &[rmt_item32_t] {
        &self.items
    }
}

mod chip {
    use core::marker::PhantomData;
    use esp_idf_sys::*;

    /// RMT peripheral channel.
    pub trait HwChannel {
        fn channel() -> rmt_channel_t;
    }

    macro_rules! impl_channel {
        ($instance:ident: $channel:expr) => {
            pub struct $instance {
                _marker: PhantomData<rmt_channel_t>,
            }

            impl $instance {
                /// # Safety
                ///
                /// It is safe to instantiate this channel exactly one time.
                pub unsafe fn new() -> Self {
                    $instance {
                        _marker: PhantomData,
                    }
                }
            }

            impl HwChannel for $instance {
                fn channel() -> rmt_channel_t {
                    $channel
                }
            }
        };
    }

    impl_channel!(CHANNEL0: rmt_channel_t_RMT_CHANNEL_0);
    impl_channel!(CHANNEL1: rmt_channel_t_RMT_CHANNEL_1);
    impl_channel!(CHANNEL2: rmt_channel_t_RMT_CHANNEL_2);
    impl_channel!(CHANNEL3: rmt_channel_t_RMT_CHANNEL_3);

    pub struct Peripheral {
        pub channel0: CHANNEL0,
        pub channel1: CHANNEL1,
        pub channel2: CHANNEL2,
        pub channel3: CHANNEL3,
    }

    impl Peripheral {
        pub unsafe fn new() -> Self {
            Self {
                channel0: CHANNEL0::new(),
                channel1: CHANNEL1::new(),
                channel2: CHANNEL2::new(),
                channel3: CHANNEL3::new(),
            }
        }
    }
}
