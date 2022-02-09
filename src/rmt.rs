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

use crate::gpio::OutputPin;
use config::WriterConfig;
use core::cell::RefCell;
use core::convert::TryFrom;
use core::mem::ManuallyDrop;
use core::time::Duration;
use embedded_hal::digital::PinState;
use esp_idf_sys::{
    esp, rmt_channel_t_RMT_CHANNEL_0, rmt_channel_t_RMT_CHANNEL_1, rmt_channel_t_RMT_CHANNEL_2,
    rmt_channel_t_RMT_CHANNEL_3, rmt_config, rmt_config_t, rmt_config_t__bindgen_ty_1,
    rmt_driver_install, rmt_get_counter_clock, rmt_item32_t, rmt_item32_t__bindgen_ty_1,
    rmt_item32_t__bindgen_ty_1__bindgen_ty_1, rmt_mode_t_RMT_MODE_TX, rmt_tx_config_t,
    rmt_write_items, EspError, EOVERFLOW, ESP_ERR_INVALID_ARG, RMT_CHANNEL_FLAGS_AWARE_DFS,
};

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Channel {
    // TODO: Work out the different number of channels per chip.
    Channel0,
    Channel1,
    Channel2,
    Channel3,
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
        ticks_hz: u32,
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
    pub fn new_with_duration(ticks_hz: u32, duration: Duration) -> Result<Self, EspError> {
        let ticks = duration
            .as_nanos()
            .checked_mul(ticks_hz as u128)
            .ok_or(EspError::from(EOVERFLOW as i32).unwrap())?
            / 1_000_000_000;
        let ticks = u16::try_from(ticks).map_err(|_| EspError::from(EOVERFLOW as i32).unwrap())?;

        Self::new(ticks)
    }
}

pub mod config {
    use embedded_hal::digital::PinState;

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

        Ok(Self { channel })
    }

    pub fn counter_clock(&self) -> Result<u32, EspError> {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(self.channel as u32, &mut ticks_hz) })?;
        Ok(ticks_hz)
    }

    /// Start sending the pulses.
    pub fn start<D>(&mut self, data: D) -> Result<(), EspError>
    where
        D: Data,
    {
        let items = data.as_slice();
        dbg!(items.as_ptr());
        dbg!(items.len());

        let b: &[u16] =
            unsafe { ::std::slice::from_raw_parts(items.as_ptr() as *const u16, items.len() * 4) };
        dbg!(b);

        esp!(unsafe {
            rmt_write_items(
                self.channel as u32,
                items.as_ptr(),
                items.len() as i32,
                true, // TODO: Blocking.
            )
        })
    }

    pub fn stop(&self) -> Result<(), EspError> {
        todo!()
    }
}

fn pulses_to_internal<const N: usize>(s: [Pulse; N]) -> [rmt_item32_t; (N + 1) / 2] {
    todo!()
}

pub trait Data {
    fn as_slice(&self) -> &[rmt_item32_t];
}

pub struct VecData {
    items: Vec<rmt_item32_t>,
    next_item_is_new: bool,
}

struct VecDataInner {}

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

pub struct StackWriterData<const N: usize>([rmt_item32_t; N]);

impl<const N: usize> Data for StackWriterData<N> {
    fn as_slice(&self) -> &[rmt_item32_t] {
        self.0.as_slice()
    }
}

// pub fn pulses_to_data<const N: usize>(s: &[Pulse; N]) -> StackWriterData<{ (N + 1) / 2 }> {
//     todo!()
// }
//
// impl<const N: usize> From<&[Pulse; N]> for StackWriterData<{ (N + 1) / 2 }> {
//     fn from(pulses: &[Pulse; N]) -> Self {
//         let slice = [rmt_item32_t {
//             __bindgen_anon_1: rmt_item32_t__bindgen_ty_1 {
//                 __bindgen_anon_1: rmt_item32_t__bindgen_ty_1__bindgen_ty_1::default(),
//             },
//         }; (N + 1) / 2];
//         StackWriterData(slice)
//     }
// }
//
// fn brainstorming() {
//     let pulse = Pulse::new(PinState::High, PulseTicks::max());
//     //     // If from slice is even, /2. If odd, /2 + 1
//     //     let wd: WriterData<1> = WriterData::from_slice(&[pulse]);
//     //     let wd: WriterData<2> = WriterData::from_slice(&[pulse, pulse]);
//     //     let wd: WriterData<3> = WriterData::from_slice(&[pulse, pulse, pulse]);
//     //     let wd: WriterData<4> = WriterData::from_slice(&[pulse, pulse, pulse, pulse]);
//
//     let a: [rmt_item32_t; 0] = pulses_to_internal([]);
//     let a: [rmt_item32_t; 1] = pulses_to_internal([pulse]);
//     let a: [rmt_item32_t; 1] = pulses_to_internal([pulse, pulse]);
//     let a: [rmt_item32_t; 2] = pulses_to_internal([pulse, pulse, pulse]);
//     let a: [rmt_item32_t; 2] = pulses_to_internal([pulse, pulse, pulse, pulse]);
//
//     let a: StackWriterData<1> = pulses_to_data(&[pulse]);
//     let a: StackWriterData<1> = pulses_to_data(&[pulse, pulse]);
//     let a: StackWriterData<2> = pulses_to_data(&[pulse, pulse, pulse]);
//     let a: StackWriterData<2> = pulses_to_data(&[pulse, pulse, pulse, pulse]);
//
//     let _: StackWriterData<1> = (&[pulse]).into();
//     let _: StackWriterData<1> = (&[pulse, pulse]).into();
//     let _: StackWriterData<2> = (&[pulse, pulse, pulse]).into();
// }
