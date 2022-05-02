//! Remote Control (RMT) module driver.
//!
//! The RMT (Remote Control) module driver can be used to send infrared remote control
//! signals. Due to flexibility of RMT module, the driver can also be used to generate or receive
//! many other types of signals.
//!
//! This module is an abstraction around the [IDF RMT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
//! implementation. It is recommended to read before using this module.
//!
//! This is implementation currently supports transmission only.
//!
//! Not supported:
//! * Interrupts.
//! * Receiving.
//! * Change of config after initialisation.
//!
//! # Example
//!
//! ```
//! // Prepare the config.
//! let config = TransmitConfig::new().clock_divider(1);
//!
//! // Retrieve the output pin and channel from peripherals.
//! let peripherals = Peripherals::take().unwrap();
//! let pin = peripherals.pins.gpio18.into_output()?;
//! let channel = peripherals.rmt.channel0;
//!
//! // Create an RMT transmitter.
//! let tx = Transmit::new(pin, channel, &config)?;
//!
//! // Prepare signal pulse signal to be sent.
//! let low = Pulse::new(PinState::Low, PulseTicks::new(10)?);
//! let high = Pulse::new(PinState::High, PulseTicks::new(10)?);
//! let mut signal = FixedLengthSignal::<2>::new();
//! signal.set(0, &(low, high))?;
//! signal.set(1, &(high, low))?;
//!
//! // Transmit the signal.
//! tx.start(signal)?;
//!```
//!
//! See the `examples/` folder of this repository for more.
//!
//! # Loading pulses
//! There are two ways of preparing pulse signal. [FixedLengthSignal] and [VariableLengthSignal]. These
//! implement the [Signal] trait.
//!
//! [FixedLengthSignal] lives on the stack and must have the items set in pairs of [Pulse]s. This is
//! due to the internal implementation of RMT, and const generics limitations.
//!
//! [VariableLengthSignal] allows you to use the heap and incrementally add pulse items without knowing the size
//! ahead of time.

extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use crate::gpio::OutputPin;
use crate::units::Hertz;
pub use chip::*;
use config::TransmitConfig;
use core::convert::TryFrom;
use core::time::Duration;
use esp_idf_sys::*;

/// A `Low` (0) or `High` (1) state for a pin.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinState {
    Low,
    High,
}

/// A `Pulse` contains a pin state and a tick count, used in creating a [`Signal`].
///
/// The real time duration of a tick depends on the [`TransmitConfig::clock_divider`] setting.
///
/// You can create a `Pulse` with a [`Duration`] by using [`Pulse::new_with_duration`].
///
/// # Example
/// ```rust
/// # use esp_idf_hal::rmt::PulseTicks;
/// let pulse = Pulse::new(PinState::High, PulseTicks::new(32));
/// ```
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Pulse {
    pub ticks: PulseTicks,
    pub pin_state: PinState,
}

impl Pulse {
    pub fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
        Pulse { pin_state, ticks }
    }

    /// Create a [`Pulse`] using a [`Duration`].
    ///
    /// The ticks frequency, which depends on the clock divider set on the channel within a
    /// [`Transmit`]. To get the frequency for the `ticks_hz` argument, use [`Transmit::counter_clock()`].
    ///
    /// # Example
    /// ```
    /// # use esp_idf_sys::EspError;
    /// # use esp_idf_hal::gpio::Output;
    /// # use esp_idf_hal::rmt::Channel::Channel0;
    /// # fn example() -> Result<(), EspError> {
    /// # let peripherals = Peripherals::take()?;
    /// # let led = peripherals.pins.gpio18.into_output()?;
    /// # let channel = peripherals.rmt.channel0;
    /// # let config = TransmitConfig::new()?;
    /// let tx = Transmit::new(led, channel, &config)?;
    /// let ticks_hz = tx.counter_clock()?;
    /// let pulse = Pulse::new_with_duration(ticks_hz, PinState::High, Duration::from_nanos(500))?;
    /// # }
    /// ```
    pub fn new_with_duration(
        ticks_hz: Hertz,
        pin_state: PinState,
        duration: &Duration,
    ) -> Result<Self, EspError> {
        let ticks = PulseTicks::new_with_duration(ticks_hz, duration)?;
        Ok(Self::new(pin_state, ticks))
    }
}

/// Number of ticks, restricting the range to 0 to 32,767.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767;

    /// Needs to be unsigned 15 bits: 0-32767 inclusive, otherwise an [ESP_ERR_INVALID_ARG] is
    /// returned.
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
    pub fn new_with_duration(ticks_hz: Hertz, duration: &Duration) -> Result<Self, EspError> {
        let ticks = duration_to_ticks(ticks_hz, duration)?;
        let ticks = u16::try_from(ticks).map_err(|_| EspError::from(EOVERFLOW as i32).unwrap())?;
        Self::new(ticks)
    }
}

/// A utility to convert a duration into ticks, depending on the clock ticks.
pub fn duration_to_ticks(ticks_hz: Hertz, duration: &Duration) -> Result<u128, EspError> {
    Ok(duration
        .as_nanos()
        .checked_mul(u32::from(ticks_hz) as u128)
        .ok_or_else(|| EspError::from(EOVERFLOW as i32).unwrap())?
        / 1_000_000_000)
}

/// Types used for configuring the [`rmt`][crate::rmt] module.
///
/// [`TransmitConfig`] is used when creating a [`Transmit`][crate::rmt::Transmit] instance.
///
/// # Example
/// ```
/// # use esp_idf_hal::units::FromValueType;
/// let carrier = CarrierConfig::new()
///     .duty_percent(DutyPercent::new(50)?)
///     .frequency(611.Hz());
///
/// let config = TransmitConfig::new()
///     .carrier(Some(carrier))
///     .looping(Loop::Endless)
///     .clock_divider(255);
///
/// ```
pub mod config {
    use super::PinState;
    use crate::units::{FromValueType, Hertz};
    use esp_idf_sys::{EspError, ESP_ERR_INVALID_ARG};

    /// A percentage from 0 to 100%, used to specify the duty percentage in [`CarrierConfig`].
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct DutyPercent(pub(super) u8);

    impl DutyPercent {
        /// Must be between 0 and 100, otherwise an error is returned.
        pub fn new(v: u8) -> Result<Self, EspError> {
            if v > 100 {
                Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
            } else {
                Ok(Self(v))
            }
        }
    }

    /// Configuration for when enabling a carrier frequency.
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct CarrierConfig {
        /// Frequency of the carrier in Hz.
        pub frequency: Hertz,

        /// Level of the RMT output, when the carrier is applied.
        pub carrier_level: PinState,

        /// Duty cycle of the carrier signal in percent (%).
        pub duty_percent: DutyPercent,
    }

    impl Default for CarrierConfig {
        /// Defaults from `<https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101>`
        fn default() -> Self {
            Self {
                frequency: 38.kHz().into(),
                carrier_level: PinState::High,
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

        pub fn duty_percent(mut self, duty: DutyPercent) -> Self {
            self.duty_percent = duty;
            self
        }
    }

    /// Configuration setting for looping a signal.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum Loop {
        None,
        Endless,
        #[cfg(not(any(esp32, esp32c2)))]
        Count(u32),
    }

    /// Used when creating a [`Transmit`][crate::rmt::Transmit] instance.
    pub struct TransmitConfig {
        pub clock_divider: u8,
        pub mem_block_num: u8,
        pub carrier: Option<CarrierConfig>,
        // TODO: `loop` is taken. Maybe can change to `repeat` even though it doesn't match the IDF.
        pub looping: Loop,

        /// Enable and set the signal level on the output if idle.
        pub idle: Option<PinState>,

        /// Channel can work during APB clock scaling.
        ///
        /// When set, RMT channel will take REF_TICK or XTAL as source clock. The benefit is, RMT
        /// channel can continue work even when APB clock is changing.
        pub aware_dfs: bool,
    }

    impl Default for TransmitConfig {
        /// Defaults from `<https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101>`
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

    impl TransmitConfig {
        pub fn new() -> Self {
            Self::default()
        }

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

/// The RMT transmitter.
///
/// Use [`Transmit::start()`] or [`Transmit::start_blocking()`] to transmit pulses.
///
/// See the [rmt module][crate::rmt] for more information.
pub struct Transmit<P: OutputPin, C: HwChannel> {
    pin: P,
    channel: C,
}

impl<P: OutputPin, C: HwChannel> Transmit<P, C> {
    /// Initialise the rmt module with the specified pin, channel and configuration.
    ///
    /// To uninstall the driver and return ownership of the `channel` and `pin` use
    /// [`Transmit::release()`].
    ///
    /// Internally this calls `rmt_config()` and `rmt_driver_install()`.
    pub fn new(pin: P, channel: C, config: &TransmitConfig) -> Result<Self, EspError> {
        let mut flags = 0;
        if config.aware_dfs {
            flags |= RMT_CHANNEL_FLAGS_AWARE_DFS;
        }

        let carrier_en = config.carrier.is_some();
        let carrier = config.carrier.unwrap_or_default();

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
                    loop_en: config.looping != config::Loop::None,
                    #[cfg(not(any(esp32, esp32c2)))]
                    loop_count: match config.looping {
                        config::Loop::Count(count) if count > 0 && count < 1024 => count,
                        _ => 0,
                    },
                },
            },
        };

        unsafe {
            esp!(rmt_config(&sys_config))?;
            esp!(rmt_driver_install(C::channel(), 0, 0))?;
        }

        Ok(Self { pin, channel })
    }

    /// Get speed of the channel’s internal counter clock.
    ///
    /// This calls [rmt_get_counter_clock()][rmt_get_counter_clock]
    /// internally. It is used for calculating the number of ticks per second for pulses.
    ///
    /// See [Pulse::new_with_duration()].
    ///
    /// [rmt_get_counter_clock]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html#_CPPv421rmt_get_counter_clock13rmt_channel_tP8uint32_t
    pub fn counter_clock(&self) -> Result<Hertz, EspError> {
        let mut ticks_hz: u32 = 0;
        esp!(unsafe { rmt_get_counter_clock(C::channel(), &mut ticks_hz) })?;
        Ok(ticks_hz.into())
    }

    /// Start sending the given signal without blocking.
    ///
    /// `signal` is captured for safety so that the user can't change the data while transmitting.
    pub fn start<S>(&mut self, signal: S) -> Result<(), EspError>
    where
        S: Signal,
    {
        self.write_items(&signal, false)
    }

    /// Start sending the given signal while blocking.
    pub fn start_blocking<S>(&mut self, signal: &S) -> Result<(), EspError>
    where
        S: Signal,
    {
        self.write_items(signal, true)
    }

    fn write_items<S>(&mut self, signal: &S, block: bool) -> Result<(), EspError>
    where
        S: Signal,
    {
        let items = signal.as_slice();
        esp!(unsafe { rmt_write_items(C::channel(), items.as_ptr(), items.len() as i32, block,) })
    }

    /// Stop transmitting.
    pub fn stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_tx_stop(C::channel()) })
    }

    /// Stop transmitting and release the driver.
    ///
    /// This will return the pin and channel.
    pub fn release(mut self) -> Result<(P, C), EspError> {
        self.stop()?;
        esp!(unsafe { rmt_driver_uninstall(C::channel()) })?;
        Ok((self.pin, self.channel))
    }

    pub fn set_looping(&mut self, looping: config::Loop) -> Result<(), EspError> {
        esp!(unsafe { rmt_set_tx_loop_mode(C::channel(), looping != config::Loop::None) })?;

        #[cfg(not(any(esp32, esp32c2)))]
        esp!(unsafe {
            rmt_set_tx_loop_count(
                C::channel(),
                match looping {
                    config::Loop::Count(count) if count > 0 && count < 1024 => count,
                    _ => 0,
                },
            )
        })?;

        Ok(())
    }
}

/// Signal storage for [`Transmit`] in a format ready for the RMT driver.
pub trait Signal {
    fn as_slice(&self) -> &[rmt_item32_t];
}

/// Stack based signal storage for an RMT signal.
///
/// Use this if you know the length of the pulses ahead of time and prefer to use the stack.
///
/// Internally RMT uses pairs of pulses as part of its data structure. This implementation
/// you need to [`set`][FixedLengthSignal::set()] a two [`Pulse`]s for each index.
///
/// ```rust
/// # use esp_idf_hal::rmt::FixedLengthSignal;
/// let p1 = Pulse::new(PinState::High, PulseTicks::new(10));
/// let p2 = Pulse::new(PinState::Low, PulseTicks::new(11));
/// let p3 = Pulse::new(PinState::High, PulseTicks::new(12));
/// let p4 = Pulse::new(PinState::Low, PulseTicks::new(13));
///
/// let mut s = FixedLengthSignal::new();
/// s.set(0, &(p1, p2));
/// s.set(1, &(p3, p4));
/// ```
#[derive(Clone)]
pub struct FixedLengthSignal<const N: usize>([rmt_item32_t; N]);

#[cfg(all(esp_idf_version_major = "4", esp32))]
#[allow(non_camel_case_types)]
type rmt_item32_t__bindgen_ty_1 = rmt_item32_s__bindgen_ty_1;
#[cfg(all(esp_idf_version_major = "4", esp32))]
#[allow(non_camel_case_types)]
#[allow(dead_code)]
type rmt_item32_t__bindgen_ty_1__bindgen_ty_1 = rmt_item32_s__bindgen_ty_1__bindgen_ty_1;

impl<const N: usize> FixedLengthSignal<N> {
    /// Creates a new array of size `<N>`, where the number of pulses is `N * 2`.
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

    /// Set a pair of [`Pulse`]s at a position in the array.
    pub fn set(&mut self, index: usize, pair: &(Pulse, Pulse)) -> Result<(), EspError> {
        let item = self
            .0
            .get_mut(index)
            .ok_or_else(|| EspError::from(ERANGE as i32).unwrap())?;

        // SAFETY: We're overriding all 32 bits, so it doesn't matter what was here before.
        let inner = unsafe { &mut item.__bindgen_anon_1.__bindgen_anon_1 };
        inner.set_level0(pair.0.pin_state as u32);
        inner.set_duration0(pair.0.ticks.0 as u32);
        inner.set_level1(pair.1.pin_state as u32);
        inner.set_duration1(pair.1.ticks.0 as u32);

        Ok(())
    }
}

impl<const N: usize> Signal for FixedLengthSignal<N> {
    fn as_slice(&self) -> &[rmt_item32_t] {
        &self.0
    }
}

impl<const N: usize> Default for FixedLengthSignal<N> {
    fn default() -> Self {
        Self::new()
    }
}

// TODO: impl<const N: usize> From<&[Pulse; N]> for StackSignal<{ (N + 1) / 2 }> {
// Implementing this caused the compiler to crash!

/// `Vec` heap based storage for an RMT signal.
///
/// Use this for when you don't know the final size of your signal data.
///
/// # Example
/// ```rust
/// let mut signal = VariableLengthSignal::new();
/// signal.push(Pulse::new(PinState::High, PulseTicks::new(10)));
/// signal.push(Pulse::new(PinState::Low, PulseTicks::new(9)));
/// ```

#[derive(Clone, Default)]
#[cfg(feature = "alloc")]
pub struct VariableLengthSignal {
    items: Vec<rmt_item32_t>,

    // Items contain two pulses. Track if we're adding a new pulse to the first one (true) or if
    // we're changing the second one (false).
    next_item_is_new: bool,
}

#[cfg(feature = "alloc")]
impl VariableLengthSignal {
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
            next_item_is_new: true,
        }
    }

    /// Add [`Pulse`]s to the end of the signal.
    pub fn push<'p, I>(&mut self, pulses: I) -> Result<(), EspError>
    where
        I: IntoIterator<Item = &'p Pulse>,
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

    /// Delete all pulses.
    pub fn clear(&mut self) {
        self.next_item_is_new = true;
        self.items.clear();
    }
}

#[cfg(feature = "alloc")]
impl Signal for VariableLengthSignal {
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

    // SOC_RMT_CHANNELS_PER_GROUP defines how many channels there are.

    impl_channel!(CHANNEL0: rmt_channel_t_RMT_CHANNEL_0);
    impl_channel!(CHANNEL1: rmt_channel_t_RMT_CHANNEL_1);
    impl_channel!(CHANNEL2: rmt_channel_t_RMT_CHANNEL_2);
    impl_channel!(CHANNEL3: rmt_channel_t_RMT_CHANNEL_3);
    #[cfg(any(esp32, esp32s3))]
    impl_channel!(CHANNEL4: rmt_channel_t_RMT_CHANNEL_4);
    #[cfg(any(esp32, esp32s3))]
    impl_channel!(CHANNEL5: rmt_channel_t_RMT_CHANNEL_5);
    #[cfg(any(esp32, esp32s3))]
    impl_channel!(CHANNEL6: rmt_channel_t_RMT_CHANNEL_6);
    #[cfg(any(esp32, esp32s3))]
    impl_channel!(CHANNEL7: rmt_channel_t_RMT_CHANNEL_7);

    pub struct Peripheral {
        pub channel0: CHANNEL0,
        pub channel1: CHANNEL1,
        pub channel2: CHANNEL2,
        pub channel3: CHANNEL3,
        #[cfg(any(esp32, esp32s3))]
        pub channel4: CHANNEL4,
        #[cfg(any(esp32, esp32s3))]
        pub channel5: CHANNEL5,
        #[cfg(any(esp32, esp32s3))]
        pub channel6: CHANNEL6,
        #[cfg(any(esp32, esp32s3))]
        pub channel7: CHANNEL7,
    }

    impl Peripheral {
        /// Creates a new instance of the RMT peripheral. Typically one wants
        /// to use the instance [`rmt`](crate::peripherals::Peripherals::rmt) from
        /// the device peripherals obtained via
        /// [`peripherals::Peripherals::take()`](crate::peripherals::Peripherals::take()).
        ///
        /// # Safety
        ///
        /// It is safe to instantiate the RMT peripheral exactly one time.
        /// Care has to be taken that this has not already been done elsewhere.
        pub unsafe fn new() -> Self {
            Self {
                channel0: CHANNEL0::new(),
                channel1: CHANNEL1::new(),
                channel2: CHANNEL2::new(),
                channel3: CHANNEL3::new(),
                #[cfg(any(esp32, esp32s3))]
                channel4: CHANNEL4::new(),
                #[cfg(any(esp32, esp32s3))]
                channel5: CHANNEL5::new(),
                #[cfg(any(esp32, esp32s3))]
                channel6: CHANNEL6::new(),
                #[cfg(any(esp32, esp32s3))]
                channel7: CHANNEL7::new(),
            }
        }
    }
}
