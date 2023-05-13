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
//! let config = Config::new().clock_divider(1);
//!
//! // Retrieve the output pin and channel from peripherals.
//! let peripherals = Peripherals::take().unwrap();
//! let channel = peripherals.rmt.channel0;
//! let pin = peripherals.pins.gpio18;
//!
//! // Create an RMT transmitter.
//! let tx = TxRmtDriver::new(channel, pin, &config)?;
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

use core::cell::UnsafeCell;
use core::convert::{TryFrom, TryInto};
use core::marker::PhantomData;
use core::ptr;
use core::time::Duration;

#[cfg(feature = "alloc")]
extern crate alloc;

use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::gpio::OutputPin;
use crate::interrupt::IntrFlags;
use crate::peripheral::Peripheral;
use crate::units::Hertz;

use config::ReceiveConfig;
use config::TransmitConfig;

pub use chip::*;

// Might not always be available in the generated `esp-idf-sys` bindings
const ERR_ERANGE: esp_err_t = 34;
const ERR_EOVERFLOW: esp_err_t = 139;

pub type RmtTransmitConfig = config::TransmitConfig;
pub type RmtReceiveConfig = config::ReceiveConfig;

/// A `Low` (0) or `High` (1) state for a pin.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinState {
    Low,
    High,
}

impl From<u32> for PinState {
    fn from(state: u32) -> Self {
        if state == 0 {
            Self::Low
        } else {
            Self::High
        }
    }
}

/// A `Pulse` contains a pin state and a tick count, used in creating a [`Signal`].
///
/// The real time duration of a tick depends on the [`Config::clock_divider`] setting.
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
    pub const fn zero() -> Self {
        Self::new(PinState::Low, PulseTicks::zero())
    }

    pub const fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
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
    /// # let config = Config::new()?;
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

impl Default for Pulse {
    fn default() -> Self {
        Self::zero()
    }
}

/// Number of ticks, restricting the range to 0 to 32,767.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767;

    pub const fn zero() -> Self {
        Self(0)
    }

    /// Use the maximum value of 32767.
    pub const fn max() -> Self {
        Self(Self::MAX)
    }

    /// Needs to be unsigned 15 bits: 0-32767 inclusive, otherwise an [ESP_ERR_INVALID_ARG] is
    /// returned.
    pub fn new(ticks: u16) -> Result<Self, EspError> {
        if ticks > Self::MAX {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
        } else {
            Ok(Self(ticks))
        }
    }

    /// Convert a `Duration` into `PulseTicks`.
    ///
    /// See `Pulse::new_with_duration()` for details.
    pub fn new_with_duration(ticks_hz: Hertz, duration: &Duration) -> Result<Self, EspError> {
        Self::new(duration_to_ticks(ticks_hz, duration)?)
    }

    pub fn ticks(&self) -> u16 {
        self.0
    }

    pub fn duration(&self, ticks_hz: Hertz) -> Result<Duration, EspError> {
        ticks_to_duration(ticks_hz, self.ticks())
    }
}

impl Default for PulseTicks {
    fn default() -> Self {
        Self::zero()
    }
}

/// A utility to convert a duration into ticks, depending on the clock ticks.
pub fn duration_to_ticks(ticks_hz: Hertz, duration: &Duration) -> Result<u16, EspError> {
    let ticks = duration
        .as_nanos()
        .checked_mul(u32::from(ticks_hz) as u128)
        .ok_or_else(|| EspError::from(ERR_EOVERFLOW).unwrap())?
        / 1_000_000_000;

    u16::try_from(ticks).map_err(|_| EspError::from(ERR_EOVERFLOW).unwrap())
}

/// A utility to convert ticks into duration, depending on the clock ticks.
pub fn ticks_to_duration(ticks_hz: Hertz, ticks: u16) -> Result<Duration, EspError> {
    let duration = 1_000_000_000_u128
        .checked_mul(ticks as u128)
        .ok_or_else(|| EspError::from(ERR_EOVERFLOW).unwrap())?
        / u32::from(ticks_hz) as u128;

    u64::try_from(duration)
        .map(Duration::from_nanos)
        .map_err(|_| EspError::from(ERR_EOVERFLOW).unwrap())
}

pub type TxRmtConfig = config::TransmitConfig;
pub type RxRmtConfig = config::ReceiveConfig;

/// Types used for configuring the [`rmt`][crate::rmt] module.
///
/// [`Config`] is used when creating a [`Transmit`][crate::rmt::Transmit] instance.
///
/// # Example
/// ```
/// # use esp_idf_hal::units::FromValueType;
/// let carrier = CarrierConfig::new()
///     .duty_percent(DutyPercent::new(50)?)
///     .frequency(611.Hz());
///
/// let config = Config::new()
///     .carrier(Some(carrier))
///     .looping(Loop::Endless)
///     .clock_divider(255);
///
/// ```
pub mod config {
    use enumset::EnumSet;
    use esp_idf_sys::{EspError, ESP_ERR_INVALID_ARG};

    use super::PinState;
    use crate::{
        interrupt::IntrFlags,
        units::{FromValueType, Hertz},
    };

    /// A percentage from 0 to 100%, used to specify the duty percentage in [`CarrierConfig`].
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct DutyPercent(pub(super) u8);

    impl DutyPercent {
        /// Must be between 0 and 100, otherwise an error is returned.
        pub fn new(v: u8) -> Result<Self, EspError> {
            if v > 100 {
                Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
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

    impl CarrierConfig {
        pub fn new() -> Self {
            Self {
                frequency: 38.kHz().into(),
                carrier_level: PinState::High,
                duty_percent: DutyPercent(33),
            }
        }

        #[must_use]
        pub fn frequency(mut self, hz: Hertz) -> Self {
            self.frequency = hz;
            self
        }

        #[must_use]
        pub fn carrier_level(mut self, state: PinState) -> Self {
            self.carrier_level = state;
            self
        }

        #[must_use]
        pub fn duty_percent(mut self, duty: DutyPercent) -> Self {
            self.duty_percent = duty;
            self
        }
    }

    impl Default for CarrierConfig {
        /// Defaults from `<https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101>`
        fn default() -> Self {
            Self::new()
        }
    }

    /// Configuration setting for looping a signal.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum Loop {
        None,
        Endless,
        #[cfg(any(
            all(not(esp_idf_version_major = "4"), not(esp_idf_version_major = "5")),
            all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")),
            not(esp32)
        ))]
        Count(u32),
    }

    /// Used when creating a [`Transmit`][crate::rmt::Transmit] instance.
    #[derive(Debug, Clone)]
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

        pub intr_flags: EnumSet<IntrFlags>,
    }

    impl TransmitConfig {
        pub fn new() -> Self {
            Self {
                aware_dfs: false,
                mem_block_num: 1,
                clock_divider: 80,
                looping: Loop::None,
                carrier: None,
                idle: Some(PinState::Low),
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }

        #[must_use]
        pub fn aware_dfs(mut self, enable: bool) -> Self {
            self.aware_dfs = enable;
            self
        }

        #[must_use]
        pub fn mem_block_num(mut self, mem_block_num: u8) -> Self {
            self.mem_block_num = mem_block_num;
            self
        }

        #[must_use]
        pub fn clock_divider(mut self, divider: u8) -> Self {
            self.clock_divider = divider;
            self
        }

        #[must_use]
        pub fn looping(mut self, looping: Loop) -> Self {
            self.looping = looping;
            self
        }

        #[must_use]
        pub fn carrier(mut self, carrier: Option<CarrierConfig>) -> Self {
            self.carrier = carrier;
            self
        }

        #[must_use]
        pub fn idle(mut self, idle: Option<PinState>) -> Self {
            self.idle = idle;
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = flags;
            self
        }
    }

    impl Default for TransmitConfig {
        /// Defaults from `<https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L101>`
        fn default() -> Self {
            Self::new()
        }
    }

    /// Used when creating a [`Receive`][crate::rmt::Receive] instance.
    #[derive(Debug, Clone)]
    pub struct ReceiveConfig {
        pub clock_divider: u8,
        pub mem_block_num: u8,
        pub idle_threshold: u16,
        pub filter_ticks_thresh: u8,
        pub filter_en: bool,
        pub carrier: Option<CarrierConfig>,
        pub intr_flags: EnumSet<IntrFlags>,
    }

    impl ReceiveConfig {
        pub fn new() -> Self {
            Self::default()
        }

        #[must_use]
        pub fn clock_divider(mut self, divider: u8) -> Self {
            self.clock_divider = divider;
            self
        }

        #[must_use]
        pub fn mem_block_num(mut self, mem_block_num: u8) -> Self {
            self.mem_block_num = mem_block_num;
            self
        }

        #[must_use]
        pub fn idle_threshold(mut self, threshold: u16) -> Self {
            self.idle_threshold = threshold;
            self
        }

        #[must_use]
        pub fn filter_ticks_thresh(mut self, threshold: u8) -> Self {
            self.filter_ticks_thresh = threshold;
            self
        }

        #[must_use]
        pub fn filter_en(mut self, enable: bool) -> Self {
            self.filter_en = enable;
            self
        }

        #[must_use]
        pub fn carrier(mut self, carrier: Option<CarrierConfig>) -> Self {
            self.carrier = carrier;
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = flags;
            self
        }
    }

    impl Default for ReceiveConfig {
        /// Defaults from `<https://github.com/espressif/esp-idf/blob/master/components/driver/include/driver/rmt.h#L110>`
        fn default() -> Self {
            Self {
                clock_divider: 80,        // one microsecond clock period
                mem_block_num: 1, // maximum of 448 rmt items can be captured (mem_block_num=0 will have max 512 rmt items)
                idle_threshold: 12000, // 1.2 milliseconds, pulse greater than this will generate interrupt
                filter_ticks_thresh: 100, // 100 microseconds, pulses less than this will be ignored
                filter_en: true,
                carrier: None,
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }
    }
}

/// The RMT transmitter driver.
///
/// Use [`TxRmtDriver::start()`] or [`TxRmtDriver::start_blocking()`] to transmit pulses.
///
/// See the [rmt module][crate::rmt] for more information.

pub struct TxRmtDriver<'d> {
    channel: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TxRmtDriver<'d> {
    /// Initialise the rmt module with the specified pin, channel and configuration.
    ///
    /// To uninstall the driver just drop it.
    ///
    /// Internally this calls `rmt_config()` and `rmt_driver_install()`.
    pub fn new<C: RmtChannel>(
        _channel: impl Peripheral<P = C> + 'd,
        pin: impl Peripheral<P = impl OutputPin> + 'd,
        config: &TransmitConfig,
    ) -> Result<Self, EspError> {
        crate::into_ref!(pin);

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
                    #[cfg(any(
                        all(not(esp_idf_version_major = "4"), not(esp_idf_version_major = "5")),
                        all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")),
                        not(esp32)
                    ))]
                    loop_count: match config.looping {
                        config::Loop::Count(count) if count > 0 && count < 1024 => count,
                        _ => 0,
                    },
                },
            },
        };

        unsafe {
            esp!(rmt_config(&sys_config))?;
            esp!(rmt_driver_install(
                C::channel(),
                0,
                IntrFlags::to_native(config.intr_flags) as _
            ))?;
        }

        Ok(Self {
            channel: C::channel() as _,
            _p: PhantomData,
        })
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
        esp!(unsafe { rmt_get_counter_clock(self.channel(), &mut ticks_hz) })?;
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
        esp!(unsafe { rmt_write_items(self.channel(), items.as_ptr(), items.len() as i32, block) })
    }

    /// Transmit all items in `iter` without blocking.
    ///
    /// Note that this requires `iter` to be [`Box`]ed for an allocation free version see [`Self::start_iter_blocking`].
    ///
    /// ### Warning
    ///
    /// Iteration of `iter` happens inside an interrupt handler so beware of side-effects
    /// that don't work in interrupt handlers. Iteration must also be fast so that there
    /// are no time-gaps between successive transmissions where the perhipheral has to
    /// wait for items. This can cause weird behavior and can be counteracted with
    /// increasing [`Config::mem_block_num`] or making iteration more efficient.
    #[cfg(feature = "alloc")]
    pub fn start_iter<T>(&mut self, iter: T) -> Result<(), EspError>
    where
        T: Iterator<Item = rmt_item32_t> + Send + 'static,
    {
        let iter = alloc::boxed::Box::new(UnsafeCell::new(iter));
        unsafe {
            esp!(rmt_translator_init(
                self.channel(),
                Some(Self::translate_iterator::<T, true>),
            ))?;

            esp!(rmt_write_sample(
                self.channel(),
                alloc::boxed::Box::leak(iter) as *const _ as _,
                1,
                false
            ))
        }
    }

    /// Transmit all items in `iter`, blocking until all items are transmitted.
    ///
    /// This method does not require any allocations since the thread is paused until all
    /// items are transmitted. The iterator lives on the stack and will be dropped after
    /// all items are written and before this method returns.
    ///
    /// ### Warning
    ///
    /// Iteration of `iter` happens inside an interrupt handler so beware of side-effects
    /// that don't work in interrupt handlers. Iteration must also be fast so that there
    /// are no time-gaps between successive transmissions where the perhipheral has to
    /// wait for items. This can cause weird behavior and can be counteracted with
    /// increasing [`Config::mem_block_num`] or making iteration more efficient.

    pub fn start_iter_blocking<T>(&mut self, iter: T) -> Result<(), EspError>
    where
        T: Iterator<Item = rmt_item32_t> + Send,
    {
        let iter = UnsafeCell::new(iter);
        unsafe {
            // TODO: maybe use a separate struct so that we don't have to do this when
            // transmitting the same iterator type.
            esp!(rmt_translator_init(
                self.channel(),
                Some(Self::translate_iterator::<T, false>),
            ))?;
            esp!(rmt_write_sample(
                self.channel(),
                &iter as *const _ as _,
                24,
                true
            ))
        }
    }

    /// The translator that turns an iterator into `rmt_item32_t` elements. Most of the
    /// magic happens here.
    ///
    /// The general idea is that we can fill a buffer (`dest`) of `rmt_item32_t` items of
    /// length `wanted_num` with the items that we get from the iterator. Then we can tell
    /// the peripheral driver how many items we filled in by setting `item_num`. The
    /// driver will call this function over-and-over until `translated_size` is equal to
    /// `src_size` so when the iterator returns [`None`] we set `translated_size` to
    /// `src_size` to signal that there are no more items to translate.
    ///
    /// The compiler will generate this function for every different call to
    /// [`Self::start_iter_blocking`] and [`Self::start_iter`] with different iterator
    /// types because of the type parameter. This is done to avoid the double indirection
    /// that we'd have to do when using a trait object since references to trait objects
    /// are fat-pointers (2 `usize` wide) and we only get a narrow pointer (`src`).
    /// Using a trait object has the addional overhead that every call to `Iterator::next`
    /// would also be indirect (through the `vtable`) and couldn't be inlined.
    unsafe extern "C" fn translate_iterator<T, const DEALLOC_ITER: bool>(
        src: *const core::ffi::c_void,
        mut dest: *mut rmt_item32_t,
        src_size: usize,
        wanted_num: usize,
        translated_size: *mut usize,
        item_num: *mut usize,
    ) where
        T: Iterator<Item = rmt_item32_t>,
    {
        // An `UnsafeCell` is needed here because we're casting a `*const` to a `*mut`.
        // Safe because this is the only existing reference.
        let iter = &mut *UnsafeCell::raw_get(src as *const UnsafeCell<T>);

        let mut i = 0;
        let finished = loop {
            if i >= wanted_num {
                break 0;
            }

            if let Some(item) = iter.next() {
                *dest = item;
                dest = dest.add(1);
                i += 1;
            } else {
                // Only deallocate the iter if the const generics argument is `true`
                // otherwise we could be deallocating stack memory.
                #[cfg(feature = "alloc")]
                if DEALLOC_ITER {
                    drop(alloc::boxed::Box::from_raw(iter));
                }
                break src_size;
            }
        };

        *item_num = i;
        *translated_size = finished;
    }

    /// Stop transmitting.
    pub fn stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_tx_stop(self.channel()) })
    }

    pub fn set_looping(&mut self, looping: config::Loop) -> Result<(), EspError> {
        esp!(unsafe { rmt_set_tx_loop_mode(self.channel(), looping != config::Loop::None) })?;

        #[cfg(not(any(esp32, esp32c2)))]
        esp!(unsafe {
            rmt_set_tx_loop_count(
                self.channel(),
                match looping {
                    config::Loop::Count(count) if count > 0 && count < 1024 => count,
                    _ => 0,
                },
            )
        })?;

        Ok(())
    }

    pub fn channel(&self) -> rmt_channel_t {
        self.channel as _
    }
}

impl<'d> Drop for TxRmtDriver<'d> {
    /// Stop transmitting and release the driver.
    fn drop(&mut self) {
        self.stop().unwrap();
        esp!(unsafe { rmt_driver_uninstall(self.channel()) }).unwrap();
    }
}

unsafe impl<'d> Send for TxRmtDriver<'d> {}

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
            .ok_or_else(|| EspError::from(ERR_ERANGE).unwrap())?;

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

#[cfg(feature = "alloc")]
#[derive(Clone, Default)]
pub struct VariableLengthSignal {
    items: alloc::vec::Vec<rmt_item32_t>,

    // Items contain two pulses. Track if we're adding a new pulse to the first one (true) or if
    // we're changing the second one (false).
    next_item_is_new: bool,
}

#[cfg(feature = "alloc")]
impl VariableLengthSignal {
    pub const fn new() -> Self {
        Self {
            items: alloc::vec::Vec::new(),
            next_item_is_new: true,
        }
    }

    /// Create a new [`VariableLengthSignal`] with a a given capacity. This is
    /// more efficent than not specifying the capacity with `new( )` as the
    /// memory manager only needs to allocate the underlying array once.
    ///
    /// - `capacity` is the number of [`Pulse`]s which can be pushes before reallocating
    pub fn with_capacity(capacity: usize) -> Self {
        // half the size, rounding up, because each entry in the [`Vec`] holds upto 2 pulses each
        let vec_size = (capacity + 1) / 2;
        Self {
            items: alloc::vec::Vec::with_capacity(vec_size),
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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Receive {
    Read(usize),
    Overflow(usize),
    Timeout,
}

/// The RMT receiver.
///
/// Use [`RxRmtDriver::start()`] to receive pulses.
///
/// See the [rmt module][crate::rmt] for more information.
pub struct RxRmtDriver<'d> {
    channel: u8,
    next_ringbuf_item: Option<(*mut rmt_item32_t, usize)>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> RxRmtDriver<'d> {
    /// Initialise the rmt module with the specified pin, channel and configuration.
    ///
    /// To uninstall the driver just drop it.
    ///
    /// Internally this calls `rmt_config()` and `rmt_driver_install()`.

    pub fn new<C: RmtChannel>(
        _channel: impl Peripheral<P = C> + 'd,
        pin: impl Peripheral<P = impl InputPin> + 'd,
        config: &ReceiveConfig,
        ring_buf_size: usize,
    ) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        #[cfg(not(any(esp32, esp32c2)))]
        let carrier_en = config.carrier.is_some();

        #[cfg(not(any(esp32, esp32c2)))]
        let carrier = config.carrier.unwrap_or_default();

        let sys_config = rmt_config_t {
            rmt_mode: rmt_mode_t_RMT_MODE_RX,
            channel: C::channel(),
            gpio_num: pin.pin(),
            clk_div: config.clock_divider,
            mem_block_num: config.mem_block_num,
            flags: 0,
            __bindgen_anon_1: rmt_config_t__bindgen_ty_1 {
                rx_config: rmt_rx_config_t {
                    idle_threshold: config.idle_threshold,
                    filter_ticks_thresh: config.filter_ticks_thresh,
                    filter_en: config.filter_en,
                    #[cfg(not(any(esp32, esp32c2)))]
                    rm_carrier: carrier_en,
                    #[cfg(not(any(esp32, esp32c2)))]
                    carrier_freq_hz: carrier.frequency.into(),
                    #[cfg(not(any(esp32, esp32c2)))]
                    carrier_level: carrier.carrier_level as u32,
                    #[cfg(not(any(esp32, esp32c2)))]
                    carrier_duty_percent: carrier.duty_percent.0,
                },
            },
        };

        unsafe {
            esp!(rmt_config(&sys_config))?;
            esp!(rmt_driver_install(
                C::channel(),
                ring_buf_size * 4,
                IntrFlags::to_native(config.intr_flags) as _
            ))?;
        }

        Ok(Self {
            channel: C::channel() as _,
            next_ringbuf_item: None,
            _p: PhantomData,
        })
    }

    pub fn channel(&self) -> rmt_channel_t {
        self.channel as _
    }

    /// Start receiving
    pub fn start(&self) -> Result<(), EspError> {
        esp!(unsafe { rmt_rx_start(self.channel(), true) })
    }

    /// Stop receiving
    pub fn stop(&self) -> Result<(), EspError> {
        esp!(unsafe { rmt_rx_stop(self.channel()) })
    }

    pub fn receive(
        &mut self,
        buf: &mut [(Pulse, Pulse)],
        ticks_to_wait: TickType_t,
    ) -> Result<Receive, EspError> {
        if let Some(items) = self.fetch_ringbuf_next_item(ticks_to_wait)? {
            if items.len() <= buf.len() {
                for (index, item) in items.iter().enumerate() {
                    let item = unsafe { item.__bindgen_anon_1.__bindgen_anon_1 };

                    buf[index] = (
                        Pulse::new(
                            item.level0().into(),
                            PulseTicks::new(item.duration0().try_into().unwrap()).unwrap(),
                        ),
                        Pulse::new(
                            item.level1().into(),
                            PulseTicks::new(item.duration1().try_into().unwrap()).unwrap(),
                        ),
                    );
                }

                let len = items.len();

                self.return_ringbuf_item()?;

                Ok(Receive::Read(len))
            } else {
                Ok(Receive::Overflow(items.len()))
            }
        } else {
            Ok(Receive::Timeout)
        }
    }

    fn fetch_ringbuf_next_item(
        &mut self,
        ticks_to_wait: TickType_t,
    ) -> Result<Option<&[rmt_item32_t]>, EspError> {
        if let Some((rmt_items, length)) = self.next_ringbuf_item {
            Ok(Some(unsafe {
                core::slice::from_raw_parts(rmt_items, length)
            }))
        } else {
            let mut ringbuf_handle = ptr::null_mut();
            esp!(unsafe { rmt_get_ringbuf_handle(self.channel(), &mut ringbuf_handle) })?;

            let mut length = 0;
            let rmt_items: *mut rmt_item32_t = unsafe {
                xRingbufferReceive(ringbuf_handle.cast(), &mut length, ticks_to_wait).cast()
            };

            if rmt_items.is_null() {
                Ok(None)
            } else {
                let length = length / 4;
                self.next_ringbuf_item = Some((rmt_items, length));

                Ok(Some(unsafe {
                    core::slice::from_raw_parts(rmt_items, length)
                }))
            }
        }
    }

    fn return_ringbuf_item(&mut self) -> Result<(), EspError> {
        let mut ringbuf_handle = ptr::null_mut();
        esp!(unsafe { rmt_get_ringbuf_handle(self.channel(), &mut ringbuf_handle) })?;

        if let Some((rmt_items, _)) = self.next_ringbuf_item.take() {
            unsafe {
                vRingbufferReturnItem(ringbuf_handle, rmt_items.cast());
            }
        } else {
            unreachable!();
        }

        Ok(())
    }
}

impl<'d> Drop for RxRmtDriver<'d> {
    /// Stop receiving and release the driver.
    fn drop(&mut self) {
        self.stop().unwrap();
        esp!(unsafe { rmt_driver_uninstall(self.channel()) }).unwrap();
    }
}

unsafe impl<'d> Send for RxRmtDriver<'d> {}

mod chip {
    use esp_idf_sys::*;

    /// RMT peripheral channel.
    pub trait RmtChannel {
        fn channel() -> rmt_channel_t;
    }

    macro_rules! impl_channel {
        ($instance:ident: $channel:expr) => {
            crate::impl_peripheral!($instance);

            impl RmtChannel for $instance {
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

    pub struct RMT {
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

    impl RMT {
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
