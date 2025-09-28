// TODO: merge with rmt_legacy code/add changes back to it?
// TODO: I think it would be better to have the code in this module and have rmt_legacy re-export it?

use core::time::Duration;

use esp_idf_sys::*;

use crate::units::Hertz;

// Might not always be available in the generated `esp-idf-sys` bindings
const ERR_EOVERFLOW: esp_err_t = 139;

/// A `Low` (0) or `High` (1) state for a pin.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinState {
    Low,
    High,
}

impl From<bool> for PinState {
    fn from(state: bool) -> Self {
        (state as u32).into()
    }
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
    /// Returns a low `Pulse` with zero ticks.
    pub const fn zero() -> Self {
        Self::new(PinState::Low, PulseTicks::zero())
    }

    /// Create a [`Pulse`] using a pin state and a tick count.
    pub const fn new(pin_state: PinState, ticks: PulseTicks) -> Self {
        Self { pin_state, ticks }
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
    pub const fn new_with_duration(
        ticks_hz: Hertz,
        pin_state: PinState,
        duration: Duration,
    ) -> Result<Self, EspError> {
        match PulseTicks::new_with_duration(ticks_hz, duration) {
            Ok(ticks) => Ok(Self::new(pin_state, ticks)),
            Err(error) => Err(error),
        }
    }
}

impl Default for Pulse {
    fn default() -> Self {
        Self::zero()
    }
}

/// Number of ticks, restricting the range to 0 to 32,767.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Default)]
pub struct PulseTicks(u16);

impl PulseTicks {
    const MAX: u16 = 32767; // = 2^15 - 1

    /// Use zero ticks.
    pub const fn zero() -> Self {
        Self(0)
    }

    /// Use the maximum value of 32767.
    pub const fn max() -> Self {
        Self(Self::MAX)
    }

    /// Needs to be unsigned 15 bits: 0-32767 inclusive, otherwise an [ESP_ERR_INVALID_ARG] is
    /// returned.
    pub const fn new(ticks: u16) -> Result<Self, EspError> {
        if ticks > Self::MAX {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
        } else {
            Ok(Self(ticks))
        }
    }

    /// Convert a `Duration` into `PulseTicks`.
    ///
    /// See `Pulse::new_with_duration()` for details.
    pub const fn new_with_duration(ticks_hz: Hertz, duration: Duration) -> Result<Self, EspError> {
        match duration_to_ticks(ticks_hz, duration) {
            Ok(ticks) => Self::new(ticks),
            Err(error) => Err(error),
        }
    }

    /// Returns the number of ticks.
    pub const fn ticks(&self) -> u16 {
        self.0
    }

    /// Returns the duration it takes for the number of ticks, depending on the given clock ticks.
    pub const fn duration(&self, ticks_hz: Hertz) -> Result<Duration, EspError> {
        ticks_to_duration(ticks_hz, self.ticks())
    }
}

const ONE_SECOND_IN_NANOS: u128 = Duration::from_secs(1).as_nanos();

/// A utility to convert a duration into ticks, depending on the clock ticks.
pub const fn duration_to_ticks(ticks_hz: Hertz, duration: Duration) -> Result<u16, EspError> {
    let Some(ticks) = duration.as_nanos().checked_mul(ticks_hz.0 as u128) else {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    };

    // To get the result we calculate:
    // duration (s) * ticks_hz (ticks/s) = ticks
    //
    // The above calculates with the duration in nanoseconds,
    // which is why it has to be divided by 1_000_000_000 (= 1s)
    // to get the correct result.

    let ticks = ticks / ONE_SECOND_IN_NANOS;

    if ticks > u16::MAX as u128 {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    }

    Ok(ticks as u16)
}

/// A utility to convert ticks into duration, depending on the clock ticks.
pub const fn ticks_to_duration(ticks_hz: Hertz, ticks: u16) -> Result<Duration, EspError> {
    let Some(duration) = ONE_SECOND_IN_NANOS.checked_mul(ticks as u128) else {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    };

    let duration = duration / ticks_hz.0 as u128;

    if duration > u64::MAX as u128 {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    }

    Ok(Duration::from_nanos(duration as u64))
}
