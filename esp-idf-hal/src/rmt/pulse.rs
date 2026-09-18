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

/// A [`Pulse`] defines for how long the output pin should be high or low.
///
/// The duration is defined through the [`PulseTicks`] type.
///
/// <div class="warning">The conversion from
/// ticks to real time depends on the selected resolution of the RMT channel.</div>
///
/// # Example
///
/// ```rust
/// # use esp_idf_hal::rmt::{PulseTicks, Pulse, PinState};
/// let pulse = Pulse::new(PinState::High, PulseTicks::new(32));
/// ```
///
/// You can create a [`Pulse`] with a [`Duration`] by using [`Pulse::new_with_duration`]:
/// ```rust
/// # use esp_idf_sys::EspError;
/// # use core::time::Duration;
/// # fn example() -> Result<(), EspError> {
/// use esp_idf_hal::rmt::{Pulse, PinState, PulseTicks};
/// use esp_idf_hal::units::FromValueType;
///
/// let ticks = 1_000_000.Hz(); // 1 MHz
/// let pulse = Pulse::new_with_duration(1_000_000.Hz(), PinState::High, Duration::from_nanos(300))?;
/// # Ok(())
/// # }
/// ```
/// [`Duration`]: core::time::Duration
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
    /// To convert the duration into ticks, the resolution (clock ticks) set for the
    /// RMT channel must be provided ([`TxChannelConfig::resolution`](crate::rmt::config::TxChannelConfig::resolution)).
    ///
    /// # Errors
    ///
    /// If the duration is too long to be represented as ticks with the given resolution,
    /// an error with the code [`ERR_EOVERFLOW`] or [`ESP_ERR_INVALID_ARG`] will be returned.
    pub const fn new_with_duration(
        resolution: Hertz,
        pin_state: PinState,
        duration: Duration,
    ) -> Result<Self, EspError> {
        match PulseTicks::new_with_duration(resolution, duration) {
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

    /// Create a `PulseTicks` from the given number of ticks.
    ///
    /// # Errors
    ///
    /// If the given number of ticks is larger than [`PulseTicks::max()`], an error with the code
    /// [`ESP_ERR_INVALID_ARG`] will be returned.
    pub const fn new(ticks: u16) -> Result<Self, EspError> {
        if ticks > Self::MAX {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
        } else {
            Ok(Self(ticks))
        }
    }

    /// Constructs a [`PulseTicks`] from a [`Duration`].
    ///
    /// # Errors
    ///
    /// If the duration is too long to be represented as ticks with the given resolution,
    /// an error with the code [`ERR_EOVERFLOW`] or [`ESP_ERR_INVALID_ARG`] will be returned.
    pub const fn new_with_duration(
        resolution: Hertz,
        duration: Duration,
    ) -> Result<Self, EspError> {
        match duration_to_ticks(resolution, duration) {
            Ok(ticks) => Self::new(ticks),
            Err(error) => Err(error),
        }
    }

    /// Returns the number of ticks.
    pub const fn ticks(&self) -> u16 {
        self.0
    }

    /// Returns the duration it takes for the number of ticks, depending on the given clock ticks.
    ///
    /// # Panics
    ///
    /// This function panics if the conversion from ticks to duration overflows.
    pub const fn duration(&self, resolution: Hertz) -> Duration {
        match ticks_to_duration(resolution, self.ticks()) {
            Ok(duration) => duration,
            Err(_) => panic!("Overflow while converting ticks to duration"),
        }
    }
}

const ONE_SECOND_IN_NANOS: u128 = Duration::from_secs(1).as_nanos();

/// A utility to convert a duration into ticks, depending on the clock ticks.
pub const fn duration_to_ticks(resolution: Hertz, duration: Duration) -> Result<u16, EspError> {
    let Some(ticks) = duration.as_nanos().checked_mul(resolution.0 as u128) else {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    };

    // To get the result we calculate:
    // duration (s) * resolution (ticks/s) = ticks
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
pub const fn ticks_to_duration(resolution: Hertz, ticks: u16) -> Result<Duration, EspError> {
    let Some(duration) = ONE_SECOND_IN_NANOS.checked_mul(ticks as u128) else {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    };

    let duration = duration / resolution.0 as u128;

    if duration > u64::MAX as u128 {
        return Err(EspError::from_infallible::<ERR_EOVERFLOW>());
    }

    Ok(Duration::from_nanos(duration as u64))
}
