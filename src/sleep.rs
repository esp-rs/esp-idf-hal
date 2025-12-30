//! Driver for light and deep sleep.
//!
//! The ESP can be put into light or deep sleep. In light sleep, the CPU and peripherals
//! are still powered, but do not run (clock-gated). When woken up, the CPU and its
//! peripherals keep their states and the CPU continues running from the same place in the
//! code.
//!
//! Deep sleep is similar to light sleep, but the CPU as well as the SRAM are also powered down.
//! Only RTC and the ULP coprocessor (on chips where it is avalable) continue to be powered up.
//! When woking up, the CPU does not keep its state and the program starts from the beginning.
//!
//! When entering either light or deep sleep, one or more wakeup sources might be enabled.
//! In this driver, the various wakeup sources are defined as different structs, which
//! can be added to a light or deep sleep struct. This struct can then be used to perform
//! a sleep operation with the given wakeup sources.
//!
//! The wakeup sources available depends on the ESP chip and type of sleep. The driver
//! intends to enforce these constraints at compile time where possible, i.e. if it builds
//! it should work.

use core::fmt;
use core::marker::PhantomData;
use core::time::Duration;

use esp_idf_sys::*;

use crate::gpio::{GPIOMode, InputMode, Level, PinDriver, PinId};
use crate::uart::UartDriver;

#[cfg(not(any(esp32c2, esp32c3)))]
pub use rtc::*;
#[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
pub use touch::*;
#[cfg(any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32p4))]
pub use ulp::*;

/// A wakeup source that can wakeup from light sleep
pub trait LightSleepWakeup {
    /// Apply the wakeup source
    fn apply(&self) -> Result<(), EspError>;
}

impl<T> LightSleepWakeup for &T
where
    T: LightSleepWakeup,
{
    fn apply(&self) -> Result<(), EspError> {
        (**self).apply()
    }
}

impl LightSleepWakeup for &dyn LightSleepWakeup {
    fn apply(&self) -> Result<(), EspError> {
        (**self).apply()
    }
}

/// A wakeup source that can wakeup from deep sleep
pub trait DeepSleepWakeup {
    /// Apply the wakeup source
    fn apply(&self) -> Result<(), EspError>;
}

impl<T> DeepSleepWakeup for &T
where
    T: DeepSleepWakeup,
{
    fn apply(&self) -> Result<(), EspError> {
        (**self).apply()
    }
}

impl DeepSleepWakeup for &dyn DeepSleepWakeup {
    fn apply(&self) -> Result<(), EspError> {
        (**self).apply()
    }
}

/// Trait for types that can provide a wakeup pin for a given mode
pub trait WakeupPin<M> {
    /// Get the pin for wakeup
    fn pin(&self) -> PinId;

    /// Get the level for wakeup
    fn level(&self) -> Level;
}

impl<M, T> WakeupPin<M> for &T
where
    T: WakeupPin<M>,
{
    fn pin(&self) -> PinId {
        (**self).pin()
    }

    fn level(&self) -> Level {
        (**self).level()
    }
}

impl<M> WakeupPin<M> for &dyn WakeupPin<M> {
    fn pin(&self) -> PinId {
        (**self).pin()
    }

    fn level(&self) -> Level {
        (**self).level()
    }
}

impl<P: InputMode + GPIOMode> WakeupPin<Gpio> for (&PinDriver<'_, P>, Level) {
    fn pin(&self) -> PinId {
        self.0.pin()
    }

    fn level(&self) -> Level {
        self.1
    }
}

/// Will wake the CPU up after a given duration
#[derive(Debug)]
pub struct TimerWakeup {
    pub duration: Duration,
}

impl TimerWakeup {
    /// Create a new TimerWakeup struct
    ///
    /// # Arguments
    /// - duration: The duration after which to wake up
    ///
    /// # Returns
    /// - A TimerWakeup struct
    pub const fn new(duration: Duration) -> Self {
        Self { duration }
    }
}

impl LightSleepWakeup for TimerWakeup {
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_enable_timer_wakeup(self.duration.as_micros() as u64) })
    }
}

impl DeepSleepWakeup for TimerWakeup {
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_enable_timer_wakeup(self.duration.as_micros() as u64) })
    }
}

/// Marker type for GPIO wakeup
pub struct Gpio;

/// Will wake up the CPU based on changes in GPIO pins.
/// It takes a `PinDriver` as input, which means that any required configurations on the pin
/// can be done before adding it to the PinsWakeup struct.
pub struct PinsWakeup<P, M>
where
    P: IntoIterator + Clone,
    P::Item: WakeupPin<M>,
{
    pins: P,
    _t: PhantomData<M>,
}

impl<P, M> PinsWakeup<P, M>
where
    P: IntoIterator + Clone,
    P::Item: WakeupPin<M>,
{
    /// Create a new PinsWakeup struct
    ///
    /// # Arguments
    /// - pins: The pins to use for wakeup
    ///
    /// # Returns
    /// - A PinsWakeup struct
    pub const fn new(pins: P) -> Self {
        Self {
            pins,
            _t: PhantomData,
        }
    }

    fn mask(&self, level: Level) -> u64 {
        self.pins
            .clone()
            .into_iter()
            .filter(|wp| wp.level() == level)
            .fold(0u64, |mut m, wp| {
                m |= 1 << wp.pin();
                m
            })
    }
}

impl<P> LightSleepWakeup for PinsWakeup<P, Gpio>
where
    P: IntoIterator + Clone,
    P::Item: WakeupPin<Gpio>,
{
    fn apply(&self) -> Result<(), EspError> {
        for wp in self.pins.clone().into_iter() {
            let intr_level = match wp.level() {
                Level::Low => gpio_int_type_t_GPIO_INTR_LOW_LEVEL,
                Level::High => gpio_int_type_t_GPIO_INTR_HIGH_LEVEL,
            };

            esp!(unsafe { gpio_wakeup_enable(wp.pin() as _, intr_level) })?;
        }

        esp!(unsafe { esp_sleep_enable_gpio_wakeup() })
    }
}

#[cfg(any(esp32c2, esp32c3))]
impl<P> DeepSleepWakeup for PinsWakeup<P, Gpio>
where
    P: IntoIterator + Clone,
    P::Item: WakeupPin<Gpio>,
{
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe {
            esp_deep_sleep_enable_gpio_wakeup(
                self.mask(Level::Low),
                esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_LOW,
            )
        })?;

        esp!(unsafe {
            esp_deep_sleep_enable_gpio_wakeup(
                self.mask(Level::High),
                esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH,
            )
        })
    }
}

impl<P> fmt::Debug for PinsWakeup<P, Gpio>
where
    P: IntoIterator + Clone,
    P::Item: WakeupPin<Gpio>,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "PinsWakeup<GPIO> {{ pins: [")?;

        for wp in self.pins.clone().into_iter() {
            write!(f, "({} {:?}), ", wp.pin(), wp.level())?;
        }
        write!(f, "")
    }
}

/// Will wake up the CPU when the number of positive edges higher than a threshold
/// is detected on the UART RX pin. Is only available for light sleep.
pub struct UartWakeup<'a> {
    pub driver: &'a UartDriver<'a>,
    pub threshold: i32,
}

impl<'a> UartWakeup<'a> {
    /// Create a new UartWakeup struct
    ///
    /// # Arguments
    /// - driver: The UART driver to use for wakeup
    /// - threshold: The number of positive edges to detect before waking up
    ///
    /// # Returns
    /// - A UartWakeup struct
    pub const fn new(driver: &'a UartDriver<'a>, threshold: i32) -> Self {
        Self { driver, threshold }
    }
}

impl LightSleepWakeup for UartWakeup<'_> {
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { uart_set_wakeup_threshold(self.driver.port(), self.threshold) })?;
        esp!(unsafe { esp_sleep_enable_uart_wakeup(self.driver.port() as _) })
    }
}

impl<'a> fmt::Debug for UartWakeup<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "UartWakeup")
    }
}

#[cfg(not(any(esp32c2, esp32c3)))]
mod rtc {
    use core::fmt;

    use esp_idf_sys::*;

    use crate::gpio::{InputMode, Level, PinDriver, PinId, RTCMode};

    use super::{DeepSleepWakeup, LightSleepWakeup, PinsWakeup, WakeupPin};

    /// Marker type for RTC wakeup
    pub struct Rtc;

    impl<P: InputMode + RTCMode> WakeupPin<Rtc> for (&PinDriver<'_, P>, Level) {
        fn pin(&self) -> PinId {
            self.0.pin()
        }

        fn level(&self) -> Level {
            self.1
        }
    }

    impl<P> PinsWakeup<P, Rtc>
    where
        P: IntoIterator + Clone,
        P::Item: WakeupPin<Rtc>,
    {
        fn rtc_apply(&self) -> Result<(), EspError> {
            #[cfg(any(esp32, esp32s3))]
            esp!(unsafe {
                esp_sleep_pd_config(
                    esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                    esp_sleep_pd_option_t_ESP_PD_OPTION_ON,
                )
            })?;

            esp!(unsafe {
                esp_sleep_enable_ext1_wakeup(
                    self.mask(Level::Low),
                    esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ALL_LOW,
                )
            })?;
            esp!(unsafe {
                esp_sleep_enable_ext1_wakeup(
                    self.mask(Level::High),
                    esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_HIGH,
                )
            })
        }
    }

    impl<P> LightSleepWakeup for PinsWakeup<P, Rtc>
    where
        P: IntoIterator + Clone,
        P::Item: WakeupPin<Rtc>,
    {
        fn apply(&self) -> Result<(), EspError> {
            self.rtc_apply()
        }
    }

    impl<P> DeepSleepWakeup for PinsWakeup<P, Rtc>
    where
        P: IntoIterator + Clone,
        P::Item: WakeupPin<Rtc>,
    {
        fn apply(&self) -> Result<(), EspError> {
            self.rtc_apply()
        }
    }

    impl<P> fmt::Debug for PinsWakeup<P, Rtc>
    where
        P: IntoIterator + Clone,
        P::Item: WakeupPin<Rtc>,
    {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            write!(f, "PinsWakeup<Rtc> {{ pins: [")?;

            for wp in self.pins.clone().into_iter() {
                write!(f, "({} {:?}), ", wp.pin(), wp.level())?;
            }
            write!(f, "")
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
mod touch {
    use esp_idf_sys::*;

    /// Will wake up the CPU when a touchpad is touched.
    #[derive(Debug)]
    pub struct TouchWakeup;

    impl super::LightSleepWakeup for TouchWakeup {
        fn apply(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_touchpad_wakeup() })
        }
    }

    impl super::DeepSleepWakeup for TouchWakeup {
        fn apply(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_touchpad_wakeup() })
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32p4))]
mod ulp {
    use esp_idf_sys::*;

    /// Will let the ULP co-processor wake up the CPU. Requires that the ULP is started.
    #[derive(Debug)]
    pub struct UlpWakeup;

    impl super::LightSleepWakeup for UlpWakeup {
        fn apply(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_ulp_wakeup() })
        }
    }

    impl super::DeepSleepWakeup for UlpWakeup {
        fn apply(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_ulp_wakeup() })
        }
    }
}

/// Put the CPU into light sleep mode with the given wakeup sources
///
/// # Arguments
/// - wakeups: The wakeup sources to use
///   Note that the API does not enforce that each wakeup source is used
///   at most once in the wakeup chain. Thus, if a wakeup source is used
///   more than once, only the last one will take effect.
///
/// # Returns
/// - Ok(()) if the CPU was put into light sleep mode successfully
/// - Err(EspError) if there was an error
pub fn light_sleep<W>(wakeups: W) -> Result<(), EspError>
where
    W: IntoIterator,
    W::Item: LightSleepWakeup,
{
    esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })?;

    for wakeup in wakeups {
        wakeup.apply()?;
    }

    esp!(unsafe { esp_light_sleep_start() })
}

/// Put the CPU into deep sleep mode with the given wakeup sources.
/// This function will _not_ return if entering deep sleep was successful.
///
/// # Arguments
/// - wakeups: The wakeup sources to use
///   Note that the API does not enforce that each wakeup source is used
///   at most once in the wakeup chain. Thus, if a wakeup source is used
///   more than once, only the last one will take effect.
///
/// # Returns
/// - EspError if there was an error
pub fn deep_sleep<W>(wakeups: W) -> EspError
where
    W: IntoIterator,
    W::Item: DeepSleepWakeup,
{
    if let Err(e) =
        esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })
    {
        return e;
    }

    for wakeup in wakeups {
        if let Err(e) = wakeup.apply() {
            return e;
        }
    }

    // In esp idf version 5.x the esp_deep_sleep_start() function CAN return
    // if deep sleep was rejected.
    unsafe { esp_deep_sleep_start() }
}
