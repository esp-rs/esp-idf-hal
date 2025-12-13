//! Driver for Light and Deep Sleep.
//!
//! # Example
//!
//! ```rust
//! // Light Sleep with RTC wakeup (Legacy/Specific use case)
//! LightSleep::new()
//!     .unwrap()
//!     .wakeup_on_rtc(&pin, RtcWakeLevel::AnyHigh) // Now available!
//!     .unwrap()
//!     .enter()
//!     .unwrap();
//! ```

use core::time::Duration;
use esp_idf_sys::*;

// Re-export specific types for user convenience
pub use self::source::rtc::{ChainedRtcWakeupPins, RtcWakeLevel, RtcWakeupPins};

// =============================================================================
// Module: Wakeup Sources
// =============================================================================

mod source {
    pub mod timer {
        use core::time::Duration;
        use esp_idf_sys::*;

        pub fn configure(duration: Duration) -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_timer_wakeup(duration.as_micros() as u64) })
        }
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub mod rtc {
        use crate::gpio::{PinDriver, PinId, RTCMode};
        use esp_idf_sys::*;

        // --- RTC Wakeup Levels ---
        #[derive(Debug, Copy, Clone, PartialEq, Eq)]
        pub enum RtcWakeLevel {
            AllLow,
            AnyHigh,
        }

        impl RtcWakeLevel {
            pub fn mode(&self) -> u32 {
                match self {
                    Self::AllLow => esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ALL_LOW,
                    Self::AnyHigh => esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_HIGH,
                }
            }
        }

        // --- Chaining Trait & Implementation ---

        /// Trait for items that can produce a sequence of RTC Pin IDs.
        pub trait RtcWakeupPins {
            type Iterator<'a>: Iterator<Item = PinId>
            where
                Self: 'a;

            fn iter(&self) -> Self::Iterator<'_>;

            fn chain<P: RtcWakeupPins>(self, other: P) -> ChainedRtcWakeupPins<Self, P>
            where
                Self: Sized,
            {
                ChainedRtcWakeupPins {
                    first: self,
                    second: other,
                }
            }
        }

        /// Base Case: A single PinDriver implements the trait.
        impl<P> RtcWakeupPins for &PinDriver<'_, P>
        where
            P: RTCMode,
        {
            type Iterator<'a> = core::iter::Once<PinId> where Self: 'a;

            fn iter(&self) -> Self::Iterator<'_> {
                core::iter::once(self.pin())
            }
        }

        /// Recursive Case: A chain of two RtcWakeupPins.
        pub struct ChainedRtcWakeupPins<F, S> {
            first: F,
            second: S,
        }

        impl<F, S> RtcWakeupPins for ChainedRtcWakeupPins<F, S>
        where
            F: RtcWakeupPins,
            S: RtcWakeupPins,
        {
            type Iterator<'a> = core::iter::Chain<F::Iterator<'a>, S::Iterator<'a>> where Self: 'a;

            fn iter(&self) -> Self::Iterator<'_> {
                self.first.iter().chain(self.second.iter())
            }
        }

        // Support passing references to chains
        impl<F, S> RtcWakeupPins for &ChainedRtcWakeupPins<F, S>
        where
            F: RtcWakeupPins,
            S: RtcWakeupPins,
        {
            type Iterator<'a> = core::iter::Chain<F::Iterator<'a>, S::Iterator<'a>> where Self: 'a;

            fn iter(&self) -> Self::Iterator<'_> {
                self.first.iter().chain(self.second.iter())
            }
        }

        // --- Configuration Logic ---

        pub fn configure<P: RtcWakeupPins>(pins: P, level: RtcWakeLevel) -> Result<(), EspError> {
            let mut mask: u64 = 0;
            for pin_id in pins.iter() {
                mask |= 1 << pin_id;
            }

            #[cfg(any(esp32, esp32s3))]
            esp!(unsafe {
                esp_sleep_pd_config(
                    esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                    esp_sleep_pd_option_t_ESP_PD_OPTION_ON,
                )
            })?;

            esp!(unsafe { esp_sleep_enable_ext1_wakeup(mask, level.mode()) })
        }
    }

    pub mod gpio {
        use crate::gpio::{Level, PinId};
        use esp_idf_sys::*;

        pub fn configure_light(pin: PinId, level: Level) -> Result<(), EspError> {
            let intr = match level {
                Level::Low => gpio_int_type_t_GPIO_INTR_LOW_LEVEL,
                Level::High => gpio_int_type_t_GPIO_INTR_HIGH_LEVEL,
            };
            esp!(unsafe { gpio_wakeup_enable(pin as _, intr) })?;
            esp!(unsafe { esp_sleep_enable_gpio_wakeup() })
        }

        #[cfg(any(esp32c2, esp32c3))]
        pub fn configure_deep(pin: PinId, level: Level) -> Result<(), EspError> {
            let mask = 1 << pin;
            let mode = match level {
                Level::Low => esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_LOW,
                Level::High => esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_HIGH,
            };
            esp!(unsafe { esp_deep_sleep_enable_gpio_wakeup(mask, mode) })
        }
    }

    pub mod uart {
        use crate::uart::UartDriver;
        use esp_idf_sys::*;

        pub fn configure(uart: &UartDriver, threshold: i32) -> Result<(), EspError> {
            esp!(unsafe { uart_set_wakeup_threshold(uart.port(), threshold) })?;
            esp!(unsafe { esp_sleep_enable_uart_wakeup(uart.port() as _) })
        }
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub mod touch {
        use esp_idf_sys::*;
        pub fn configure() -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_touchpad_wakeup() })
        }
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub mod ulp {
        use esp_idf_sys::*;
        pub fn configure() -> Result<(), EspError> {
            esp!(unsafe { esp_sleep_enable_ulp_wakeup() })
        }
    }
}

// =============================================================================
// Module: Drivers
// =============================================================================

use crate::gpio::{GPIOMode, Level, PinDriver};
use crate::uart::UartDriver;

/// Helper to clear previous wakeup configuration
fn reset_wakeup_sources() -> Result<(), EspError> {
    esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })
}

// -----------------------------------------------------------------------------
// Light Sleep
// -----------------------------------------------------------------------------

/// Light Sleep Driver.
///
/// Stateless handle. Configuration is applied immediately.
pub struct LightSleep;

impl LightSleep {
    pub fn new() -> Result<Self, EspError> {
        reset_wakeup_sources()?;
        Ok(Self)
    }

    pub fn wakeup_on_timer(self, duration: Duration) -> Result<Self, EspError> {
        source::timer::configure(duration)?;
        Ok(self)
    }

    pub fn wakeup_on_uart(self, uart: &UartDriver, threshold: i32) -> Result<Self, EspError> {
        source::uart::configure(uart, threshold)?;
        Ok(self)
    }

    /// Wake up on standard GPIO (digital controller).
    ///
    /// This is the preferred method for light sleep as it allows per-pin logic.
    pub fn wakeup_on_gpio<M: GPIOMode>(
        self,
        pin: &PinDriver<M>,
        level: Level,
    ) -> Result<Self, EspError> {
        source::gpio::configure_light(pin.pin(), level)?;
        Ok(self)
    }

    /// Wake up on RTC Pins (EXT1 controller).
    ///
    /// Available on ESP32, S2, S3. Uses the low-power RTC controller.
    /// Note: All pins in the chain must trigger on the same logic level.
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_rtc<P>(self, pins: P, level: source::rtc::RtcWakeLevel) -> Result<Self, EspError>
    where
        P: source::rtc::RtcWakeupPins,
    {
        source::rtc::configure(pins, level)?;
        Ok(self)
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_touch(self) -> Result<Self, EspError> {
        source::touch::configure()?;
        Ok(self)
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_ulp(self) -> Result<Self, EspError> {
        source::ulp::configure()?;
        Ok(self)
    }

    pub fn enter(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_light_sleep_start() })
    }
}

// -----------------------------------------------------------------------------
// Deep Sleep
// -----------------------------------------------------------------------------

/// Deep Sleep Driver.
///
/// Stateless handle. Configuration is applied immediately.
pub struct DeepSleep;

impl DeepSleep {
    pub fn new() -> Result<Self, EspError> {
        reset_wakeup_sources()?;
        Ok(Self)
    }

    pub fn wakeup_on_timer(self, duration: Duration) -> Result<Self, EspError> {
        source::timer::configure(duration)?;
        Ok(self)
    }

    /// Wake up on RTC Pins (ESP32/S2/S3).
    ///
    /// Accepts a chain of pins (or a single pin) and a single wakeup level.
    /// The bitmask is calculated and applied immediately.
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_rtc<P>(self, pins: P, level: source::rtc::RtcWakeLevel) -> Result<Self, EspError>
    where
        P: source::rtc::RtcWakeupPins,
    {
        source::rtc::configure(pins, level)?;
        Ok(self)
    }

    /// Wake up on GPIO (ESP32C2/C3 only).
    #[cfg(any(esp32c2, esp32c3))]
    pub fn wakeup_on_gpio<M: GPIOMode>(
        self,
        pin: &PinDriver<M>,
        level: Level,
    ) -> Result<Self, EspError> {
        source::gpio::configure_deep(pin.pin(), level)?;
        Ok(self)
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_touch(self) -> Result<Self, EspError> {
        source::touch::configure()?;
        Ok(self)
    }

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_ulp(self) -> Result<Self, EspError> {
        source::ulp::configure()?;
        Ok(self)
    }

    pub fn enter(self) -> ! {
        unsafe { esp_deep_sleep_start() }
        #[allow(unreachable_code)]
        {
            panic!("Deep sleep failed to start");
        }
    }
}