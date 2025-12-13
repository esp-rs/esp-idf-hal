use core::time::Duration;
use esp_idf_sys::*;

pub use self::source::rtc::{ChainedRtcWakeupPins, RtcWakeLevel, RtcWakeupPins};

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

        impl<P> RtcWakeupPins for &PinDriver<'_, P>
        where
            P: RTCMode,
        {
            type Iterator<'a> = core::iter::Once<PinId> where Self: 'a;

            fn iter(&self) -> Self::Iterator<'_> {
                core::iter::once(self.pin())
            }
        }

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

use crate::gpio::{GPIOMode, Level, PinDriver};
use crate::uart::UartDriver;

fn reset_wakeup_sources() -> Result<(), EspError> {
    esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })
}

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

    pub fn wakeup_on_gpio<M: GPIOMode>(
        self,
        pin: &PinDriver<M>,
        level: Level,
    ) -> Result<Self, EspError> {
        source::gpio::configure_light(pin.pin(), level)?;
        Ok(self)
    }

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

    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub fn wakeup_on_rtc<P>(self, pins: P, level: source::rtc::RtcWakeLevel) -> Result<Self, EspError>
    where
        P: source::rtc::RtcWakeupPins,
    {
        source::rtc::configure(pins, level)?;
        Ok(self)
    }

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