//! Driver for light and deep sleep.
//!
//! The ESP can be put into light or deep sleep. In light sleep, the CPU and peripherals
//! are still powered, but do not run (clock-gated). When woken up, the CPU and its
//! peripherals keep their states and the CPU continues running from the same place in the
//! code.
//!
//! Deep sleep is similar to light sleep, but the CPU is also powered down. Only RTC and
//! possibly ULP coprocessor is running. When woking up, the CPU does not keep its state,
//! and the program starts from the beginning.
//!
//! When entering either light or deep sleep, one or more wakeup sources must be enabled.
//! In this driver, the various wakeup sources are defined as different structs, which
//! can be added to a light or deep sleep struct. This struct can then be used to perform
//! a sleep operation with the given wakeup sources.
//!
//! The wakeup sources available depends on the ESP chip and type of sleep. The driver
//! intends to enforce these constraints at compile time where possible, i.e. if it builds
//! it should work.
//!

use core::fmt;
#[cfg(not(any(esp32, esp32s2, esp32s3)))]
use core::marker::PhantomData;
use core::time::Duration;
use esp_idf_sys::*;

#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::gpio::Pin;
use crate::gpio::{AnyInputPin, Input, Level, PinDriver};
use crate::uart::UartDriver;

/// Will wake the CPU up after a given duration
#[derive(Debug)]
pub struct TimerWakeup {
    pub duration: Duration,
}

impl TimerWakeup {
    pub const fn new(duration: Duration) -> Self {
        Self { duration }
    }

    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_enable_timer_wakeup(self.duration.as_micros() as u64) })?;
        Ok(())
    }
}

/// Will wake up the CPU based on changes in RTC GPIO pins. Is available for both light and
/// deep sleep. However, for light sleep, the GpioWakeup struct offers more flexibility.
/// Pullup and pulldown can be enabled for each pin. This settings will be applied when
/// deep sleep is enabled.
#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct RtcWakeup<'a> {
    pub pins: &'a [RtcWakeupPin],
    pub wake_level: RtcWakeLevel,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'a> RtcWakeup<'a> {
    pub const fn new(pins: &'a [RtcWakeupPin], wake_level: RtcWakeLevel) -> Self {
        Self { pins, wake_level }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'a> RtcWakeup<'a> {
    fn mask(&self) -> u64 {
        let mut m: u64 = 0;
        for pin in self.pins.iter() {
            m |= 1 << pin.pin.pin();
        }
        m
    }

    fn apply(&self) -> Result<(), EspError> {
        #[cfg(any(esp32, esp32s3))]
        for pin in self.pins {
            if pin.pullup || pin.pulldown {
                esp!(unsafe {
                    esp_sleep_pd_config(
                        esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                        esp_sleep_pd_option_t_ESP_PD_OPTION_ON,
                    )
                })?;

                if pin.pullup {
                    esp!(unsafe { rtc_gpio_pullup_en(pin.pin.pin()) })?;
                }
                if pin.pulldown {
                    esp!(unsafe { rtc_gpio_pulldown_en(pin.pin.pin()) })?;
                }
            }
        }

        if !self.pins.is_empty() {
            /* Do not use ext0, as ext1 with one pin does the same */
            esp!(unsafe { esp_sleep_enable_ext1_wakeup(self.mask(), self.wake_level.mode()) })?;
        }

        Ok(())
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'a> fmt::Debug for RtcWakeup<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RtcWakeup {{ pins: [")?;

        for pin in self.pins.iter() {
            write!(f, "{:?} ", pin.pin.pin())?;
        }
        write!(
            f,
            "] mask: {:#x} wake_level: {:?} }}",
            self.mask(),
            self.wake_level
        )
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum RtcWakeLevel {
    AllLow,
    AnyHigh,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl RtcWakeLevel {
    pub fn mode(&self) -> u32 {
        match self {
            RtcWakeLevel::AllLow => esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ALL_LOW,
            RtcWakeLevel::AnyHigh => esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_HIGH,
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct RtcWakeupPin {
    pub pin: AnyInputPin, // this should use RtcInput somehow
    pub pullup: bool,
    pub pulldown: bool,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl RtcWakeupPin {
    pub const fn new(pin: AnyInputPin, pullup: bool, pulldown: bool) -> Self {
        Self {
            pin,
            pullup,
            pulldown,
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl fmt::Debug for RtcWakeupPin {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "RtcWakeupPin {{ pin: {:?} pullup {:?} pulldown {:?}}}",
            self.pin.pin(),
            self.pullup,
            self.pulldown
        )
    }
}

/// Will wake up the CPU based on changes in GPIO pins. Is only available for light sleep.
/// It takes PinDriver as input, which means that any required configurations on the pin
/// can be done before adding it to the GpioWakeup struct.
#[derive(Debug)]
pub struct GpioWakeup<'a> {
    pub pins: &'a [GpioWakeupPin<'a>],
}

impl<'a> GpioWakeup<'a> {
    pub const fn new(pins: &'a [GpioWakeupPin<'a>]) -> Self {
        Self { pins }
    }
}

impl<'a> GpioWakeup<'a> {
    fn apply(&self) -> Result<(), EspError> {
        for pin in self.pins.iter() {
            let intr_level = match pin.wake_level {
                Level::Low => gpio_int_type_t_GPIO_INTR_LOW_LEVEL,
                Level::High => gpio_int_type_t_GPIO_INTR_HIGH_LEVEL,
            };
            esp!(unsafe { gpio_wakeup_enable(pin.pin.pin(), intr_level) })?;
        }
        esp!(unsafe { esp_sleep_enable_gpio_wakeup() })?;
        Ok(())
    }
}

pub struct GpioWakeupPin<'a> {
    pub pin: PinDriver<'a, AnyInputPin, Input>,
    pub wake_level: Level,
}

impl<'a> GpioWakeupPin<'a> {
    pub const fn new(
        pin: PinDriver<'a, AnyInputPin, Input>,
        wake_level: Level,
    ) -> Result<Self, EspError> {
        Ok(Self { pin, wake_level })
    }
}

impl<'a> fmt::Debug for GpioWakeupPin<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "GpioWakeupPin {{ pin: {:?} wake_level: {:?} }}",
            self.pin.pin(),
            self.wake_level
        )
    }
}

/// Will wake up the CPU when the number of positive edges higher than a threshold
/// is detected on the UART RX pin. Is only available for light sleep.
pub struct UartWakeup<'a> {
    pub uart: &'a UartDriver<'a>,
    pub threshold: i32,
}

impl<'a> UartWakeup<'a> {
    pub const fn new(uart: &'a UartDriver<'a>, threshold: i32) -> Self {
        Self { uart, threshold }
    }

    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { uart_set_wakeup_threshold(self.uart.port(), self.threshold) })?;
        esp!(unsafe { esp_sleep_enable_uart_wakeup(self.uart.port()) })?;
        Ok(())
    }
}

impl<'a> fmt::Debug for UartWakeup<'a> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "UartWakeup")
    }
}

/// Will wake up the CPU when a touchpad is touched.
#[cfg(any(esp32, esp32s2, esp32s3))]
#[derive(Debug)]
pub struct TouchWakeup;

#[cfg(any(esp32, esp32s2, esp32s3))]
impl TouchWakeup {
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_enable_touchpad_wakeup() })?;
        Ok(())
    }
}

/// Will let the ULP co-processor wake up the CPU. Requires that the ULP is started.
#[cfg(any(esp32, esp32s2, esp32s3))]
#[derive(Debug)]
pub struct UlpWakeup;

#[cfg(any(esp32, esp32s2, esp32s3))]
impl UlpWakeup {
    fn apply(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_enable_ulp_wakeup() })?;
        Ok(())
    }
}

/// Struct for light sleep. Add wakeup sources to this struct, and then call sleep().
#[derive(Debug, Default)]
pub struct LightSleep<'a> {
    pub timer: Option<TimerWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub rtc: Option<RtcWakeup<'a>>,
    pub gpio: Option<GpioWakeup<'a>>,
    pub uart: Option<UartWakeup<'a>>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub ulp: Option<UlpWakeup>,
}

impl<'a> LightSleep<'a> {
    pub fn sleep(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })?;

        if let Some(timer) = &self.timer {
            timer.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(rtc) = &self.rtc {
            rtc.apply()?;
        }

        if let Some(gpio) = &self.gpio {
            gpio.apply()?;
        }

        if let Some(uart) = &self.uart {
            uart.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(touch) = &self.touch {
            touch.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(ulp) = &self.ulp {
            ulp.apply()?;
        }

        esp!(unsafe { esp_light_sleep_start() })?;
        Ok(())
    }
}

/// Struct for deep sleep. Add wakeup sources to this struct, and then call sleep().
#[derive(Debug, Default)]
pub struct DeepSleep<'a> {
    pub timer: Option<TimerWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub rtc: Option<RtcWakeup<'a>>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub ulp: Option<UlpWakeup>,
    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    pub _p: PhantomData<&'a ()>,
}

impl<'a> DeepSleep<'a> {
    pub fn prepare(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })?;

        if let Some(timer) = &self.timer {
            timer.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(rtc) = &self.rtc {
            rtc.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(touch) = &self.touch {
            touch.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(ulp) = &self.ulp {
            ulp.apply()?;
        }
        Ok(())
    }

    // In esp idf version 5.x the esp_deep_sleep_start() function CAN return
    // if deep sleep was rejected. So in order for this function to return !
    // it needs to panic. This cause unreachable code for v4.4 and earlier,
    // thus the warning is disabled.
    #[allow(unreachable_code)] 
    pub fn sleep(&self) -> ! {
        unsafe { esp_deep_sleep_start() }   
        // Normally not reached, but if it is, then panic will restart the
        // chip, which is similar to what would happen with a very short deep
        // sleep. 
        panic!("Failed to deep sleep, will panic instead");
    }

    pub fn prepare_and_sleep(&self) -> Result<(), EspError> {
        self.prepare()?;
        self.sleep();
    }
}
