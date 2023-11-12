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
use core::time::Duration;
use esp_idf_sys::*;

use crate::gpio::{GPIOMode, InputPin, Level, PinDriver};
#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::gpio::{RTCMode, RTCPin};
use crate::uart::UartDriver;
use core::marker::PhantomData;

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
pub struct RtcWakeup<P>
where
    P: RtcWakeupPins,
{
    pub pins: P,
    pub wake_level: RtcWakeLevel,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<P> RtcWakeup<P>
where
    P: RtcWakeupPins,
{
    fn mask(&self) -> u64 {
        let mut m: u64 = 0;
        for pin in self.pins.iter() {
            m |= 1 << pin;
        }
        m
    }

    fn apply(&self) -> Result<(), EspError> {
        #[cfg(any(esp32, esp32s3))]
        esp!(unsafe {
            esp_sleep_pd_config(
                esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                esp_sleep_pd_option_t_ESP_PD_OPTION_ON,
            )
        })?;

        esp!(unsafe { esp_sleep_enable_ext1_wakeup(self.mask(), self.wake_level.mode()) })?;

        Ok(())
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<P> fmt::Debug for RtcWakeup<P>
where
    P: RtcWakeupPins,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RtcWakeup {{ pins: [")?;

        for pin in self.pins.iter() {
            write!(f, "{:?} ", pin)?;
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

pub trait RtcWakeupPinTrait {
    fn pin(&self) -> i32;
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub trait RtcWakeupPins {
    type Iterator<'a>: Iterator<Item = i32>
    where
        Self: 'a;

    fn iter(&self) -> Self::Iterator<'_>;
}
#[cfg(not(any(esp32, esp32s2, esp32s3)))]
pub trait RtcWakeupPins {}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<P> RtcWakeupPins for P
where
    P: RtcWakeupPinTrait,
{
    type Iterator<'a> = core::iter::Once<i32> where Self: 'a;

    fn iter(&self) -> Self::Iterator<'_> {
        core::iter::once(self.pin())
    }
}

pub struct EmptyRtcWakeupPins;

#[cfg(any(esp32, esp32s2, esp32s3))]
impl EmptyRtcWakeupPins {
    pub fn chain<O>(other: O) -> ChainedRtcWakeupPins<Self, O>
    where
        O: RtcWakeupPins,
    {
        ChainedRtcWakeupPins {
            first: Self,
            second: other,
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl RtcWakeupPins for EmptyRtcWakeupPins {
    type Iterator<'a> = core::iter::Empty<i32> where Self: 'a;

    fn iter(&self) -> Self::Iterator<'_> {
        core::iter::empty()
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
impl RtcWakeupPins for EmptyRtcWakeupPins {}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct ChainedRtcWakeupPins<F, S> {
    first: F,
    second: S,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<F, S> ChainedRtcWakeupPins<F, S> {
    pub fn chain<O>(self, other: O) -> ChainedRtcWakeupPins<Self, O>
    where
        F: RtcWakeupPins,
        S: RtcWakeupPins,
        O: RtcWakeupPins,
    {
        ChainedRtcWakeupPins {
            first: self,
            second: other,
        }
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
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

#[cfg(any(esp32, esp32s2, esp32s3))]
pub struct RtcWakeupPin<'d, P, M>
where
    P: InputPin + RTCPin + 'd,
    M: RTCMode + 'd,
{
    pub pindriver: &'d PinDriver<'d, P, M>,
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<'d, P, M> RtcWakeupPinTrait for RtcWakeupPin<'d, P, M>
where
    P: InputPin + RTCPin + 'd,
    M: RTCMode + 'd,
{
    fn pin(&self) -> i32 {
        self.pindriver.pin()
    }
}

/// Will wake up the CPU based on changes in GPIO pins. Is only available for light sleep.
/// It takes PinDriver as input, which means that any required configurations on the pin
/// can be done before adding it to the GpioWakeup struct.
pub struct GpioWakeup<P>
where
    P: GpioWakeupPins,
{
    pub pins: P,
}

impl<P> GpioWakeup<P>
where
    P: GpioWakeupPins,
{
    fn apply(&self) -> Result<(), EspError> {
        for pin in self.pins.iter() {
            let intr_level = match pin.1 {
                Level::Low => gpio_int_type_t_GPIO_INTR_LOW_LEVEL,
                Level::High => gpio_int_type_t_GPIO_INTR_HIGH_LEVEL,
            };
            esp!(unsafe { gpio_wakeup_enable(pin.0, intr_level) })?;
        }
        esp!(unsafe { esp_sleep_enable_gpio_wakeup() })?;
        Ok(())
    }
}

impl<P> fmt::Debug for GpioWakeup<P>
where
    P: GpioWakeupPins,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "GpioWakeup {{ pins: [")?;

        for pin in self.pins.iter() {
            write!(f, "({} {:?}), ", pin.0, pin.1)?;
        }
        write!(f, "")
    }
}

#[cfg(any(esp32c2, esp32c3))]
pub struct GpioDeepWakeup<P>
where
    P: GpioWakeupPins,
{
    pub pins: P,
}

#[cfg(any(esp32c2, esp32c3))]
impl<P> GpioDeepWakeup<P>
where
    P: GpioWakeupPins,
{
    fn mask(&self, level: Level) -> u64 {
        let mut m: u64 = 0;
        for pin in self.pins.iter() {
            if pin.1 == level {
                m |= 1 << pin.0;
            }
        }
        m
    }

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
        })?;

        Ok(())
    }
}

#[cfg(any(esp32c2, esp32c3))]
impl<P> fmt::Debug for GpioDeepWakeup<P>
where
    P: GpioWakeupPins,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "GpioDeepWakeup {{ pins: [")?;

        for pin in self.pins.iter() {
            write!(f, "({} {:?}), ", pin.0, pin.1)?;
        }
        write!(f, "")
    }
}

pub trait GpioWakeupPinTrait {
    fn pin(&self) -> i32;
    fn wake_level(&self) -> Level;
}

pub trait GpioWakeupPins {
    type Iterator<'a>: Iterator<Item = (i32, Level)>
    where
        Self: 'a;

    fn iter(&self) -> Self::Iterator<'_>;
}

impl<P> GpioWakeupPins for P
where
    P: GpioWakeupPinTrait,
{
    type Iterator<'a> = core::iter::Once<(i32, Level)> where Self: 'a;

    fn iter(&self) -> Self::Iterator<'_> {
        core::iter::once((self.pin(), self.wake_level()))
    }
}

pub struct EmptyGpioWakeupPins;

impl EmptyGpioWakeupPins {
    pub fn chain<O>(other: O) -> ChainedGpioWakeupPins<Self, O>
    where
        O: GpioWakeupPins,
    {
        ChainedGpioWakeupPins {
            first: Self,
            second: other,
        }
    }
}

impl GpioWakeupPins for EmptyGpioWakeupPins {
    type Iterator<'a> = core::iter::Empty<(i32, Level)> where Self: 'a;

    fn iter(&self) -> Self::Iterator<'_> {
        core::iter::empty()
    }
}

pub struct ChainedGpioWakeupPins<F, S> {
    first: F,
    second: S,
}

impl<F, S> ChainedGpioWakeupPins<F, S> {
    pub fn chain<O>(self, other: O) -> ChainedGpioWakeupPins<Self, O>
    where
        F: GpioWakeupPins,
        S: GpioWakeupPins,
        O: GpioWakeupPins,
    {
        ChainedGpioWakeupPins {
            first: self,
            second: other,
        }
    }
}

impl<F, S> GpioWakeupPins for ChainedGpioWakeupPins<F, S>
where
    F: GpioWakeupPins,
    S: GpioWakeupPins,
{
    type Iterator<'a> = core::iter::Chain<F::Iterator<'a>, S::Iterator<'a>> where Self: 'a;

    fn iter(&self) -> Self::Iterator<'_> {
        self.first.iter().chain(self.second.iter())
    }
}

pub struct GpioWakeupPin<'d, P, M>
where
    P: InputPin + 'd,
    M: GPIOMode + 'd,
{
    pub pindriver: &'d PinDriver<'d, P, M>,
    pub wake_level: Level,
}

impl<'d, P, M> GpioWakeupPinTrait for GpioWakeupPin<'d, P, M>
where
    P: InputPin + 'd,
    M: GPIOMode + 'd,
{
    fn pin(&self) -> i32 {
        self.pindriver.pin()
    }

    fn wake_level(&self) -> Level {
        self.wake_level
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
pub struct LightSleep<'a, R, G>
where
    R: RtcWakeupPins,
    G: GpioWakeupPins,
{
    timer: Option<TimerWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    rtc: Option<RtcWakeup<R>>,
    gpio: Option<GpioWakeup<G>>,
    uart: Option<UartWakeup<'a>>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    ulp: Option<UlpWakeup>,
    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    _p: PhantomData<R>,
}

pub fn make_light_sleep_no_pins(
    timer: Option<TimerWakeup>,
    uart: Option<UartWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] ulp: Option<UlpWakeup>,
) -> LightSleep<EmptyRtcWakeupPins, EmptyGpioWakeupPins> {
    LightSleep {
        timer,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rtc: None,
        gpio: None,
        uart,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        touch,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        ulp,
        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        _p: PhantomData,
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub fn make_light_sleep_rtc_pins<R: RtcWakeupPins>(
    timer: Option<TimerWakeup>,
    rtc: Option<RtcWakeup<R>>,
    uart: Option<UartWakeup>,
    touch: Option<TouchWakeup>,
    ulp: Option<UlpWakeup>,
) -> LightSleep<R, EmptyGpioWakeupPins> {
    LightSleep {
        timer,
        rtc,
        gpio: None,
        uart,
        touch,
        ulp,
    }
}

pub fn make_light_sleep_gpio_pins<G: GpioWakeupPins>(
    timer: Option<TimerWakeup>,
    gpio: Option<GpioWakeup<G>>,
    uart: Option<UartWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] ulp: Option<UlpWakeup>,
) -> LightSleep<EmptyRtcWakeupPins, G> {
    LightSleep {
        timer,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rtc: None,
        gpio,
        uart,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        touch,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        ulp,
        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        _p: PhantomData,
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub fn make_light_sleep_rtc_gpio_pins<R: RtcWakeupPins, G: GpioWakeupPins>(
    timer: Option<TimerWakeup>,
    rtc: Option<RtcWakeup<R>>,
    gpio: Option<GpioWakeup<G>>,
    uart: Option<UartWakeup>,
    touch: Option<TouchWakeup>,
    ulp: Option<UlpWakeup>,
) -> LightSleep<R, G> {
    LightSleep {
        timer,
        rtc,
        gpio,
        uart,
        touch,
        ulp,
    }
}

impl<'a, R, P> LightSleep<'a, R, P>
where
    R: RtcWakeupPins,
    P: GpioWakeupPins,
{
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

impl<'a, R, P> fmt::Debug for LightSleep<'a, R, P>
where
    R: RtcWakeupPins,
    P: GpioWakeupPins,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "LightSleep: {{timer: {:?}, ", self.timer)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "rtc: {:?}, ", self.rtc)?;
        write!(f, "gpio: {:?}, ", self.gpio)?;
        write!(f, "uart: {:?}, ", self.uart)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "touch: {:?}, ", self.touch)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "ulp: {:?}, ", self.ulp)?;
        write!(f, "}}")
    }
}

/// Struct for deep sleep. Add wakeup sources to this struct, and then call sleep().
pub struct DeepSleep<R, P>
where
    R: RtcWakeupPins,
    P: GpioWakeupPins,
{
    pub timer: Option<TimerWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub rtc: Option<RtcWakeup<R>>,
    #[cfg(any(esp32c2, esp32c3))]
    pub gpio: Option<GpioDeepWakeup<P>>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub ulp: Option<UlpWakeup>,
    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    pub _p: PhantomData<R>,
    #[cfg(not(any(esp32c2, esp32c3)))]
    pub _p: PhantomData<P>,
}

pub fn make_deep_sleep_no_pins(
    timer: Option<TimerWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] touch: Option<TouchWakeup>,
    #[cfg(any(esp32, esp32s2, esp32s3))] ulp: Option<UlpWakeup>,
) -> DeepSleep<EmptyRtcWakeupPins, EmptyGpioWakeupPins> {
    DeepSleep {
        timer,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rtc: None,
        #[cfg(any(esp32c2, esp32c3))]
        gpio: None,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        touch,
        #[cfg(any(esp32, esp32s2, esp32s3))]
        ulp,
        _p: PhantomData,
    }
}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub fn make_deep_sleep_rtc_pins<R: RtcWakeupPins>(
    timer: Option<TimerWakeup>,
    rtc: Option<RtcWakeup<R>>,
    touch: Option<TouchWakeup>,
    ulp: Option<UlpWakeup>,
) -> DeepSleep<R, EmptyGpioWakeupPins> {
    DeepSleep {
        timer,
        rtc,
        touch,
        ulp,
        _p: PhantomData,
    }
}

#[cfg(any(esp32c2, esp32c3))]
pub fn make_deep_sleep_gpio_pins<P: GpioWakeupPins>(
    timer: Option<TimerWakeup>,
    gpio: Option<GpioDeepWakeup<P>>,
) -> DeepSleep<EmptyRtcWakeupPins, P> {
    DeepSleep {
        timer,
        gpio,
        _p: PhantomData,
    }
}

impl<R, P> DeepSleep<R, P>
where
    R: RtcWakeupPins,
    P: GpioWakeupPins,
{
    pub fn prepare(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_sleep_disable_wakeup_source(esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL) })?;

        if let Some(timer) = &self.timer {
            timer.apply()?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        if let Some(rtc) = &self.rtc {
            rtc.apply()?;
        }

        #[cfg(any(esp32c2, esp32c3))]
        if let Some(gpio) = &self.gpio {
            gpio.apply()?;
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

impl<R, P> fmt::Debug for DeepSleep<R, P>
where
    R: RtcWakeupPins,
    P: GpioWakeupPins,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "DeepSleep: {{timer: {:?}, ", self.timer)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "rtc: {:?}, ", self.rtc)?;
        #[cfg(any(esp32c2, esp32c3))]
        write!(f, "gpio: {:?}, ", self.gpio)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "touch: {:?}, ", self.touch)?;
        #[cfg(any(esp32, esp32s2, esp32s3))]
        write!(f, "ulp: {:?}, ", self.ulp)?;
        write!(f, "}}")
    }
}
