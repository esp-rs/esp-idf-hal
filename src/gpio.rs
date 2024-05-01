//! GPIO and pin configuration

use core::marker::PhantomData;

#[cfg(feature = "alloc")]
extern crate alloc;

use esp_idf_sys::*;

use crate::adc::Adc;
use crate::peripheral::{Peripheral, PeripheralRef};

pub use chip::*;

/// A trait implemented by every pin instance
pub trait Pin: Peripheral<P = Self> + Sized + Send + 'static {
    fn pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an input pin
pub trait InputPin: Pin + Into<AnyInputPin> {
    fn downgrade_input(self) -> AnyInputPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of
/// operating as an output pin
pub trait OutputPin: Pin + Into<AnyOutputPin> {
    fn downgrade_output(self) -> AnyOutputPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of
/// operating as an input and output pin
pub trait IOPin: InputPin + OutputPin + Into<AnyIOPin> {
    fn downgrade(self) -> AnyIOPin {
        self.into()
    }
}

/// A marker trait designating a pin which is capable of
/// operating as an RTC pin
pub trait RTCPin: Pin {
    fn rtc_pin(&self) -> i32;
}

pub(crate) mod sealed {
    pub trait ADCPin {
        // NOTE: Will likely disappear in subsequent versions,
        // once ADC support pops up in e-hal1. Hence sealed
        const CHANNEL: super::adc_channel_t;
    }
}

/// A marker trait designating a pin which is capable of
/// operating as an ADC pin
pub trait ADCPin: sealed::ADCPin + Pin {
    type Adc: Adc;

    fn adc_channel(&self) -> adc_channel_t {
        Self::CHANNEL
    }
}

/// A marker trait designating a pin which is capable of
/// operating as a DAC pin
#[cfg(any(esp32, esp32s2))]
pub trait DACPin: Pin {
    fn dac_channel(&self) -> dac_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a touch pin
#[cfg(any(esp32, esp32s2, esp32s3))]
pub trait TouchPin: Pin {
    fn touch_channel(&self) -> touch_pad_t;
}

/// Generic Gpio input-output pin
pub struct AnyIOPin {
    pin: i32,
    _p: PhantomData<*const ()>,
}

impl AnyIOPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is
    /// already instantiated and used elsewhere, or if it is not set
    /// already in the mode of operation which is being instantiated
    pub unsafe fn new(pin: i32) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }

    /// Creates an `Option<AnyIOPin>::None` for pins that are
    /// optional in APIs.
    pub const fn none() -> Option<Self> {
        None
    }
}

crate::impl_peripheral_trait!(AnyIOPin);

impl Pin for AnyIOPin {
    fn pin(&self) -> i32 {
        self.pin
    }
}

impl InputPin for AnyIOPin {}
impl OutputPin for AnyIOPin {}
impl IOPin for AnyIOPin {}

/// Generic Gpio input pin
pub struct AnyInputPin {
    pin: i32,
    _p: PhantomData<*const ()>,
}

impl AnyInputPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is
    /// already instantiated and used elsewhere, or if it is not set
    /// already in the mode of operation which is being instantiated
    pub unsafe fn new(pin: i32) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }

    /// Creates an `Option<AnyInputPin>::None` for pins that are
    /// optional in APIs.
    pub const fn none() -> Option<Self> {
        None
    }
}

crate::impl_peripheral_trait!(AnyInputPin);

impl Pin for AnyInputPin {
    fn pin(&self) -> i32 {
        self.pin
    }
}

impl InputPin for AnyInputPin {}

impl From<AnyIOPin> for AnyInputPin {
    fn from(pin: AnyIOPin) -> Self {
        unsafe { Self::new(pin.pin()) }
    }
}

/// Generic Gpio output pin
pub struct AnyOutputPin {
    pin: i32,
    _p: PhantomData<*const ()>,
}

impl AnyOutputPin {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is
    /// already instantiated and used elsewhere, or if it is not set
    /// already in the mode of operation which is being instantiated
    pub unsafe fn new(pin: i32) -> Self {
        Self {
            pin,
            _p: PhantomData,
        }
    }

    /// Creates an `Option<AnyOutputPin>::None` for pins that are
    /// optional in APIs.
    pub const fn none() -> Option<Self> {
        None
    }
}

crate::impl_peripheral_trait!(AnyOutputPin);

impl Pin for AnyOutputPin {
    fn pin(&self) -> i32 {
        self.pin
    }
}

impl OutputPin for AnyOutputPin {}

impl From<AnyIOPin> for AnyOutputPin {
    fn from(pin: AnyIOPin) -> Self {
        unsafe { Self::new(pin.pin()) }
    }
}

/// Interrupt types
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum InterruptType {
    PosEdge,
    NegEdge,
    AnyEdge,
    LowLevel,
    HighLevel,
}

impl From<InterruptType> for gpio_int_type_t {
    fn from(interrupt_type: InterruptType) -> gpio_int_type_t {
        match interrupt_type {
            InterruptType::PosEdge => gpio_int_type_t_GPIO_INTR_POSEDGE,
            InterruptType::NegEdge => gpio_int_type_t_GPIO_INTR_NEGEDGE,
            InterruptType::AnyEdge => gpio_int_type_t_GPIO_INTR_ANYEDGE,
            InterruptType::LowLevel => gpio_int_type_t_GPIO_INTR_LOW_LEVEL,
            InterruptType::HighLevel => gpio_int_type_t_GPIO_INTR_HIGH_LEVEL,
        }
    }
}

impl From<InterruptType> for u8 {
    fn from(interrupt_type: InterruptType) -> u8 {
        let int_type: gpio_int_type_t = interrupt_type.into();

        int_type as u8
    }
}

/// Drive strength (values are approximates)
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

impl From<DriveStrength> for gpio_drive_cap_t {
    fn from(strength: DriveStrength) -> gpio_drive_cap_t {
        match strength {
            DriveStrength::I5mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_0,
            DriveStrength::I10mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_1,
            DriveStrength::I20mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_2,
            DriveStrength::I40mA => gpio_drive_cap_t_GPIO_DRIVE_CAP_3,
        }
    }
}

impl From<gpio_drive_cap_t> for DriveStrength {
    #[allow(non_upper_case_globals)]
    fn from(cap: gpio_drive_cap_t) -> DriveStrength {
        match cap {
            gpio_drive_cap_t_GPIO_DRIVE_CAP_0 => DriveStrength::I5mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_1 => DriveStrength::I10mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_2 => DriveStrength::I20mA,
            gpio_drive_cap_t_GPIO_DRIVE_CAP_3 => DriveStrength::I40mA,
            other => panic!("Unknown GPIO pin drive capability: {}", other),
        }
    }
}

// Pull setting for an input.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Pull {
    Floating,
    Up,
    Down,
    UpDown,
}

impl From<Pull> for gpio_pull_mode_t {
    fn from(pull: Pull) -> gpio_pull_mode_t {
        match pull {
            Pull::Floating => gpio_pull_mode_t_GPIO_FLOATING,
            Pull::Up => gpio_pull_mode_t_GPIO_PULLUP_ONLY,
            Pull::Down => gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,
            Pull::UpDown => gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN,
        }
    }
}

/// Digital input or output level.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Level {
    Low,
    High,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(val: Level) -> bool {
        match val {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl core::ops::Not for Level {
    type Output = Level;

    fn not(self) -> Self::Output {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }
}

impl From<embedded_hal_0_2::digital::v2::PinState> for Level {
    fn from(state: embedded_hal_0_2::digital::v2::PinState) -> Self {
        match state {
            embedded_hal_0_2::digital::v2::PinState::Low => Self::Low,
            embedded_hal_0_2::digital::v2::PinState::High => Self::High,
        }
    }
}

impl From<Level> for embedded_hal_0_2::digital::v2::PinState {
    fn from(level: Level) -> Self {
        match level {
            Level::Low => Self::Low,
            Level::High => Self::High,
        }
    }
}

impl From<embedded_hal::digital::PinState> for Level {
    fn from(state: embedded_hal::digital::PinState) -> Self {
        match state {
            embedded_hal::digital::PinState::Low => Self::Low,
            embedded_hal::digital::PinState::High => Self::High,
        }
    }
}

impl From<Level> for embedded_hal::digital::PinState {
    fn from(level: Level) -> Self {
        match level {
            Level::Low => Self::Low,
            Level::High => Self::High,
        }
    }
}

pub trait GPIOMode {}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
pub trait RTCMode {}

pub trait InputMode {
    const RTC: bool;
}

pub trait OutputMode {
    const RTC: bool;
}

pub struct Disabled;
pub struct Input;
pub struct Output;
pub struct InputOutput;
#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
pub struct RtcDisabled;
#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
pub struct RtcInput;
#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
pub struct RtcOutput;
#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
pub struct RtcInputOutput;

impl InputMode for Input {
    const RTC: bool = false;
}

impl InputMode for InputOutput {
    const RTC: bool = false;
}

impl OutputMode for Output {
    const RTC: bool = false;
}

impl OutputMode for InputOutput {
    const RTC: bool = false;
}

impl GPIOMode for Disabled {}
impl GPIOMode for Input {}
impl GPIOMode for InputOutput {}
impl GPIOMode for Output {}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl InputMode for RtcInput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl InputMode for RtcInputOutput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl OutputMode for RtcOutput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl OutputMode for RtcInputOutput {
    const RTC: bool = true;
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl RTCMode for RtcDisabled {}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl RTCMode for RtcInput {}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl RTCMode for RtcInputOutput {}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl RTCMode for RtcOutput {}

/// A driver for a GPIO pin.
///
/// The driver can set the pin as a disconnected/disabled one, input, or output pin, or both or analog.
/// On some chips (i.e. esp32 and esp32s*), the driver can also set the pin in RTC IO mode.
/// Depending on the current operating mode, different sets of functions are available.
///
/// The mode-setting depends on the capabilities of the pin as well, i.e. input-only pins cannot be set
/// into output or input-output mode.
pub struct PinDriver<'d, T: Pin, MODE> {
    pin: PeripheralRef<'d, T>,
    _mode: PhantomData<MODE>,
}

impl<'d, T: Pin> PinDriver<'d, T, Disabled> {
    /// Creates the driver for a pin in disabled state.
    #[inline]
    pub fn disabled(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_disabled()
    }
}

impl<'d, T: InputPin> PinDriver<'d, T, Input> {
    /// Creates the driver for a pin in input state.
    #[inline]
    pub fn input(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input()
    }
}

impl<'d, T: InputPin + OutputPin> PinDriver<'d, T, InputOutput> {
    /// Creates the driver for a pin in input-output state.
    #[inline]
    pub fn input_output(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input_output()
    }
}

impl<'d, T: InputPin + OutputPin> PinDriver<'d, T, InputOutput> {
    /// Creates the driver for a pin in input-output open-drain state.
    #[inline]
    pub fn input_output_od(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_input_output_od()
    }
}

impl<'d, T: OutputPin> PinDriver<'d, T, Output> {
    /// Creates the driver for a pin in output state.
    #[inline]
    pub fn output(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_output()
    }
}

impl<'d, T: OutputPin> PinDriver<'d, T, Output> {
    /// Creates the driver for a pin in output open-drain state.
    #[inline]
    pub fn output_od(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_output_od()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: Pin + RTCPin> PinDriver<'d, T, RtcDisabled> {
    /// Creates the driver for a pin in disabled state.
    #[inline]
    pub fn rtc_disabled(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_disabled()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: InputPin + RTCPin> PinDriver<'d, T, RtcInput> {
    /// Creates the driver for a pin in RTC input state.
    #[inline]
    pub fn rtc_input(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: InputPin + OutputPin + RTCPin> PinDriver<'d, T, RtcInputOutput> {
    /// Creates the driver for a pin in RTC input-output state.
    #[inline]
    pub fn rtc_input_output(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input_output()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: InputPin + OutputPin + RTCPin> PinDriver<'d, T, RtcInputOutput> {
    /// Creates the driver for a pin in RTC input-output open-drain state.
    #[inline]
    pub fn rtc_input_output_od(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_input_output_od()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: OutputPin + RTCPin> PinDriver<'d, T, RtcOutput> {
    /// Creates the driver for a pin in RTC output state.
    #[inline]
    pub fn rtc_output(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_output()
    }
}

#[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
impl<'d, T: OutputPin + RTCPin> PinDriver<'d, T, RtcOutput> {
    /// Creates the driver for a pin in RTC output open-drain state.
    #[inline]
    pub fn rtc_output_od(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_rtc_output_od()
    }
}

impl<'d, T: Pin, MODE> PinDriver<'d, T, MODE> {
    /// Returns the pin number.
    pub fn pin(&self) -> i32 {
        self.pin.pin()
    }

    /// Put the pin into disabled mode.
    pub fn into_disabled(self) -> Result<PinDriver<'d, T, Disabled>, EspError> {
        self.into_mode(gpio_mode_t_GPIO_MODE_DISABLE)
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn into_input(self) -> Result<PinDriver<'d, T, Input>, EspError>
    where
        T: InputPin,
    {
        self.into_mode(gpio_mode_t_GPIO_MODE_INPUT)
    }

    /// Put the pin into input + output mode.
    #[inline]
    pub fn into_input_output(self) -> Result<PinDriver<'d, T, InputOutput>, EspError>
    where
        T: InputPin + OutputPin,
    {
        self.into_mode(gpio_mode_t_GPIO_MODE_INPUT_OUTPUT)
    }

    /// Put the pin into input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    #[inline]
    pub fn into_input_output_od(self) -> Result<PinDriver<'d, T, InputOutput>, EspError>
    where
        T: InputPin + OutputPin,
    {
        self.into_mode(gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD)
    }

    /// Put the pin into output mode.
    #[inline]
    pub fn into_output(self) -> Result<PinDriver<'d, T, Output>, EspError>
    where
        T: OutputPin,
    {
        self.into_mode(gpio_mode_t_GPIO_MODE_OUTPUT)
    }

    /// Put the pin into output Open Drain mode.
    #[inline]
    pub fn into_output_od(self) -> Result<PinDriver<'d, T, Output>, EspError>
    where
        T: OutputPin,
    {
        self.into_mode(gpio_mode_t_GPIO_MODE_OUTPUT_OD)
    }

    /// Put the pin into RTC disabled mode.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_disabled(self) -> Result<PinDriver<'d, T, RtcDisabled>, EspError>
    where
        T: RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_DISABLED)
    }

    /// Put the pin into RTC input mode.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_input(self) -> Result<PinDriver<'d, T, RtcInput>, EspError>
    where
        T: InputPin + RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_ONLY)
    }

    /// Put the pin into RTC input + output mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_input_output(self) -> Result<PinDriver<'d, T, RtcInputOutput>, EspError>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_OUTPUT)
    }

    /// Put the pin into RTC input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_input_output_od(self) -> Result<PinDriver<'d, T, RtcInputOutput>, EspError>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_OUTPUT_OD)
    }

    /// Put the pin into RTC output mode.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_output(self) -> Result<PinDriver<'d, T, RtcOutput>, EspError>
    where
        T: OutputPin + RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_OUTPUT_ONLY)
    }

    /// Put the pin into RTC output Open Drain mode.
    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    pub fn into_rtc_output_od(self) -> Result<PinDriver<'d, T, RtcOutput>, EspError>
    where
        T: OutputPin + RTCPin,
    {
        self.into_rtc_mode(rtc_gpio_mode_t_RTC_GPIO_MODE_OUTPUT_OD)
    }

    #[inline]
    fn into_mode<M>(mut self, mode: gpio_mode_t) -> Result<PinDriver<'d, T, M>, EspError>
    where
        T: Pin,
    {
        let pin = unsafe { self.pin.clone_unchecked() };
        drop(self);

        if mode != gpio_mode_t_GPIO_MODE_DISABLE {
            esp!(unsafe { gpio_set_direction(pin.pin(), mode) })?;
        }

        Ok(PinDriver {
            pin,
            _mode: PhantomData,
        })
    }

    #[inline]
    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    fn into_rtc_mode<M>(mut self, mode: rtc_gpio_mode_t) -> Result<PinDriver<'d, T, M>, EspError>
    where
        T: RTCPin,
    {
        let pin = unsafe { self.pin.clone_unchecked() };

        drop(self);

        #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
        {
            esp!(unsafe { rtc_gpio_init(pin.pin()) })?;
            esp!(unsafe { rtc_gpio_set_direction(pin.pin(), mode) })?;
        }

        Ok(PinDriver {
            pin,
            _mode: PhantomData,
        })
    }

    #[inline]
    pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError>
    where
        MODE: OutputMode,
    {
        let mut cap: gpio_drive_cap_t = 0;

        if MODE::RTC {
            #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
            esp!(unsafe { rtc_gpio_get_drive_capability(self.pin.pin(), &mut cap) })?;

            #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_get_drive_capability(self.pin.pin(), &mut cap) })?;
        }

        Ok(cap.into())
    }

    #[inline]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
            esp!(unsafe { rtc_gpio_set_drive_capability(self.pin.pin(), strength.into()) })?;

            #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_set_drive_capability(self.pin.pin(), strength.into()) })?;
        }

        Ok(())
    }

    #[inline]
    pub fn is_high(&self) -> bool
    where
        MODE: InputMode,
    {
        self.get_level().into()
    }

    #[inline]
    pub fn is_low(&self) -> bool
    where
        MODE: InputMode,
    {
        !self.is_high()
    }

    #[inline]
    pub fn get_level(&self) -> Level
    where
        MODE: InputMode,
    {
        let res;

        if MODE::RTC {
            #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
            {
                res = if unsafe { rtc_gpio_get_level(self.pin.pin()) } != 0 {
                    Level::High
                } else {
                    Level::Low
                };
            }

            #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
            unreachable!();
        } else if unsafe { gpio_get_level(self.pin.pin()) } != 0 {
            res = Level::High;
        } else {
            res = Level::Low;
        }

        res
    }

    #[inline]
    pub fn is_set_high(&self) -> bool
    where
        MODE: OutputMode,
    {
        !self.is_set_low()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool
    where
        MODE: OutputMode,
    {
        self.get_output_level() == Level::Low
    }

    /// What level output is set to
    #[inline]
    fn get_output_level(&self) -> Level
    where
        MODE: OutputMode,
    {
        // TODO: Implement for RTC mode

        let pin = self.pin.pin() as u32;

        #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
        let is_set_high = unsafe { (*(GPIO_OUT_REG as *const u32) >> pin) & 0x01 != 0 };
        #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
        let is_set_high = if pin <= 31 {
            // GPIO0 - GPIO31
            unsafe { (*(GPIO_OUT_REG as *const u32) >> pin) & 0x01 != 0 }
        } else {
            // GPIO32+
            unsafe { (*(GPIO_OUT1_REG as *const u32) >> (pin - 32)) & 0x01 != 0 }
        };

        if is_set_high {
            Level::High
        } else {
            Level::Low
        }
    }

    #[inline]
    pub fn set_high(&mut self) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        self.set_level(Level::High)
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        self.set_level(Level::Low)
    }

    #[inline]
    pub fn set_level(&mut self, level: Level) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        let on = match level {
            Level::Low => 0,
            Level::High => 1,
        };

        if MODE::RTC {
            #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
            esp!(unsafe { rtc_gpio_set_level(self.pin.pin(), on) })?;

            #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_set_level(self.pin.pin(), on) })?;
        }

        Ok(())
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }

    pub fn set_pull(&mut self, pull: Pull) -> Result<(), EspError>
    where
        T: InputPin + OutputPin,
        MODE: InputMode,
    {
        if MODE::RTC {
            #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
            unsafe {
                match pull {
                    Pull::Down => {
                        esp!(rtc_gpio_pulldown_en(self.pin.pin()))?;
                        esp!(rtc_gpio_pullup_dis(self.pin.pin()))?;
                    }
                    Pull::Up => {
                        esp!(rtc_gpio_pulldown_dis(self.pin.pin()))?;
                        esp!(rtc_gpio_pullup_en(self.pin.pin()))?;
                    }
                    Pull::UpDown => {
                        esp!(rtc_gpio_pulldown_en(self.pin.pin()))?;
                        esp!(rtc_gpio_pullup_en(self.pin.pin()))?;
                    }
                    Pull::Floating => {
                        esp!(rtc_gpio_pulldown_dis(self.pin.pin()))?;
                        esp!(rtc_gpio_pullup_dis(self.pin.pin()))?;
                    }
                }
            }

            #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_set_pull_mode(self.pin.pin(), pull.into()) })?;
        }

        Ok(())
    }

    /// Subscribes the provided callback for ISR notifications.
    /// As a side effect, interrupts will be disabled, so to receive a notification, one has
    /// to also call `PinDriver::enable_interrupt` after calling this method.
    ///
    /// Note that `PinDriver::enable_interrupt` should also be called after
    /// each received notification **from non-ISR context**, because the driver will automatically
    /// disable ISR interrupts on each received ISR notification (so as to avoid IWDT triggers).
    ///
    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe<F: FnMut() + Send + 'static>(
        &mut self,
        callback: F,
    ) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        self.internal_subscribe(callback)
    }

    /// Subscribes the provided callback for ISR notifications.
    /// As a side effect, interrupts will be disabled, so to receive a notification, one has
    /// to also call `PinDriver::enable_interrupt` after calling this method.
    ///
    /// Note that `PinDriver::enable_interrupt` should also be called after
    /// each received notification **from non-ISR context**, because the driver will automatically
    /// disable ISR interrupts on each received ISR notification (so as to avoid IWDT triggers).
    ///
    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// Additionally, this method - in contrast to method `subscribe` - allows
    /// the passed-in callback/closure to be non-`'static`. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    /// scope where the driver is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the driver,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the driver might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent and owned by an ISR routine,
    /// which means that if the driver is forgotten, Rust is free to e.g. unwind the stack
    /// and the ISR routine will end up with references to variables that no longer exist.
    ///
    /// The destructor of the driver takes care - prior to the driver being dropped and e.g.
    /// the stack being unwind - to unsubscribe the ISR routine.
    /// Unfortunately, when the driver is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe_nonstatic<F: FnMut() + Send + 'd>(
        &mut self,
        callback: F,
    ) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        self.internal_subscribe(callback)
    }

    #[cfg(feature = "alloc")]
    fn internal_subscribe(&mut self, callback: impl FnMut() + Send + 'd) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        extern crate alloc;

        self.disable_interrupt()?;

        let callback: alloc::boxed::Box<dyn FnMut() + Send + 'd> = alloc::boxed::Box::new(callback);
        unsafe {
            chip::PIN_ISR_HANDLER[self.pin.pin() as usize] = Some(core::mem::transmute::<
                alloc::boxed::Box<dyn FnMut() + Send>,
                alloc::boxed::Box<dyn FnMut() + Send>,
            >(callback));
        }

        Ok(())
    }

    pub fn unsubscribe(&mut self) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        unsafe {
            unsubscribe_pin(self.pin.pin())?;
        }

        Ok(())
    }

    /// Enables or re-enables the interrupt
    ///
    /// Note that the interrupt is automatically disabled each time an interrupt is triggered
    /// (or else we risk entering a constant interrupt processing loop while the pin is in low/high state
    /// and the interrupt type is set to non-edge)
    ///
    /// Therefore - to continue receiving ISR interrupts - user needs to call `enable_interrupt`
    /// - **from a non-ISR context** - after each successful interrupt triggering.
    pub fn enable_interrupt(&mut self) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        enable_isr_service()?;

        unsafe {
            esp!(gpio_isr_handler_add(
                self.pin.pin(),
                Some(Self::handle_isr),
                self.pin.pin() as u32 as *mut core::ffi::c_void,
            ))
        }
    }

    pub fn disable_interrupt(&mut self) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        use core::sync::atomic::Ordering;

        if ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { gpio_isr_handler_remove(self.pin.pin()) })?;
        }

        Ok(())
    }

    pub fn set_interrupt_type(&mut self, interrupt_type: InterruptType) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        esp!(unsafe { gpio_set_intr_type(self.pin.pin(), interrupt_type.into()) })?;

        Ok(())
    }

    unsafe extern "C" fn handle_isr(user_ctx: *mut core::ffi::c_void) {
        let pin = user_ctx as u32;

        // IMPORTANT: MUST be done or else the ESP IDF GPIO driver will continue calling us in a loop
        // - particularly when the interrupt type is set to non-edge triggering (pin high or low) -
        // which will eventually cause the Interrupt WatchDog to kick in
        gpio_intr_disable(pin as _);

        PIN_NOTIF[pin as usize].notify_lsb();

        #[cfg(feature = "alloc")]
        {
            if let Some(unsafe_callback) = unsafe { &mut PIN_ISR_HANDLER[pin as usize] } {
                (unsafe_callback)();
            }
        }
    }
}

impl<T: Pin, MODE: InputMode> PinDriver<'_, T, MODE> {
    pub async fn wait_for(&mut self, interrupt_type: InterruptType) -> Result<(), EspError> {
        self.disable_interrupt()?;

        let notif = &chip::PIN_NOTIF[self.pin.pin() as usize];

        notif.reset();

        match interrupt_type {
            InterruptType::LowLevel => {
                if self.is_low() {
                    return Ok(());
                }
            }
            InterruptType::HighLevel => {
                if self.is_high() {
                    return Ok(());
                }
            }
            _ => (),
        }

        self.set_interrupt_type(interrupt_type)?;
        self.enable_interrupt()?;

        notif.wait().await;

        Ok(())
    }

    pub async fn wait_for_high(&mut self) -> Result<(), EspError> {
        self.wait_for(InterruptType::HighLevel).await
    }

    pub async fn wait_for_low(&mut self) -> Result<(), EspError> {
        self.wait_for(InterruptType::LowLevel).await
    }

    pub async fn wait_for_rising_edge(&mut self) -> Result<(), EspError> {
        self.wait_for(InterruptType::PosEdge).await
    }

    pub async fn wait_for_falling_edge(&mut self) -> Result<(), EspError> {
        self.wait_for(InterruptType::NegEdge).await
    }

    pub async fn wait_for_any_edge(&mut self) -> Result<(), EspError> {
        self.wait_for(InterruptType::AnyEdge).await
    }
}

impl<'d, T: Pin, MODE> Drop for PinDriver<'d, T, MODE> {
    fn drop(&mut self) {
        gpio_reset_without_pull(self.pin.pin()).unwrap();
    }
}

unsafe impl<'d, T: Pin, MODE> Send for PinDriver<'d, T, MODE> {}

impl<'d, T: Pin, MODE> embedded_hal_0_2::digital::v2::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    type Error = EspError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_low(self))
    }
}

use crate::embedded_hal_error;
embedded_hal_error!(
    GpioError,
    embedded_hal::digital::Error,
    embedded_hal::digital::ErrorKind
);

fn to_gpio_err(err: EspError) -> GpioError {
    GpioError::other(err)
}

impl<'d, T: Pin, MODE> embedded_hal::digital::ErrorType for PinDriver<'d, T, MODE> {
    type Error = GpioError;
}

impl<'d, T: Pin, MODE> embedded_hal::digital::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_low(self))
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::InputPin for &PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(PinDriver::is_low(self))
    }
}

impl<'d, T: Pin, MODE> embedded_hal_0_2::digital::v2::OutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    type Error = EspError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low)
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::OutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High).map_err(to_gpio_err)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low).map_err(to_gpio_err)
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::StatefulOutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level().into())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_output_level()))
    }
}

// TODO: Will become possible once the `PinDriver::setXXX`` methods become non-`&mut`, which they really are, internally
// impl<'d, T: Pin, MODE> embedded_hal::digital::StatefulOutputPin for &PinDriver<'d, T, MODE>
// where
//     MODE: OutputMode,
// {
//     fn is_set_high(&mut self) -> Result<bool, Self::Error> {
//         Ok(self.get_output_level().into())
//     }

//     fn is_set_low(&mut self) -> Result<bool, Self::Error> {
//         Ok(!bool::from(self.get_output_level()))
//     }
// }

impl<'d, T: Pin, MODE> embedded_hal_0_2::digital::v2::StatefulOutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level().into())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_output_level()))
    }
}

impl<'d, T: Pin, MODE> embedded_hal_0_2::digital::v2::ToggleableOutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    type Error = EspError;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::from(!bool::from(self.get_output_level())))
    }
}

impl<T: Pin, MODE: InputMode> embedded_hal_async::digital::Wait for PinDriver<'_, T, MODE> {
    async fn wait_for_high(&mut self) -> Result<(), GpioError> {
        self.wait_for_high().await?;

        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), GpioError> {
        self.wait_for_low().await?;

        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), GpioError> {
        self.wait_for_rising_edge().await?;

        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), GpioError> {
        self.wait_for_falling_edge().await?;

        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), GpioError> {
        self.wait_for_any_edge().await?;

        Ok(())
    }
}

static ISR_ALLOC_FLAGS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

static ISR_SERVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

static ISR_SERVICE_ENABLED_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

pub fn init_isr_alloc_flags(flags: enumset::EnumSet<crate::interrupt::InterruptType>) {
    ISR_ALLOC_FLAGS.store(
        crate::interrupt::InterruptType::to_native(flags),
        core::sync::atomic::Ordering::SeqCst,
    );
}

pub fn enable_isr_service() -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        let _guard = ISR_SERVICE_ENABLED_CS.enter();

        if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { gpio_install_isr_service(ISR_ALLOC_FLAGS.load(Ordering::SeqCst) as _) })?;

            ISR_SERVICE_ENABLED.store(true, Ordering::SeqCst);
        }
    }

    Ok(())
}

pub(crate) unsafe fn rtc_reset_pin(pin: i32) -> Result<(), EspError> {
    gpio_reset_without_pull(pin)?;

    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5)))]
    esp!(rtc_gpio_init(pin))?;

    Ok(())
}

// The default esp-idf gpio_reset function sets a pull-up. If that behaviour is
// not desired this function can be used instead.
#[inline]
fn gpio_reset_without_pull(pin: gpio_num_t) -> Result<(), EspError> {
    let cfg = gpio_config_t {
        pin_bit_mask: (1u64 << pin),
        mode: esp_idf_sys::gpio_mode_t_GPIO_MODE_DISABLE,
        pull_up_en: esp_idf_sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: esp_idf_sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
        intr_type: esp_idf_sys::gpio_int_type_t_GPIO_INTR_DISABLE,
        #[cfg(esp32h2)]
        hys_ctrl_mode: esp_idf_sys::gpio_hys_ctrl_mode_t_GPIO_HYS_SOFT_DISABLE,
    };

    unsafe {
        unsubscribe_pin(pin)?;
        esp!(gpio_config(&cfg))?;
    }
    Ok(())
}

unsafe fn unsubscribe_pin(pin: i32) -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        esp!(gpio_isr_handler_remove(pin))?;

        chip::PIN_NOTIF[pin as usize].reset();

        #[cfg(feature = "alloc")]
        {
            chip::PIN_ISR_HANDLER[pin as usize] = None;
        }
    }

    Ok(())
}

#[cfg(feature = "alloc")]
#[allow(clippy::declare_interior_mutable_const)] // OK because this is only used as an array initializer
const PIN_ISR_INIT: Option<alloc::boxed::Box<dyn FnMut() + Send + 'static>> = None;

#[allow(clippy::declare_interior_mutable_const)] // OK because this is only used as an array initializer
const PIN_NOTIF_INIT: crate::interrupt::asynch::HalIsrNotification =
    crate::interrupt::asynch::HalIsrNotification::new();

macro_rules! impl_input {
    ($pxi:ident: $pin:expr) => {
        crate::impl_peripheral!($pxi);

        impl Pin for $pxi {
            fn pin(&self) -> i32 {
                $pin
            }
        }

        impl InputPin for $pxi {}

        impl From<$pxi> for AnyInputPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input!($pxi: $pin);

        impl OutputPin for $pxi {}

        impl IOPin for $pxi {}

        impl From<$pxi> for AnyOutputPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }

        impl From<$pxi> for AnyIOPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }
    };
}

macro_rules! impl_rtc {
    ($pxi:ident: $pin:expr, RTC: $rtc:expr) => {
        impl RTCPin for $pxi {
            fn rtc_pin(&self) -> i32 {
                $rtc
            }
        }
    };

    ($pxi:ident: $pin:expr, NORTC: $rtc:expr) => {};
}

macro_rules! impl_adc {
    ($pxi:ident: $pin:expr, ADC1: $adc:expr) => {
        impl sealed::ADCPin for $pxi {
            const CHANNEL: adc_channel_t = $adc;
        }

        impl ADCPin for $pxi {
            type Adc = ADC1;

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, ADC2: $adc:expr) => {
        impl sealed::ADCPin for $pxi {
            const CHANNEL: adc_channel_t = $adc;
        }

        #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
        impl ADCPin for $pxi {
            type Adc = ADC2;

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, NOADC: $adc:expr) => {};
}

macro_rules! impl_dac {
    ($pxi:ident: $pin:expr, DAC: $dac:expr) => {
        #[cfg(any(esp32, esp32s2))]
        impl DACPin for $pxi {
            fn dac_channel(&self) -> dac_channel_t {
                $dac
            }
        }
    };

    ($pxi:ident: $pin:expr, NODAC: $dac:expr) => {};
}

macro_rules! impl_touch {
    ($pxi:ident: $pin:expr, TOUCH: $touch:expr) => {
        #[cfg(any(esp32, esp32s2, esp32s3))]
        impl TouchPin for $pxi {
            fn touch_channel(&self) -> touch_pad_t {
                $touch
            }
        }
    };

    ($pxi:ident: $pin:expr, NOTOUCH: $touch:expr) => {};
}

macro_rules! pin {
    ($pxi:ident: $pin:expr, Input, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr) => {
        impl_input!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };

    ($pxi:ident: $pin:expr, IO, $rtc:ident: $rtcno:expr, $adc:ident: $adcno:expr, $dac:ident: $dacno:expr, $touch:ident: $touchno:expr) => {
        impl_input_output!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc: $rtcno);
        impl_adc!($pxi: $pin, $adc: $adcno);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };
}

#[cfg(esp32)]
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 40] =
        [PIN_ISR_INIT; 40];

    #[allow(clippy::type_complexity)]
    pub(crate) static PIN_NOTIF: [HalIsrNotification; 40] = [PIN_NOTIF_INIT; 40];

    // NOTE: Gpio26 - Gpio32 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0, IO, RTC:11, ADC2:1, NODAC:0, TOUCH:1);
    pin!(Gpio1:1, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2, IO, RTC:12, ADC2:2, NODAC:0, TOUCH:2);
    pin!(Gpio3:3, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4, IO, RTC:10, ADC2:0, NODAC:0, TOUCH:0);
    pin!(Gpio5:5, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, RTC:15, ADC2:5, NODAC:0, TOUCH:5);
    pin!(Gpio13:13, IO, RTC:14, ADC2:4, NODAC:0, TOUCH:4);
    pin!(Gpio14:14, IO, RTC:16, ADC2:6, NODAC:0, TOUCH:6);
    pin!(Gpio15:15, IO, RTC:13, ADC2:3, NODAC:0, TOUCH:3);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio22:22, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio23:23, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio25:25, IO, RTC:6, ADC2:8, DAC:1, NOTOUCH:0);
    pin!(Gpio26:26, IO, RTC:7, ADC2:9, DAC:2, NOTOUCH:0);
    pin!(Gpio27:27, IO, RTC:17, ADC2:7, NODAC:0, TOUCH:7);
    pin!(Gpio32:32, IO, RTC:9, ADC1:4, NODAC:0, TOUCH:9);
    pin!(Gpio33:33, IO, RTC:8, ADC1:5, NODAC:0, TOUCH:8);
    pin!(Gpio34:34, Input, RTC:4, ADC1:6, NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, Input, RTC:5, ADC1:7, NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, Input, RTC:0, ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, Input, RTC:1, ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, Input, RTC:2, ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, Input, RTC:3, ADC1:3, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
        pub gpio22: Gpio22,
        pub gpio23: Gpio23,
        pub gpio25: Gpio25,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
        pub gpio34: Gpio34,
        pub gpio35: Gpio35,
        pub gpio36: Gpio36,
        pub gpio37: Gpio37,
        pub gpio38: Gpio38,
        pub gpio39: Gpio39,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
                gpio22: Gpio22::new(),
                gpio23: Gpio23::new(),
                gpio25: Gpio25::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio32: Gpio32::new(),
                gpio33: Gpio33::new(),
                gpio34: Gpio34::new(),
                gpio35: Gpio35::new(),
                gpio36: Gpio36::new(),
                gpio37: Gpio37::new(),
                gpio38: Gpio38::new(),
                gpio39: Gpio39::new(),
            }
        }
    }
}

#[cfg(any(esp32s2, esp32s3))]
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 49] =
        [PIN_ISR_INIT; 49];

    #[allow(clippy::type_complexity)]
    pub(crate) static PIN_NOTIF: [HalIsrNotification; 49] = [PIN_NOTIF_INIT; 49];

    // NOTE: Gpio26 - Gpio32 (and Gpio33 - Gpio37 if using Octal RAM/Flash) are used
    //       by SPI0/SPI1 for external PSRAM/SPI Flash and are not recommended for
    //       other uses
    pin!(Gpio0:0, IO, RTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1, IO, RTC:1, ADC1:0, NODAC:0, TOUCH:1);
    pin!(Gpio2:2, IO, RTC:2, ADC1:1, NODAC:0, TOUCH:2);
    pin!(Gpio3:3, IO, RTC:3, ADC1:2, NODAC:0, TOUCH:3);
    pin!(Gpio4:4, IO, RTC:4, ADC1:3, NODAC:0, TOUCH:4);
    pin!(Gpio5:5, IO, RTC:5, ADC1:4, NODAC:0, TOUCH:5);
    pin!(Gpio6:6, IO, RTC:6, ADC1:5, NODAC:0, TOUCH:6);
    pin!(Gpio7:7, IO, RTC:7, ADC1:6, NODAC:0, TOUCH:7);
    pin!(Gpio8:8, IO, RTC:8, ADC1:7, NODAC:0, TOUCH:8);
    pin!(Gpio9:9, IO, RTC:9, ADC1:8, NODAC:0, TOUCH:9);
    pin!(Gpio10:10, IO, RTC:10, ADC1:9, NODAC:0, TOUCH:10);
    pin!(Gpio11:11, IO, RTC:11, ADC2:0, NODAC:0, TOUCH:11);
    pin!(Gpio12:12, IO, RTC:12, ADC2:1, NODAC:0, TOUCH:12);
    pin!(Gpio13:13, IO, RTC:13, ADC2:2, NODAC:0, TOUCH:13);
    pin!(Gpio14:14, IO, RTC:14, ADC2:3, NODAC:0, TOUCH:14);
    pin!(Gpio15:15, IO, RTC:15, ADC2:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, RTC:16, ADC2:5, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s2)]
    pin!(Gpio17:17, IO, RTC:17, ADC2:6, DAC:1, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio17:17, IO, RTC:17, ADC2:6, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s2)]
    pin!(Gpio18:18, IO, RTC:18, ADC2:7, DAC:2, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio18:18, IO, RTC:18, ADC2:7, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, RTC:19, ADC2:8, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, RTC:20, ADC2:9, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, RTC:21, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio31:31, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio32:32, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio33:33, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio34:34, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio35:35, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio36:36, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio37:37, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio38:38, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio39:39, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio40:40, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio41:41, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio42:42, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio43:43, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio44:44, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio45:45, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s2)]
    pin!(Gpio46:46, Input, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio46:46, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio47:47, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(esp32s3)]
    pin!(Gpio48:48, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio28: Gpio28,
        pub gpio29: Gpio29,
        pub gpio30: Gpio30,
        pub gpio31: Gpio31,
        pub gpio32: Gpio32,
        pub gpio33: Gpio33,
        pub gpio34: Gpio34,
        pub gpio35: Gpio35,
        pub gpio36: Gpio36,
        pub gpio37: Gpio37,
        pub gpio38: Gpio38,
        pub gpio39: Gpio39,
        pub gpio40: Gpio40,
        pub gpio41: Gpio41,
        pub gpio42: Gpio42,
        pub gpio43: Gpio43,
        pub gpio44: Gpio44,
        pub gpio45: Gpio45,
        pub gpio46: Gpio46,
        #[cfg(esp32s3)]
        pub gpio47: Gpio47,
        #[cfg(esp32s3)]
        pub gpio48: Gpio48,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio28: Gpio28::new(),
                gpio29: Gpio29::new(),
                gpio30: Gpio30::new(),
                gpio31: Gpio31::new(),
                gpio32: Gpio32::new(),
                gpio33: Gpio33::new(),
                gpio34: Gpio34::new(),
                gpio35: Gpio35::new(),
                gpio36: Gpio36::new(),
                gpio37: Gpio37::new(),
                gpio38: Gpio38::new(),
                gpio39: Gpio39::new(),
                gpio40: Gpio40::new(),
                gpio41: Gpio41::new(),
                gpio42: Gpio42::new(),
                gpio43: Gpio43::new(),
                gpio44: Gpio44::new(),
                gpio45: Gpio45::new(),
                gpio46: Gpio46::new(),
                #[cfg(esp32s3)]
                gpio47: Gpio47::new(),
                #[cfg(esp32s3)]
                gpio48: Gpio48::new(),
            }
        }
    }
}

#[cfg(esp32c3)]
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 22] =
        [PIN_ISR_INIT; 22];

    pub(crate) static PIN_NOTIF: [HalIsrNotification; 22] = [PIN_NOTIF_INIT; 22];

    // NOTE: Gpio12 - Gpio17 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO,   RTC:0,  ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO,   RTC:1,  ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO,   RTC:2,  ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO,   RTC:3,  ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO,   RTC:4,  ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO,   RTC:5,  ADC2:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
            }
        }
    }
}

#[cfg(esp32c2)]
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::ADC1;

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 20] =
        [PIN_ISR_INIT; 20];

    pub(crate) static PIN_NOTIF: [HalIsrNotification; 20] = [PIN_NOTIF_INIT; 20];

    // NOTE: Gpio12 - Gpio17 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO,   RTC:0,  ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO,   RTC:1,  ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO,   RTC:2,  ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO,   RTC:3,  ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO,   RTC:4,  ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO,   RTC:5, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
            }
        }
    }
}

#[cfg(esp32h2)]
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::ADC1;

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 20] =
        [PIN_ISR_INIT; 20];

    pub(crate) static PIN_NOTIF: [HalIsrNotification; 20] = [PIN_NOTIF_INIT; 20];

    // NOTE: Gpio12 - Gpio17 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1,   IO, NORTC:0,  ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2,   IO, NORTC:0,  ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3,   IO, NORTC:0,  ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4,   IO, NORTC:0,  ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5,   IO, NORTC:0,  ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6,   IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7,   IO,   RTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8,   IO,   RTC:1, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9,   IO,   RTC:2, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO,   RTC:3, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO,   RTC:4, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO,   RTC:5, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO,   RTC:6, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO,   RTC:7, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
            }
        }
    }
}

// TODO: Implement esp32c6 glitch filters

#[cfg(any(esp32c5, esp32c6, esp32p4))] // TODO: Implement proper pin layout for esp32c5 and esp32p4
mod chip {
    #[cfg(feature = "alloc")]
    extern crate alloc;

    #[cfg(feature = "alloc")]
    use alloc::boxed::Box;

    use esp_idf_sys::*;

    use crate::interrupt::asynch::HalIsrNotification;

    use crate::adc::ADC1;

    use super::*;

    #[allow(clippy::type_complexity)]
    #[cfg(feature = "alloc")]
    pub(crate) static mut PIN_ISR_HANDLER: [Option<Box<dyn FnMut() + Send + 'static>>; 30] =
        [PIN_ISR_INIT; 30];

    #[allow(clippy::type_complexity)]
    pub(crate) static PIN_NOTIF: [HalIsrNotification; 30] = [PIN_NOTIF_INIT; 30];

    // NOTE: Gpio26 - Gpio32 (and Gpio33 - Gpio37 if using Octal RAM/Flash) are used
    //       by SPI0/SPI1 for external PSRAM/SPI Flash and are not recommended for
    //       other uses
    pin!(Gpio0:0, IO, RTC:0, ADC1:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio1:1, IO, RTC:1, ADC1:1, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2, IO, RTC:2, ADC1:2, NODAC:0, NOTOUCH:0);
    pin!(Gpio3:3, IO, RTC:3, ADC1:3, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4, IO, RTC:4, ADC1:4, NODAC:0, NOTOUCH:0);
    pin!(Gpio5:5, IO, RTC:5, ADC1:5, NODAC:0, NOTOUCH:0);
    pin!(Gpio6:6, IO, RTC:6, ADC1:6, NODAC:0, NOTOUCH:0);
    pin!(Gpio7:7, IO, RTC:7, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio8:8, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio9:9, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio13:13, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio14:14, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio15:15, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio20:20, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio22:22, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio23:23, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio24:24, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio25:25, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0,
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        pub gpio5: Gpio5,
        pub gpio6: Gpio6,
        pub gpio7: Gpio7,
        pub gpio8: Gpio8,
        pub gpio9: Gpio9,
        pub gpio10: Gpio10,
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        pub gpio16: Gpio16,
        pub gpio17: Gpio17,
        pub gpio18: Gpio18,
        pub gpio19: Gpio19,
        pub gpio20: Gpio20,
        pub gpio21: Gpio21,
        pub gpio22: Gpio22,
        pub gpio23: Gpio23,
        pub gpio24: Gpio24,
        pub gpio25: Gpio25,
        pub gpio26: Gpio26,
        pub gpio27: Gpio27,
        pub gpio28: Gpio28,
        pub gpio29: Gpio29,
        pub gpio30: Gpio30,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::new(),
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                gpio5: Gpio5::new(),
                gpio6: Gpio6::new(),
                gpio7: Gpio7::new(),
                gpio8: Gpio8::new(),
                gpio9: Gpio9::new(),
                gpio10: Gpio10::new(),
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                gpio16: Gpio16::new(),
                gpio17: Gpio17::new(),
                gpio18: Gpio18::new(),
                gpio19: Gpio19::new(),
                gpio20: Gpio20::new(),
                gpio21: Gpio21::new(),
                gpio22: Gpio22::new(),
                gpio23: Gpio23::new(),
                gpio24: Gpio24::new(),
                gpio25: Gpio25::new(),
                gpio26: Gpio26::new(),
                gpio27: Gpio27::new(),
                gpio28: Gpio28::new(),
                gpio29: Gpio29::new(),
                gpio30: Gpio30::new(),
            }
        }
    }
}
