//! GPIO and pin configuration

use core::marker::PhantomData;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::boxed::Box;

#[cfg(not(feature = "riscv-ulp-hal"))]
use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::sys::*;

use crate::adc::Adc;
use crate::peripheral::{Peripheral, PeripheralRef};

pub use chip::*;

/// A trait implemented by every pin instance
pub trait Pin: Peripheral<P = Self> + Sized + Send + 'static {
    fn pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an input pin
pub trait InputPin: Pin {}

/// A marker trait designating a pin which is capable of
/// operating as an output pin
pub trait OutputPin: Pin {}

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
#[cfg(all(not(esp32c3), not(esp32s3)))]
pub trait DACPin: Pin {
    fn dac_channel(&self) -> dac_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a touch pin
#[cfg(not(esp32c3))]
pub trait TouchPin: Pin {
    fn touch_channel(&self) -> touch_pad_t;
}

/// Generic Gpio input-output pin
pub struct AnyPin {
    pin: i32,
    _p: PhantomData<*const ()>,
}

impl AnyPin {
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
}

crate::impl_peripheral_trait!(AnyPin);

impl Pin for AnyPin {
    fn pin(&self) -> i32 {
        self.pin
    }
}

impl InputPin for AnyPin {}
impl OutputPin for AnyPin {}

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
}

crate::impl_peripheral_trait!(AnyInputPin);

impl Pin for AnyInputPin {
    fn pin(&self) -> i32 {
        self.pin
    }
}

impl InputPin for AnyInputPin {}

/// Interrupt types
#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
pub enum InterruptType {
    PosEdge,
    NegEdge,
    AnyEdge,
    LowLevel,
    HighLevel,
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
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

/// Drive strength (values are approximates)
#[cfg(not(feature = "riscv-ulp-hal"))]
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
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

#[cfg(not(feature = "riscv-ulp-hal"))]
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

#[cfg(not(feature = "riscv-ulp-hal"))]
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
#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
pub struct RtcDisabled;
#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
pub struct RtcInput;
#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
pub struct RtcOutput;
#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
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

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
impl InputMode for RtcInput {
    const RTC: bool = true;
}

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
impl InputMode for RtcInputOutput {
    const RTC: bool = true;
}

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
impl OutputMode for RtcOutput {
    const RTC: bool = true;
}

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
impl OutputMode for RtcInputOutput {
    const RTC: bool = true;
}

/// A driver for a GPIO pin.
///
/// The driver can set the pin as a disconnected/disabled one, input, or output pin, or both or analog. Depending
/// on the current operating mode, different sets of functions are available.
///
/// The mode-setting depends on the capabilities of the pin as well, i.e. input-only pins cannot be set into output mode.
pub struct PinDriver<'d, T: Pin, MODE> {
    pin: PeripheralRef<'d, T>,
    _mode: PhantomData<MODE>,
}

impl<'d, T: Pin, MODE> PinDriver<'d, T, MODE> {
    /// Wrap the pin in a `Flex`.
    ///
    /// The pin remains disconnected. The initial output level is unspecified, but can be changed
    /// before the pin is put into output mode.
    ///
    #[inline]
    pub fn new(pin: impl Peripheral<P = T> + 'd) -> Result<PinDriver<'d, T, Disabled>, EspError> {
        let pin = pin.into_ref();

        Self {
            pin,
            _mode: PhantomData,
        }
        .into_disabled()
    }

    /// Put the pin into disabled mode.
    pub fn into_disabled(mut self) -> Result<PinDriver<'d, T, Disabled>, EspError> {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_DISABLE) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into input mode.
    #[inline]
    pub fn into_input(mut self) -> Result<PinDriver<'d, T, Input>, EspError>
    where
        T: InputPin,
    {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_INPUT) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into input + output mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn into_input_output(mut self) -> Result<PinDriver<'d, T, InputOutput>, EspError>
    where
        T: InputPin + OutputPin,
    {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn into_input_output_od(mut self) -> Result<PinDriver<'d, T, InputOutput>, EspError>
    where
        T: InputPin + OutputPin,
    {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn into_output(mut self) -> Result<PinDriver<'d, T, Output>, EspError>
    where
        T: OutputPin,
    {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_OUTPUT) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into output Open Drain mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn into_output_od(mut self) -> Result<PinDriver<'d, T, Output>, EspError>
    where
        T: OutputPin,
    {
        self.reset()?;

        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_OUTPUT_OD) })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC disabled mode.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_disabled(mut self) -> Result<PinDriver<'d, T, RtcDisabled>, EspError> {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(self.pin.pin(), rtc_gpio_mode_t_RTC_GPIO_MODE_DISABLED)
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC input mode.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_input(mut self) -> Result<PinDriver<'d, T, RtcInput>, EspError>
    where
        T: InputPin,
    {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(self.pin.pin(), rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_ONLY)
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC input + output mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_input_output(mut self) -> Result<PinDriver<'d, T, RtcInputOutput>, EspError>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(self.pin.pin(), rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_OUTPUT)
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC input + output Open Drain mode.
    ///
    /// This is commonly used for "open drain" mode.
    /// the hardware will drive the line low if you set it to low, and will leave it floating if you set
    /// it to high, in which case you can read the input to figure out whether another device
    /// is driving the line low.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_input_output_od(mut self) -> Result<PinDriver<'d, T, RtcInputOutput>, EspError>
    where
        T: InputPin + OutputPin + RTCPin,
    {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(
                self.pin.pin(),
                rtc_gpio_mode_t_RTC_GPIO_MODE_INPUT_OUTPUT_OD,
            )
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_output(mut self) -> Result<PinDriver<'d, T, RtcOutput>, EspError>
    where
        T: OutputPin + RTCPin,
    {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(self.pin.pin(), rtc_gpio_mode_t_RTC_GPIO_MODE_OUTPUT_ONLY)
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    /// Put the pin into RTC output Open Drain mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    pub fn into_rtc_output_od(mut self) -> Result<PinDriver<'d, T, RtcOutput>, EspError>
    where
        T: OutputPin + RTCPin,
    {
        self.rtc_reset()?;

        esp!(unsafe {
            rtc_gpio_set_direction(self.pin.pin(), rtc_gpio_mode_t_RTC_GPIO_MODE_OUTPUT_OD)
        })?;

        Ok(PinDriver {
            pin: unsafe { self.pin.clone_unchecked() },
            _mode: PhantomData,
        })
    }

    #[inline]
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError>
    where
        MODE: OutputMode,
    {
        let mut cap: gpio_drive_cap_t = 0;

        if MODE::RTC {
            #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
            esp!(unsafe { rtc_gpio_get_drive_capability(self.pin.pin(), &mut cap as *mut _) })?;

            #[cfg(any(feature = "riscv-ulp-hal", esp32c3))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_get_drive_capability(self.pin.pin(), &mut cap as *mut _) })?;
        }

        Ok(cap.into())
    }

    #[inline]
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError>
    where
        MODE: OutputMode,
    {
        if MODE::RTC {
            #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
            esp!(unsafe { rtc_gpio_set_drive_capability(self.pin.pin(), strength.into()) })?;

            #[cfg(any(feature = "riscv-ulp-hal", esp32c3))]
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
        !self.is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool
    where
        MODE: InputMode,
    {
        self.get_level().into()
    }

    #[inline]
    pub fn get_level(&self) -> Level
    where
        MODE: InputMode,
    {
        if MODE::RTC {
            #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
            let mode = if unsafe { rtc_gpio_get_level(self.pin.pin()) } != 0 {
                Level::High
            } else {
                Level::Low
            };

            #[cfg(any(feature = "riscv-ulp-hal", esp32c3))]
            let mode = unreachable!();

            mode
        } else if unsafe { gpio_get_level(self.pin.pin()) } != 0 {
            Level::High
        } else {
            Level::Low
        }
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
    #[cfg(not(feature = "riscv-ulp-hal"))]
    fn get_output_level(&self) -> Level
    where
        MODE: OutputMode,
    {
        // TODO: Implement for RTC mode

        let pin = self.pin.pin() as u32;

        #[cfg(esp32c3)]
        let is_set_high = unsafe { (*(GPIO_OUT_REG as *const u32) >> pin) & 0x01 != 0 };
        #[cfg(not(esp32c3))]
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

    /// What level output is set to
    #[inline]
    #[cfg(feature = "riscv-ulp-hal")]
    fn get_output_level(&self) -> Level
    where
        MODE: OutputMode,
    {
        if unsafe { gpio_get_output_level(self.pin.pin()) } != 0 {
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
            #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
            esp!(unsafe { rtc_gpio_set_level(self.pin.pin(), on) })?;

            #[cfg(any(feature = "riscv-ulp-hal", esp32c3))]
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

    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub fn enable_interrupt(&mut self) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        esp_result!(unsafe { gpio_intr_enable(self.pin.pin()) }, ())
    }

    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub fn disable_interrupt(&mut self) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        esp!(unsafe { gpio_intr_disable(self.pin.pin()) })?;
        esp!(unsafe { gpio_set_intr_type(self.pin.pin(), gpio_int_type_t_GPIO_INTR_DISABLE) })?;
        unsafe { unregister_irq_handler(self.pin.pin() as usize) };

        Ok(())
    }

    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub fn set_interrupt_type(&mut self, interrupt_type: InterruptType) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        esp_result!(
            unsafe { gpio_set_intr_type(self.pin.pin(), interrupt_type.into()) },
            ()
        )
    }

    pub fn set_pull(&mut self, pull: Pull) -> Result<(), EspError>
    where
        T: InputPin + OutputPin,
        MODE: InputMode,
    {
        if MODE::RTC {
            #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
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

            #[cfg(any(feature = "riscv-ulp-hal", esp32c3))]
            unreachable!();
        } else {
            esp!(unsafe { gpio_set_pull_mode(self.pin.pin(), pull.into()) })?;
        }

        Ok(())
    }

    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub unsafe fn subscribe(
        &mut self,
        callback: impl FnMut() + Send + 'static,
    ) -> Result<(), EspError>
    where
        MODE: InputMode,
    {
        let callback = PinNotifySubscription::subscribe(self.pin.pin(), callback)?;

        register_irq_handler(self.pin.pin() as usize, callback);

        self.enable_interrupt()?;

        Ok(())
    }

    fn reset(&mut self) -> Result<(), EspError> {
        #[cfg(not(feature = "riscv-ulp-hal"))]
        let res = {
            #[cfg(feature = "alloc")]
            {
                esp!(unsafe { gpio_intr_disable(self.pin.pin()) })?;
                esp!(unsafe {
                    gpio_set_intr_type(self.pin.pin(), gpio_int_type_t_GPIO_INTR_DISABLE)
                })?;
                unsafe { unregister_irq_handler(self.pin.pin() as usize) };
            }

            esp!(unsafe { gpio_reset_pin(self.pin.pin()) })?;
            Ok(())
        };

        #[cfg(feature = "riscv-ulp-hal")]
        let res = Ok(());

        res
    }

    #[cfg(not(feature = "riscv-ulp-hal"))]
    fn rtc_reset(&mut self) -> Result<(), EspError> {
        rtc_reset_pin(self.pin.pin())
    }
}

impl<'d, T: Pin, MODE> Drop for PinDriver<'d, T, MODE> {
    fn drop(&mut self) {
        esp!(unsafe { gpio_set_direction(self.pin.pin(), gpio_mode_t_GPIO_MODE_DISABLE) }).unwrap();
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
pub(crate) fn rtc_reset_pin(pin: i32) -> Result<(), EspError> {
    #[cfg(feature = "alloc")]
    {
        esp!(unsafe { gpio_intr_disable(pin) })?;
        esp!(unsafe { gpio_set_intr_type(pin, gpio_int_type_t_GPIO_INTR_DISABLE) })?;
        unsafe { unregister_irq_handler(pin as usize) };
    }

    esp!(unsafe { gpio_reset_pin(pin) })?;

    #[cfg(all(not(feature = "riscv-ulp-hal"), not(esp32c3)))]
    esp!(unsafe { rtc_gpio_init(pin) })?;

    Ok(())
}

impl<'d, T: Pin, MODE> embedded_hal_0_2::digital::v2::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    type Error = EspError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_level().into())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_level()))
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::ErrorType for PinDriver<'d, T, MODE> {
    type Error = EspError;
}

impl<'d, T: Pin, MODE> embedded_hal::digital::blocking::InputPin for PinDriver<'d, T, MODE>
where
    MODE: InputMode,
{
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_level().into())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!bool::from(self.get_level()))
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

impl<'d, T: Pin, MODE> embedded_hal::digital::blocking::OutputPin for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low)
    }
}

impl<'d, T: Pin, MODE> embedded_hal::digital::blocking::StatefulOutputPin for PinDriver<'d, T, MODE>
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

impl<'d, T: Pin, MODE> embedded_hal::digital::blocking::ToggleableOutputPin
    for PinDriver<'d, T, MODE>
where
    MODE: OutputMode,
{
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::from(!bool::from(self.get_output_level())))
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl UnsafeCallback {
    #[allow(clippy::type_complexity)]
    pub fn from(boxed: &mut Box<Box<dyn FnMut() + 'static>>) -> Self {
        Self(boxed.as_mut())
    }

    pub unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    pub fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    pub unsafe fn call(&mut self) {
        let reference = self.0.as_mut().unwrap();

        (reference)();
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static ISR_SERVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static ISR_SERVICE_ENABLED_CS: crate::cs::CriticalSection = crate::cs::CriticalSection::new();

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
unsafe extern "C" fn irq_handler(unsafe_callback: *mut esp_idf_sys::c_types::c_void) {
    let mut unsafe_callback = UnsafeCallback::from_ptr(unsafe_callback);
    unsafe_callback.call();
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
fn enable_isr_service() -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        let _ = ISR_SERVICE_ENABLED_CS.enter();

        if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { esp_idf_sys::gpio_install_isr_service(0) })?;

            ISR_SERVICE_ENABLED.store(true, Ordering::SeqCst);
        }
    }

    Ok(())
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
type ClosureBox = Box<Box<dyn FnMut()>>;

/// The PinNotifySubscription represents the association between an InputPin and
/// a registered isr handler.
/// When the PinNotifySubscription is dropped, the isr handler is unregistered.
#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
pub(crate) struct PinNotifySubscription(i32, ClosureBox);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl PinNotifySubscription {
    fn subscribe(pin: i32, callback: impl FnMut() + 'static) -> Result<Self, EspError> {
        enable_isr_service()?;

        let callback: Box<dyn FnMut() + 'static> = Box::new(callback);
        let mut callback = Box::new(callback);

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        esp!(unsafe {
            esp_idf_sys::gpio_isr_handler_add(pin, Some(irq_handler), unsafe_callback.as_ptr())
        })?;

        Ok(Self(pin, callback))
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl Drop for PinNotifySubscription {
    fn drop(self: &mut PinNotifySubscription) {
        esp!(unsafe { esp_idf_sys::gpio_isr_handler_remove(self.0) }).expect("Error unsubscribing");
    }
}

/// # Safety
///
/// - Access to `IRQ_HANDLERS` is not guarded because we only call register
///   from the context of Pin which is owned and in the rights state
///
// Clippy in the CI seems to wrongfully catch a only_used_in_recursion
// lint error in this function. We'll ignore it until it's fixed.
#[allow(clippy::only_used_in_recursion)]
#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
unsafe fn register_irq_handler(pin_number: usize, p: PinNotifySubscription) {
    chip::IRQ_HANDLERS[pin_number] = Some(p);
}

/// # Safety
///
/// - Access to `IRQ_HANDLERS` is not guarded because we only call register
///   from the context of Pin which is owned and in the rights state
///
#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
unsafe fn unregister_irq_handler(pin_number: usize) {
    chip::IRQ_HANDLERS[pin_number].take();
}

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

#[allow(unused)]
macro_rules! impl_input_only {
    ($pxi:ident: $pin:expr) => {
        impl_input!($pxi: $pin);

        impl $pxi {
            pub fn downgrade(self) -> AnyInputPin {
                self.into()
            }
        }
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input!($pxi: $pin);

        impl OutputPin for $pxi {}

        impl From<$pxi> for AnyPin {
            fn from(pin: $pxi) -> Self {
                unsafe { Self::new(pin.pin()) }
            }
        }

        impl $pxi {
            pub fn downgrade(self) -> AnyPin {
                self.into()
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
        #[cfg(all(not(esp32c3), not(esp32s3)))]
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
        #[cfg(not(esp32c3))]
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
        impl_input_only!($pxi: $pin);
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
    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

    #[cfg(feature = "riscv-ulp-hal")]
    use crate::riscv_ulp_hal::sys::*;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub(crate) static mut IRQ_HANDLERS: [Option<PinNotifySubscription>; 40] = [
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None, None, None, None, None, None, None,
    ];

    // NOTE: Gpio26 - Gpio32 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0, IO, RTC:11, ADC2:1, NODAC:0, TOUCH:1);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio1:1, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2, IO, RTC:12, ADC2:2, NODAC:0, TOUCH:2);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio3:3, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4, IO, RTC:10, ADC2:0, NODAC:0, TOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio5:5, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio6:6, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio7:7, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio8:8, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio9:9, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, RTC:15, ADC2:5, NODAC:0, TOUCH:5);
    pin!(Gpio13:13, IO, RTC:14, ADC2:4, NODAC:0, TOUCH:4);
    pin!(Gpio14:14, IO, RTC:16, ADC2:6, NODAC:0, TOUCH:6);
    pin!(Gpio15:15, IO, RTC:13, ADC2:3, NODAC:0, TOUCH:3);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio22:22, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
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
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio1: Gpio1,
        pub gpio2: Gpio2,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio3: Gpio3,
        pub gpio4: Gpio4,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio5: Gpio5,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio6: Gpio6,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio7: Gpio7,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio8: Gpio8,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio9: Gpio9,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio10: Gpio10,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio11: Gpio11,
        pub gpio12: Gpio12,
        pub gpio13: Gpio13,
        pub gpio14: Gpio14,
        pub gpio15: Gpio15,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio16: Gpio16,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio17: Gpio17,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio18: Gpio18,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio19: Gpio19,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio21: Gpio21,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio22: Gpio22,
        #[cfg(not(feature = "riscv-ulp-hal"))]
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
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio1: Gpio1::new(),
                gpio2: Gpio2::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio3: Gpio3::new(),
                gpio4: Gpio4::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio5: Gpio5::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio6: Gpio6::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio7: Gpio7::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio8: Gpio8::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio9: Gpio9::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio10: Gpio10::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio11: Gpio11::new(),
                gpio12: Gpio12::new(),
                gpio13: Gpio13::new(),
                gpio14: Gpio14::new(),
                gpio15: Gpio15::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio16: Gpio16::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio17: Gpio17::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio18: Gpio18::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio19: Gpio19::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio21: Gpio21::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio22: Gpio22::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
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
    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
    pub(crate) static mut IRQ_HANDLERS: [Option<PinNotifySubscription>; 49] = [
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None,
    ];

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
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio31:31, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio32:32, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio33:33, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio34:34, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio35:35, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio36:36, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio37:37, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio38:38, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio39:39, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio40:40, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio41:41, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio42:42, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio43:43, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio44:44, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pin!(Gpio45:45, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s2, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio46:46, Input, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio46:46, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
    pin!(Gpio47:47, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
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
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio26: Gpio26,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio27: Gpio27,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio28: Gpio28,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio29: Gpio29,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio30: Gpio30,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio31: Gpio31,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio32: Gpio32,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio33: Gpio33,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio34: Gpio34,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio35: Gpio35,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio36: Gpio36,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio37: Gpio37,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio38: Gpio38,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio39: Gpio39,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio40: Gpio40,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio41: Gpio41,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio42: Gpio42,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio43: Gpio43,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio44: Gpio44,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio45: Gpio45,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio46: Gpio46,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
        pub gpio47: Gpio47,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
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
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio26: Gpio26::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio27: Gpio27::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio28: Gpio28::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio29: Gpio29::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio30: Gpio30::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio31: Gpio31::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio32: Gpio32::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio33: Gpio33::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio34: Gpio34::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio35: Gpio35::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio36: Gpio36::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio37: Gpio37::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio38: Gpio38::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio39: Gpio39::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio40: Gpio40::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio41: Gpio41::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio42: Gpio42::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio43: Gpio43::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio44: Gpio44::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio45: Gpio45::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio46: Gpio46::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio47: Gpio47::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio48: Gpio48::new(),
            }
        }
    }
}

#[cfg(esp32c3)]
#[cfg(not(feature = "riscv-ulp-hal"))]
mod chip {
    use esp_idf_sys::*;

    use crate::adc::{ADC1, ADC2};

    use super::*;

    #[cfg(feature = "alloc")]
    pub(crate) static mut IRQ_HANDLERS: [Option<PinNotifySubscription>; 22] = [
        None, None, None, None, None, None, None, None, None, None, None, None, None, None, None,
        None, None, None, None, None, None, None,
    ];

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
