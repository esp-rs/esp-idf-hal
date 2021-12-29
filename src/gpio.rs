//! GPIO and pin configuration
//!
//! ESP32 has very flexible pin assignment via the GPIO mux. It also has a separate RTC mux for
//! low power and analog functions.
//!
//! To support this flexibility two sets of traits are supported:
//! - The various embedded_hal properties
//! - Dedicated [Pin], [InputPin], [OutputPin], [RTCInputPin] and [RTCOutputPin]
//!
//! The advantage of using the dedicated traits in peripherals is that the configuration of the
//! IO can be done inside the peripheral instead of having to be done upfront.

use core::marker::PhantomData;

#[cfg(not(feature = "ulp"))]
use esp_idf_sys::*;

#[cfg(feature = "ulp")]
use crate::ulp::sys::*;

pub use chip::*;

/// Extension trait to split a GPIO peripheral into independent pins and registers
pub trait GpioExt {
    /// The type to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self) -> Self::Parts;
}

/// Functions available on pins with pull up/down resistors
//
// This is split into a separate trait from OutputPin, because for pins which also connect to
// the RTCIO mux, the pull up/down needs to be set via the RTCIO mux.
pub trait Pull {
    type Error;

    /// Enable internal pull up resistor, disable pull down
    fn set_pull_up(&mut self) -> Result<&mut Self, Self::Error>;

    /// Enable internal pull down resistor, disable pull up
    fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error>;

    /// Enable internal pull up and down resistors
    fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error>;

    /// Disable internal pull up and down resistors
    fn set_floating(&mut self) -> Result<&mut Self, Self::Error>;
}

pub trait Pin: Send {
    type Error;

    fn pin(&self) -> i32;

    fn reset(&mut self) -> Result<(), Self::Error>;
}

pub trait InputPin: Pin {}

pub trait OutputPin: Pin {}

pub trait RTCPin: Pin {
    fn rtc_pin() -> i32;
}

pub trait ADCPin: Pin {
    fn adc_unit() -> adc_unit_t;
    fn adc_channel() -> adc_channel_t;
}

#[cfg(all(not(esp32c3), not(esp32s3)))]
pub trait DACPin: Pin {
    fn dac_channel() -> dac_channel_t;
}

#[cfg(not(esp32c3))]
pub trait TouchPin: Pin {
    fn touch_channel() -> touch_pad_t;
}

pub struct Input;

pub struct Output;

pub struct InputOutput;

pub struct Disabled;

pub struct Unknown;

/// Generic $GpioX pin
pub struct GpioPin<MODE> {
    pin: i32,
    _mode: PhantomData<MODE>,
}

impl<MODE> GpioPin<MODE>
where
    MODE: Send,
{
    fn new(pin: i32) -> GpioPin<MODE> {
        Self {
            pin,
            _mode: PhantomData,
        }
    }

    fn get_input_level(&self) -> bool {
        (unsafe { gpio_get_level(self.pin) } != 0)
    }

    #[cfg(not(feature = "ulp"))]
    fn get_output_level(&self) -> bool {
        let pin = self.pin as u32;

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

        is_set_high
    }

    #[cfg(feature = "ulp")]
    fn get_output_level(&self) -> bool {
        (unsafe { gpio_get_output_level(self.pin) } != 0)
    }

    fn set_output_level(&mut self, on: bool) -> Result<(), EspError> {
        esp_result!(unsafe { gpio_set_level(self.pin(), (on as u8).into()) }, ())
    }

    #[cfg(not(feature = "ulp"))]
    pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError> {
        let mut cap: gpio_drive_cap_t = 0;

        esp!(unsafe { gpio_get_drive_capability(self.pin(), &mut cap as *mut _) })?;

        Ok(cap.into())
    }

    #[cfg(not(feature = "ulp"))]
    pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError> {
        esp!(unsafe { gpio_set_drive_capability(self.pin(), strength.into()) })?;

        Ok(())
    }
}

impl<MODE> Pin for GpioPin<MODE>
where
    MODE: Send,
{
    type Error = EspError;

    fn pin(&self) -> i32
    where
        Self: Sized,
    {
        self.pin
    }

    fn reset(&mut self) -> Result<(), Self::Error> {
        #[cfg(not(feature = "ulp"))]
        let res = esp_result!(unsafe { gpio_reset_pin(self.pin) }, ());
        #[cfg(feature = "ulp")]
        let res = Ok(());

        res
    }
}

impl InputPin for GpioPin<Input> {}

impl embedded_hal::digital::v2::InputPin for GpioPin<Input> {
    type Error = EspError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_input_level())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.get_input_level())
    }
}

impl OutputPin for GpioPin<Output> {}

impl embedded_hal::digital::v2::OutputPin for GpioPin<Output> {
    type Error = EspError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output_level(true)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output_level(false)
    }
}

impl embedded_hal::digital::v2::StatefulOutputPin for GpioPin<Output> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.get_output_level())
    }
}

impl embedded_hal::digital::v2::toggleable::Default for GpioPin<Output> {}

impl InputPin for GpioPin<InputOutput> {}

impl embedded_hal::digital::v2::InputPin for GpioPin<InputOutput> {
    type Error = EspError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_input_level())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.get_input_level())
    }
}

impl OutputPin for GpioPin<InputOutput> {}

impl embedded_hal::digital::v2::OutputPin for GpioPin<InputOutput> {
    type Error = EspError;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_output_level(true)
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_output_level(false)
    }
}

impl embedded_hal::digital::v2::StatefulOutputPin for GpioPin<InputOutput> {
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.get_output_level())
    }
}

impl embedded_hal::digital::v2::toggleable::Default for GpioPin<InputOutput> {}

/// Interrupt events
///
/// *Note: ESP32 has a bug (3.14), which prevents correct triggering of interrupts when
/// multiple GPIO's are configured for edge triggering in a group (GPIO0-31 is one group,
/// GPIO32-39 is the other group). This can be worked around by using level triggering on the
/// GPIO with edge triggering on the CPU.*
//
// Value must correspond to values in the register
#[cfg(not(feature = "ulp"))]
#[derive(Copy, Clone)]
pub enum Event {
    /// Trigger on the rising edge
    RisingEdge = 1,
    /// Trigger on the falling edge
    FallingEdge = 2,
    /// Trigger on any edge
    AnyEdge = 3,
    /// Trigger while low level
    LowLevel = 4,
    /// Trigger while high level
    HighLevel = 5,
}

/// Input mode via RTC (type state)
pub struct RTCInput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Pulled up + pulled down input (type state)
pub struct PullUpDown;

/// Output mode via RTC (type state)
pub struct RTCOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Input-output mode via RTC (type state)
pub struct RTCInputOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Push pull output (type state)
pub struct PushPull;

/// Analog mode (type state)
pub struct Analog;

/// Drive strength (values are approximates)
#[cfg(not(feature = "ulp"))]
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

#[cfg(not(feature = "ulp"))]
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

#[cfg(not(feature = "ulp"))]
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

macro_rules! impl_hal_input_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal::digital::v2::InputPin for $pxi<$mode> {
            type Error = EspError;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { gpio_get_level($pxi::<$mode>::runtime_pin()) } != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }
    };
}

macro_rules! impl_hal_output_pin {
    ($pxi:ident: $mode:ident) => {
        impl $pxi<$mode> {
            #[cfg(not(feature = "ulp"))]
            pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError> {
                let mut cap: gpio_drive_cap_t = 0;

                esp!(unsafe { gpio_get_drive_capability(self.pin(), &mut cap as *mut _) })?;

                Ok(cap.into())
            }

            #[cfg(not(feature = "ulp"))]
            pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_drive_capability(self.pin(), strength.into()) })?;

                Ok(())
            }
        }

        impl embedded_hal::digital::v2::OutputPin for $pxi<$mode> {
            type Error = EspError;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                esp_result!(
                    unsafe { gpio_set_level($pxi::<$mode>::runtime_pin(), 1) },
                    ()
                )
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                esp_result!(
                    unsafe { gpio_set_level($pxi::<$mode>::runtime_pin(), 0) },
                    ()
                )
            }
        }

        impl embedded_hal::digital::v2::StatefulOutputPin for $pxi<$mode> {
            #[cfg(not(feature = "ulp"))]
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                let pin = $pxi::<$mode>::runtime_pin() as u32;

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

                Ok(is_set_high)
            }

            #[cfg(feature = "ulp")]
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { gpio_get_output_level($pxi::<$mode>::runtime_pin()) } != 0)
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_set_high()?)
            }
        }

        impl embedded_hal::digital::v2::ToggleableOutputPin for $pxi<$mode> {
            type Error = EspError;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_set_high()? {
                    Ok(self.set_low()?)
                } else {
                    Ok(self.set_high()?)
                }
            }
        }
    };
}

macro_rules! impl_pull {
    ($pxi:ident: $mode:ident) => {
        impl Pull for $pxi<$mode> {
            type Error = EspError;

            fn set_pull_up(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(
                            $pxi::<$mode>::runtime_pin(),
                            gpio_pull_mode_t_GPIO_PULLUP_ONLY,
                        )
                    },
                    self
                )
            }

            fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(
                            $pxi::<$mode>::runtime_pin(),
                            gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,
                        )
                    },
                    self
                )
            }

            fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(
                            $pxi::<$mode>::runtime_pin(),
                            gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN,
                        )
                    },
                    self
                )
            }

            fn set_floating(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(
                            $pxi::<$mode>::runtime_pin(),
                            gpio_pull_mode_t_GPIO_FLOATING,
                        )
                    },
                    self
                )
            }
        }
    };
}

macro_rules! impl_input_base {
    ($pxi:ident: $pin:expr) => {
        pub struct $pxi<MODE> {
            _mode: PhantomData<MODE>,
        }

        #[cfg(not(feature = "ulp"))]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            #[inline(always)]
            fn runtime_pin() -> i32 {
                $pin
            }

            /// Degrades a concrete pin (e.g. [`Gpio1`]) to a generic pin
            /// struct that can also be used with periphals.
            pub fn degrade(self) -> GpioPin<MODE> {
                GpioPin::new($pin)
            }

            pub fn into_unknown(self) -> $pxi<Unknown> {
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> Pin for $pxi<MODE>
        where
            MODE: Send,
        {
            type Error = EspError;

            #[inline(always)]
            fn pin(&self) -> i32 {
                $pin
            }

            fn reset(&mut self) -> Result<(), Self::Error> {
                #[cfg(not(feature = "ulp"))]
                let res = esp_result!(unsafe { gpio_reset_pin(self.pin()) }, ());
                #[cfg(feature = "ulp")]
                let res = Ok(());

                res
            }
        }

        impl<MODE> InputPin for $pxi<MODE> where MODE: Send {}

        impl_hal_input_pin!($pxi: Input);
    };
}

#[allow(unused)]
macro_rules! impl_input_only {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);

        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            /// # Safety
            ///
            /// Care should be taken not to instantiate a pin which is already used elsewhere
            pub unsafe fn new() -> $pxi<Unknown> {
                $pxi { _mode: PhantomData }
            }

            pub fn into_disabled(self) -> Result<$pxi<Disabled>, EspError> {
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_DISABLE,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_input(mut self) -> Result<$pxi<Input>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction($pxi::<MODE>::runtime_pin(), gpio_mode_t_GPIO_MODE_INPUT)
                    },
                    $pxi { _mode: PhantomData }
                )
            }
        }
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);

        impl<MODE> OutputPin for $pxi<MODE> where MODE: Send {}

        impl_hal_input_pin!($pxi: InputOutput);
        impl_hal_output_pin!($pxi: InputOutput);
        impl_hal_output_pin!($pxi: Output);
        impl_pull!($pxi: Input);
        impl_pull!($pxi: InputOutput);

        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            /// # Safety
            ///
            /// Care should be taken not to instantiate a pin which is already used elsewhere
            pub unsafe fn new() -> $pxi<Unknown> {
                $pxi { _mode: PhantomData }
            }

            pub fn into_disabled(self) -> Result<$pxi<Disabled>, EspError> {
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_DISABLE,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_input(mut self) -> Result<$pxi<Input>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction($pxi::<MODE>::runtime_pin(), gpio_mode_t_GPIO_MODE_INPUT)
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_input_output(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_INPUT_OUTPUT,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_input_output_od(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_output(mut self) -> Result<$pxi<Output>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_OUTPUT,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }

            pub fn into_output_od(mut self) -> Result<$pxi<Output>, EspError> {
                self.reset()?;
                esp_result!(
                    unsafe {
                        gpio_set_direction(
                            $pxi::<MODE>::runtime_pin(),
                            gpio_mode_t_GPIO_MODE_OUTPUT_OD,
                        )
                    },
                    $pxi { _mode: PhantomData }
                )
            }
        }
    };
}

macro_rules! impl_rtc {
    ($pxi:ident: $pin:expr, RTC: $rtc:expr) => {
        #[cfg(feature = "ulp")]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            #[inline(always)]
            fn runtime_pin() -> i32 {
                $rtc
            }

            /// Degrades a concrete pin (e.g. [`Gpio1`]) to a generic pin
            /// struct that can also be used with periphals.
            pub fn degrade(self) -> GpioPin<MODE> {
                GpioPin::new($pin)
            }

            pub fn into_unknown(self) -> $pxi<Unknown> {
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> RTCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn rtc_pin() -> i32 {
                $rtc
            }
        }
    };

    ($pxi:ident: $pin:expr, NORTC: $rtc:expr) => {};
}

macro_rules! impl_adc {
    ($pxi:ident: $pin:expr, ADC1: $adc:expr) => {
        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit() -> adc_unit_t {
                adc_unit_t_ADC_UNIT_1
            }

            fn adc_channel() -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, ADC2: $adc:expr) => {
        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit() -> adc_unit_t {
                adc_unit_t_ADC_UNIT_2
            }

            fn adc_channel() -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, NOADC: $adc:expr) => {};
}

macro_rules! impl_dac {
    ($pxi:ident: $pin:expr, DAC: $dac:expr) => {
        #[cfg(all(not(esp32c3), not(esp32s3)))]
        impl<MODE> DACPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn dac_channel() -> dac_channel_t {
                $dac
            }
        }
    };

    ($pxi:ident: $pin:expr, NODAC: $dac:expr) => {};
}

macro_rules! impl_touch {
    ($pxi:ident: $pin:expr, TOUCH: $touch:expr) => {
        #[cfg(not(esp32c3))]
        impl<MODE> TouchPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn touch_channel() -> touch_pad_t {
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
    use core::marker::PhantomData;

    use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};
    #[cfg(not(feature = "ulp"))]
    use esp_idf_sys::*;

    use super::*;
    #[cfg(feature = "ulp")]
    use crate::ulp::sys::*;

    // NOTE: Gpio26 - Gpio32 are used by SPI0/SPI1 for external PSRAM/SPI Flash and
    //       are not recommended for other uses
    pin!(Gpio0:0, IO, RTC:11, ADC2:1, NODAC:0, TOUCH:1);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio1:1, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio2:2, IO, RTC:12, ADC2:2, NODAC:0, TOUCH:2);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio3:3, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio4:4, IO, RTC:10, ADC2:0, NODAC:0, TOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio5:5, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio6:6, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio7:7, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio8:8, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio9:9, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio10:10, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio11:11, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    pin!(Gpio12:12, IO, RTC:15, ADC2:5, NODAC:0, TOUCH:5);
    pin!(Gpio13:13, IO, RTC:14, ADC2:4, NODAC:0, TOUCH:4);
    pin!(Gpio14:14, IO, RTC:16, ADC2:6, NODAC:0, TOUCH:6);
    pin!(Gpio15:15, IO, RTC:13, ADC2:3, NODAC:0, TOUCH:3);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio16:16, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio17:17, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio18:18, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio19:19, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio21:21, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio22:22, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
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
        pub gpio0: Gpio0<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio5: Gpio5<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio6: Gpio6<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio7: Gpio7<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio8: Gpio8<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio9: Gpio9<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio10: Gpio10<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio16: Gpio16<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio17: Gpio17<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio18: Gpio18<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio19: Gpio19<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio21: Gpio21<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio22: Gpio22<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio23: Gpio23<Unknown>,
        pub gpio25: Gpio25<Unknown>,
        pub gpio26: Gpio26<Unknown>,
        pub gpio27: Gpio27<Unknown>,
        pub gpio32: Gpio32<Unknown>,
        pub gpio33: Gpio33<Unknown>,
        pub gpio34: Gpio34<Unknown>,
        pub gpio35: Gpio35<Unknown>,
        pub gpio36: Gpio36<Unknown>,
        pub gpio37: Gpio37<Unknown>,
        pub gpio38: Gpio38<Unknown>,
        pub gpio39: Gpio39<Unknown>,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instnatiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio5: Gpio5::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio6: Gpio6::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio7: Gpio7::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio8: Gpio8::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio9: Gpio9::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio10: Gpio10::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio16: Gpio16::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio17: Gpio17::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio18: Gpio18::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio19: Gpio19::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio21: Gpio21::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio22: Gpio22::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio23: Gpio23::<Unknown>::new(),
                gpio25: Gpio25::<Unknown>::new(),
                gpio26: Gpio26::<Unknown>::new(),
                gpio27: Gpio27::<Unknown>::new(),
                gpio32: Gpio32::<Unknown>::new(),
                gpio33: Gpio33::<Unknown>::new(),
                gpio34: Gpio34::<Unknown>::new(),
                gpio35: Gpio35::<Unknown>::new(),
                gpio36: Gpio36::<Unknown>::new(),
                gpio37: Gpio37::<Unknown>::new(),
                gpio38: Gpio38::<Unknown>::new(),
                gpio39: Gpio39::<Unknown>::new(),
            }
        }
    }
}

#[cfg(any(esp32s2, esp32s3))]
mod chip {
    use core::marker::PhantomData;

    use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};
    #[cfg(not(feature = "ulp"))]
    use esp_idf_sys::*;

    use super::*;
    #[cfg(feature = "ulp")]
    use crate::ulp::sys::*;

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
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio26:26, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio27:27, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio28:28, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio29:29, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio30:30, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio31:31, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio32:32, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio33:33, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio34:34, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio35:35, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio36:36, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio37:37, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio38:38, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio39:39, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio40:40, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio41:41, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio42:42, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio43:43, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio44:44, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(not(feature = "ulp"))]
    pin!(Gpio45:45, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s2, not(feature = "ulp")))]
    pin!(Gpio46:46, Input, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "ulp")))]
    pin!(Gpio46:46, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "ulp")))]
    pin!(Gpio47:47, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);
    #[cfg(all(esp32s3, not(feature = "ulp")))]
    pin!(Gpio48:48, IO, NORTC:0, NOADC:0, NODAC:0, NOTOUCH:0);

    pub struct Pins {
        pub gpio0: Gpio0<Unknown>,
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        pub gpio5: Gpio5<Unknown>,
        pub gpio6: Gpio6<Unknown>,
        pub gpio7: Gpio7<Unknown>,
        pub gpio8: Gpio8<Unknown>,
        pub gpio9: Gpio9<Unknown>,
        pub gpio10: Gpio10<Unknown>,
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        pub gpio16: Gpio16<Unknown>,
        pub gpio17: Gpio17<Unknown>,
        pub gpio18: Gpio18<Unknown>,
        pub gpio19: Gpio19<Unknown>,
        pub gpio20: Gpio20<Unknown>,
        pub gpio21: Gpio21<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio26: Gpio26<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio27: Gpio27<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio28: Gpio28<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio29: Gpio29<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio30: Gpio30<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio31: Gpio31<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio32: Gpio32<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio33: Gpio33<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio34: Gpio34<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio35: Gpio35<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio36: Gpio36<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio37: Gpio37<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio38: Gpio38<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio39: Gpio39<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio40: Gpio40<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio41: Gpio41<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio42: Gpio42<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio43: Gpio43<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio44: Gpio44<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio45: Gpio45<Unknown>,
        #[cfg(not(feature = "ulp"))]
        pub gpio46: Gpio46<Unknown>,
        #[cfg(all(esp32s3, not(feature = "ulp")))]
        pub gpio47: Gpio47<Unknown>,
        #[cfg(all(esp32s3, not(feature = "ulp")))]
        pub gpio48: Gpio48<Unknown>,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instnatiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                gpio5: Gpio5::<Unknown>::new(),
                gpio6: Gpio6::<Unknown>::new(),
                gpio7: Gpio7::<Unknown>::new(),
                gpio8: Gpio8::<Unknown>::new(),
                gpio9: Gpio9::<Unknown>::new(),
                gpio10: Gpio10::<Unknown>::new(),
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                gpio16: Gpio16::<Unknown>::new(),
                gpio17: Gpio17::<Unknown>::new(),
                gpio18: Gpio18::<Unknown>::new(),
                gpio19: Gpio19::<Unknown>::new(),
                gpio20: Gpio20::<Unknown>::new(),
                gpio21: Gpio21::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio26: Gpio26::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio27: Gpio27::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio28: Gpio28::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio29: Gpio29::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio30: Gpio30::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio31: Gpio31::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio32: Gpio32::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio33: Gpio33::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio34: Gpio34::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio35: Gpio35::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio36: Gpio36::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio37: Gpio37::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio38: Gpio38::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio39: Gpio39::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio40: Gpio40::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio41: Gpio41::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio42: Gpio42::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio43: Gpio43::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio44: Gpio44::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio45: Gpio45::<Unknown>::new(),
                #[cfg(not(feature = "ulp"))]
                gpio46: Gpio46::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "ulp")))]
                gpio47: Gpio47::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "ulp")))]
                gpio48: Gpio48::<Unknown>::new(),
            }
        }
    }
}

#[cfg(esp32c3)]
#[cfg(not(feature = "ulp"))]
mod chip {
    use core::marker::PhantomData;

    use embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _};
    use esp_idf_sys::*;

    use super::*;

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
        pub gpio0: Gpio0<Unknown>,
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        pub gpio5: Gpio5<Unknown>,
        pub gpio6: Gpio6<Unknown>,
        pub gpio7: Gpio7<Unknown>,
        pub gpio8: Gpio8<Unknown>,
        pub gpio9: Gpio9<Unknown>,
        pub gpio10: Gpio10<Unknown>,
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        pub gpio16: Gpio16<Unknown>,
        pub gpio17: Gpio17<Unknown>,
        pub gpio18: Gpio18<Unknown>,
        pub gpio19: Gpio19<Unknown>,
        pub gpio20: Gpio20<Unknown>,
        pub gpio21: Gpio21<Unknown>,
    }

    impl Pins {
        /// # Safety
        ///
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                gpio5: Gpio5::<Unknown>::new(),
                gpio6: Gpio6::<Unknown>::new(),
                gpio7: Gpio7::<Unknown>::new(),
                gpio8: Gpio8::<Unknown>::new(),
                gpio9: Gpio9::<Unknown>::new(),
                gpio10: Gpio10::<Unknown>::new(),
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                gpio16: Gpio16::<Unknown>::new(),
                gpio17: Gpio17::<Unknown>::new(),
                gpio18: Gpio18::<Unknown>::new(),
                gpio19: Gpio19::<Unknown>::new(),
                gpio20: Gpio20::<Unknown>::new(),
                gpio21: Gpio21::<Unknown>::new(),
            }
        }
    }
}
