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

use {
    core::{convert::Infallible, marker::PhantomData},
    embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _},
};

use esp_idf_sys::*;

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

pub trait Pin {
    type Error;

    fn pin() -> i32;

    fn reset(&mut self) -> Result<(), Self::Error>;
}

pub trait InputPin: Pin {
}

pub trait OutputPin: Pin {
}

pub struct Input;

pub struct Output;

pub struct InputOutput;

pub struct Disabled;

pub struct Unknown;

/// Interrupt events
///
/// *Note: ESP32 has a bug (3.14), which prevents correct triggering of interrupts when
/// multiple GPIO's are configured for edge triggering in a group (GPIO0-31 is one group,
/// GPIO32-39 is the other group). This can be worked around by using level triggering on the
/// GPIO with edge triggering on the CPU.*
//
// Value must correspond to values in the register
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
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

macro_rules! impl_hal_input_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal::digital::v2::InputPin for $pxi<$mode> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe {gpio_get_level($pxi::<$mode>::pin())} != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }
    };
}

macro_rules! impl_hal_output_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal::digital::v2::OutputPin for $pxi<$mode> {
            type Error = EspError;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                esp_result!(unsafe {gpio_set_level($pxi::<$mode>::pin(), 1)}, ())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                esp_result!(unsafe {gpio_set_level($pxi::<$mode>::pin(), 0)}, ())
            }
        }

        impl embedded_hal::digital::v2::StatefulOutputPin for $pxi<$mode> {
            //type Error = Infallible;

            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe {gpio_get_level($pxi::<$mode>::pin())} != 0)
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
                esp_result!(unsafe {gpio_set_pull_mode($pxi::<$mode>::pin(), gpio_pull_mode_t_GPIO_PULLUP_ONLY)}, self)
            }

            fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(unsafe {gpio_set_pull_mode($pxi::<$mode>::pin(), gpio_pull_mode_t_GPIO_PULLDOWN_ONLY)}, self)
            }

            fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(unsafe {gpio_set_pull_mode($pxi::<$mode>::pin(), gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN)}, self)
            }

            fn set_floating(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(unsafe {gpio_set_pull_mode($pxi::<$mode>::pin(), gpio_pull_mode_t_GPIO_FLOATING)}, self)
            }
        }
    };
}

macro_rules! impl_input_base {
    ($pxi:ident: $pin:expr) => {
        pub struct $pxi<MODE> {
            _mode: PhantomData<MODE>,
        }

        impl<MODE> Pin for $pxi<MODE> {
            type Error = EspError;

            fn pin() -> i32 {$pin}

            fn reset(&mut self) -> Result<(), Self::Error> {Ok(())}
        }

        impl<MODE> InputPin for $pxi<MODE> {
        }

        impl_hal_input_pin!($pxi: Input);
    };
}

macro_rules! impl_input_only {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);

        impl<MODE> $pxi<MODE> {
            pub unsafe fn new() -> $pxi<Unknown> {
                $pxi {_mode: PhantomData}
            }

            pub fn into_disabled(self) -> Result<$pxi<Disabled>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_DISABLE)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_input(self) -> Result<$pxi<Input>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_INPUT)},
                    $pxi {_mode: PhantomData})
            }
        }
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);

        impl<MODE> OutputPin for $pxi<MODE> {
        }

        impl_hal_input_pin!($pxi: InputOutput);
        impl_hal_output_pin!($pxi: Output);
        impl_pull!($pxi: Input);
        impl_pull!($pxi: InputOutput);

        impl<MODE> $pxi<MODE> {
            pub unsafe fn new() -> $pxi<Unknown> {
                $pxi {_mode: PhantomData}
            }

            pub fn into_disabled(self) -> Result<$pxi<Disabled>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_DISABLE)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_input(self) -> Result<$pxi<Input>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_INPUT)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_input_output(self) -> Result<$pxi<InputOutput>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_input_output_od(self) -> Result<$pxi<InputOutput>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_output(self) -> Result<$pxi<Output>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_OUTPUT)},
                    $pxi {_mode: PhantomData})
            }

            pub fn into_output_od(self) -> Result<$pxi<Output>, EspError> {
                esp_result!(
                    unsafe {gpio_set_direction($pxi::<MODE>::pin(), gpio_mode_t_GPIO_MODE_OUTPUT_OD)},
                    $pxi {_mode: PhantomData})
            }
        }
    };
}

impl_input_output!(Gpio0: 0);
impl_input_output!(Gpio1: 1);
impl_input_output!(Gpio2: 2);
impl_input_output!(Gpio6: 6);
impl_input_output!(Gpio7: 7);
impl_input_output!(Gpio8: 8);
impl_input_output!(Gpio19: 19);
impl_input_output!(Gpio22: 22);
impl_input_only!(Gpio29: 22);