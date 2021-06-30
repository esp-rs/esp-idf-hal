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

pub trait RTCPin: Pin {
    fn rtc_pin() -> i32;
}

pub trait ADCPin: Pin {
    fn adc_unit() -> adc_unit_t;
    fn adc_channel() -> adc_channel_t;
}

pub trait DACPin: Pin {
    fn dac_channel() -> dac_channel_t;
}

pub trait TouchPin: Pin {
    fn touch_channel() -> touch_pad_t;
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
            type Error = EspError;

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

            #[inline(always)]
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
        impl_hal_output_pin!($pxi: InputOutput);
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

macro_rules! impl_rtc {
    ($pxi:ident: $pin:expr, RTC:$rtc:expr) => {
        impl<MODE> RTCPin for $pxi<MODE> {
            fn rtc_pin() -> i32 {
                $rtc
            }
        }
    };

    ($pxi:ident: $pin:expr, NORTC:$rtc:expr) => {
    };
}

macro_rules! impl_adc {
    ($pxi:ident: $pin:expr, ADC1:$adc:expr) => {
        impl<MODE> ADCPin for $pxi<MODE> {
            fn adc_unit() -> adc_unit_t {
                adc_unit_t_ADC_UNIT_1
            }

            fn adc_channel() -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, ADC2:$adc:expr) => {
        impl<MODE> ADCPin for $pxi<MODE> {
            fn adc_unit() -> adc_unit_t {
                adc_unit_t_ADC_UNIT_2
            }

            fn adc_channel() -> adc_channel_t {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, NOADC:$adc:expr) => {
    };
}

macro_rules! impl_dac {
    ($pxi:ident: $pin:expr, DAC:$dac:expr) => {
        impl<MODE> DACPin for $pxi<MODE> {
            fn dac_channel() -> dac_channel_t {
                $dac
            }
        }
    };

    ($pxi:ident: $pin:expr, NODAC:$dac:expr) => {
    };
}

macro_rules! impl_touch {
    ($pxi:ident: $pin:expr, TOUCH:$touch:expr) => {
        impl<MODE> TouchPin for $pxi<MODE> {
            fn touch_channel() -> touch_pad_t {
                $touch
            }
        }
    };

    ($pxi:ident: $pin:expr, NOTOUCH:$touch:expr) => {
    };
}

macro_rules! pin {
    ($pxi:ident: $pin:expr, Input, $rtc:ident:$rtcno:expr, $adc:ident:$adcno:expr, $dac:ident:$dacno:expr, $touch:ident:$touchno:expr) => {
        impl_input_only!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc:$rtcno);
        impl_adc!($pxi: $pin, $adc:$adcno);
        impl_dac!($pxi: $pin, $dac:$dacno);
        impl_touch!($pxi: $pin, $touch:$touchno);
    };

    ($pxi:ident: $pin:expr, IO, $rtc:ident:$rtcno:expr, $adc:ident:$adcno:expr, $dac:ident:$dacno:expr, $touch:ident:$touchno:expr) => {
        impl_input_output!($pxi: $pin);
        impl_rtc!($pxi: $pin, $rtc:$rtcno);
        impl_adc!($pxi: $pin, $adc:$adcno);
        impl_dac!($pxi: $pin, $dac:$dacno);
        impl_touch!($pxi: $pin, $touch:$touchno);
    };
}

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
pin!(Gpio39:39, Input, RTC:3, ADC1:3, NODAC:0, NOTOUCH:0);

// Not mapped: 20, 24, 28, 29, 30, 31, 37, 38

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
    pub gpio21: Gpio21<Unknown>,
    pub gpio22: Gpio22<Unknown>,
    pub gpio23: Gpio23<Unknown>,
    pub gpio25: Gpio25<Unknown>,
    pub gpio26: Gpio26<Unknown>,
    pub gpio27: Gpio27<Unknown>,
    pub gpio32: Gpio32<Unknown>,
    pub gpio33: Gpio33<Unknown>,
    pub gpio34: Gpio34<Unknown>,
    pub gpio35: Gpio35<Unknown>,
    pub gpio36: Gpio36<Unknown>,
    pub gpio39: Gpio39<Unknown>,
}

impl Pins {
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
            gpio21: Gpio21::<Unknown>::new(),
            gpio22: Gpio22::<Unknown>::new(),
            gpio23: Gpio23::<Unknown>::new(),
            gpio25: Gpio25::<Unknown>::new(),
            gpio26: Gpio26::<Unknown>::new(),
            gpio27: Gpio27::<Unknown>::new(),
            gpio32: Gpio32::<Unknown>::new(),
            gpio33: Gpio33::<Unknown>::new(),
            gpio34: Gpio34::<Unknown>::new(),
            gpio35: Gpio35::<Unknown>::new(),
            gpio36: Gpio36::<Unknown>::new(),
            gpio39: Gpio39::<Unknown>::new(),
        }
    }
}
