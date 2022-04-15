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

use crate::adc;

pub use chip::*;

/// A trait implemented by every pin insance
pub trait Pin: Send {
    type Error;

    fn pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an input pin, even if its current mode
/// might be a different one
pub trait InputPin: Pin {}

/// A marker trait designating a pin which is capable of
/// operating as an output pin, even if its current mode
/// might be a different one
pub trait OutputPin: Pin {}

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

    /// Enable internal pull up resistor, disable pull down
    fn into_pull_up(mut self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.set_pull_up()?;

        Ok(self)
    }

    /// Enable internal pull down resistor, disable pull up
    fn into_pull_down(mut self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.set_pull_down()?;

        Ok(self)
    }

    /// Enable internal pull up and down resistors
    fn into_pull_up_down(mut self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.set_pull_up_down()?;

        Ok(self)
    }

    /// Disable internal pull up and down resistors
    fn into_floating(mut self) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        self.set_floating()?;

        Ok(self)
    }
}

pub trait RTCPin: Pin {
    fn rtc_pin(&self) -> i32;
}

/// A marker trait designating a pin which is capable of
/// operating as an ADC pin, even if its current mode
/// might be a different one
pub trait ADCPin: Pin {
    fn adc_unit(&self) -> adc_unit_t;
    fn adc_channel(&self) -> adc_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a DAC pin, even if its current mode
/// might be a different one
#[cfg(all(not(esp32c3), not(esp32s3)))]
pub trait DACPin: Pin {
    fn dac_channel(&self) -> dac_channel_t;
}

/// A marker trait designating a pin which is capable of
/// operating as a touch pin, even if its current mode
/// might be a different one
#[cfg(not(esp32c3))]
pub trait TouchPin: Pin {
    fn touch_channel(&self) -> touch_pad_t;
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
pub trait SubscribedPin: Pin {}

pub struct Input;

pub struct Output;

pub struct InputOutput;

pub struct Disabled;

pub struct Unknown;

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
pub struct SubscribedInput;

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
static ISR_SERVICE_ENABLED: crate::mutex::Mutex<bool> = crate::mutex::Mutex::new(false);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
unsafe extern "C" fn irq_handler(unsafe_callback: *mut esp_idf_sys::c_types::c_void) {
    let mut unsafe_callback = UnsafeCallback::from_ptr(unsafe_callback);
    unsafe_callback.call();
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
fn enable_isr_service() -> Result<(), EspError> {
    let mut service_enabled = ISR_SERVICE_ENABLED.lock();
    if !*service_enabled {
        if let Err(e) = esp!(unsafe { esp_idf_sys::gpio_install_isr_service(0) }) {
            return Err(e);
        } else {
            *service_enabled = true;
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
    fn subscribe<P>(pin: &mut P, callback: impl FnMut() + 'static) -> Result<Self, EspError>
    where
        P: InputPin + Pin,
    {
        enable_isr_service()?;

        let pin_number: i32 = pin.pin();

        let callback: Box<dyn FnMut() + 'static> = Box::new(callback);
        let mut callback = Box::new(callback);

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        esp!(unsafe {
            esp_idf_sys::gpio_isr_handler_add(
                pin_number,
                Some(irq_handler),
                unsafe_callback.as_ptr(),
            )
        })?;

        Ok(Self(pin_number, callback))
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl Drop for PinNotifySubscription {
    fn drop(self: &mut PinNotifySubscription) {
        esp!(unsafe { esp_idf_sys::gpio_isr_handler_remove(self.0) }).expect("Error unsubscribing");
    }
}

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

macro_rules! impl_base {
    ($pxi:ident) => {
        #[allow(dead_code)]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            fn reset(&mut self) -> Result<(), EspError> {
                #[cfg(not(feature = "riscv-ulp-hal"))]
                let res = {
                    #[cfg(feature = "alloc")]
                    self.disable_interrupt()?;

                    esp!(unsafe { gpio_reset_pin(self.pin()) })?;
                    Ok(())
                };

                #[cfg(feature = "riscv-ulp-hal")]
                let res = Ok(());

                res
            }

            fn get_input_level(&self) -> bool {
                (unsafe { gpio_get_level(self.pin()) } != 0)
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            fn get_output_level(&self) -> bool {
                let pin = self.pin() as u32;

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

            #[cfg(feature = "riscv-ulp-hal")]
            fn get_output_level(&self) -> bool {
                (unsafe { gpio_get_output_level(self.pin()) } != 0)
            }

            fn set_output_level(&mut self, on: bool) -> Result<(), EspError> {
                esp_result!(unsafe { gpio_set_level(self.pin(), (on as u8).into()) }, ())
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            pub fn get_drive_strength(&self) -> Result<DriveStrength, EspError> {
                let mut cap: gpio_drive_cap_t = 0;

                esp!(unsafe { gpio_get_drive_capability(self.pin(), &mut cap as *mut _) })?;

                Ok(cap.into())
            }

            #[cfg(not(feature = "riscv-ulp-hal"))]
            pub fn set_drive_strength(&mut self, strength: DriveStrength) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_drive_capability(self.pin(), strength.into()) })?;

                Ok(())
            }

            fn set_disabled(&mut self) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_DISABLE,) })?;

                Ok(())
            }

            fn set_input(&mut self) -> Result<(), EspError> {
                self.reset()?;
                esp!(unsafe { gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT) })?;

                Ok(())
            }

            fn set_input_output(&mut self) -> Result<(), EspError> {
                self.reset()?;
                esp!(unsafe {
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT)
                })?;

                Ok(())
            }

            fn set_input_output_od(&mut self) -> Result<(), EspError> {
                self.reset()?;
                esp!(unsafe {
                    gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD)
                })?;

                Ok(())
            }

            fn set_output(&mut self) -> Result<(), EspError> {
                self.reset()?;
                esp!(unsafe { gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_OUTPUT,) })?;

                Ok(())
            }

            fn set_output_od(&mut self) -> Result<(), EspError> {
                self.reset()?;
                esp!(unsafe { gpio_set_direction(self.pin(), gpio_mode_t_GPIO_MODE_OUTPUT_OD,) })?;

                Ok(())
            }

            #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
            fn enable_interrupt(&mut self) -> Result<(), EspError> {
                esp!(unsafe { gpio_intr_enable(self.pin()) })?;

                Ok(())
            }

            #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
            fn disable_interrupt(&mut self) -> Result<(), EspError> {
                esp!(unsafe { gpio_intr_disable(self.pin()) })?;
                esp!(unsafe { gpio_set_intr_type(self.pin(), gpio_int_type_t_GPIO_INTR_DISABLE) })?;
                unsafe { unregister_irq_handler(self.pin() as usize) };

                Ok(())
            }

            #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
            fn set_interrupt_type(
                &mut self,
                interrupt_type: InterruptType,
            ) -> Result<(), EspError> {
                esp!(unsafe { gpio_set_intr_type(self.pin(), interrupt_type.into()) })?;

                Ok(())
            }
        }

        impl<MODE> embedded_hal::digital::ErrorType for $pxi<MODE> {
            type Error = EspError;
        }
    };
}

macro_rules! impl_pull {
    ($pxi:ident: $mode:ty) => {
        impl Pull for $pxi<$mode> {
            type Error = EspError;

            fn set_pull_up(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLUP_ONLY,) },
                    self
                )
            }

            fn set_pull_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLDOWN_ONLY,) },
                    self
                )
            }

            fn set_pull_up_down(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe {
                        gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN)
                    },
                    self
                )
            }

            fn set_floating(&mut self) -> Result<&mut Self, Self::Error> {
                esp_result!(
                    unsafe { gpio_set_pull_mode(self.pin(), gpio_pull_mode_t_GPIO_FLOATING,) },
                    self
                )
            }
        }
    };
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

macro_rules! impl_input_base {
    ($pxi:ident: $pin:expr) => {
        pub struct $pxi<MODE> {
            _mode: PhantomData<MODE>,
        }

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

            pub fn into_unknown(self) -> Result<$pxi<Unknown>, EspError> {
                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_disabled(mut self) -> Result<$pxi<Disabled>, EspError> {
                self.set_disabled()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_input(mut self) -> Result<$pxi<Input>, EspError> {
                self.set_input()?;

                Ok($pxi { _mode: PhantomData })
            }

            /// # Safety
            ///
            /// The callback passed to this method is executed in the context of an
            /// interrupt handler. So you should take care of what is done in it.
            #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
            pub unsafe fn into_subscribed(
                mut self,
                callback: impl FnMut() + Send + 'static,
                interrupt_type: InterruptType,
            ) -> Result<$pxi<SubscribedInput>, EspError> {
                self.set_input()?;

                self.set_interrupt_type(interrupt_type)?;

                let callback = PinNotifySubscription::subscribe(&mut self, callback)?;

                register_irq_handler(self.pin() as usize, callback);

                self.enable_interrupt()?;

                Ok($pxi { _mode: PhantomData })
            }

            /// Degrades a concrete pin (e.g. [`Gpio1`]) to a generic pin
            /// struct that can also be used with periphals.
            pub fn degrade(self) -> GpioPin<MODE> {
                unsafe { GpioPin::new($pin) }
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
        }

        impl<MODE> InputPin for $pxi<MODE> where MODE: Send {}

        impl_base!($pxi);
        impl_hal_input_pin!($pxi: Input);

        #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
        impl_hal_input_pin!($pxi: SubscribedInput);
    };
}

#[allow(unused)]
macro_rules! impl_input_only {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);
    };
}

macro_rules! impl_input_output {
    ($pxi:ident: $pin:expr) => {
        impl_input_base!($pxi: $pin);
        impl_pull!($pxi: Input);
        impl_pull!($pxi: InputOutput);

        #[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
        impl_pull!($pxi: SubscribedInput);

        impl_hal_input_pin!($pxi: InputOutput);

        impl<MODE> OutputPin for $pxi<MODE> where MODE: Send {}

        impl_hal_output_pin!($pxi: InputOutput);
        impl_hal_output_pin!($pxi: Output);

        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_input_output(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.set_input_output()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_input_output_od(mut self) -> Result<$pxi<InputOutput>, EspError> {
                self.set_input_output_od()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_output(mut self) -> Result<$pxi<Output>, EspError> {
                self.set_output()?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_output_od(mut self) -> Result<$pxi<Output>, EspError> {
                self.set_output_od()?;

                Ok($pxi { _mode: PhantomData })
            }
        }
    };
}

macro_rules! impl_rtc {
    ($pxi:ident: $pin:expr, RTC: $rtc:expr) => {
        impl<MODE> RTCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn rtc_pin(&self) -> i32 {
                $rtc
            }
        }
    };

    ($pxi:ident: $pin:expr, NORTC: $rtc:expr) => {};
}

macro_rules! impl_adc_pull {
    ($pxi:ident, ADC1) => {
        impl_adc_pull!($pxi, adc::ADC1);
    };

    ($pxi:ident, ADC2) => {
        impl_adc_pull!($pxi, adc::ADC2);
    };

    ($pxi:ident, NOADC) => {};

    ($pxi:ident, $adc:ty) => {
        impl_pull!($pxi: adc::Atten0dB<$adc>);
        impl_pull!($pxi: adc::Atten2p5dB<$adc>);
        impl_pull!($pxi: adc::Atten6dB<$adc>);
        impl_pull!($pxi: adc::Atten11dB<$adc>);
    };
}

macro_rules! impl_adc {
    ($pxi:ident: $pin:expr, ADC1: $adc:expr) => {
        #[cfg(not(feature = "riscv-ulp-hal"))]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_analog_atten_0db(
                mut self,
            ) -> Result<$pxi<adc::Atten0dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_0) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_2p5db(
                mut self,
            ) -> Result<$pxi<adc::Atten2p5dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_2_5) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_6db(
                mut self,
            ) -> Result<$pxi<adc::Atten6dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_6) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_11db(
                mut self,
            ) -> Result<$pxi<adc::Atten11dB<adc::ADC1>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc1_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_11) })?;

                Ok($pxi { _mode: PhantomData })
            }
        }

        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit(&self) -> adc_unit_t {
                adc_unit_t_ADC_UNIT_1
            }

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }

        impl<AN> embedded_hal_0_2::adc::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC1> + Send,
        {
            type ID = u8;

            fn channel() -> Self::ID {
                $adc
            }
        }
    };

    ($pxi:ident: $pin:expr, ADC2: $adc:expr) => {
        #[cfg(not(feature = "riscv-ulp-hal"))]
        impl<MODE> $pxi<MODE>
        where
            MODE: Send,
        {
            pub fn into_analog_atten_0db(
                mut self,
            ) -> Result<$pxi<adc::Atten0dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_0) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_2p5db(
                mut self,
            ) -> Result<$pxi<adc::Atten2p5dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_2_5) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_6db(
                mut self,
            ) -> Result<$pxi<adc::Atten6dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_6) })?;

                Ok($pxi { _mode: PhantomData })
            }

            pub fn into_analog_atten_11db(
                mut self,
            ) -> Result<$pxi<adc::Atten11dB<adc::ADC2>>, EspError> {
                self.reset()?;
                esp!(unsafe { adc2_config_channel_atten($adc, adc_atten_t_ADC_ATTEN_DB_11) })?;

                Ok($pxi { _mode: PhantomData })
            }
        }

        impl<MODE> ADCPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn adc_unit(&self) -> adc_unit_t {
                adc_unit_t_ADC_UNIT_2
            }

            fn adc_channel(&self) -> adc_channel_t {
                $adc
            }
        }

        impl<AN> embedded_hal_0_2::adc::Channel<AN> for $pxi<AN>
        where
            AN: adc::Analog<adc::ADC2> + Send,
        {
            type ID = u8;

            fn channel() -> Self::ID {
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
        impl<MODE> TouchPin for $pxi<MODE>
        where
            MODE: Send,
        {
            fn touch_channel(&self) -> touch_pad_t {
                $touch
            }
        }
    };

    ($pxi:ident: $pin:expr, NOTOUCH: $touch:expr) => {};
}

macro_rules! impl_hal_input_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal_0_2::digital::v2::InputPin for $pxi<$mode> {
            type Error = EspError;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_input_level())
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_input_level())
            }
        }

        impl embedded_hal::digital::blocking::InputPin for $pxi<$mode> {
            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_input_level())
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_input_level())
            }
        }
    };
}

macro_rules! impl_hal_output_pin {
    ($pxi:ident: $mode:ident) => {
        impl embedded_hal_0_2::digital::v2::OutputPin for $pxi<$mode> {
            type Error = EspError;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(true)
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(false)
            }
        }

        impl embedded_hal::digital::blocking::OutputPin for $pxi<$mode> {
            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(true)
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(false)
            }
        }

        impl embedded_hal::digital::blocking::StatefulOutputPin for $pxi<$mode> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_output_level())
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_output_level())
            }
        }

        impl embedded_hal_0_2::digital::v2::StatefulOutputPin for $pxi<$mode> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                Ok(self.get_output_level())
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.get_output_level())
            }
        }

        impl embedded_hal_0_2::digital::v2::ToggleableOutputPin for $pxi<$mode> {
            type Error = EspError;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(!self.get_output_level())
            }
        }

        impl embedded_hal::digital::blocking::ToggleableOutputPin for $pxi<$mode> {
            fn toggle(&mut self) -> Result<(), Self::Error> {
                self.set_output_level(!self.get_output_level())
            }
        }
    };
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
        impl_adc_pull!($pxi, $adc);
        impl_dac!($pxi: $pin, $dac: $dacno);
        impl_touch!($pxi: $pin, $touch: $touchno);
    };
}

/// Generic $GpioX pin
pub struct GpioPin<MODE> {
    pin: i32,
    _mode: PhantomData<MODE>,
}

impl<MODE> GpioPin<MODE>
where
    MODE: Send,
{
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Pin, if it is
    /// already instantiated and used elsewhere, or if it is not set
    /// already in the mode of operation which is being instantiated
    pub unsafe fn new(pin: i32) -> GpioPin<MODE> {
        Self {
            pin,
            _mode: PhantomData,
        }
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
}

impl InputPin for GpioPin<Input> {}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl InputPin for GpioPin<SubscribedInput> {}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl SubscribedPin for GpioPin<SubscribedInput> {}

impl OutputPin for GpioPin<Output> {}

impl InputPin for GpioPin<InputOutput> {}

impl OutputPin for GpioPin<InputOutput> {}

impl_base!(GpioPin);
impl_hal_input_pin!(GpioPin: Input);
impl_hal_input_pin!(GpioPin: InputOutput);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
impl_hal_input_pin!(GpioPin: SubscribedInput);

impl_hal_output_pin!(GpioPin: InputOutput);
impl_hal_output_pin!(GpioPin: Output);

#[cfg(esp32)]
mod chip {
    use core::marker::PhantomData;

    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

    use super::*;

    #[cfg(feature = "riscv-ulp-hal")]
    use crate::riscv_ulp_hal::sys::*;

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
        pub gpio0: Gpio0<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio1: Gpio1<Unknown>,
        pub gpio2: Gpio2<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio3: Gpio3<Unknown>,
        pub gpio4: Gpio4<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio5: Gpio5<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio6: Gpio6<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio7: Gpio7<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio8: Gpio8<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio9: Gpio9<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio10: Gpio10<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio11: Gpio11<Unknown>,
        pub gpio12: Gpio12<Unknown>,
        pub gpio13: Gpio13<Unknown>,
        pub gpio14: Gpio14<Unknown>,
        pub gpio15: Gpio15<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio16: Gpio16<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio17: Gpio17<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio18: Gpio18<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio19: Gpio19<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio21: Gpio21<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio22: Gpio22<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
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
        /// Care should be taken not to instantiate the Pins structure, if it is
        /// already instantiated and used elsewhere
        pub unsafe fn new() -> Self {
            Self {
                gpio0: Gpio0::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio1: Gpio1::<Unknown>::new(),
                gpio2: Gpio2::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio3: Gpio3::<Unknown>::new(),
                gpio4: Gpio4::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio5: Gpio5::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio6: Gpio6::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio7: Gpio7::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio8: Gpio8::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio9: Gpio9::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio10: Gpio10::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio11: Gpio11::<Unknown>::new(),
                gpio12: Gpio12::<Unknown>::new(),
                gpio13: Gpio13::<Unknown>::new(),
                gpio14: Gpio14::<Unknown>::new(),
                gpio15: Gpio15::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio16: Gpio16::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio17: Gpio17::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio18: Gpio18::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio19: Gpio19::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio21: Gpio21::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio22: Gpio22::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
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

    #[cfg(not(feature = "riscv-ulp-hal"))]
    use esp_idf_sys::*;

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
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio26: Gpio26<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio27: Gpio27<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio28: Gpio28<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio29: Gpio29<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio30: Gpio30<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio31: Gpio31<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio32: Gpio32<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio33: Gpio33<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio34: Gpio34<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio35: Gpio35<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio36: Gpio36<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio37: Gpio37<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio38: Gpio38<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio39: Gpio39<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio40: Gpio40<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio41: Gpio41<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio42: Gpio42<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio43: Gpio43<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio44: Gpio44<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio45: Gpio45<Unknown>,
        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub gpio46: Gpio46<Unknown>,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
        pub gpio47: Gpio47<Unknown>,
        #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
        pub gpio48: Gpio48<Unknown>,
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
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio26: Gpio26::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio27: Gpio27::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio28: Gpio28::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio29: Gpio29::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio30: Gpio30::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio31: Gpio31::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio32: Gpio32::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio33: Gpio33::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio34: Gpio34::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio35: Gpio35::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio36: Gpio36::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio37: Gpio37::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio38: Gpio38::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio39: Gpio39::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio40: Gpio40::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio41: Gpio41::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio42: Gpio42::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio43: Gpio43::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio44: Gpio44::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio45: Gpio45::<Unknown>::new(),
                #[cfg(not(feature = "riscv-ulp-hal"))]
                gpio46: Gpio46::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio47: Gpio47::<Unknown>::new(),
                #[cfg(all(esp32s3, not(feature = "riscv-ulp-hal")))]
                gpio48: Gpio48::<Unknown>::new(),
            }
        }
    }
}

#[cfg(esp32c3)]
#[cfg(not(feature = "riscv-ulp-hal"))]
mod chip {
    use core::marker::PhantomData;

    use esp_idf_sys::*;

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
