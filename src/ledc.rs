#![cfg(not(feature = "riscv-ulp-hal"))]

//! LED Control peripheral (which also crates PWM signals for other purposes)
//!
//! Interface to the [LED Control (LEDC)
//! peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/ledc.html)
//!
//! This is an initial implementation supporting the generation of PWM signals
//! but no chrome and spoilers like fading.
//!
//! # Examples
//!
//! Create a 25 kHz PWM signal with 75 % duty cycle on GPIO 1
//! ```
//! use embedded_hal::pwm::blocking::PwmPin;
//! use esp_idf_hal::ledc::{config::TimerConfig, Channel, Timer};
//! use esp_idf_hal::peripherals::Peripherals;
//! use esp_idf_hal::prelude::*;
//!
//! let peripherals = Peripherals::take().unwrap();
//! let config = TimerConfig::default().frequency(25.kHz().into());
//! let timer = Timer::new(peripherals.ledc.timer0, &config)?;
//! let channel = Channel::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio1)?;
//!
//! let max_duty = channel.get_max_duty()?;
//! channel.set_duty(max_duty * 3 / 4);
//! ```
//!
//! See the `examples/` folder of this repository for more.

use core::{borrow::Borrow, marker::PhantomData};

use crate::gpio::OutputPin;
use crate::mutex::Mutex;
use embedded_hal::pwm::blocking::PwmPin;
use esp_idf_sys::*;

pub use chip::*;

type Duty = u32;

const IDLE_LEVEL: u32 = 0;

static FADE_FUNC_INSTALLED: Mutex<bool> = Mutex::new(false);

/// Types for configuring the LED Control peripheral
pub mod config {
    use super::*;
    use crate::units::*;

    pub use chip::Resolution;

    pub struct TimerConfig {
        pub frequency: Hertz,
        pub resolution: Resolution,
        pub speed_mode: ledc_mode_t,
    }

    impl TimerConfig {
        #[must_use]
        pub fn frequency(mut self, f: Hertz) -> Self {
            self.frequency = f;
            self
        }

        #[must_use]
        pub fn resolution(mut self, r: Resolution) -> Self {
            self.resolution = r;
            self
        }

        #[must_use]
        pub fn speed_mode(mut self, mode: ledc_mode_t) -> Self {
            self.speed_mode = mode;
            self
        }
    }

    impl Default for TimerConfig {
        fn default() -> Self {
            TimerConfig {
                frequency: 1000.Hz(),
                resolution: Resolution::Bits8,
                speed_mode: ledc_mode_t_LEDC_LOW_SPEED_MODE,
            }
        }
    }
}

/// LED Control timer abstraction
pub struct Timer<T: HwTimer> {
    instance: T,
    speed_mode: ledc_mode_t,
    max_duty: Duty,
}

impl<T: HwTimer> Timer<T> {
    /// Creates a new LED Control timer abstraction
    pub fn new(instance: T, config: &config::TimerConfig) -> Result<Self, EspError> {
        let timer_config = ledc_timer_config_t {
            speed_mode: config.speed_mode,
            timer_num: T::timer(),
            __bindgen_anon_1: ledc_timer_config_t__bindgen_ty_1 {
                duty_resolution: config.resolution.timer_bits(),
            },
            freq_hz: config.frequency.into(),
            clk_cfg: ledc_clk_cfg_t_LEDC_AUTO_CLK,
        };

        // SAFETY: We own the instance and therefor are safe to configure it.
        esp!(unsafe { ledc_timer_config(&timer_config) })?;

        Ok(Timer {
            instance,
            speed_mode: config.speed_mode,
            max_duty: config.resolution.max_duty(),
        })
    }

    /// Pauses the timer. Operation can be resumed with
    /// [`resume()`](Timer::resume()).
    pub fn pause(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_pause(self.speed_mode, T::timer()) })?;
        Ok(())
    }

    /// Stops the timer and releases its hardware resource
    pub fn release(mut self) -> Result<T, EspError> {
        self.reset()?;
        Ok(self.instance)
    }

    fn reset(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_rst(self.speed_mode, T::timer()) })?;
        Ok(())
    }

    /// Resumes the operation of a previously paused timer
    pub fn resume(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_resume(self.speed_mode, T::timer()) })?;
        Ok(())
    }
}

/// LED Control output channel abstraction
pub struct Channel<C: HwChannel, H: HwTimer, T: Borrow<Timer<H>>, P: OutputPin> {
    instance: C,
    _hw_timer: PhantomData<H>,
    timer: T,
    pin: P,
    duty: Duty,
}

// TODO: Stop channel when the instance gets dropped. It seems that we can't
// have both at the same time: a method for releasing its hardware resources
// and implementing Drop.
impl<C: HwChannel, H: HwTimer, T: Borrow<Timer<H>>, P: OutputPin> Channel<C, H, T, P> {
    /// Creates a new LED Control output channel abstraction
    pub fn new(instance: C, timer: T, pin: P) -> Result<Self, EspError> {
        let duty = 0;
        let channel_config = ledc_channel_config_t {
            speed_mode: timer.borrow().speed_mode,
            channel: C::channel(),
            timer_sel: H::timer(),
            intr_type: ledc_intr_type_t_LEDC_INTR_DISABLE,
            gpio_num: pin.pin(),
            duty: duty as u32,
            ..Default::default()
        };

        let mut installed = FADE_FUNC_INSTALLED.lock();
        if !*installed {
            // It looks like ledc_channel_config requires the face function to
            // be installed. I don't see why this is nescessary yet but hey,
            // let the Wookie win for now.
            //
            // TODO: Check whether it's worth to track its install status and
            // remove it if no longer needed.
            esp!(unsafe { ledc_fade_func_install(0) })?;
            *installed = true;
        }
        drop(installed);

        // SAFETY: As long as we have borrowed the timer, we are safe to use
        // it.
        esp!(unsafe { ledc_channel_config(&channel_config) })?;

        Ok(Channel {
            instance,
            _hw_timer: PhantomData,
            timer,
            pin,
            duty,
        })
    }

    /// Stops the output channel and releases its hardware resource and GPIO
    /// pin
    pub fn release(mut self) -> Result<(C, P), EspError> {
        self.stop()?;
        Ok((self.instance, self.pin))
    }

    fn get_duty(&self) -> Duty {
        self.duty
    }

    fn get_max_duty(&self) -> Duty {
        self.timer.borrow().max_duty
    }

    fn disable(&mut self) -> Result<(), EspError> {
        self.update_duty(0)?;
        Ok(())
    }

    fn enable(&mut self) -> Result<(), EspError> {
        self.update_duty(self.duty)?;
        Ok(())
    }

    fn set_duty(&mut self, duty: Duty) -> Result<(), EspError> {
        // Clamp the actual duty cycle to the current maximum as done by other
        // Pwm/PwmPin implementations.
        //
        // TODO: Why does calling self.get_max_duty() result in the compiler
        // error 'expected `u32`, found enum `Result`' when our method returns
        // Duty?
        let clamped = duty.min(self.timer.borrow().max_duty);
        self.duty = clamped;
        self.update_duty(clamped)?;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_stop(self.timer.borrow().speed_mode, C::channel(), IDLE_LEVEL) })?;
        Ok(())
    }

    fn update_duty(&mut self, duty: Duty) -> Result<(), EspError> {
        esp!(unsafe {
            ledc_set_duty_and_update(
                self.timer.borrow().speed_mode,
                C::channel(),
                duty as u32,
                Default::default(),
            )
        })?;
        Ok(())
    }
}

impl<C: HwChannel, H: HwTimer, T: Borrow<Timer<H>>, P: OutputPin> PwmPin for Channel<C, H, T, P> {
    type Duty = Duty;
    type Error = EspError;

    fn disable(&mut self) -> Result<(), Self::Error> {
        self.disable()
    }

    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enable()
    }

    fn get_duty(&self) -> Result<Self::Duty, Self::Error> {
        Ok(self.get_duty())
    }

    fn get_max_duty(&self) -> Result<Self::Duty, Self::Error> {
        Ok(self.get_max_duty())
    }

    fn set_duty(&mut self, duty: Duty) -> Result<(), Self::Error> {
        self.set_duty(duty)
    }
}

impl<C: HwChannel, H: HwTimer, T: Borrow<Timer<H>>, P: OutputPin> embedded_hal_0_2::PwmPin
    for Channel<C, H, T, P>
{
    type Duty = Duty;

    fn disable(&mut self) {
        if let Err(e) = self.disable() {
            panic!("disabling PWM failed: {}", e);
        }
    }

    fn enable(&mut self) {
        if let Err(e) = self.enable() {
            panic!("enabling PWM failed: {}", e);
        }
    }

    fn get_duty(&self) -> Self::Duty {
        self.get_duty()
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.get_max_duty()
    }

    fn set_duty(&mut self, duty: Duty) {
        if let Err(e) = self.set_duty(duty) {
            panic!("updating duty failed: {}", e);
        }
    }
}

mod chip {
    use core::marker::PhantomData;
    use esp_idf_sys::*;

    pub enum Resolution {
        Bits1,
        Bits2,
        Bits3,
        Bits4,
        Bits5,
        Bits6,
        Bits7,
        Bits8,
        Bits9,
        Bits10,
        Bits11,
        Bits12,
        Bits13,
        Bits14,
        #[cfg(esp32)]
        Bits15,
        #[cfg(esp32)]
        Bits16,
        #[cfg(esp32)]
        Bits17,
        #[cfg(esp32)]
        Bits18,
        #[cfg(esp32)]
        Bits19,
        #[cfg(esp32)]
        Bits20,
    }

    impl Resolution {
        pub const fn bits(&self) -> usize {
            match self {
                Resolution::Bits1 => 1,
                Resolution::Bits2 => 2,
                Resolution::Bits3 => 3,
                Resolution::Bits4 => 4,
                Resolution::Bits5 => 5,
                Resolution::Bits6 => 6,
                Resolution::Bits7 => 7,
                Resolution::Bits8 => 8,
                Resolution::Bits9 => 9,
                Resolution::Bits10 => 10,
                Resolution::Bits11 => 11,
                Resolution::Bits12 => 12,
                Resolution::Bits13 => 13,
                Resolution::Bits14 => 14,
                #[cfg(esp32)]
                Resolution::Bits15 => 15,
                #[cfg(esp32)]
                Resolution::Bits16 => 16,
                #[cfg(esp32)]
                Resolution::Bits17 => 17,
                #[cfg(esp32)]
                Resolution::Bits18 => 18,
                #[cfg(esp32)]
                Resolution::Bits19 => 19,
                #[cfg(esp32)]
                Resolution::Bits20 => 20,
            }
        }

        pub const fn max_duty(&self) -> u32 {
            (1 << self.bits()) - 1
        }

        pub(crate) const fn timer_bits(&self) -> ledc_timer_bit_t {
            match self {
                Resolution::Bits1 => ledc_timer_bit_t_LEDC_TIMER_1_BIT,
                Resolution::Bits2 => ledc_timer_bit_t_LEDC_TIMER_2_BIT,
                Resolution::Bits3 => ledc_timer_bit_t_LEDC_TIMER_3_BIT,
                Resolution::Bits4 => ledc_timer_bit_t_LEDC_TIMER_4_BIT,
                Resolution::Bits5 => ledc_timer_bit_t_LEDC_TIMER_5_BIT,
                Resolution::Bits6 => ledc_timer_bit_t_LEDC_TIMER_6_BIT,
                Resolution::Bits7 => ledc_timer_bit_t_LEDC_TIMER_7_BIT,
                Resolution::Bits8 => ledc_timer_bit_t_LEDC_TIMER_8_BIT,
                Resolution::Bits9 => ledc_timer_bit_t_LEDC_TIMER_9_BIT,
                Resolution::Bits10 => ledc_timer_bit_t_LEDC_TIMER_10_BIT,
                Resolution::Bits11 => ledc_timer_bit_t_LEDC_TIMER_11_BIT,
                Resolution::Bits12 => ledc_timer_bit_t_LEDC_TIMER_12_BIT,
                Resolution::Bits13 => ledc_timer_bit_t_LEDC_TIMER_13_BIT,
                Resolution::Bits14 => ledc_timer_bit_t_LEDC_TIMER_14_BIT,
                #[cfg(esp32)]
                Resolution::Bits15 => ledc_timer_bit_t_LEDC_TIMER_15_BIT,
                #[cfg(esp32)]
                Resolution::Bits16 => ledc_timer_bit_t_LEDC_TIMER_16_BIT,
                #[cfg(esp32)]
                Resolution::Bits17 => ledc_timer_bit_t_LEDC_TIMER_17_BIT,
                #[cfg(esp32)]
                Resolution::Bits18 => ledc_timer_bit_t_LEDC_TIMER_18_BIT,
                #[cfg(esp32)]
                Resolution::Bits19 => ledc_timer_bit_t_LEDC_TIMER_19_BIT,
                #[cfg(esp32)]
                Resolution::Bits20 => ledc_timer_bit_t_LEDC_TIMER_20_BIT,
            }
        }
    }

    /// LED Control peripheral timer
    pub trait HwTimer {
        fn timer() -> ledc_timer_t;
    }

    /// LED Control peripheral output channel
    pub trait HwChannel {
        fn channel() -> ledc_channel_t;
    }

    macro_rules! impl_timer {
        ($instance:ident: $timer:expr) => {
            pub struct $instance {
                _marker: PhantomData<ledc_timer_t>,
            }

            impl $instance {
                /// # Safety
                ///
                /// It is safe to instantiate this timer exactly one time.
                pub unsafe fn new() -> Self {
                    $instance {
                        _marker: PhantomData,
                    }
                }
            }

            impl HwTimer for $instance {
                fn timer() -> ledc_timer_t {
                    $timer
                }
            }
        };
    }

    impl_timer!(TIMER0: ledc_timer_t_LEDC_TIMER_0);
    impl_timer!(TIMER1: ledc_timer_t_LEDC_TIMER_1);
    impl_timer!(TIMER2: ledc_timer_t_LEDC_TIMER_2);
    impl_timer!(TIMER3: ledc_timer_t_LEDC_TIMER_3);

    macro_rules! impl_channel {
        ($instance:ident: $channel:expr) => {
            pub struct $instance {
                _marker: PhantomData<ledc_channel_t>,
            }

            impl $instance {
                /// # Safety
                ///
                /// It is safe to instantiate this output channel exactly one
                /// time.
                pub unsafe fn new() -> Self {
                    $instance {
                        _marker: PhantomData,
                    }
                }
            }

            impl HwChannel for $instance {
                fn channel() -> ledc_channel_t {
                    $channel
                }
            }
        };
    }

    impl_channel!(CHANNEL0: ledc_channel_t_LEDC_CHANNEL_0);
    impl_channel!(CHANNEL1: ledc_channel_t_LEDC_CHANNEL_1);
    impl_channel!(CHANNEL2: ledc_channel_t_LEDC_CHANNEL_2);
    impl_channel!(CHANNEL3: ledc_channel_t_LEDC_CHANNEL_3);
    impl_channel!(CHANNEL4: ledc_channel_t_LEDC_CHANNEL_4);
    impl_channel!(CHANNEL5: ledc_channel_t_LEDC_CHANNEL_5);
    #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
    impl_channel!(CHANNEL6: ledc_channel_t_LEDC_CHANNEL_6);
    #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
    impl_channel!(CHANNEL7: ledc_channel_t_LEDC_CHANNEL_7);

    /// The LED Control device peripheral
    pub struct Peripheral {
        pub timer0: TIMER0,
        pub timer1: TIMER1,
        pub timer2: TIMER2,
        pub timer3: TIMER3,
        pub channel0: CHANNEL0,
        pub channel1: CHANNEL1,
        pub channel2: CHANNEL2,
        pub channel3: CHANNEL3,
        pub channel4: CHANNEL4,
        pub channel5: CHANNEL5,
        #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
        pub channel6: CHANNEL6,
        #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
        pub channel7: CHANNEL7,
    }

    impl Peripheral {
        /// Creates a new instance of the LEDC peripheral. Typically one wants
        /// to use the instance [`ledc`](crate::peripherals::Peripherals::ledc) from
        /// the device peripherals obtained via
        /// [`peripherals::Peripherals::take()`](crate::peripherals::Peripherals::take()).
        ///
        /// # Safety
        ///
        /// It is safe to instantiate the LEDC peripheral exactly one time.
        /// Care has to be taken that this has not already been done elsewhere.
        pub unsafe fn new() -> Self {
            Self {
                timer0: TIMER0::new(),
                timer1: TIMER1::new(),
                timer2: TIMER2::new(),
                timer3: TIMER3::new(),
                channel0: CHANNEL0::new(),
                channel1: CHANNEL1::new(),
                channel2: CHANNEL2::new(),
                channel3: CHANNEL3::new(),
                channel4: CHANNEL4::new(),
                channel5: CHANNEL5::new(),
                #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
                channel6: CHANNEL6::new(),
                #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
                channel7: CHANNEL7::new(),
            }
        }
    }
}
