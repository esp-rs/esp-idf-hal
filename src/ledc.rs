//! LED Control peripheral (which also creates PWM signals for other purposes)
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
//! use esp_idf_hal::ledc::{config::TimerConfig, Channel, LedcDriver, LedcTimerDriver, Timer};
//! use esp_idf_hal::peripherals::Peripherals;
//! use esp_idf_hal::prelude::*;
//!
//! let peripherals = Peripherals::take().unwrap();
//! let timer_driver = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default().frequency(25.kHz().into()));
//! let mut driver = LedcDriver::new(peripherals.ledc.channel0, timer_driver, peripherals.pins.gpio1)?;
//!
//! let max_duty = driver.get_max_duty()?;
//! driver.set_duty(max_duty * 3 / 4)?;
//! ```
//!
//! See the `examples/` folder of this repository for more.

use core::borrow::Borrow;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use esp_idf_sys::*;

use crate::gpio::OutputPin;
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::task::CriticalSection;
use crate::units::*;

pub use chip::*;

type Duty = u32;
type HPoint = Duty;

const IDLE_LEVEL: u32 = 0;

static FADE_FUNC_INSTALLED: AtomicBool = AtomicBool::new(false);
static FADE_FUNC_INSTALLED_CS: CriticalSection = CriticalSection::new();

crate::embedded_hal_error!(
    PwmError,
    embedded_hal::pwm::Error,
    embedded_hal::pwm::ErrorKind
);

/// Types for configuring the LED Control peripheral
pub mod config {
    use super::*;

    pub use chip::Resolution;

    #[derive(Clone, Debug, Eq, PartialEq)]
    pub struct TimerConfig {
        pub frequency: Hertz,
        pub resolution: Resolution,
    }

    impl TimerConfig {
        pub const fn new() -> Self {
            Self {
                frequency: Hertz(1000),
                resolution: Resolution::Bits8,
            }
        }

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
    }

    impl Default for TimerConfig {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// LED Control timer driver
pub struct LedcTimerDriver<'d, T>
where
    T: LedcTimer,
{
    _timer: PeripheralRef<'d, T>,
    max_duty: Duty,
    _p: PhantomData<&'d mut ()>,
}

impl<'d, T> LedcTimerDriver<'d, T>
where
    T: LedcTimer,
{
    pub fn new(
        timer: impl Peripheral<P = T> + 'd,
        config: &config::TimerConfig,
    ) -> Result<Self, EspError> {
        crate::into_ref!(timer);

        let timer_config = ledc_timer_config_t {
            speed_mode: T::SpeedMode::SPEED_MODE,
            timer_num: T::timer() as _,
            #[cfg(esp_idf_version_major = "4")]
            __bindgen_anon_1: ledc_timer_config_t__bindgen_ty_1 {
                duty_resolution: config.resolution.timer_bits(),
            },
            #[cfg(not(esp_idf_version_major = "4"))]
            duty_resolution: config.resolution.timer_bits(),
            freq_hz: config.frequency.into(),
            #[cfg(any(esp_idf_version_major = "4", esp_idf_version_minor = "0"))]
            clk_cfg: ledc_clk_cfg_t_LEDC_AUTO_CLK,
            #[cfg(not(any(esp_idf_version_major = "4", esp_idf_version_minor = "0")))]
            clk_cfg: soc_periph_ledc_clk_src_legacy_t_LEDC_AUTO_CLK,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
                all(esp_idf_version_major = "5", esp_idf_version_minor = "1")
            )))]
            deconfigure: false,
        };

        // SAFETY: We own the instance and therefor are safe to configure it.
        esp!(unsafe { ledc_timer_config(&timer_config) })?;

        Ok(Self {
            _timer: timer,
            max_duty: config.resolution.max_duty(),
            _p: PhantomData,
        })
    }

    /// Pauses the timer. Operation can be resumed with [`resume_timer()`].
    pub fn pause(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_pause(T::SpeedMode::SPEED_MODE, self.timer()) })?;
        Ok(())
    }

    /// Resumes the operation of the previously paused timer
    pub fn resume(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_resume(T::SpeedMode::SPEED_MODE, self.timer()) })?;
        Ok(())
    }

    /// Set the frequency of the timer.
    pub fn set_frequency(&mut self, frequency: Hertz) -> Result<(), EspError> {
        esp!(unsafe { ledc_set_freq(T::SpeedMode::SPEED_MODE, T::timer(), frequency.into()) })?;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_timer_rst(T::SpeedMode::SPEED_MODE, T::timer()) })?;
        Ok(())
    }

    pub fn timer(&self) -> ledc_timer_t {
        T::timer()
    }
}

impl<'d, T> Drop for LedcTimerDriver<'d, T>
where
    T: LedcTimer,
{
    fn drop(&mut self) {
        self.reset().unwrap();
    }
}

unsafe impl<'d, T> Send for LedcTimerDriver<'d, T> where T: LedcTimer {}

/// LED Control driver
pub struct LedcDriver<'d> {
    channel: u8,
    timer: u8,
    duty: Duty,
    hpoint: HPoint,
    speed_mode: ledc_mode_t,
    max_duty: Duty,
    _p: PhantomData<&'d mut ()>,
}

// TODO: Stop channel when the instance gets dropped. It seems that we can't
// have both at the same time: a method for releasing its hardware resources
// and implementing Drop.
impl<'d> LedcDriver<'d> {
    /// Creates a new LED Control driver
    pub fn new<C, T, B>(
        _channel: impl Peripheral<P = C> + 'd,
        timer_driver: B,
        pin: impl Peripheral<P = impl OutputPin> + 'd,
    ) -> Result<Self, EspError>
    where
        C: LedcChannel<SpeedMode = <T as LedcTimer>::SpeedMode>,
        T: LedcTimer + 'd,
        B: Borrow<LedcTimerDriver<'d, T>>,
    {
        if !FADE_FUNC_INSTALLED.load(Ordering::SeqCst) {
            let _guard = FADE_FUNC_INSTALLED_CS.enter();

            if !FADE_FUNC_INSTALLED.load(Ordering::SeqCst) {
                // It looks like ledc_channel_config requires the face function to
                // be installed. I don't see why this is nescessary yet but hey,
                // let the Wookie win for now.
                //
                // TODO: Check whether it's worth to track its install status and
                // remove it if no longer needed.
                esp!(unsafe { ledc_fade_func_install(0) })?;

                FADE_FUNC_INSTALLED.store(true, Ordering::SeqCst);
            }
        }

        let mut driver = LedcDriver {
            duty: 0,
            hpoint: 0,
            speed_mode: T::SpeedMode::SPEED_MODE,
            max_duty: timer_driver.borrow().max_duty,
            timer: timer_driver.borrow().timer() as _,
            channel: C::channel() as _,
            _p: PhantomData,
        };

        driver.config_with_pin(pin)?;

        Ok(driver)
    }

    /// Applies LEDC configuration with a specific pin
    /// Can be used to reconfigure the LEDC driver with a different pin
    pub fn config_with_pin(
        &mut self,
        pin: impl Peripheral<P = impl OutputPin> + 'd,
    ) -> Result<(), EspError> {
        crate::into_ref!(pin);

        let channel_config = ledc_channel_config_t {
            speed_mode: self.speed_mode,
            channel: self.channel as u32,
            timer_sel: self.timer as u32,
            intr_type: ledc_intr_type_t_LEDC_INTR_DISABLE,
            gpio_num: pin.pin(),
            duty: self.duty,
            hpoint: self.hpoint as _,
            ..Default::default()
        };

        // SAFETY: As long as we have borrowed the timer, we are safe to use
        // it.
        esp!(unsafe { ledc_channel_config(&channel_config) })?;
        Ok(())
    }

    pub fn get_duty(&self) -> Duty {
        self.duty
    }

    pub fn get_hpoint(&self) -> HPoint {
        self.hpoint
    }

    pub fn get_max_duty(&self) -> Duty {
        self.max_duty
    }

    pub fn disable(&mut self) -> Result<(), EspError> {
        self.update_duty(0, 0)?;
        Ok(())
    }

    pub fn enable(&mut self) -> Result<(), EspError> {
        self.update_duty(self.duty, self.hpoint)?;
        Ok(())
    }

    pub fn set_duty(&mut self, duty: Duty) -> Result<(), EspError> {
        self.set_duty_with_hpoint(duty, self.hpoint)
    }

    pub fn set_hpoint(&mut self, hpoint: HPoint) -> Result<(), EspError> {
        self.set_duty_with_hpoint(self.duty, hpoint)
    }

    pub fn set_duty_with_hpoint(&mut self, duty: Duty, hpoint: HPoint) -> Result<(), EspError> {
        // Clamp the actual duty cycle to the current maximum as done by other
        // Pwm/PwmPin implementations.
        let max_duty = self.get_max_duty();
        let clamped_duty = duty.min(max_duty);
        let clamped_hpoint = hpoint.min(max_duty);
        self.duty = clamped_duty;
        self.hpoint = clamped_hpoint;
        self.update_duty(clamped_duty, clamped_hpoint)?;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { ledc_stop(self.speed_mode, self.channel(), IDLE_LEVEL,) })?;
        Ok(())
    }

    fn update_duty(&mut self, duty: Duty, hpoint: HPoint) -> Result<(), EspError> {
        esp!(unsafe { ledc_set_duty_and_update(self.speed_mode, self.channel(), duty, hpoint) })?;
        Ok(())
    }

    pub fn channel(&self) -> ledc_channel_t {
        self.channel as _
    }

    pub fn timer(&self) -> ledc_timer_t {
        self.timer as _
    }
}

impl<'d> Drop for LedcDriver<'d> {
    fn drop(&mut self) {
        self.stop().unwrap();
    }
}

unsafe impl<'d> Send for LedcDriver<'d> {}

impl<'d> embedded_hal::pwm::ErrorType for LedcDriver<'d> {
    type Error = PwmError;
}

fn to_pwm_err(err: EspError) -> PwmError {
    PwmError::other(err)
}

impl<'d> embedded_hal::pwm::SetDutyCycle for LedcDriver<'d> {
    fn max_duty_cycle(&self) -> u16 {
        let duty = self.get_max_duty();
        let duty_cap: u16 = if duty > u16::MAX as u32 {
            u16::MAX
        } else {
            duty as u16
        };
        duty_cap
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), PwmError> {
        self.set_duty(duty as u32).map_err(to_pwm_err)
    }

    fn set_duty_cycle_fully_on(&mut self) -> Result<(), PwmError> {
        self.set_duty(self.get_max_duty()).map_err(to_pwm_err)
    }

    fn set_duty_cycle_fully_off(&mut self) -> Result<(), PwmError> {
        self.set_duty(0).map_err(to_pwm_err)
    }

    fn set_duty_cycle_fraction(&mut self, num: u16, denom: u16) -> Result<(), PwmError> {
        let duty = num as u32 * self.max_duty_cycle() as u32 / denom as u32;
        self.set_duty_cycle(duty as u16)
    }

    fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), PwmError> {
        self.set_duty_cycle_fraction(percent as u16, 100)
    }
}

impl<'d> embedded_hal_0_2::PwmPin for LedcDriver<'d> {
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
    use esp_idf_sys::*;

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
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
            // when using the maximum resultion, the duty cycle must not exceed 2^N - 1 to avoid timer overflow
            if cfg!(esp32) && self.bits() == 20 || cfg!(not(esp32)) && self.bits() == 14 {
                (1 << self.bits()) - 1
            } else {
                1 << self.bits()
            }
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

    /// Speed mode for the LED Control peripheral
    /// The ESP32 supports two speed modes: low and high speed
    /// All others support only low speed mode.
    pub trait SpeedMode: Send + Sync + 'static {
        const SPEED_MODE: ledc_mode_t;
        const HIGH_SPEED: bool;
    }

    /// Low speed mode for the LED Control peripheral
    pub struct LowSpeed;

    impl SpeedMode for LowSpeed {
        const SPEED_MODE: ledc_mode_t = ledc_mode_t_LEDC_LOW_SPEED_MODE;
        const HIGH_SPEED: bool = false;
    }

    #[cfg(esp32)]
    /// High speed mode for the LED Control peripheral (ESP32 only)
    pub struct HighSpeed;

    #[cfg(esp32)]
    impl SpeedMode for HighSpeed {
        const SPEED_MODE: ledc_mode_t = ledc_mode_t_LEDC_HIGH_SPEED_MODE;
        const HIGH_SPEED: bool = true;
    }

    /// LED Control peripheral timer
    pub trait LedcTimer {
        type SpeedMode: SpeedMode;

        fn timer() -> ledc_timer_t;
    }

    /// LED Control peripheral output channel
    pub trait LedcChannel {
        type SpeedMode: SpeedMode;

        fn channel() -> ledc_channel_t;
    }

    macro_rules! impl_timer {
        ($typ:ty; $instance:ident: $timer:expr) => {
            crate::impl_peripheral!($instance);

            impl LedcTimer for $instance {
                type SpeedMode = $typ;

                fn timer() -> ledc_timer_t {
                    $timer
                }
            }
        };
    }

    impl_timer!(LowSpeed; TIMER0: ledc_timer_t_LEDC_TIMER_0);
    impl_timer!(LowSpeed; TIMER1: ledc_timer_t_LEDC_TIMER_1);
    impl_timer!(LowSpeed; TIMER2: ledc_timer_t_LEDC_TIMER_2);
    impl_timer!(LowSpeed; TIMER3: ledc_timer_t_LEDC_TIMER_3);

    #[cfg(esp32)]
    impl_timer!(HighSpeed; HTIMER0: ledc_timer_t_LEDC_TIMER_0);
    #[cfg(esp32)]
    impl_timer!(HighSpeed; HTIMER1: ledc_timer_t_LEDC_TIMER_1);
    #[cfg(esp32)]
    impl_timer!(HighSpeed; HTIMER2: ledc_timer_t_LEDC_TIMER_2);
    #[cfg(esp32)]
    impl_timer!(HighSpeed; HTIMER3: ledc_timer_t_LEDC_TIMER_3);

    macro_rules! impl_channel {
        ($typ:ty; $instance:ident: $channel:expr) => {
            crate::impl_peripheral!($instance);

            impl LedcChannel for $instance {
                type SpeedMode = $typ;

                fn channel() -> ledc_channel_t {
                    $channel
                }
            }
        };
    }

    impl_channel!(LowSpeed; CHANNEL0: ledc_channel_t_LEDC_CHANNEL_0);
    impl_channel!(LowSpeed; CHANNEL1: ledc_channel_t_LEDC_CHANNEL_1);
    impl_channel!(LowSpeed; CHANNEL2: ledc_channel_t_LEDC_CHANNEL_2);
    impl_channel!(LowSpeed; CHANNEL3: ledc_channel_t_LEDC_CHANNEL_3);
    impl_channel!(LowSpeed; CHANNEL4: ledc_channel_t_LEDC_CHANNEL_4);
    impl_channel!(LowSpeed; CHANNEL5: ledc_channel_t_LEDC_CHANNEL_5);
    #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
    impl_channel!(LowSpeed; CHANNEL6: ledc_channel_t_LEDC_CHANNEL_6);
    #[cfg(any(esp32, esp32s2, esp32s3, esp8684))]
    impl_channel!(LowSpeed; CHANNEL7: ledc_channel_t_LEDC_CHANNEL_7);

    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL0: ledc_channel_t_LEDC_CHANNEL_0);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL1: ledc_channel_t_LEDC_CHANNEL_1);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL2: ledc_channel_t_LEDC_CHANNEL_2);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL3: ledc_channel_t_LEDC_CHANNEL_3);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL4: ledc_channel_t_LEDC_CHANNEL_4);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL5: ledc_channel_t_LEDC_CHANNEL_5);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL6: ledc_channel_t_LEDC_CHANNEL_6);
    #[cfg(esp32)]
    impl_channel!(HighSpeed; HCHANNEL7: ledc_channel_t_LEDC_CHANNEL_7);

    /// The LED Control device peripheral
    pub struct LEDC {
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

    impl LEDC {
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

    /// The LED Control device peripheral (high speed channels, ESP32 only)
    #[cfg(esp32)]
    pub struct HLEDC {
        pub timer0: HTIMER0,
        pub timer1: HTIMER1,
        pub timer2: HTIMER2,
        pub timer3: HTIMER3,
        pub channel0: HCHANNEL0,
        pub channel1: HCHANNEL1,
        pub channel2: HCHANNEL2,
        pub channel3: HCHANNEL3,
        pub channel4: HCHANNEL4,
        pub channel5: HCHANNEL5,
        pub channel6: HCHANNEL6,
        pub channel7: HCHANNEL7,
    }

    #[cfg(esp32)]
    impl HLEDC {
        /// Creates a new instance of the HLEDC peripheral. Typically one wants
        /// to use the instance [`ledc`](crate::peripherals::Peripherals::fledc) from
        /// the device peripherals obtained via
        /// [`peripherals::Peripherals::take()`](crate::peripherals::Peripherals::take()).
        ///
        /// # Safety
        ///
        /// It is safe to instantiate the HLEDC peripheral exactly one time.
        /// Care has to be taken that this has not already been done elsewhere.
        pub unsafe fn new() -> Self {
            Self {
                timer0: HTIMER0::new(),
                timer1: HTIMER1::new(),
                timer2: HTIMER2::new(),
                timer3: HTIMER3::new(),
                channel0: HCHANNEL0::new(),
                channel1: HCHANNEL1::new(),
                channel2: HCHANNEL2::new(),
                channel3: HCHANNEL3::new(),
                channel4: HCHANNEL4::new(),
                channel5: HCHANNEL5::new(),
                channel6: HCHANNEL6::new(),
                channel7: HCHANNEL7::new(),
            }
        }
    }
}
