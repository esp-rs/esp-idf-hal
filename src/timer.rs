use core::marker::PhantomData;

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

use esp_idf_sys::*;

use crate::delay::*;
use crate::gpio::*;
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::units::*;

crate::embedded_hal_error!(
    TimerError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

pub type TimerConfig = config::Config;

/// Timer configuration
pub mod config {
    use crate::units::*;
    use core::time::Duration;

    #[derive(Copy, Clone)]
    pub struct Config {
        pub divider: u32,
        #[cfg(any(esp32s2, esp32s3, esp32c3))]
        pub xtal: bool,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn divider(mut self, divider: u32) -> Self {
            self.divider = divider;
            self
        }

        #[must_use]
        #[cfg(any(esp32s2, esp32s3, esp32c3))]
        pub fn xtal(mut self, xtal: bool) -> Self {
            self.xtal = xtal;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                divider: 2,
                #[cfg(any(esp32s2, esp32s3, esp32c3))]
                xtal: false,
            }
        }
    }
}

pub trait Timer: Send {
    fn group() -> timer_group_t;
    fn index() -> timer_idx_t;
}

pub struct TimerDriver<'d, TIMER>
where
    TIMER: Timer,
{
    _timer: PeripheralRef<'d, TIMER>,
}

impl<'d, TIMER> TimerDriver<'d, TIMER>
where
    TIMER: Timer,
{
    pub fn new(
        timer: impl Peripheral<P = TIMER> + 'd,
        config: &config::Config,
    ) -> Result<TimerDriver<'d, TIMER>, EspError> {
        crate::into_ref!(timer);

        esp!(unsafe {
            timer_init(
                TIMER::group(),
                TIMER::index(),
                &timer_config_t {
                    alarm_en: timer_alarm_t_TIMER_ALARM_DIS,
                    counter_en: timer_start_t_TIMER_PAUSE,
                    counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
                    auto_reload: timer_autoreload_t_TIMER_AUTORELOAD_DIS,
                    intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
                    divider: config.divider,
                    #[cfg(any(esp32s2, esp32s3, esp32c3))]
                    clk_src: if config.xtal {
                        timer_src_clk_t_TIMER_SRC_CLK_XTAL
                    } else {
                        timer_src_clk_t_TIMER_SRC_CLK_ARB
                    },
                },
            )
        })?;

        Ok(TimerDriver { _timer: timer })
    }

    pub fn enable(&mut self, enable: bool) -> Result<(), EspError> {
        if enable {
            esp!(unsafe { timer_start(TIMER::group(), TIMER::index()) })?;
        } else {
            esp!(unsafe { timer_pause(TIMER::group(), TIMER::index()) })?;
        }

        Ok(())
    }

    pub fn counter(&self) -> Result<u64, EspError> {
        let mut value = 0_u64;

        esp!(unsafe {
            timer_get_counter_value(TIMER::group(), TIMER::index(), &mut value as *mut _)
        })?;

        Ok(value)
    }

    pub fn set_counter(&mut self, value: u64) -> Result<(), EspError> {
        esp!(unsafe { timer_set_counter_value(TIMER::group(), TIMER::index(), value) })?;

        Ok(())
    }

    pub fn enable_alarm(&mut self, enable: bool) -> Result<(), EspError> {
        esp!(unsafe {
            timer_set_alarm(
                TIMER::group(),
                TIMER::index(),
                if enable {
                    timer_alarm_t_TIMER_ALARM_DIS
                } else {
                    timer_alarm_t_TIMER_ALARM_DIS
                },
            )
        })?;

        Ok(())
    }

    pub fn alarm(&self) -> Result<u64, EspError> {
        let mut value = 0_u64;

        esp!(unsafe { timer_get_alarm_value(TIMER::group(), TIMER::index(), &mut value) })?;

        Ok(value)
    }

    pub fn set_alarm(&mut self, value: u64) -> Result<(), EspError> {
        esp!(unsafe { timer_set_alarm_value(TIMER::group(), TIMER::index(), value) })?;

        Ok(())
    }
}

impl<'d, TIMER: Timer> Drop for TimerDriver<'d, TIMER> {
    fn drop(&mut self) {
        esp!(unsafe { timer_deinit(TIMER::group(), TIMER::index()) }).unwrap();
    }
}

unsafe impl<'d, TIMER: Timer> Send for TimerDriver<'d, TIMER> {}

macro_rules! impl_timer {
    ($timer:ident: $group:expr, $index:expr) => {
        crate::impl_peripheral!($timer);

        impl Timer for $timer {
            #[inline(always)]
            fn group() -> timer_group_t {
                $group
            }

            #[inline(always)]
            fn index() -> timer_idx_t {
                $index
            }
        }
    };
}

impl_timer!(TIMER00: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);
#[cfg(not(esp32c3))]
impl_timer!(TIMER01: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_1);
impl_timer!(TIMER10: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
#[cfg(not(esp32c3))]
impl_timer!(TIMER11: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
