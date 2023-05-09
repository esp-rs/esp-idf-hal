use core::marker::PhantomData;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::boxed::Box;

pub type TimerConfig = config::Config;

/// Timer configuration
pub mod config {
    #[derive(Copy, Clone)]
    pub struct Config {
        pub divider: u32,
        #[cfg(any(esp32s2, esp32s3, esp32c3))]
        pub xtal: bool,

        /// Enable or disable counter reload function when alarm event occurs.
        ///
        /// Enabling this makes the hardware automatically reset the counter
        /// to the value set by [`TimerDriver::set_counter`](super::TimerDriver::set_counter) when the alarm is fired.
        /// This allows creating timers that automatically fire at a given interval
        /// without the software having to do anything after the timer setup.
        pub auto_reload: bool,
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

        #[must_use]
        pub fn auto_reload(mut self, auto_reload: bool) -> Self {
            self.auto_reload = auto_reload;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                divider: 80,
                #[cfg(any(esp32s2, esp32s3, esp32c3))]
                xtal: false,
                auto_reload: false,
            }
        }
    }
}

pub trait Timer: Send {
    fn group() -> timer_group_t;
    fn index() -> timer_idx_t;
}

pub struct TimerDriver<'d> {
    timer: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TimerDriver<'d> {
    pub fn new<TIMER: Timer>(
        _timer: impl Peripheral<P = TIMER> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        esp!(unsafe {
            timer_init(
                TIMER::group(),
                TIMER::index(),
                &timer_config_t {
                    alarm_en: timer_alarm_t_TIMER_ALARM_DIS,
                    counter_en: timer_start_t_TIMER_PAUSE,
                    counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
                    auto_reload: if config.auto_reload {
                        timer_autoreload_t_TIMER_AUTORELOAD_EN
                    } else {
                        timer_autoreload_t_TIMER_AUTORELOAD_DIS
                    },
                    intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
                    divider: config.divider,
                    #[cfg(all(any(esp32s2, esp32s3, esp32c3), esp_idf_version_major = "4"))]
                    clk_src: if config.xtal {
                        timer_src_clk_t_TIMER_SRC_CLK_XTAL
                    } else {
                        timer_src_clk_t_TIMER_SRC_CLK_APB
                    },
                    #[cfg(not(esp_idf_version_major = "4"))]
                    clk_src: 0,
                },
            )
        })?;

        Ok(TimerDriver {
            timer: ((TIMER::group() as u8) << 4) | (TIMER::index() as u8),
            _p: PhantomData,
        })
    }

    pub fn enable(&mut self, enable: bool) -> Result<(), EspError> {
        self.check();

        if enable {
            esp!(unsafe { timer_start(self.group(), self.index()) })?;
        } else {
            esp!(unsafe { timer_pause(self.group(), self.index()) })?;
        }

        Ok(())
    }

    pub fn counter(&self) -> Result<u64, EspError> {
        let value = if crate::interrupt::active() {
            unsafe { timer_group_get_counter_value_in_isr(self.group(), self.index()) }
        } else {
            let mut value = 0_u64;

            esp!(unsafe { timer_get_counter_value(self.group(), self.index(), &mut value) })?;

            value
        };

        Ok(value)
    }

    pub fn set_counter(&mut self, value: u64) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_set_counter_value(self.group(), self.index(), value) })?;

        Ok(())
    }

    pub fn enable_alarm(&mut self, enable: bool) -> Result<(), EspError> {
        if crate::interrupt::active() {
            if enable {
                unsafe {
                    timer_group_enable_alarm_in_isr(self.group(), self.index());
                }
            } else {
                panic!("Disabling alarm from an ISR is not supported");
            }
        } else {
            esp!(unsafe {
                timer_set_alarm(
                    self.group(),
                    self.index(),
                    if enable {
                        timer_alarm_t_TIMER_ALARM_EN
                    } else {
                        timer_alarm_t_TIMER_ALARM_DIS
                    },
                )
            })?;
        }

        Ok(())
    }

    pub fn alarm(&self) -> Result<u64, EspError> {
        self.check();

        let mut value = 0_u64;

        esp!(unsafe { timer_get_alarm_value(self.group(), self.index(), &mut value) })?;

        Ok(value)
    }

    pub fn set_alarm(&mut self, value: u64) -> Result<(), EspError> {
        if crate::interrupt::active() {
            unsafe {
                timer_group_set_alarm_value_in_isr(self.group(), self.index(), value);
            }
        } else {
            esp!(unsafe { timer_set_alarm_value(self.group(), self.index(), value) })?;
        }

        Ok(())
    }

    pub fn enable_interrupt(&mut self) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_enable_intr(self.group(), self.index()) })?;

        Ok(())
    }

    pub fn disable_interrupt(&mut self) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_disable_intr(self.group(), self.index()) })?;

        Ok(())
    }

    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(&mut self, callback: impl FnMut() + 'static) -> Result<(), EspError> {
        self.check();

        self.unsubscribe()?;

        let callback: Box<dyn FnMut() + 'static> = Box::new(callback);

        ISR_HANDLERS[(self.group() * timer_group_t_TIMER_GROUP_MAX + self.index()) as usize] =
            Some(Box::new(callback));

        esp!(timer_isr_callback_add(
            self.group(),
            self.index(),
            Some(Self::handle_isr),
            UnsafeCallback::from(
                ISR_HANDLERS
                    [(self.group() * timer_group_t_TIMER_GROUP_MAX + self.index()) as usize]
                    .as_mut()
                    .unwrap(),
            )
            .as_ptr(),
            0
        ))?;

        self.enable_interrupt()?;

        Ok(())
    }

    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) -> Result<(), EspError> {
        self.check();

        unsafe {
            let subscribed = ISR_HANDLERS
                [(self.group() * timer_group_t_TIMER_GROUP_MAX + self.index()) as usize]
                .is_some();

            if subscribed {
                esp!(timer_disable_intr(self.group(), self.index()))?;
                esp!(timer_isr_callback_remove(self.group(), self.index()))?;

                ISR_HANDLERS
                    [(self.group() * timer_group_t_TIMER_GROUP_MAX + self.index()) as usize] = None;
            }
        }

        Ok(())
    }

    fn check(&self) {
        if crate::interrupt::active() {
            panic!("This function cannot be called from an ISR");
        }
    }

    #[cfg(feature = "alloc")]
    unsafe extern "C" fn handle_isr(unsafe_callback: *mut core::ffi::c_void) -> bool {
        crate::interrupt::with_isr_yield_signal(move || {
            UnsafeCallback::from_ptr(unsafe_callback).call();
        })
    }

    pub fn group(&self) -> timer_group_t {
        (self.timer >> 4) as _
    }

    pub fn index(&self) -> timer_idx_t {
        (self.timer & 0xf) as _
    }
}

impl<'d> Drop for TimerDriver<'d> {
    fn drop(&mut self) {
        #[cfg(feature = "alloc")]
        {
            self.unsubscribe().unwrap();
        }

        esp!(unsafe { timer_deinit(self.group(), self.index()) }).unwrap();
    }
}

unsafe impl<'d> Send for TimerDriver<'d> {}

#[cfg(feature = "alloc")]
struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

#[cfg(feature = "alloc")]
impl UnsafeCallback {
    #[allow(clippy::type_complexity)]
    pub fn from(boxed: &mut Box<Box<dyn FnMut() + 'static>>) -> Self {
        Self(boxed.as_mut())
    }

    pub unsafe fn from_ptr(ptr: *mut core::ffi::c_void) -> Self {
        Self(ptr.cast())
    }

    pub fn as_ptr(&self) -> *mut core::ffi::c_void {
        self.0.cast()
    }

    pub unsafe fn call(&mut self) {
        let reference = self.0.as_mut().unwrap();

        (reference)();
    }
}

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

#[allow(clippy::type_complexity)]
#[cfg(not(any(esp32, esp32s2, esp32s3)))]
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [Option<Box<Box<dyn FnMut()>>>; 2] = [None, None];

#[allow(clippy::type_complexity)]
#[cfg(any(esp32, esp32s2, esp32s3))]
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [Option<Box<Box<dyn FnMut()>>>; 4] = [None, None, None, None];

impl_timer!(TIMER00: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_timer!(TIMER01: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_1);
#[cfg(not(esp32c2))]
impl_timer!(TIMER10: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_timer!(TIMER11: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
