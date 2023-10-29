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
    divider: u32,
    isr_registered: bool,
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

        Ok(Self {
            timer: ((TIMER::group() as u8) << 4) | (TIMER::index() as u8),
            divider: config.divider,
            isr_registered: false,
            _p: PhantomData,
        })
    }

    pub fn tick_hz(&self) -> u64 {
        let hz;

        #[cfg(esp_idf_version_major = "4")]
        {
            hz = TIMER_BASE_CLK / self.divider;
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            hz = APB_CLK_FREQ / self.divider;
        }

        hz as _
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

        if !self.isr_registered {
            // Driver will complain if we try to register when ISR CB is already registered
            esp!(unsafe {
                timer_isr_callback_add(
                    self.group(),
                    self.index(),
                    Some(Self::handle_isr),
                    (self.group() * timer_idx_t_TIMER_MAX + self.index()) as *mut core::ffi::c_void,
                    0,
                )
            })?;

            self.isr_registered = true;
        }

        Ok(())
    }

    pub fn disable_interrupt(&mut self) -> Result<(), EspError> {
        self.check();

        if self.isr_registered {
            // Driver will complain if we try to deregister when ISR callback is not registered
            esp!(unsafe { timer_isr_callback_remove(self.group(), self.index()) })?;

            self.isr_registered = false;
        }

        Ok(())
    }

    pub async fn delay(&mut self, counter: u64) -> Result<(), EspError> {
        self.enable(false)?;
        self.enable_alarm(false)?;
        self.set_counter(0)?;
        self.set_alarm(counter)?;

        self.reset_wait();

        self.enable_interrupt()?;
        self.enable_alarm(true)?;
        self.enable(true)?;

        self.wait().await
    }

    pub fn reset_wait(&mut self) {
        let notif = &PIN_NOTIF[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize];
        notif.reset();
    }

    pub async fn wait(&mut self) -> Result<(), EspError> {
        let notif = &PIN_NOTIF[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize];

        notif.wait().await;

        Ok(())
    }

    /// Subscribes the provided callback for ISR notifications.
    /// As a side effect, interrupts will be disabled, so to receive a notification, one has
    /// to also call `TimerDriver::enable_interrupt` after calling this method.
    ///
    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(&mut self, callback: impl FnMut() + Send + 'd) -> Result<(), EspError> {
        self.check();

        self.disable_interrupt()?;

        let callback: Box<dyn FnMut() + Send + 'd> = Box::new(callback);

        ISR_HANDLERS[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize] =
            Some(unsafe { core::mem::transmute(callback) });

        Ok(())
    }

    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) -> Result<(), EspError> {
        self.check();

        self.disable_interrupt()?;

        unsafe {
            ISR_HANDLERS[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize] = None;
        }

        Ok(())
    }

    fn check(&self) {
        if crate::interrupt::active() {
            panic!("This function cannot be called from an ISR");
        }
    }

    unsafe extern "C" fn handle_isr(index: *mut core::ffi::c_void) -> bool {
        use core::num::NonZeroU32;

        let index = index as usize;

        crate::interrupt::with_isr_yield_signal(move || {
            #[cfg(feature = "alloc")]
            {
                if let Some(handler) = ISR_HANDLERS[index].as_mut() {
                    handler();
                }
            }

            PIN_NOTIF[index].notify(NonZeroU32::new(1).unwrap());
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
        self.disable_interrupt().unwrap();

        #[cfg(feature = "alloc")]
        unsafe {
            ISR_HANDLERS[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize] = None;
        }

        PIN_NOTIF[(self.group() * timer_idx_t_TIMER_MAX + self.index()) as usize].reset();

        esp!(unsafe { timer_deinit(self.group(), self.index()) }).unwrap();
    }
}

unsafe impl<'d> Send for TimerDriver<'d> {}

#[cfg(feature = "nightly")]
impl<'d> embedded_hal_async::delay::DelayUs for TimerDriver<'d> {
    async fn delay_us(&mut self, us: u32) {
        let counter = core::cmp::max((self.tick_hz() * us as u64) / 1000000, 1);

        self.delay(counter).await.unwrap();
    }

    async fn delay_ms(&mut self, ms: u32) {
        let counter = core::cmp::max((self.tick_hz() * ms as u64) / 1000, 1);

        self.delay(counter).await.unwrap();
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
static mut ISR_HANDLERS: [Option<Box<dyn FnMut() + Send + 'static>>; 2] = [None, None];

#[allow(clippy::type_complexity)]
#[cfg(not(any(esp32, esp32s2, esp32s3)))]
pub(crate) static PIN_NOTIF: [crate::interrupt::asynch::HalIsrNotification; 2] = [
    crate::interrupt::asynch::HalIsrNotification::new(),
    crate::interrupt::asynch::HalIsrNotification::new(),
];

#[allow(clippy::type_complexity)]
#[cfg(any(esp32, esp32s2, esp32s3))]
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [Option<Box<dyn FnMut() + Send + 'static>>; 4] = [None, None, None, None];

#[allow(clippy::type_complexity)]
#[cfg(any(esp32, esp32s2, esp32s3))]
pub(crate) static PIN_NOTIF: [crate::interrupt::asynch::HalIsrNotification; 4] = [
    crate::interrupt::asynch::HalIsrNotification::new(),
    crate::interrupt::asynch::HalIsrNotification::new(),
    crate::interrupt::asynch::HalIsrNotification::new(),
    crate::interrupt::asynch::HalIsrNotification::new(),
];

impl_timer!(TIMER00: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_timer!(TIMER01: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_1);
#[cfg(not(esp32c2))]
impl_timer!(TIMER10: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_timer!(TIMER11: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_1);
