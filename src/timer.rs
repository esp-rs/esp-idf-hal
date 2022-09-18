use esp_idf_sys::*;

use crate::peripheral::{Peripheral, PeripheralRef};

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
                divider: 80,
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
                    timer_alarm_t_TIMER_ALARM_EN
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

    pub fn enable_interrupt(&mut self) -> Result<(), EspError> {
        esp!(unsafe { timer_enable_intr(TIMER::group(), TIMER::index()) })?;

        Ok(())
    }

    pub fn disable_interrupt(&mut self) -> Result<(), EspError> {
        esp!(unsafe { timer_disable_intr(TIMER::group(), TIMER::index()) })?;

        Ok(())
    }

    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(&mut self, callback: impl FnMut() + 'static) -> Result<(), EspError> {
        self.unsubscribe()?;

        let callback: Box<dyn FnMut() + 'static> = Box::new(callback);

        ISR_HANDLERS[(TIMER::group() * timer_group_t_TIMER_GROUP_MAX + TIMER::index()) as usize] =
            Some(Box::new(callback));

        esp!(timer_isr_callback_add(
            TIMER::group(),
            TIMER::index(),
            Some(Self::handle_isr),
            UnsafeCallback::from(
                ISR_HANDLERS
                    [(TIMER::group() * timer_group_t_TIMER_GROUP_MAX + TIMER::index()) as usize]
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
        unsafe {
            let subscribed = ISR_HANDLERS
                [(TIMER::group() * timer_group_t_TIMER_GROUP_MAX + TIMER::index()) as usize]
                .is_some();

            if subscribed {
                esp!(timer_disable_intr(TIMER::group(), TIMER::index()))?;
                esp!(timer_isr_callback_remove(TIMER::group(), TIMER::index()))?;

                ISR_HANDLERS
                    [(TIMER::group() * timer_group_t_TIMER_GROUP_MAX + TIMER::index()) as usize] =
                    None;
            }
        }

        Ok(())
    }

    #[cfg(feature = "alloc")]
    unsafe extern "C" fn handle_isr(unsafe_callback: *mut c_types::c_void) -> bool {
        crate::interrupt::with_isr_yield_signal(move || {
            UnsafeCallback::from_ptr(unsafe_callback).call();
        })
    }
}

impl<'d, TIMER: Timer> Drop for TimerDriver<'d, TIMER> {
    fn drop(&mut self) {
        #[cfg(feature = "alloc")]
        {
            self.unsubscribe().unwrap();
        }

        esp!(unsafe { timer_deinit(TIMER::group(), TIMER::index()) }).unwrap();
    }
}

unsafe impl<'d, TIMER: Timer> Send for TimerDriver<'d, TIMER> {}

#[cfg(feature = "alloc")]
struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

#[cfg(feature = "alloc")]
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
#[cfg(esp32c3)]
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [Option<Box<Box<dyn FnMut()>>>; 2] = [None, None];

#[allow(clippy::type_complexity)]
#[cfg(not(esp32c3))]
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [Option<Box<Box<dyn FnMut()>>>; 4] = [None, None, None, None];

impl_timer!(TIMER00: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);
#[cfg(not(esp32c3))]
impl_timer!(TIMER01: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_1);
impl_timer!(TIMER10: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
#[cfg(not(esp32c3))]
impl_timer!(TIMER11: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);

#[cfg(any(
    feature = "embassy-time-timer00",
    feature = "embassy-time-timer01",
    feature = "embassy-time-timer10",
    feature = "embassy-time-timer11"
))]
mod embassy_time {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

    use esp_idf_sys::*;

    #[cfg(all(
        esp32c3,
        any(feature = "embassy-time-timer01", feature = "embassy-time-timer11")
    ))]
    compile_error!("Features `embassy-time-timer01` and `embassy-time-timer11` are not available for `esp32c3`");

    #[cfg(feature = "embassy-time-timer00")]
    type EmbassyTimer = super::TIMER00;
    #[cfg(feature = "embassy-time-timer01")]
    type EmbassyTimer = super::TIMER01;
    #[cfg(feature = "embassy-time-timer10")]
    type EmbassyTimer = super::TIMER10;
    #[cfg(feature = "embassy-time-timer11")]
    type EmbassyTimer = super::TIMER11;

    use ::embassy_time::driver::{AlarmHandle, Driver};

    use super::Timer;

    #[derive(Copy, Clone)]
    struct Alarm {
        timestamp: u64,
        callback: fn(*mut ()),
        ctx: *mut (),
    }

    impl Alarm {
        const fn new() -> Self {
            Self {
                timestamp: u64::MAX,
                callback: Self::noop,
                ctx: core::ptr::null_mut(),
            }
        }

        fn noop(_ctx: *mut ()) {}
    }

    struct EspDriver<const MAX_ALARMS: usize = 16> {
        alarms: UnsafeCell<[Alarm; MAX_ALARMS]>,
        next_alarm: AtomicU8,
        initialized: AtomicBool,
        cs_mutex: crate::cs::CriticalSection,
        cs_inter: crate::interrupt::CriticalSection,
    }

    impl<const MAX_ALARMS: usize> EspDriver<MAX_ALARMS> {
        pub const fn new() -> Self {
            Self {
                alarms: UnsafeCell::new([Alarm::new(); MAX_ALARMS]),
                next_alarm: AtomicU8::new(0),
                initialized: AtomicBool::new(false),
                cs_mutex: crate::cs::CriticalSection::new(),
                cs_inter: crate::interrupt::CriticalSection::new(),
            }
        }

        fn initialize(&self) {
            if !self.initialized.load(Ordering::SeqCst) {
                if crate::interrupt::active() {
                    unreachable!();
                }

                let _guard = self.cs_mutex.enter();

                if !self.initialized.load(Ordering::SeqCst) {
                    esp!(unsafe {
                        timer_init(
                            EmbassyTimer::group(),
                            EmbassyTimer::index(),
                            &timer_config_t {
                                alarm_en: timer_alarm_t_TIMER_ALARM_EN,
                                counter_en: timer_start_t_TIMER_START,
                                counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
                                auto_reload: timer_autoreload_t_TIMER_AUTORELOAD_DIS,
                                intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
                                divider: 80,
                                #[cfg(all(
                                    any(esp32s2, esp32s3, esp32c3),
                                    esp_idf_version_major = "4"
                                ))]
                                clk_src: timer_src_clk_t_TIMER_SRC_CLK_APB,
                                #[cfg(not(esp_idf_version_major = "4"))]
                                clk_src: 0,
                            },
                        )
                    })
                    .unwrap();

                    esp!(unsafe {
                        timer_isr_callback_add(
                            EmbassyTimer::group(),
                            EmbassyTimer::index(),
                            Some(Self::handle_isr),
                            self as *const _ as *mut _,
                            0,
                        )
                    })
                    .unwrap();

                    esp!(unsafe {
                        timer_enable_intr(EmbassyTimer::group(), EmbassyTimer::index())
                    })
                    .unwrap();

                    self.initialized.store(true, Ordering::SeqCst);
                }
            }
        }

        fn update_timer_alarm(&self) {
            let alarms = unsafe { &mut self.alarms.get().as_mut().unwrap() };

            let timestamp = alarms
                .iter()
                .min_by_key(|alarm| alarm.timestamp)
                .unwrap()
                .timestamp;

            esp!(unsafe {
                timer_set_alarm_value(EmbassyTimer::group(), EmbassyTimer::index(), timestamp)
            })
            .unwrap();
        }

        fn handle_alarm(&self) {
            let mut now = 0_u64;

            esp!(unsafe {
                timer_get_counter_value(
                    EmbassyTimer::group(),
                    EmbassyTimer::index(),
                    &mut now as *mut _,
                )
            })
            .unwrap();

            let alarms = unsafe { &mut self.alarms.get().as_mut().unwrap() };

            loop {
                let next_alarm = alarms
                    .iter_mut()
                    .min_by_key(|alarm| alarm.timestamp)
                    .unwrap();

                if next_alarm.timestamp < now {
                    next_alarm.timestamp = u64::MAX;
                    (next_alarm.callback)(next_alarm.ctx);
                } else {
                    break;
                }
            }

            self.update_timer_alarm();
        }

        unsafe extern "C" fn handle_isr(this: *mut c_types::c_void) -> bool {
            crate::interrupt::with_isr_yield_signal(move || {
                let this = (this as *const Self).as_ref().unwrap();

                this.handle_alarm();
            })
        }
    }

    unsafe impl<const MAX_ALARMS: usize> Send for EspDriver<MAX_ALARMS> {}
    unsafe impl<const MAX_ALARMS: usize> Sync for EspDriver<MAX_ALARMS> {}

    impl<const MAX_ALARMS: usize> Driver for EspDriver<MAX_ALARMS> {
        fn now(&self) -> u64 {
            self.initialize();

            let mut now = 0_u64;

            esp!(unsafe {
                timer_get_counter_value(
                    EmbassyTimer::group(),
                    EmbassyTimer::index(),
                    &mut now as *mut _,
                )
            })
            .unwrap();

            now
        }

        unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
            self.initialize();

            let id = self
                .next_alarm
                .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                    if x < MAX_ALARMS as u8 {
                        Some(x + 1)
                    } else {
                        None
                    }
                });

            match id {
                Ok(id) => Some(AlarmHandle::new(id)),
                Err(_) => None,
            }
        }

        fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
            self.initialize();

            let _guard = self.cs_inter.enter();

            let alarms = unsafe { &mut self.alarms.get().as_mut().unwrap() };

            alarms[alarm.id() as usize].callback = callback;
            alarms[alarm.id() as usize].ctx = ctx;
        }

        fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) {
            self.initialize();

            let _guard = self.cs_inter.enter();

            let alarms = unsafe { &mut self.alarms.get().as_mut().unwrap() };

            alarms[alarm.id() as usize].timestamp = timestamp;

            self.update_timer_alarm();
        }
    }

    embassy_time::time_driver_impl!(static DRIVER: EspDriver = EspDriver::new());
}
