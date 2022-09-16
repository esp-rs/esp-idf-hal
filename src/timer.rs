use esp_idf_sys::*;

use crate::peripheral::{Peripheral, PeripheralRef};

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
                    #[cfg(any(esp32s2, esp32s3, esp32c3))]
                    clk_src: if config.xtal {
                        timer_src_clk_t_TIMER_SRC_CLK_XTAL
                    } else {
                        timer_src_clk_t_TIMER_SRC_CLK_APB
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

    pub fn unsubscribe(&mut self) -> Result<(), EspError> {
        unsafe {
            unsubscribe_timer(TIMER::group(), TIMER::index())?;
        }

        Ok(())
    }

    unsafe extern "C" fn handle_isr(unsafe_callback: *mut c_types::c_void) -> bool {
        crate::interrupt::with_isr_yield_signal(move || {
            UnsafeCallback::from_ptr(unsafe_callback).call();
        })
    }
}

impl<'d, TIMER: Timer> Drop for TimerDriver<'d, TIMER> {
    fn drop(&mut self) {
        esp!(unsafe { timer_deinit(TIMER::group(), TIMER::index()) }).unwrap();
    }
}

unsafe impl<'d, TIMER: Timer> Send for TimerDriver<'d, TIMER> {}

unsafe fn unsubscribe_timer(group: timer_group_t, index: timer_idx_t) -> Result<(), EspError> {
    let subscribed =
        ISR_HANDLERS[(group * timer_group_t_TIMER_GROUP_MAX + index) as usize].is_some();

    if subscribed {
        esp!(timer_disable_intr(group, index))?;
        esp!(timer_isr_callback_remove(group, index))?;

        ISR_HANDLERS[(group * timer_group_t_TIMER_GROUP_MAX + index) as usize] = None;
    }

    Ok(())
}

struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

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

#[cfg(esp32c3)]
static mut ISR_HANDLERS: [Option<Box<Box<dyn FnMut()>>>; 2] = [None, None];

#[cfg(not(esp32c3))]
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
    use core::sync::atomic::{AtomicBool, Ordering};

    use esp_idf_sys::esp_timer_get_time;

    #[cfg(any(feature = "embassy-time-timer01", feature = "embassy-time-timer11"))]
    compile_error!("Features `embassy-time-timer01` and `embassy-time-timer11`");

    #[cfg(feature = "embassy-time-timer00")]
    type TIMER = super::TIMER00;
    #[cfg(feature = "embassy-time-timer01")]
    type TIMER = super::TIMER01;
    #[cfg(feature = "embassy-time-timer10")]
    type TIMER = super::TIMER10;
    #[cfg(feature = "embassy-time-timer11")]
    type TIMER = super::TIMER11;

    use ::embassy_time::driver::{AlarmHandle, Driver};

    static TAKEN: AtomicBool = AtomicBool::new(false);

    struct EspDriver(UnsafeCell<Option<super::TimerDriver<'static, TIMER>>>);

    impl EspDriver {
        pub const fn new() -> Self {
            Self(UnsafeCell::new(None))
        }

        unsafe fn driver(&self) -> &mut super::TimerDriver<'static, TIMER> {
            self.0.get().as_mut().unwrap().as_mut().unwrap()
        }
    }

    unsafe impl Send for EspDriver {}
    unsafe impl Sync for EspDriver {}

    impl Driver for EspDriver {
        fn now(&self) -> u64 {
            unsafe { esp_timer_get_time() as _ }
        }

        unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
            if TAKEN.swap(true, Ordering::SeqCst) {
                None
            } else {
                let mut timer =
                    super::TimerDriver::new(TIMER::new(), &super::TimerConfig::new()).unwrap();

                timer.set_counter(0).unwrap();

                *self.0.get().as_mut().unwrap() = Some(timer);

                Some(AlarmHandle::new(0))
            }
        }

        fn set_alarm_callback(&self, _alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
            unsafe {
                self.driver().subscribe(move || callback(ctx)).unwrap();
            }
        }

        fn set_alarm(&self, _alarm: AlarmHandle, timestamp: u64) {
            unsafe { self.driver() }.set_alarm(timestamp).unwrap();
        }
    }

    embassy_time::time_driver_impl!(static DRIVER: EspDriver = EspDriver::new());
}
