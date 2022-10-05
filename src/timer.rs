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
        self.check();

        if enable {
            esp!(unsafe { timer_start(TIMER::group(), TIMER::index()) })?;
        } else {
            esp!(unsafe { timer_pause(TIMER::group(), TIMER::index()) })?;
        }

        Ok(())
    }

    pub fn counter(&self) -> Result<u64, EspError> {
        let value = if crate::interrupt::active() {
            unsafe { timer_group_get_counter_value_in_isr(TIMER::group(), TIMER::index()) }
        } else {
            let mut value = 0_u64;

            esp!(unsafe {
                timer_get_counter_value(TIMER::group(), TIMER::index(), &mut value as *mut _)
            })?;

            value
        };

        Ok(value)
    }

    pub fn set_counter(&mut self, value: u64) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_set_counter_value(TIMER::group(), TIMER::index(), value) })?;

        Ok(())
    }

    pub fn enable_alarm(&mut self, enable: bool) -> Result<(), EspError> {
        if crate::interrupt::active() {
            if enable {
                unsafe {
                    timer_group_enable_alarm_in_isr(TIMER::group(), TIMER::index());
                }
            } else {
                panic!("Disabling alarm from an ISR is not supported");
            }
        } else {
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
        }

        Ok(())
    }

    pub fn alarm(&self) -> Result<u64, EspError> {
        self.check();

        let mut value = 0_u64;

        esp!(unsafe { timer_get_alarm_value(TIMER::group(), TIMER::index(), &mut value) })?;

        Ok(value)
    }

    pub fn set_alarm(&mut self, value: u64) -> Result<(), EspError> {
        if crate::interrupt::active() {
            unsafe {
                timer_group_set_alarm_value_in_isr(TIMER::group(), TIMER::index(), value);
            }
        } else {
            esp!(unsafe { timer_set_alarm_value(TIMER::group(), TIMER::index(), value) })?;
        }

        Ok(())
    }

    pub fn enable_interrupt(&mut self) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_enable_intr(TIMER::group(), TIMER::index()) })?;

        Ok(())
    }

    pub fn disable_interrupt(&mut self) -> Result<(), EspError> {
        self.check();

        esp!(unsafe { timer_disable_intr(TIMER::group(), TIMER::index()) })?;

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
        self.check();

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

    fn check(&self) {
        if crate::interrupt::active() {
            panic!("This function cannot be called from an ISR");
        }
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

#[cfg(feature = "embassy-time")]
pub mod embassy_time {
    use core::cell::RefCell;
    use core::cmp::Ordering;
    use core::task::Waker;

    use embassy_sync::blocking_mutex::Mutex;

    use embassy_time::queue::TimerQueue;
    use embassy_time::Instant;

    use heapless::sorted_linked_list::{LinkedIndexU8, Min, SortedLinkedList};

    use crate::interrupt::embassy_sync::CriticalSectionRawMutex;

    #[derive(Debug)]
    struct Timer {
        at: Instant,
        waker: Waker,
    }

    impl PartialEq for Timer {
        fn eq(&self, other: &Self) -> bool {
            self.at == other.at
        }
    }

    impl Eq for Timer {}

    impl PartialOrd for Timer {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            self.at.partial_cmp(&other.at)
        }
    }

    impl Ord for Timer {
        fn cmp(&self, other: &Self) -> Ordering {
            self.at.cmp(&other.at)
        }
    }

    pub struct AlarmContext {
        pub callback: fn(*mut ()),
        pub ctx: *mut (),
    }

    impl AlarmContext {
        const fn new() -> Self {
            Self {
                callback: Self::noop,
                ctx: core::ptr::null_mut(),
            }
        }

        fn set(&mut self, callback: fn(*mut ()), ctx: *mut ()) {
            self.callback = callback;
            self.ctx = ctx;
        }

        fn noop(_ctx: *mut ()) {}
    }

    unsafe impl Send for AlarmContext {}

    pub trait Alarm {
        fn new(context: &AlarmContext) -> Self;
        fn schedule(&mut self, timestamp: u64);
    }

    struct InnerQueue<A> {
        queue: SortedLinkedList<Timer, LinkedIndexU8, Min, 128>,
        alarm: Option<A>,
        alarm_context: AlarmContext,
        alarm_at: Instant,
    }

    impl<A: Alarm> InnerQueue<A> {
        const fn new() -> Self {
            Self {
                queue: SortedLinkedList::new_u8(),
                alarm: None,
                alarm_context: AlarmContext::new(),
                alarm_at: Instant::MAX,
            }
        }

        fn schedule_wake(&mut self, at: Instant, waker: &Waker) {
            self.initialize();

            self.queue
                .find_mut(|timer| timer.waker.will_wake(waker))
                .map(|mut timer| {
                    timer.at = at;
                    timer.finish();
                })
                .unwrap_or_else(|| {
                    let mut timer = Timer {
                        waker: waker.clone(),
                        at,
                    };

                    loop {
                        match self.queue.push(timer) {
                            Ok(()) => break,
                            Err(e) => timer = e,
                        }

                        self.queue.pop().unwrap().waker.wake();
                    }
                });

            // Don't wait for the alarm callback to trigger and directly
            // dispatch all timers that are already due
            //
            // Then update the alarm if necessary
            self.dispatch();
        }

        fn dispatch(&mut self) {
            let now = Instant::now();

            while self.queue.peek().filter(|timer| timer.at <= now).is_some() {
                self.queue.pop().unwrap().waker.wake();
            }

            self.update_alarm();
        }

        fn update_alarm(&mut self) {
            if let Some(timer) = self.queue.peek() {
                let new_at = timer.at;

                if self.alarm_at != new_at {
                    self.alarm_at = new_at;
                    self.alarm.as_mut().unwrap().schedule(new_at.as_ticks());
                }
            } else {
                self.alarm_at = Instant::MAX;
                self.alarm
                    .as_mut()
                    .unwrap()
                    .schedule(Instant::MAX.as_ticks());
            }
        }

        fn handle_alarm(&mut self) {
            self.alarm_at = Instant::MAX;

            self.dispatch();
        }

        fn initialize(&mut self) {
            if self.alarm.is_none() {
                self.alarm = Some(A::new(&self.alarm_context));
            }
        }
    }

    pub struct Queue<A: Alarm> {
        inner: Mutex<CriticalSectionRawMutex, RefCell<InnerQueue<A>>>,
    }

    impl<A: Alarm> Queue<A> {
        pub const fn new() -> Self {
            Self {
                inner: Mutex::new(RefCell::new(InnerQueue::new())),
            }
        }

        fn schedule_wake(&'static self, at: Instant, waker: &Waker) {
            self.inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                inner
                    .alarm_context
                    .set(Self::handle_alarm_callback, self as *const _ as _);
                inner.schedule_wake(at, waker);
            });
        }

        fn handle_alarm(&self) {
            self.inner.lock(|inner| inner.borrow_mut().handle_alarm());
        }

        fn handle_alarm_callback(ctx: *mut ()) {
            unsafe { (ctx as *const Self).as_ref().unwrap() }.handle_alarm();
        }
    }

    impl<A: Alarm> TimerQueue for Queue<A> {
        fn schedule_wake(&'static self, at: Instant, waker: &Waker) {
            Queue::schedule_wake(self, at, waker);
        }
    }

    #[cfg(any(
        feature = "embassy-time-isr-queue-timer00",
        feature = "embassy-time-isr-queue-timer01",
        feature = "embassy-time-isr-queue-timer10",
        feature = "embassy-time-isr-queue-timer11",
    ))]
    pub mod queue {
        use esp_idf_sys::*;

        use crate::timer::Timer;

        #[cfg(all(
            esp32c3,
            any(
                feature = "embassy-time-isr-queue-timer01",
                feature = "embassy-time-isr-queue-timer11"
            )
        ))]
        compile_error!("Features `embassy-time-isr-queue-timer01` and `embassy-time-isr-queue-timer11` are not available for `esp32c3`");

        #[cfg(feature = "embassy-time-isr-queue-timer00")]
        type EmbassyTimer = crate::timer::TIMER00;
        #[cfg(feature = "embassy-time-isr-queue-timer01")]
        type EmbassyTimer = crate::timer::TIMER01;
        #[cfg(feature = "embassy-time-isr-queue-timer10")]
        type EmbassyTimer = crate::timer::TIMER10;
        #[cfg(feature = "embassy-time-isr-queue-timer11")]
        type EmbassyTimer = crate::timer::TIMER11;

        struct AlarmImpl;

        impl AlarmImpl {
            unsafe extern "C" fn handle_isr(alarm_context: *mut c_types::c_void) -> bool {
                crate::interrupt::with_isr_yield_signal(move || {
                    let alarm_context = (alarm_context as *const super::AlarmContext)
                        .as_ref()
                        .unwrap();

                    (alarm_context.callback)(alarm_context.ctx);
                })
            }
        }

        impl super::Alarm for AlarmImpl {
            fn new(context: &super::AlarmContext) -> Self {
                unsafe {
                    esp!(timer_init(
                        EmbassyTimer::group(),
                        EmbassyTimer::index(),
                        &timer_config_t {
                            alarm_en: timer_alarm_t_TIMER_ALARM_DIS,
                            counter_en: timer_start_t_TIMER_START,
                            counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
                            auto_reload: timer_autoreload_t_TIMER_AUTORELOAD_DIS,
                            intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
                            divider: rtc_clk_apb_freq_get() / 1_000_000,
                            #[cfg(all(
                                any(esp32s2, esp32s3, esp32c3),
                                esp_idf_version_major = "4"
                            ))]
                            clk_src: timer_src_clk_t_TIMER_SRC_CLK_APB,
                            #[cfg(not(esp_idf_version_major = "4"))]
                            clk_src: 0,
                        },
                    ))
                    .unwrap();

                    esp!(timer_isr_callback_add(
                        EmbassyTimer::group(),
                        EmbassyTimer::index(),
                        Some(Self::handle_isr),
                        context as *const _ as *mut _,
                        0,
                    ))
                    .unwrap();

                    esp!(timer_enable_intr(
                        EmbassyTimer::group(),
                        EmbassyTimer::index()
                    ))
                    .unwrap();
                }

                Self
            }

            fn schedule(&mut self, timestamp: u64) {
                if crate::interrupt::active() {
                    if timestamp < u64::MAX {
                        unsafe {
                            timer_group_set_alarm_value_in_isr(
                                EmbassyTimer::group(),
                                EmbassyTimer::index(),
                                timestamp,
                            );
                        }
                        unsafe {
                            timer_group_enable_alarm_in_isr(
                                EmbassyTimer::group(),
                                EmbassyTimer::index(),
                            );
                        }
                    }
                } else {
                    esp!(unsafe {
                        timer_set_alarm(
                            EmbassyTimer::group(),
                            EmbassyTimer::index(),
                            timer_alarm_t_TIMER_ALARM_DIS,
                        )
                    })
                    .unwrap();

                    if timestamp < u64::MAX {
                        esp!(unsafe {
                            timer_set_alarm_value(
                                EmbassyTimer::group(),
                                EmbassyTimer::index(),
                                timestamp,
                            )
                        })
                        .unwrap();

                        esp!(unsafe {
                            timer_set_alarm(
                                EmbassyTimer::group(),
                                EmbassyTimer::index(),
                                timer_alarm_t_TIMER_ALARM_EN,
                            )
                        })
                        .unwrap();
                    }
                }
            }
        }

        pub fn link() -> i32 {
            42
        }

        embassy_time::timer_queue_impl!(static QUEUE: super::Queue<AlarmImpl> = super::Queue::new());
    }
}
