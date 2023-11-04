#[cfg(not(feature = "gp_timer_api"))]
use core::marker::PhantomData;

use esp_idf_sys::*;

#[cfg(not(feature = "gp_timer_api"))]
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

#[cfg(not(feature = "gp_timer_api"))]
pub trait Timer: Send {
    fn group() -> timer_group_t;
    fn index() -> timer_idx_t;
}
#[cfg(not(feature = "gp_timer_api"))]
pub struct TimerDriver<'d> {
    timer: u8,
    divider: u32,
    isr_registered: bool,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(not(feature = "gp_timer_api"))]
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

#[cfg(not(feature = "gp_timer_api"))]
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

#[cfg(not(feature = "gp_timer_api"))]
unsafe impl<'d> Send for TimerDriver<'d> {}

#[cfg(not(feature = "gp_timer_api"))]
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

#[cfg(not(feature = "gp_timer_api"))]
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
#[cfg(not(feature = "gp_timer_api"))]
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

#[cfg(not(feature = "gp_timer_api"))]
impl_timer!(TIMER00: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
#[cfg(not(feature = "gp_timer_api"))]
impl_timer!(TIMER01: timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_1);
#[cfg(not(esp32c2))]
#[cfg(not(feature = "gp_timer_api"))]
impl_timer!(TIMER10: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_0);
#[cfg(any(esp32, esp32s2, esp32s3))]
#[cfg(not(feature = "gp_timer_api"))]
impl_timer!(TIMER11: timer_group_t_TIMER_GROUP_1, timer_idx_t_TIMER_1);

// implements a wrapper for the general purpose timer api used in esp-idf 5
#[cfg(not(esp_idf_version_major = "4"))]
#[cfg(feature = "gp_timer_api")]
pub mod gp_timer {

    use super::*;

    pub enum Direction {
        Up,
        Down,
    }

    pub struct Config {
        pub xtal: bool,           // GPTimer clock source
        pub direction: Direction, // Count direction
        pub resolution: u32, // Counter resolution (working frequency) in Hz, hence, the step size of each count tick equals to (1 / resolution_hz) seconds
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn xtal(mut self, xtal: bool) -> Self {
            self.xtal = xtal;
            self
        }

        #[must_use]
        pub fn direction(mut self, direction: Direction) -> Self {
            self.direction = direction;
            self
        }

        #[must_use]
        pub fn resolution(mut self, resolution: u32) -> Self {
            self.resolution = resolution;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                xtal: false,
                direction: Direction::Up,
                resolution: 1_000_000,
            }
        }
    }
    pub trait Timer {
        fn get_handle(&self) -> gptimer_handle_t;
    }

    // This functions are allowed to run within ISR context
    pub trait ISRMethods: Timer {
        // This function will transit the timer state from “enable” to “run”.
        fn start(&self) -> Result<(), EspError> {
            esp!(unsafe { gptimer_start(self.get_handle()) })?;

            Ok(())
        }

        // This function will transit the timer state from “run” to “enable”.
        fn stop(&self) -> Result<(), EspError> {
            esp!(unsafe { gptimer_stop(self.get_handle()) })?;

            Ok(())
        }

        // When updating the raw count of an active timer, the timer will immediately start counting from the new value.
        fn set_counter(&mut self, value: u64) -> Result<(), EspError> {
            esp!(unsafe { gptimer_set_raw_count(self.get_handle(), value) })?;

            Ok(())
        }

        // This function will trigger a software capture event and then return the captured count value.
        fn counter(&self) -> Result<u64, EspError> {
            let mut value = 0_u64;
            esp!(unsafe { gptimer_get_raw_count(self.get_handle(), &mut value) })?;

            Ok(value)
        }

        // Set alarm event actions for GPTimer. The alarm event will be triggered when the counter value reaches the alarm value.
        // Additionally setting a counter reload value with auto reload enabled will cause the counter to reset to the reload value when the alarm event is triggered.
        fn set_alarm_action(&mut self, alarm_config: AlarmConfig) -> Result<(), EspError> {
            let alarm_config: gptimer_alarm_config_t = alarm_config.into();
            esp!(unsafe { gptimer_set_alarm_action(self.get_handle(), &alarm_config) })?;

            Ok(())
        }
    }

    pub trait NonISRSaveMethods: Timer {
        // This function will transit the timer state from “init” to “enable”.
        // or  from “enable” to “init” if enabel=false.
        fn enable(&self, enable: bool) -> Result<(), EspError> {
            if enable {
                esp!(unsafe { gptimer_enable(self.get_handle()) })?;
            } else {
                esp!(unsafe { gptimer_disable(self.get_handle()) })?;
            }

            Ok(())
        }

        // A timer must be in the “init” state before it can be deleted.
        fn delete(&mut self) -> Result<(), EspError> {
            esp!(unsafe { gptimer_del_timer(self.get_handle()) })?;

            Ok(())
        }

        async fn delay(&mut self, _counter: u64) -> Result<(), EspError> {
            self.enable(false)?;
            //self.enable_alarm(false)?;
            //self.set_counter(0)?;
            //self.set_alarm(counter)?;

            self.reset_wait();

            //self.enable_interrupt()?;
            //self.enable_alarm(true)?;
            self.enable(true)?;

            self.wait().await
        }

        fn reset_wait(&mut self) {
            let notif = &PIN_NOTIF[self.get_index().unwrap() as usize];
            notif.reset();
        }

        async fn wait(&mut self) -> Result<(), EspError> {
            let notif = &PIN_NOTIF[self.get_index().unwrap() as usize];

            notif.wait().await;

            Ok(())
        }

        fn get_index(&self) -> Option<u8>;
    }

    pub struct TimerDriverInISR(gptimer_handle_t);

    impl ISRMethods for TimerDriverInISR {}
    impl TimerDriverInISR {
        pub fn from_raw_handle(handle: gptimer_handle_t) -> Self {
            Self(handle)
        }
    }
    impl Timer for TimerDriverInISR {
        fn get_handle(&self) -> gptimer_handle_t {
            self.0
        }
    }

    pub struct TimerDriver {
        handle: gptimer_handle_t,
        isr_handle_idx: Option<u8>,
    }
    impl Timer for TimerDriver {
        fn get_handle(&self) -> gptimer_handle_t {
            self.handle
        }
    }
    impl ISRMethods for TimerDriver {}
    impl NonISRSaveMethods for TimerDriver {
        fn get_index(&self) -> Option<u8> {
            self.isr_handle_idx
        }
    }

    impl TimerDriver {
        pub fn new(config: &Config) -> Result<Self, EspError> {
            let mut gptimer_ptr: *mut gptimer_t = core::ptr::null_mut();
            let ptr: *mut *mut gptimer_t = &mut gptimer_ptr;

            let conf = gptimer_config_t {
                clk_src: if config.xtal {
                    soc_periph_gptimer_clk_src_t_GPTIMER_CLK_SRC_XTAL
                } else {
                    soc_periph_gptimer_clk_src_t_GPTIMER_CLK_SRC_APB
                },
                direction: match config.direction {
                    Direction::Up => gptimer_count_direction_t_GPTIMER_COUNT_UP,
                    Direction::Down => gptimer_count_direction_t_GPTIMER_COUNT_DOWN,
                },
                resolution_hz: config.resolution,
                // Flags in this context currently allow for setting the interrupt to be shared
                //flags: timer_bindgen,
                ..Default::default()
            };

            esp!(unsafe { gptimer_new_timer(&conf, ptr.cast()) })?;

            Ok(Self {
                handle: if !ptr.is_null() {
                    unsafe { *ptr }
                } else {
                    panic!("gptimer gave us a null pointer")
                },
                isr_handle_idx: None,
            })
        }

        pub unsafe fn subscribe<'d>(
            &mut self,
            callback: impl FnMut(&TimerDriverInISR, &AlarmEvent) + Send + 'd,
        ) -> Result<(), EspError> {
            // indexing for ISR_HANDLERS and PIN_NOTIF static arrays
            let guard = GP_ISR_HANDLER_GUARD.enter();

            let empty_index =
                Self::find_empty_index().unwrap_or_else(|| panic!("no empty index found"));
            self.isr_handle_idx = Some(empty_index);

            let isr_handle: Box<dyn FnMut(&TimerDriverInISR, &AlarmEvent) + Send> =
                Box::new(callback);
            GP_ISR_HANDLERS[empty_index as usize] =
                Some(unsafe { core::mem::transmute(isr_handle) });

            drop(guard);

            let ptr_index = empty_index as *mut core::ffi::c_void;

            let event_callback = gptimer_event_callbacks_t {
                on_alarm: Some(Self::callback),
            };
            esp!(unsafe {
                gptimer_register_event_callbacks(self.handle, &event_callback, ptr_index)
            })?;

            Ok(())
        }

        #[cfg(feature = "alloc")]
        pub fn unsubscribe(&mut self) -> Result<(), EspError> {
            unsafe {
                GP_ISR_HANDLERS[self.get_index().unwrap() as usize] = None;
            }

            Ok(())
        }

        unsafe extern "C" fn callback(
            handle: *mut gptimer_t,
            event: *const gptimer_alarm_event_data_t,
            user_data: *mut core::ffi::c_void,
        ) -> bool {
            use core::num::NonZeroU32;

            let driver = TimerDriverInISR::from_raw_handle(handle);
            let data: &AlarmEvent = unsafe { &*event }.into();

            let index = user_data as usize;

            crate::interrupt::with_isr_yield_signal(move || {
                #[cfg(feature = "alloc")]
                {
                    if let Some(handler) = GP_ISR_HANDLERS[index].as_mut() {
                        handler(&driver, data);
                    }
                }
                PIN_NOTIF[index].notify(NonZeroU32::new(1).unwrap());
            })
        }

        // find the first empty index in GP_ISR_HANDLER
        // Safety: not thread save standalone !!
        // To make it thread save this function should only be called in an GP_ISR_HANDLER_GUARD.enter() block
        // and the guard should live till the the returnd index slot in the GP_ISR_HANDLER is filled
        fn find_empty_index() -> Option<u8> {
            for (i, handler) in unsafe { GP_ISR_HANDLERS.iter().enumerate() } {
                if handler.is_none() {
                    return Some(i as u8);
                }
            }
            None
        }
    }

    impl Drop for TimerDriver {
        fn drop(&mut self) {
            // This function will transit the timer state from “enable” to “init”.
            // This function will disable the interrupt service if it’s installed.
            // This function will release the PM lock if it’s acquired in the gptimer_enable.
            esp!(unsafe { gptimer_disable(self.handle) }).unwrap();

            #[cfg(feature = "alloc")]
            unsafe {
                if let Some(index) = self.isr_handle_idx {
                    GP_ISR_HANDLERS[index as usize] = None;
                    PIN_NOTIF[index as usize].reset();
                }
            }

            // Delete the GPTimer handle.
            // A timer must be in the “init” state before it can be deleted.
            esp!(unsafe { gptimer_del_timer(self.handle) }).unwrap();
        }
    }

    unsafe impl Send for TimerDriver {}

    pub struct AlarmConfig {
        pub alarm_count: u64,
        pub reload_count: u64,
        pub auto_reload: bool,
    }

    impl AlarmConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn alarm_count(mut self, alarm_count: u64) -> Self {
            self.alarm_count = alarm_count;
            self
        }

        #[must_use]
        pub fn reload_count(mut self, reload_count: u64) -> Self {
            self.reload_count = reload_count;
            self
        }

        #[must_use]
        pub fn auto_reload(mut self, auto_reload: bool) -> Self {
            self.auto_reload = auto_reload;
            self
        }
    }

    // impl default for Alarm Config
    impl Default for AlarmConfig {
        fn default() -> Self {
            Self {
                alarm_count: 1000,
                reload_count: 0,
                auto_reload: false,
            }
        }
    }

    impl From<AlarmConfig> for gptimer_alarm_config_t {
        fn from(item: AlarmConfig) -> Self {
            let mut flag = gptimer_alarm_config_t__bindgen_ty_1::default();
            Self {
                alarm_count: item.alarm_count,
                reload_count: item.reload_count,
                flags: if item.auto_reload {
                    flag.set_auto_reload_on_alarm(1);
                    flag
                } else {
                    flag.set_auto_reload_on_alarm(0);
                    flag
                },
            }
        }
    }

    // maybe just reexport as new named type?

    // Rust - C conversions
    #[repr(C)]
    pub struct AlarmEvent {
        pub count_value: u64,
        pub alarm_value: u64,
    }

    impl From<&gptimer_alarm_event_data_t> for &AlarmEvent {
        fn from(item: &gptimer_alarm_event_data_t) -> Self {
            unsafe { &*(item as *const gptimer_alarm_event_data_t as *const AlarmEvent) }
        }
    }

    #[cfg(feature = "alloc")]
    static GP_ISR_HANDLER_GUARD: crate::task::CriticalSection = crate::task::CriticalSection::new();

    #[allow(clippy::type_complexity)]
    #[cfg(not(any(esp32, esp32s2, esp32s3)))]
    #[cfg(feature = "alloc")]
    static mut GP_ISR_HANDLERS: [Option<
        Box<dyn FnMut(&TimerDriverInISR, &AlarmEvent) + Send + 'static>,
    >; 2] = [None, None];

    #[allow(clippy::type_complexity)]
    #[cfg(any(esp32, esp32s2, esp32s3))]
    #[cfg(feature = "alloc")]
    static mut GP_ISR_HANDLERS: [Option<
        Box<dyn FnMut(&TimerDriverInISR, &AlarmEvent) + Send + 'static>,
    >; 4] = [None, None, None, None];
}
