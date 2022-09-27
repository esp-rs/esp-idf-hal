use core::ptr;
use core::result::Result;
use core::time::Duration;

extern crate alloc;
use alloc::boxed::Box;

use embedded_svc::sys_time::SystemTime;
use embedded_svc::timer::{self, ErrorType, OnceTimer, PeriodicTimer, Timer, TimerService};

use esp_idf_sys::*;

#[cfg(all(feature = "nightly", feature = "experimental"))]
pub use asyncify::*;

#[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
pub use isr::*;

use crate::handle::RawHandle;

struct UnsafeCallback(*mut Box<dyn FnMut()>);

impl UnsafeCallback {
    fn from(boxed: &mut Box<dyn FnMut()>) -> Self {
        Self(boxed)
    }

    unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self) {
        let reference = self.0.as_mut().unwrap();

        (reference)();
    }
}

pub struct EspTimer {
    handle: esp_timer_handle_t,
    _callback: Box<dyn FnMut()>,
}

impl EspTimer {
    pub fn is_scheduled(&self) -> Result<bool, EspError> {
        Ok(unsafe { esp_timer_is_active(self.handle) })
    }

    pub fn cancel(&self) -> Result<bool, EspError> {
        let res = unsafe { esp_timer_stop(self.handle) };

        Ok(res != ESP_OK)
    }

    pub fn after(&self, duration: Duration) -> Result<(), EspError> {
        self.cancel()?;

        esp!(unsafe { esp_timer_start_once(self.handle, duration.as_micros() as _) })?;

        Ok(())
    }

    pub fn every(&self, duration: Duration) -> Result<(), EspError> {
        self.cancel()?;

        esp!(unsafe { esp_timer_start_periodic(self.handle, duration.as_micros() as _) })?;

        Ok(())
    }

    extern "C" fn handle(arg: *mut c_types::c_void) {
        if esp_idf_hal::interrupt::active() {
            #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
            {
                let signaled = esp_idf_hal::interrupt::with_isr_yield_signal(move || unsafe {
                    UnsafeCallback::from_ptr(arg).call();
                });

                if signaled {
                    unsafe {
                        esp_idf_sys::esp_timer_isr_dispatch_need_yield();
                    }
                }
            }

            #[cfg(not(esp_idf_esp_timer_supports_isr_dispatch_method))]
            {
                unreachable!();
            }
        } else {
            unsafe {
                UnsafeCallback::from_ptr(arg).call();
            }
        }
    }
}

unsafe impl Send for EspTimer {}

impl Drop for EspTimer {
    fn drop(&mut self) {
        self.cancel().unwrap();

        while unsafe { esp_timer_delete(self.handle) } != ESP_OK {
            // Timer is still running, busy-loop
        }
    }
}

impl RawHandle for EspTimer {
    type Handle = esp_timer_handle_t;

    fn handle(&self) -> Self::Handle {
        self.handle
    }
}

impl ErrorType for EspTimer {
    type Error = EspError;
}

impl timer::Timer for EspTimer {
    fn is_scheduled(&self) -> Result<bool, Self::Error> {
        EspTimer::is_scheduled(self)
    }

    fn cancel(&mut self) -> Result<bool, Self::Error> {
        EspTimer::cancel(self)
    }
}

impl OnceTimer for EspTimer {
    fn after(&mut self, duration: Duration) -> Result<(), Self::Error> {
        EspTimer::after(self, duration)
    }
}

impl PeriodicTimer for EspTimer {
    fn every(&mut self, duration: Duration) -> Result<(), Self::Error> {
        EspTimer::every(self, duration)
    }
}

pub trait EspTimerServiceType {
    fn is_isr() -> bool;
}

#[derive(Clone, Debug)]
pub struct Task;

impl EspTimerServiceType for Task {
    fn is_isr() -> bool {
        false
    }
}

pub struct EspTimerService<T>(T)
where
    T: EspTimerServiceType;

impl<T> EspTimerService<T>
where
    T: EspTimerServiceType,
{
    pub fn now(&self) -> Duration {
        Duration::from_micros(unsafe { esp_timer_get_time() as _ })
    }

    pub fn timer(&self, callback: impl FnMut() + Send + 'static) -> Result<EspTimer, EspError> {
        let mut handle: esp_timer_handle_t = ptr::null_mut();

        let boxed_callback: Box<dyn FnMut()> = Box::new(callback);

        let mut callback = Box::new(boxed_callback);
        let unsafe_callback = UnsafeCallback::from(&mut callback);

        #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
        let dispatch_method = if T::is_isr() {
            esp_timer_dispatch_t_ESP_TIMER_ISR
        } else {
            esp_timer_dispatch_t_ESP_TIMER_TASK
        };

        #[cfg(not(esp_idf_esp_timer_supports_isr_dispatch_method))]
        let dispatch_method = esp_timer_dispatch_t_ESP_TIMER_TASK;

        esp!(unsafe {
            esp_timer_create(
                &esp_timer_create_args_t {
                    callback: Some(EspTimer::handle),
                    name: b"rust\0" as *const _ as *const _, // TODO
                    arg: unsafe_callback.as_ptr(),
                    dispatch_method,
                    skip_unhandled_events: false, // TODO
                },
                &mut handle as *mut _,
            )
        })?;

        Ok(EspTimer {
            handle,
            _callback: callback,
        })
    }
}

pub type EspTaskTimerService = EspTimerService<Task>;

impl EspTimerService<Task> {
    pub fn new() -> Result<Self, EspError> {
        Ok(Self(Task))
    }
}

impl<T> Clone for EspTimerService<T>
where
    T: EspTimerServiceType + Clone,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T> ErrorType for EspTimerService<T>
where
    T: EspTimerServiceType,
{
    type Error = EspError;
}

impl<T> TimerService for EspTimerService<T>
where
    T: EspTimerServiceType,
{
    type Timer = EspTimer;

    fn timer(
        &mut self,
        callback: impl FnMut() + Send + 'static,
    ) -> Result<Self::Timer, Self::Error> {
        EspTimerService::timer(self, callback)
    }
}

impl<T> SystemTime for EspTimerService<T>
where
    T: EspTimerServiceType,
{
    fn now(&self) -> Duration {
        EspTimerService::now(self)
    }
}

#[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
mod isr {
    use esp_idf_sys::EspError;

    #[derive(Clone, Debug)]
    pub struct ISR;

    impl super::EspTimerServiceType for ISR {
        fn is_isr() -> bool {
            true
        }
    }

    pub type EspISRTimerService = super::EspTimerService<ISR>;

    impl super::EspTimerService<ISR> {
        pub unsafe fn new() -> Result<Self, EspError> {
            Ok(Self(ISR))
        }
    }
}

#[cfg(all(feature = "nightly", feature = "experimental"))]
mod asyncify {
    use embedded_svc::utils::asyncify::timer::AsyncTimerService;
    use embedded_svc::utils::asyncify::Asyncify;

    impl Asyncify for super::EspTimerService<super::Task> {
        type AsyncWrapper<S> = AsyncTimerService<S>;
    }

    #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
    impl Asyncify for super::EspTimerService<super::ISR> {
        type AsyncWrapper<S> = AsyncTimerService<S>;
    }
}

pub mod embassy_time {
    #[cfg(feature = "embassy-time-driver")]
    pub mod driver {
        use core::cell::UnsafeCell;
        use core::sync::atomic::{AtomicU64, Ordering};

        extern crate alloc;
        use alloc::sync::{Arc, Weak};

        use heapless::Vec;

        use ::embassy_time::driver::{AlarmHandle, Driver};

        use esp_idf_hal::interrupt::CriticalSection;

        use esp_idf_sys::*;

        use crate::timer::*;

        struct Alarm {
            timer: EspTimer,
            callback: Arc<AtomicU64>,
        }

        impl Alarm {
            fn new() -> Result<Self, EspError> {
                let callback = Arc::new(AtomicU64::new(0));
                let timer_callback = Arc::downgrade(&callback);

                let service = EspTimerService::<Task>::new()?;

                let timer = service.timer(move || {
                    if let Some(callback) = Weak::upgrade(&timer_callback) {
                        Self::call(&callback);
                    }
                })?;

                Ok(Self { timer, callback })
            }

            fn set_alarm(&self, duration: u64) -> Result<(), EspError> {
                self.timer.after(Duration::from_micros(duration))?;

                Ok(())
            }

            fn set_callback(&self, callback: fn(*mut ()), ctx: *mut ()) {
                let ptr: u64 = ((callback as usize as u64) << 32) | (ctx as usize as u64);

                self.callback.store(ptr, Ordering::SeqCst);
            }

            fn invoke(&self) {
                Self::call(&self.callback);
            }

            fn call(callback: &AtomicU64) {
                let ptr: u64 = callback.load(Ordering::SeqCst);

                if ptr != 0 {
                    unsafe {
                        let func: fn(*mut ()) = core::mem::transmute((ptr >> 32) as usize);
                        let arg: *mut () = (ptr & 0xffffffff) as usize as *mut ();

                        func(arg);
                    }
                }
            }
        }

        struct EspDriver<const MAX_ALARMS: usize = 16> {
            alarms: UnsafeCell<Vec<Alarm, MAX_ALARMS>>,
            cs: CriticalSection,
        }

        impl<const MAX_ALARMS: usize> EspDriver<MAX_ALARMS> {
            const fn new() -> Self {
                Self {
                    alarms: UnsafeCell::new(Vec::new()),
                    cs: CriticalSection::new(),
                }
            }
        }

        unsafe impl<const MAX_ALARMS: usize> Send for EspDriver<MAX_ALARMS> {}
        unsafe impl<const MAX_ALARMS: usize> Sync for EspDriver<MAX_ALARMS> {}

        impl<const MAX_ALARMS: usize> Driver for EspDriver<MAX_ALARMS> {
            fn now(&self) -> u64 {
                unsafe { esp_timer_get_time() as _ }
            }

            unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
                let mut id = {
                    let _guard = self.cs.enter();

                    self.alarms.get().as_mut().unwrap().len() as u8
                };

                if (id as usize) < MAX_ALARMS {
                    let alarm = Alarm::new().unwrap();

                    {
                        let _guard = self.cs.enter();

                        id = self.alarms.get().as_mut().unwrap().len() as u8;

                        if (id as usize) == MAX_ALARMS {
                            return None;
                        }

                        self.alarms
                            .get()
                            .as_mut()
                            .unwrap()
                            .push(alarm)
                            .unwrap_or_else(|_| unreachable!());
                    }

                    Some(AlarmHandle::new(id))
                } else {
                    None
                }
            }

            fn set_alarm_callback(&self, handle: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
                let alarm = unsafe { &self.alarms.get().as_mut().unwrap()[handle.id() as usize] };

                alarm.set_callback(callback, ctx);
            }

            fn set_alarm(&self, handle: AlarmHandle, timestamp: u64) {
                let alarm = unsafe { &self.alarms.get().as_mut().unwrap()[handle.id() as usize] };

                let now = self.now();

                if now < timestamp {
                    alarm.set_alarm(timestamp - now).unwrap();
                } else {
                    alarm.invoke();
                }
            }
        }

        pub fn link() -> i32 {
            42
        }

        ::embassy_time::time_driver_impl!(static DRIVER: EspDriver = EspDriver::new());
    }

    #[cfg(feature = "embassy-time-isr-queue")]
    pub mod queue {
        use esp_idf_hal::timer::embassy_time::*;

        use esp_idf_sys::*;

        struct AlarmImpl(esp_timer_handle_t);

        impl AlarmImpl {
            unsafe extern "C" fn handle_isr(alarm_context: *mut c_types::c_void) {
                let alarm_context = (alarm_context as *const AlarmContext).as_ref().unwrap();

                if esp_idf_hal::interrupt::active() {
                    #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
                    {
                        let signaled = esp_idf_hal::interrupt::with_isr_yield_signal(move || {
                            (alarm_context.callback)(alarm_context.ctx);
                        });

                        if signaled {
                            esp_idf_sys::esp_timer_isr_dispatch_need_yield();
                        }
                    }

                    #[cfg(not(esp_idf_esp_timer_supports_isr_dispatch_method))]
                    unreachable!();
                } else {
                    (alarm_context.callback)(alarm_context.ctx);
                }
            }
        }

        unsafe impl Send for AlarmImpl {}

        impl Alarm for AlarmImpl {
            fn new(context: &AlarmContext) -> Self {
                #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
                let dispatch_method = esp_timer_dispatch_t_ESP_TIMER_ISR;

                #[cfg(not(esp_idf_esp_timer_supports_isr_dispatch_method))]
                let dispatch_method = esp_timer_dispatch_t_ESP_TIMER_TASK;

                let mut handle: esp_timer_handle_t = core::ptr::null_mut();

                esp!(unsafe {
                    esp_timer_create(
                        &esp_timer_create_args_t {
                            callback: Some(AlarmImpl::handle_isr),
                            name: b"embassy-time-queue\0" as *const _ as *const _,
                            arg: context as *const _ as *mut _,
                            dispatch_method,
                            skip_unhandled_events: false,
                        },
                        &mut handle as *mut _,
                    )
                })
                .unwrap();

                Self(handle)
            }

            fn schedule(&mut self, timestamp: u64) {
                let now = unsafe { esp_timer_get_time() as _ };
                let after = if timestamp < now { 0 } else { timestamp - now };

                unsafe {
                    esp_timer_stop(self.0);
                }

                esp!(unsafe { esp_timer_start_once(self.0, after as _) }).unwrap();
            }
        }

        pub fn link() -> i32 {
            42
        }

        ::embassy_time::timer_queue_impl!(static QUEUE: Queue<AlarmImpl> = Queue::new());
    }
}
