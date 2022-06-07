use core::ptr;
use core::result::Result;
use core::time::Duration;

extern crate alloc;
use alloc::boxed::Box;

use embedded_svc::errors::Errors;
use embedded_svc::sys_time::SystemTime;
use embedded_svc::timer::{self, OnceTimer, PeriodicTimer, Timer, TimerService};

use esp_idf_sys::*;

#[cfg(feature = "experimental")]
pub use asyncify::*;

#[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
pub use isr::*;

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
    extern "C" fn handle(arg: *mut c_types::c_void) {
        unsafe {
            #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
            let previous_yielder = if esp_idf_hal::interrupt::active() {
                esp_idf_hal::interrupt::set_isr_yielder(Some(EspISRTimerService::isr_yield))
            } else {
                None
            };

            UnsafeCallback::from_ptr(arg).call();

            #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
            if esp_idf_hal::interrupt::active() {
                esp_idf_hal::interrupt::set_isr_yielder(previous_yielder);
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

impl Errors for EspTimer {
    type Error = EspError;
}

impl timer::Timer for EspTimer {
    fn is_scheduled(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { esp_timer_is_active(self.handle) })
    }

    fn cancel(&mut self) -> Result<bool, Self::Error> {
        let res = unsafe { esp_timer_stop(self.handle) };

        Ok(res != ESP_OK)
    }
}

impl OnceTimer for EspTimer {
    fn after(&mut self, duration: Duration) -> Result<(), Self::Error> {
        self.cancel()?;

        esp!(unsafe { esp_timer_start_once(self.handle, duration.as_micros() as _) })?;

        Ok(())
    }
}

impl PeriodicTimer for EspTimer {
    fn every(&mut self, duration: Duration) -> Result<(), Self::Error> {
        self.cancel()?;

        esp!(unsafe { esp_timer_start_periodic(self.handle, duration.as_micros() as _) })?;

        Ok(())
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

impl<T> Errors for EspTimerService<T>
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

    fn timer(&mut self, callback: impl FnMut() + Send + 'static) -> Result<EspTimer, EspError> {
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

impl<T> SystemTime for EspTimerService<T>
where
    T: EspTimerServiceType,
{
    fn now(&self) -> Duration {
        Duration::from_micros(unsafe { esp_timer_get_time() as _ })
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

        pub(crate) unsafe fn isr_yield() {
            esp_idf_sys::esp_timer_isr_dispatch_need_yield();
        }
    }
}

#[cfg(feature = "experimental")]
mod asyncify {
    use embedded_svc::utils::asyncify::timer::AsyncTimerService;
    use embedded_svc::utils::asyncify::Asyncify;
    use embedded_svc::utils::asyncs::signal::AtomicSignal;
    
    impl Asyncify for super::EspTimerService<super::Task> {
        type AsyncWrapper<S> = AsyncTimerService<S, AtomicSignal<()>>;
    }

    #[cfg(esp_idf_esp_timer_supports_isr_dispatch_method)]
    impl Asyncify for super::EspTimerService<super::ISR> {
        type AsyncWrapper<S> = AsyncTimerService<S, AtomicSignal<()>>;
    }
}
