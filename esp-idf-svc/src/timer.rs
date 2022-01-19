use core::fmt::{Debug, Display};
use core::ptr;
use core::result::Result;

use embedded_svc::timer::{self, Timer};

use esp_idf_sys::*;

struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

impl UnsafeCallback {
    fn call(&self) {
        (unsafe { self.0.as_mut().unwrap() })();
    }
}

unsafe impl Send for UnsafeCallback {}

pub struct EspTimer {
    handle: esp_timer_handle_t,
    _callback: Box<Box<dyn FnMut() + 'static>>,
}

unsafe impl Send for EspTimer {}

impl EspTimer {
    extern "C" fn handle(arg: *mut c_types::c_void) {
        let callback = unsafe { (arg as *const UnsafeCallback).as_ref().unwrap() };

        callback.call();
    }
}

impl Drop for EspTimer {
    fn drop(&mut self) {
        let _ = self.cancel();

        while unsafe { esp_timer_delete(self.handle) } != ESP_OK {
            // Timer is still running, busy-loop
        }
    }
}

impl timer::Timer for EspTimer {
    type Error = EspError;

    fn once(&mut self, after: std::time::Duration) -> Result<(), Self::Error> {
        let _ = self.cancel();

        esp!(unsafe { esp_timer_start_once(self.handle, after.as_micros() as _) })?;

        Ok(())
    }

    fn periodic(&mut self, after: std::time::Duration) -> Result<(), Self::Error> {
        let _ = self.cancel();

        esp!(unsafe { esp_timer_start_periodic(self.handle, after.as_micros() as _) })?;

        Ok(())
    }

    fn is_scheduled(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { esp_timer_is_active(self.handle) })
    }

    fn cancel(&mut self) -> Result<bool, Self::Error> {
        let res = unsafe { esp_timer_stop(self.handle) };

        Ok(res != ESP_OK)
    }
}

#[derive(Clone)]
struct PrivateData;

#[derive(Clone)]
pub struct EspTimerService(PrivateData);

impl EspTimerService {
    pub fn new() -> Result<Self, EspError> {
        Ok(Self(PrivateData))
    }
}

impl timer::TimerService for EspTimerService {
    type Error = EspError;

    type Timer = EspTimer;

    fn timer<E>(
        &self,
        conf: &timer::TimerConfiguration,
        mut callback: impl FnMut() -> Result<(), E> + Send + 'static,
    ) -> Result<Self::Timer, Self::Error>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        let mut handle: esp_timer_handle_t = ptr::null_mut();

        let callback: Box<dyn FnMut() + 'static> = Box::new(move || callback().unwrap());
        let mut callback = Box::new(callback);

        let unsafe_callback = UnsafeCallback(&mut *callback as *mut _);

        esp!(unsafe {
            esp_timer_create(
                &esp_timer_create_args_t {
                    callback: Some(EspTimer::handle),
                    name: b"rust\0" as *const _ as *const _, // TODO
                    arg: &unsafe_callback as *const _ as *mut _,
                    dispatch_method: esp_timer_dispatch_t_ESP_TIMER_TASK,
                    skip_unhandled_events: conf.skip_unhandled_events,
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
