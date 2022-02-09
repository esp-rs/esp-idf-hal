use core::fmt::{Debug, Display};
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::result::Result;
use core::time::Duration;

use embedded_svc::service;
use embedded_svc::timer::{self, Timer};

use esp_idf_sys::*;

pub type EspOnce = EspTimerService<Once>;
pub type EspPeriodic = EspTimerService<Periodic>;

pub type EspOnceTimer = EspTimer<Once>;
pub type EspPeriodicTimer = EspTimer<Periodic>;

pub trait EspTimerType {
    fn is_periodic() -> bool;
}

pub struct Once(Option<Box<dyn FnOnce() + 'static>>);
pub struct Periodic(Box<dyn FnMut() + 'static>);

impl EspTimerType for Once {
    fn is_periodic() -> bool {
        false
    }
}

impl EspTimerType for Periodic {
    fn is_periodic() -> bool {
        true
    }
}

struct UnsafeCallback<T>(*mut T);

impl<T> UnsafeCallback<T> {
    fn from(boxed: &mut Box<T>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }
}

impl UnsafeCallback<Once> {
    unsafe fn call(&self) {
        let reference = self.0.as_mut().unwrap();

        if let Some(cb) = mem::replace(&mut reference.0, None) {
            cb();
        }
    }
}

impl UnsafeCallback<Periodic> {
    unsafe fn call(&self) {
        let reference = self.0.as_mut().unwrap();

        (reference.0)();
    }
}

pub struct EspTimer<T>
where
    T: EspTimerType,
{
    handle: esp_timer_handle_t,
    duration: Duration,
    _callback: Box<T>,
}

impl<T> EspTimer<T>
where
    T: EspTimerType,
{
    extern "C" fn handle(arg: *mut c_types::c_void) {
        unsafe {
            if T::is_periodic() {
                UnsafeCallback::<Periodic>::from_ptr(arg).call();
            } else {
                UnsafeCallback::<Once>::from_ptr(arg).call();
            }
        }
    }
}

unsafe impl<T> Send for EspTimer<T> where T: EspTimerType {}

impl<T> Drop for EspTimer<T>
where
    T: EspTimerType,
{
    fn drop(&mut self) {
        let _ = self.cancel();

        while unsafe { esp_timer_delete(self.handle) } != ESP_OK {
            // Timer is still running, busy-loop
        }
    }
}

impl<T> service::Service for EspTimer<T>
where
    T: EspTimerType,
{
    type Error = EspError;
}

impl<T> timer::Timer for EspTimer<T>
where
    T: EspTimerType,
{
    fn start(&mut self) -> Result<(), Self::Error> {
        let _ = self.cancel();

        if T::is_periodic() {
            esp!(unsafe { esp_timer_start_periodic(self.handle, self.duration.as_micros() as _) })?;
        } else {
            esp!(unsafe { esp_timer_start_once(self.handle, self.duration.as_micros() as _) })?;
        }

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

pub struct EspTimerService<T>(PhantomData<fn() -> T>);

impl<T> Clone for EspTimerService<T> {
    fn clone(&self) -> Self {
        Self(PhantomData)
    }
}

impl<T> EspTimerService<T>
where
    T: EspTimerType,
{
    pub fn new() -> Result<Self, EspError> {
        Ok(Self(PhantomData))
    }

    fn timer(&self, duration: Duration, mut callback: Box<T>) -> Result<EspTimer<T>, EspError> {
        let mut handle: esp_timer_handle_t = ptr::null_mut();

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        esp!(unsafe {
            esp_timer_create(
                &esp_timer_create_args_t {
                    callback: Some(EspTimer::<T>::handle),
                    name: b"rust\0" as *const _ as *const _, // TODO
                    arg: unsafe_callback.as_ptr(),
                    dispatch_method: esp_timer_dispatch_t_ESP_TIMER_TASK,
                    skip_unhandled_events: false, // TODO
                },
                &mut handle as *mut _,
            )
        })?;

        Ok(EspTimer {
            handle,
            duration,
            _callback: callback,
        })
    }
}

impl EspTimerService<Periodic> {
    pub fn into_async(
        self,
    ) -> embedded_svc::utils::nonblocking::timer::Periodic<
        esp_idf_hal::mutex::Mutex<
            embedded_svc::utils::nonblocking::timer::TimerState<EspTimer<Periodic>>,
        >,
        Self,
    > {
        embedded_svc::utils::nonblocking::timer::Periodic::new(self)
    }
}

impl EspTimerService<Once> {
    pub fn into_async(
        self,
    ) -> embedded_svc::utils::nonblocking::timer::Once<
        esp_idf_hal::mutex::Mutex<
            embedded_svc::utils::nonblocking::timer::OnceState<EspTimer<Once>>,
        >,
        Self,
    > {
        embedded_svc::utils::nonblocking::timer::Once::new(self)
    }
}

impl<T> service::Service for EspTimerService<T> {
    type Error = EspError;
}

impl timer::Once for EspTimerService<Once> {
    type Timer = EspTimer<Once>;

    fn after<E>(
        &mut self,
        duration: Duration,
        callback: impl FnOnce() -> Result<(), E> + Send + 'static,
    ) -> Result<Self::Timer, Self::Error>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        self.timer(
            duration,
            Box::new(Once(Some(Box::new(move || callback().unwrap())))),
        )
    }
}

impl timer::Periodic for EspTimerService<Periodic> {
    type Timer = EspTimer<Periodic>;

    fn every<E>(
        &mut self,
        duration: Duration,
        mut callback: impl FnMut() -> Result<(), E> + Send + 'static,
    ) -> Result<Self::Timer, Self::Error>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        self.timer(
            duration,
            Box::new(Periodic(Box::new(move || callback().unwrap()))),
        )
    }
}
