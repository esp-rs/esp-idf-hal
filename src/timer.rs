#[cfg(feature = "embassy")]
mod embassy {
    use std::{mem, ptr};

    use embassy::time::driver::{AlarmHandle, Driver};

    use esp_idf_sys::*;

    use crate::mutex::Mutex;

    #[derive(Copy, Clone)]
    struct EspTimerRegistration {
        handle: esp_timer_handle_t,
        callback: Option<(fn(*mut ()), *mut ())>,
    }

    impl EspTimerRegistration {
        const fn new() -> Self {
            Self {
                handle: ptr::null_mut(),
                callback: None,
            }
        }
    }

    unsafe impl Send for EspTimerRegistration {}
    unsafe impl Sync for EspTimerRegistration {}

    static REGISTRATIONS: Mutex<[EspTimerRegistration; 256]> =
        Mutex::new([EspTimerRegistration::new(); 256]);

    struct EspDriver;

    impl EspDriver {
        const fn to_micros(ticks: u64) -> u64 {
            ticks
        }

        const fn to_ticks(micros: u64) -> u64 {
            micros
        }

        fn deregister(index: usize) -> Option<(fn(*mut ()), *mut ())> {
            let mut guard = REGISTRATIONS.lock();

            let registration = &mut guard[index];

            if !registration.handle.is_null() {
                unsafe {
                    esp_timer_stop(registration.handle);
                }
                esp!(unsafe { esp_timer_delete(registration.handle) }).unwrap();

                registration.handle = ptr::null_mut();

                mem::replace(&mut registration.callback, None)
            } else {
                None
            }
        }

        extern "C" fn handle(arg: *mut c_types::c_void) {
            if let Some((callback, ctx)) = Self::deregister(arg as usize) {
                (callback)(ctx);
            }
        }
    }

    impl Driver for EspDriver {
        fn now(&self) -> u64 {
            let mut tv_now: timeval = Default::default();

            unsafe {
                gettimeofday(&mut tv_now as *mut _, core::ptr::null_mut());
            }

            Self::to_ticks(tv_now.tv_sec as u64 * 1000000_u64 + tv_now.tv_usec as u64)
        }

        unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
            let mut guard = REGISTRATIONS.lock();

            for (index, registration) in &mut guard.iter_mut().enumerate() {
                if registration.handle.is_null() {
                    registration.callback = None;

                    esp!(esp_timer_create(
                        &esp_timer_create_args_t {
                            callback: Some(EspDriver::handle),
                            name: b"embassy\0" as *const _ as *const _, // TODO
                            arg: index as _,
                            dispatch_method: esp_timer_dispatch_t_ESP_TIMER_TASK,
                            skip_unhandled_events: false, // TODO
                        },
                        &mut registration.handle as *mut _,
                    ))
                    .unwrap();

                    return Some(AlarmHandle::new(index as _));
                }
            }

            None
        }

        fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
            let mut guard = REGISTRATIONS.lock();

            let registration = &mut guard[alarm.id() as usize];

            if !registration.handle.is_null() {
                registration.callback = Some((callback, ctx));
            }
        }

        fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) {
            let now = self.now();

            if timestamp <= now {
                if let Some((callback, ctx)) = Self::deregister(alarm.id() as _) {
                    (callback)(ctx);
                }
            } else {
                let mut guard = REGISTRATIONS.lock();

                let registration = &mut guard[alarm.id() as usize];

                if !registration.handle.is_null() {
                    esp!(unsafe {
                        esp_timer_start_once(
                            registration.handle,
                            Self::to_micros(timestamp - now) as _,
                        )
                    })
                    .unwrap();
                }
            }
        }
    }
}
