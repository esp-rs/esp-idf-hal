use core::time::Duration;

#[cfg(not(feature = "std"))]
use esp_idf_sys::*;

use embedded_svc::mutex::Mutex;

pub struct Waitable<T> {
    #[cfg(feature = "std")]
    cvar: std::sync::Condvar,
    #[cfg(feature = "std")]
    state: std::sync::Mutex<T>,
    #[cfg(not(feature = "std"))]
    state: esp_idf_hal::mutex::Mutex<T>,
}

impl<T> Waitable<T> {
    pub fn new(state: T) -> Self {
        Self {
            #[cfg(feature = "std")]
            cvar: std::sync::Condvar::new(),
            #[cfg(feature = "std")]
            state: std::sync::Mutex::new(state),
            #[cfg(not(feature = "std"))]
            state: esp_idf_hal::mutex::Mutex::new(state),
        }
    }

    pub fn get<Q>(&self, getter: impl FnOnce(&T) -> Q) -> Q {
        getter(&Mutex::lock(&self.state))
    }

    pub fn modify<Q>(&mut self, modifier: impl FnOnce(&mut T) -> (bool, Q)) -> Q {
        let mut guard = Mutex::lock(&self.state);

        let (notify, result) = modifier(&mut *guard);

        if notify {
            self.cvar.notify_all();
        }

        result
    }

    pub fn wait_while(&self, condition: impl Fn(&T) -> bool) {
        self.wait_while_and_get(condition, |_| ());
    }

    #[allow(dead_code)]
    pub fn wait_timeout_while(&self, dur: Duration, condition: impl Fn(&T) -> bool) {
        self.wait_timeout_while_and_get(dur, condition, |_| ());
    }

    #[cfg(feature = "std")]
    pub fn wait_while_and_get<Q>(
        &self,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> Q {
        getter(
            &self
                .cvar
                .wait_while(self.state.lock().unwrap(), |state| condition(state))
                .unwrap(),
        )
    }

    #[cfg(not(feature = "std"))]
    pub fn wait_while_and_get<Q>(
        &self,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> Q {
        loop {
            {
                let state = Mutex::lock(self);

                if !condition(&state) {
                    return getter(&state);
                }
            }

            unsafe { vTaskDelay(500) };
        }
    }

    #[cfg(feature = "std")]
    pub fn wait_timeout_while_and_get<Q>(
        &self,
        dur: Duration,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> (bool, Q) {
        let (guard, result) = self
            .cvar
            .wait_timeout_while(self.state.lock().unwrap(), dur, |state| condition(state))
            .unwrap();

        (result.timed_out(), getter(&guard))
    }

    #[cfg(not(feature = "std"))]
    pub fn wait_timeout_while_and_get<Q>(
        &self,
        dur: Duration,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> (bool, Q) {
        fn micros_since_boot() -> u128 {
            unsafe { esp_timer_get_time() as _ }
        }

        let now = micros_since_boot();
        let end = now + dur.as_micros();

        loop {
            {
                let state = Mutex::lock(self.state);

                if !condition(&state) {
                    return (false, getter(&state));
                } else if micros_since_boot() > end {
                    return (true, getter(&state));
                }
            }

            unsafe { vTaskDelay(500) };
        }
    }
}
