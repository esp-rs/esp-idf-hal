use core::time::Duration;
#[cfg(feature = "std")]
use std::sync::{Condvar, Mutex};

#[cfg(not(feature = "std"))]
use esp_idf_sys::*;

#[cfg(not(feature = "std"))]
use super::time::micros_since_boot;

pub struct Waitable<T> {
    #[cfg(feature = "std")]
    cvar: Condvar,
    #[cfg(feature = "std")]
    state: Mutex<T>,
    #[cfg(not(feature = "std"))]
    state: EspMutex<T>,
}

impl<T> Waitable<T> {
    pub fn new(state: T) -> Self {
        Self {
            #[cfg(feature = "std")]
            cvar: Condvar::new(),
            #[cfg(feature = "std")]
            state: Mutex::new(state),
            #[cfg(not(feature = "std"))]
            state: EspMutex::new(state),
        }
    }

    #[cfg(feature = "std")]
    pub fn get<Q>(&self, getter: impl FnOnce(&T) -> Q) -> Q {
        getter(&self.state.lock().unwrap())
    }

    #[cfg(not(feature = "std"))]
    pub fn get<Q>(&self, getter: impl FnOnce(&T) -> Q) -> Q {
        self.state.with_lock(|state| getter(*state))
    }

    #[cfg(feature = "std")]
    pub fn modify<Q>(&mut self, modifier: impl FnOnce(&mut T) -> (bool, Q)) -> Q {
        let mut guard = self.state.lock().unwrap();

        let (notify, result) = modifier(&mut *guard);

        if notify {
            self.cvar.notify_all();
        }

        result
    }

    #[cfg(not(feature = "std"))]
    pub fn modify<Q>(&mut self, modifier: impl FnOnce(&mut T) -> (bool, Q)) {
        self.shared.with_lock(|mut state| modifier(&mut state).1)
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
        getter: impl FnOnce(&T) -> Q,
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
        getter: impl FnOnce(&T) -> Q,
    ) -> Q {
        loop {
            (cond, value) = self
                .state
                .with_lock(|state| (condition(&state), getter(&state)));

            if !cond {
                return value;
            }

            unsafe { vTaskDelay(500) };
        }
    }

    #[cfg(feature = "std")]
    pub fn wait_timeout_while_and_get<Q>(
        &self,
        dur: Duration,
        condition: impl Fn(&T) -> bool,
        getter: impl FnOnce(&T) -> Q,
    ) -> (bool, Q) {
        let (guard, result) = self
            .cvar
            .wait_timeout_while(self.state.lock().unwrap(), dur, |state| condition(state))
            .unwrap();

        (result.timed_out(), getter(&guard))
    }

    #[cfg(not(feature = "std"))]
    pub fn wait_timeout_while_and_get(
        &self,
        dur: Duration,
        condition: impl Fn(&T) -> bool,
        getter: impl FnOnce(&T) -> Q,
    ) -> (bool, Q) {
        let now = micros_since_boot();
        let end = now + dur.as_micros();

        loop {
            let (cond, value) = self
                .state
                .with_lock(|state| (condition(&state), getter(&state)));

            if !cond {
                return (false, value);
            } else if micros_since_boot() > end {
                return (true, value);
            }

            unsafe { vTaskDelay(500) };
        }
    }
}
