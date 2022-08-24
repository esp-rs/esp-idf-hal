use core::time::Duration;

use super::mutex::{Condvar, Mutex};

#[cfg(not(feature = "std"))]
use esp_idf_sys::*;

pub struct Waitable<T> {
    pub cvar: Condvar,
    pub state: Mutex<T>,
}

impl<T> Waitable<T>
where
    T: Send,
{
    pub fn new(state: T) -> Self {
        Self {
            cvar: Condvar::new(),
            state: Mutex::new(state),
        }
    }

    pub fn get<Q>(&self, getter: impl FnOnce(&T) -> Q) -> Q {
        let state = self.state.lock();

        getter(&state)
    }

    pub fn get_mut<Q>(&self, getter: impl FnOnce(&mut T) -> Q) -> Q {
        let mut state = self.state.lock();

        getter(&mut state)
    }

    pub fn wait_while(&self, condition: impl Fn(&T) -> bool) {
        self.wait_while_and_get(condition, |_| ());
    }

    #[allow(dead_code)]
    pub fn wait_timeout_while(&self, dur: Duration, condition: impl Fn(&T) -> bool) {
        self.wait_timeout_while_and_get(dur, condition, |_| ());
    }

    pub fn wait_while_and_get<Q>(
        &self,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> Q {
        let mut state = self.state.lock();

        loop {
            if !condition(&state) {
                return getter(&state);
            }

            state = self.cvar.wait(state);
        }
    }

    pub fn wait_timeout_while_and_get<Q>(
        &self,
        dur: Duration,
        condition: impl Fn(&T) -> bool,
        getter: impl Fn(&T) -> Q,
    ) -> (bool, Q) {
        let mut state = self.state.lock();

        loop {
            if !condition(&state) {
                return (false, getter(&state));
            }

            let (new_state, timeout) = self.cvar.wait_timeout(state, dur);

            state = new_state;

            if timeout {
                return (true, getter(&state));
            }
        }
    }
}
