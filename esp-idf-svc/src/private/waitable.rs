use core::time::Duration;

use super::mutex::{Condvar, Mutex};

use esp_idf_sys::EspError;
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

    pub fn wait_while<F: Fn(&T) -> Result<bool, EspError>>(
        &self,
        condition: F,
    ) -> Result<(), EspError> {
        self.wait_while_and_get(condition, |_| ())
    }

    #[allow(dead_code)]
    pub fn wait_timeout_while<F: Fn(&T) -> Result<bool, EspError>>(
        &self,
        dur: Duration,
        condition: F,
    ) -> Result<bool, EspError> {
        self.wait_timeout_while_and_get(dur, condition, |_| ())
            .map(|(timeout, _)| timeout)
    }

    pub fn wait_while_and_get<F: Fn(&T) -> Result<bool, EspError>, G: Fn(&T) -> Q, Q>(
        &self,
        condition: F,
        getter: G,
    ) -> Result<Q, EspError> {
        let mut state = self.state.lock();

        loop {
            if !condition(&state)? {
                return Ok(getter(&state));
            }

            state = self.cvar.wait(state);
        }
    }

    pub fn wait_timeout_while_and_get<F: Fn(&T) -> Result<bool, EspError>, G: Fn(&T) -> Q, Q>(
        &self,
        dur: Duration,
        condition: F,
        getter: G,
    ) -> Result<(bool, Q), EspError> {
        let mut state = self.state.lock();

        loop {
            if !condition(&state)? {
                return Ok((false, getter(&state)));
            }

            let (new_state, timeout) = self.cvar.wait_timeout(state, dur);

            state = new_state;

            if timeout {
                return Ok((true, getter(&state)));
            }
        }
    }
}
