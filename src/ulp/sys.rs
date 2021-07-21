/// A mini "esp-idf-ulp-sys" module exposing stuff on top of which the ULP HAL support is implemented
/// (currently, only GPIO) + some utilities for the riscv ULP processor

use mutex_trait::*;

pub use self::cpu::*;
pub use self::gpio::*;

pub mod cpu;
pub mod gpio;

pub type EspError = core::convert::Infallible;

#[macro_export]
macro_rules! esp_result {
    ($err:expr, $value:expr) => {{
        $err;

        Ok($value)
    }};
}

pub struct EspMutex<T>(T);

unsafe impl<T> Send for EspMutex<T> {}
unsafe impl<T> Sync for EspMutex<T> {}

impl<T> EspMutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(data)
    }
}

impl<T> Mutex for EspMutex<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        f(&mut self.0)
    }
}
