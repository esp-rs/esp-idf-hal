use core::cell::UnsafeCell;

use embedded_svc;
use mutex_trait;

use crate::*;

// NOTE: ESP-IDF-specific
const PTHREAD_MUTEX_INITIALIZER: u32 = 0xFFFFFFFF;

pub struct EspMutex<T>(UnsafeCell<pthread_mutex_t>, UnsafeCell<T>);

unsafe impl<T> Sync for EspMutex<T> {}

impl<T> EspMutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(
            UnsafeCell::new(PTHREAD_MUTEX_INITIALIZER as _),
            UnsafeCell::new(data),
        )
    }
}

impl<T> Drop for EspMutex<T> {
    fn drop(&mut self) {
        let r = unsafe { pthread_mutex_destroy(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);
    }
}

impl<T> mutex_trait::Mutex for EspMutex<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let r = unsafe { pthread_mutex_lock(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);

        let result = f(self.1.get_mut());

        let r = unsafe { pthread_mutex_unlock(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);

        result
    }
}

impl<T> embedded_svc::mutex::Mutex for EspMutex<T> {
    type Data = T;

    #[inline(always)]
    fn new(data: T) -> Self {
        Self(
            UnsafeCell::new(PTHREAD_MUTEX_INITIALIZER as _),
            UnsafeCell::new(data),
        )
    }

    #[inline(always)]
    fn with_lock<R>(&self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let r = unsafe { pthread_mutex_lock(self.0.get() as *mut _) };
        debug_assert_eq!(r, 0);

        let result = unsafe { f(&mut *self.1.get()) };

        let r = unsafe { pthread_mutex_unlock(self.0.get() as *mut _) };
        debug_assert_eq!(r, 0);

        result
    }
}
