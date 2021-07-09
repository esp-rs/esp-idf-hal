use core::cell::UnsafeCell;

use mutex_trait::*;

use crate::*;

pub trait RwLock: Mutex {
    fn lock_read<R>(&self, f: impl FnOnce(&Self::Data) -> R) -> R;
}

// NOTE: ESP-IDF-specific
const PTHREAD_MUTEX_INITIALIZER: u32 = 0xFFFFFFFF;

pub struct EspMutex<T>(UnsafeCell<pthread_mutex_t>, T);

unsafe impl<T> Send for EspMutex<T> {}
unsafe impl<T> Sync for EspMutex<T> {}

impl<T> EspMutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(UnsafeCell::new(PTHREAD_MUTEX_INITIALIZER), data)
    }
}

impl<T> Drop for EspMutex<T> {
    fn drop(&mut self) {
        let r = unsafe { pthread_mutex_destroy(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);
    }
}

impl<T> Mutex for EspMutex<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let r = unsafe { pthread_mutex_lock(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);

        let result = f(&mut self.1);

        let r = unsafe { pthread_mutex_unlock(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);

        result
    }
}

impl<T> RwLock for EspMutex<T> {
    #[inline(always)]
    fn lock_read<R>(&self, f: impl FnOnce(&Self::Data) -> R) -> R {
        let r = unsafe { pthread_mutex_lock(&mut *self.0.get() as *mut _) };
        debug_assert_eq!(r, 0);

        let result = f(&self.1);

        let r = unsafe { pthread_mutex_unlock(&mut *self.0.get() as *mut _) };
        debug_assert_eq!(r, 0);

        result
    }
}

#[cfg(feature = "std")]
pub struct EspStdMutex<T>(pub std::sync::Mutex<T>);

#[cfg(feature = "std")]
impl<T> EspStdMutex<T> {
    #[inline(always)]
    pub fn new(data: T) -> Self {
        Self(std::sync::Mutex::new(data))
    }
}

#[cfg(feature = "std")]
impl<T> Mutex for EspStdMutex<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let mut guard = self.0.lock().unwrap();

        f(&mut guard)
    }
}

#[cfg(feature = "std")]
pub struct EspStdRwLock<T>(pub std::sync::RwLock<T>);

#[cfg(feature = "std")]
impl<T> EspStdRwLock<T> {
    #[inline(always)]
    pub fn new(data: T) -> Self {
        Self(std::sync::RwLock::new(data))
    }
}

#[cfg(feature = "std")]
impl<T> Mutex for EspStdRwLock<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let mut guard = self.0.write().unwrap();

        f(&mut guard)
    }
}

#[cfg(feature = "std")]
impl<T> RwLock for EspStdRwLock<T> {
    #[inline(always)]
    fn lock_read<R>(&self, f: impl FnOnce(&Self::Data) -> R) -> R {
        let guard = self.0.read().unwrap();

        f(&guard)
    }
}
