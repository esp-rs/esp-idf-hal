use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};

use esp_idf_sys::*;

// NOTE: ESP-IDF-specific
const PTHREAD_MUTEX_INITIALIZER: u32 = 0xFFFFFFFF;

pub struct Mutex<T>(UnsafeCell<pthread_mutex_t>, UnsafeCell<T>);

impl<T> Mutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(
            UnsafeCell::new(PTHREAD_MUTEX_INITIALIZER as _),
            UnsafeCell::new(data),
        )
    }

    #[inline(always)]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        MutexGuard::new(self)
    }
}

impl<T> Drop for Mutex<T> {
    fn drop(&mut self) {
        let r = unsafe { pthread_mutex_destroy(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);
    }
}

unsafe impl<T> Sync for Mutex<T> where T: Send {}
unsafe impl<T> Send for Mutex<T> where T: Send {}

pub struct MutexGuard<'a, T>(&'a Mutex<T>, &'a mut T);

impl<'a, T> MutexGuard<'a, T> {
    #[inline(always)]
    fn new(mutex: &'a Mutex<T>) -> Self {
        let r = unsafe { pthread_mutex_lock(mutex.0.get()) };
        debug_assert_eq!(r, 0);

        Self(mutex, unsafe { mutex.1.get().as_mut().unwrap() })
    }
}

unsafe impl<T> Sync for MutexGuard<'_, T> where T: Sync {}

impl<'a, T> Drop for MutexGuard<'a, T> {
    #[inline(always)]
    fn drop(&mut self) {
        let r = unsafe { pthread_mutex_unlock(self.0 .0.get()) };
        debug_assert_eq!(r, 0);
    }
}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        self.1
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.1
    }
}

#[cfg(feature = "mutex-trait")]
impl<T> mutex_trait::Mutex for Mutex<T> {
    type Data = T;

    #[inline(always)]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let mut guard = Mutex::lock(self);

        f(&mut *guard)
    }
}

#[cfg(feature = "embedded-svc-mutex")]
impl<T> embedded_svc::mutex::Mutex for Mutex<T> {
    type Data = T;

    type Guard<'a>
    where
        T: 'a,
    = MutexGuard<'a, T>;

    #[inline(always)]
    fn new(data: Self::Data) -> Self {
        Mutex::new(data)
    }

    #[inline(always)]
    fn lock(&self) -> Self::Guard<'_> {
        Mutex::lock(self)
    }
}
