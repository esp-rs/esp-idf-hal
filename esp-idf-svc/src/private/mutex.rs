use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::ptr;
use core::time::Duration;

use crate::sys::*;

// Might not always be available in the generated `esp-idf-sys` bindings
const ERR_ETIMEDOUT: esp_err_t = 116;

// NOTE: ESP-IDF-specific
const PTHREAD_MUTEX_INITIALIZER: u32 = 0xFFFFFFFF;

struct RawMutex(UnsafeCell<pthread_mutex_t>);

impl RawMutex {
    #[inline(always)]
    pub const fn new() -> Self {
        Self(UnsafeCell::new(PTHREAD_MUTEX_INITIALIZER as _))
    }

    #[inline(always)]
    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn lock(&self) {
        let r = pthread_mutex_lock(self.0.get());
        debug_assert_eq!(r, 0);
    }

    #[inline(always)]
    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn unlock(&self) {
        let r = pthread_mutex_unlock(self.0.get());
        debug_assert_eq!(r, 0);
    }
}

impl Drop for RawMutex {
    fn drop(&mut self) {
        let r = unsafe { pthread_mutex_destroy(self.0.get_mut() as *mut _) };
        debug_assert_eq!(r, 0);
    }
}

unsafe impl Sync for RawMutex {}
unsafe impl Send for RawMutex {}

struct RawCondvar(UnsafeCell<pthread_cond_t>);

impl RawCondvar {
    pub fn new() -> Self {
        let mut cond: pthread_cond_t = Default::default();

        let r = unsafe { pthread_cond_init(&mut cond as *mut _, ptr::null()) };
        debug_assert_eq!(r, 0);

        Self(UnsafeCell::new(cond))
    }

    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn wait(&self, mutex: &RawMutex) {
        let r = pthread_cond_wait(self.0.get(), mutex.0.get());
        debug_assert_eq!(r, 0);
    }

    #[allow(clippy::missing_safety_doc)]
    pub unsafe fn wait_timeout(&self, mutex: &RawMutex, duration: Duration) -> bool {
        let mut now: timeval = core::mem::zeroed();
        gettimeofday(&mut now, core::ptr::null_mut());

        let abstime = timespec {
            tv_sec: now.tv_sec + duration.as_secs() as crate::sys::time_t,
            tv_nsec: (now.tv_usec * 1000) + duration.subsec_nanos() as i32,
        };

        let r = pthread_cond_timedwait(self.0.get(), mutex.0.get(), &abstime as *const _);
        debug_assert!(r == ERR_ETIMEDOUT || r == 0);

        r == ERR_ETIMEDOUT
    }

    pub fn notify_one(&self) {
        let r = unsafe { pthread_cond_signal(self.0.get()) };
        debug_assert_eq!(r, 0);
    }

    pub fn notify_all(&self) {
        let r = unsafe { pthread_cond_broadcast(self.0.get()) };
        debug_assert_eq!(r, 0);
    }
}

unsafe impl Sync for RawCondvar {}
unsafe impl Send for RawCondvar {}

impl Drop for RawCondvar {
    fn drop(&mut self) {
        let r = unsafe { pthread_cond_destroy(self.0.get()) };
        debug_assert_eq!(r, 0);
    }
}

pub struct Mutex<T>(RawMutex, UnsafeCell<T>);

impl<T> Mutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(RawMutex::new(), UnsafeCell::new(data))
    }

    #[inline(always)]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        MutexGuard::new(self)
    }

    #[inline(always)]
    pub fn get_mut(&mut self) -> &mut T {
        self.1.get_mut()
    }
}

unsafe impl<T> Sync for Mutex<T> where T: Send {}
unsafe impl<T> Send for Mutex<T> where T: Send {}

pub struct MutexGuard<'a, T>(&'a Mutex<T>);

impl<'a, T> MutexGuard<'a, T> {
    #[inline(always)]
    fn new(mutex: &'a Mutex<T>) -> Self {
        unsafe {
            mutex.0.lock();
        }

        Self(mutex)
    }
}

impl<T> Drop for MutexGuard<'_, T> {
    #[inline(always)]
    fn drop(&mut self) {
        unsafe {
            self.0 .0.unlock();
        }
    }
}

impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        unsafe { self.0 .1.get().as_mut().unwrap() }
    }
}

impl<T> DerefMut for MutexGuard<'_, T> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.0 .1.get().as_mut().unwrap() }
    }
}

pub struct Condvar(RawCondvar);

impl Condvar {
    pub fn new() -> Self {
        Self(RawCondvar::new())
    }

    pub fn wait<'a, T>(&self, guard: MutexGuard<'a, T>) -> MutexGuard<'a, T> {
        unsafe {
            self.0.wait(&guard.0 .0);
        }

        guard
    }

    pub fn wait_timeout<'a, T>(
        &self,
        guard: MutexGuard<'a, T>,
        duration: Duration,
    ) -> (MutexGuard<'a, T>, bool) {
        let timeout = unsafe { self.0.wait_timeout(&guard.0 .0, duration) };

        (guard, timeout)
    }

    pub fn notify_one(&self) {
        self.0.notify_one();
    }

    pub fn notify_all(&self) {
        self.0.notify_all();
    }
}

unsafe impl Sync for Condvar {}
unsafe impl Send for Condvar {}

impl Default for Condvar {
    fn default() -> Self {
        Self::new()
    }
}
