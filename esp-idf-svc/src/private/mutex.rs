use core::cell::UnsafeCell;
use core::ptr;
use core::time::Duration;

use esp_idf_sys::*;

// NOTE: ESP-IDF-specific
const PTHREAD_MUTEX_INITIALIZER: u32 = 0xFFFFFFFF;

pub type Mutex<T> = embedded_svc::utils::mutex::Mutex<RawMutex, T>;

pub type Condvar = embedded_svc::utils::mutex::Condvar<RawCondvar>;

pub struct RawMutex(UnsafeCell<pthread_mutex_t>);

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

impl embedded_svc::utils::mutex::RawMutex for RawMutex {
    #[cfg(feature = "nightly")] // Remove "nightly" condition once 1.64 is out
    #[allow(clippy::declare_interior_mutable_const)]
    const INIT: Self = RawMutex::new();

    fn new() -> Self {
        RawMutex::new()
    }

    unsafe fn lock(&self) {
        RawMutex::lock(self);
    }

    unsafe fn unlock(&self) {
        RawMutex::unlock(self);
    }
}

pub struct RawCondvar(UnsafeCell<pthread_cond_t>);

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
            tv_sec: now.tv_sec + duration.as_secs() as esp_idf_sys::time_t,
            tv_nsec: (now.tv_usec * 1000) + duration.subsec_nanos() as i32,
        };

        let r = pthread_cond_timedwait(self.0.get(), mutex.0.get(), &abstime as *const _);
        debug_assert!(r == ETIMEDOUT as i32 || r == 0);

        r == ETIMEDOUT as i32
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

impl Default for RawCondvar {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for RawCondvar {
    fn drop(&mut self) {
        let r = unsafe { pthread_cond_destroy(self.0.get()) };
        debug_assert_eq!(r, 0);
    }
}

impl embedded_svc::utils::mutex::RawCondvar for RawCondvar {
    type RawMutex = RawMutex;

    fn new() -> Self {
        RawCondvar::new()
    }

    unsafe fn wait(&self, mutex: &Self::RawMutex) {
        RawCondvar::wait(self, mutex);
    }

    unsafe fn wait_timeout(&self, mutex: &Self::RawMutex, duration: Duration) -> bool {
        RawCondvar::wait_timeout(self, mutex, duration)
    }

    fn notify_one(&self) {
        RawCondvar::notify_one(self);
    }

    fn notify_all(&self) {
        RawCondvar::notify_all(self);
    }
}
