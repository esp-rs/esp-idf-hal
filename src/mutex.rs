use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::ptr;
use core::time::Duration;

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

#[cfg(feature = "embedded-svc")]
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

pub struct Condvar(UnsafeCell<pthread_cond_t>);

impl Condvar {
    pub fn new() -> Self {
        let mut cond: pthread_cond_t = Default::default();

        let r = unsafe { pthread_cond_init(&mut cond as *mut _, ptr::null()) };
        debug_assert_eq!(r, 0);

        Self(UnsafeCell::new(cond))
    }

    pub fn wait<'a, T>(&self, guard: MutexGuard<'a, T>) -> MutexGuard<'a, T>
    where
        T: Send,
    {
        let r = unsafe { pthread_cond_wait(self.0.get(), guard.0 .0.get()) };
        debug_assert_eq!(r, 0);

        guard
    }

    pub fn wait_timeout<'a, T>(
        &self,
        guard: MutexGuard<'a, T>,
        duration: Duration,
    ) -> (MutexGuard<'a, T>, bool)
    where
        T: Send,
    {
        let abstime = timespec {
            tv_sec: duration.as_secs() as _,
            tv_nsec: duration.subsec_nanos() as _,
        };

        let r =
            unsafe { pthread_cond_timedwait(self.0.get(), guard.0 .0.get(), &abstime as *const _) };
        if r == ETIMEDOUT as i32 {
            (guard, true)
        } else {
            debug_assert_eq!(r, 0);

            (guard, false)
        }
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

impl Default for Condvar {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Condvar {
    fn drop(&mut self) {
        let r = unsafe { pthread_cond_destroy(self.0.get()) };
        debug_assert_eq!(r, 0);
    }
}

#[cfg(feature = "embedded-svc")]
impl embedded_svc::mutex::Condvar for Condvar {
    type Mutex<T>
    where
        T: Send,
    = Mutex<T>;

    #[inline(always)]
    fn new() -> Self {
        Condvar::new()
    }

    fn wait<'a, T>(
        &self,
        guard: <<Self as embedded_svc::mutex::Condvar>::Mutex<T> as embedded_svc::mutex::Mutex>::Guard<'a>,
    ) -> <<Self as embedded_svc::mutex::Condvar>::Mutex<T> as embedded_svc::mutex::Mutex>::Guard<'a>
    where
        T: Send,
    {
        Condvar::wait(self, guard)
    }

    fn wait_timeout<'a, T>(
        &self,
        guard: <<Self as embedded_svc::mutex::Condvar>::Mutex<T> as embedded_svc::mutex::Mutex>::Guard<'a>,
        duration: Duration,
    ) -> (
        <<Self as embedded_svc::mutex::Condvar>::Mutex<T> as embedded_svc::mutex::Mutex>::Guard<'a>,
        bool,
    )
    where
        T: Send,
    {
        Condvar::wait_timeout(self, guard, duration)
    }

    fn notify_one(&self) {
        Condvar::notify_one(self)
    }

    fn notify_all(&self) {
        Condvar::notify_all(self)
    }
}

#[cfg(feature = "embassy")]
pub mod embassy {
    pub enum EspMutexKind {}
    impl embassy::blocking_mutex::kind::MutexKind for EspMutexKind {
        type Mutex<T> = super::Mutex<T>;
    }

    impl<'a, T> embassy::blocking_mutex::Mutex for super::Mutex<T> {
        type Data = T;

        fn new(data: Self::Data) -> Self {
            super::Mutex::new(data)
        }

        #[inline(always)]
        fn lock<R>(&self, f: impl FnOnce(&Self::Data) -> R) -> R {
            let mut guard = super::Mutex::lock(self);

            f(&mut guard)
        }
    }
}
