use core::cell::{RefCell, RefMut};
use core::ops::{Deref, DerefMut};

pub struct Mutex<T>(RefCell<T>);

impl<T> Mutex<T> {
    #[inline(always)]
    pub const fn new(data: T) -> Self {
        Self(RefCell::new(data))
    }

    #[inline(always)]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        MutexGuard::new(self)
    }
}

unsafe impl<T> Sync for Mutex<T> where T: Send {}
unsafe impl<T> Send for Mutex<T> where T: Send {}

pub struct MutexGuard<'a, T>(&'a Mutex<T>, RefMut<'a, T>);

impl<'a, T> MutexGuard<'a, T> {
    #[inline(always)]
    fn new(mutex: &'a Mutex<T>) -> Self {
        Self(mutex, mutex.0.borrow_mut())
    }
}

unsafe impl<T> Sync for MutexGuard<'_, T> where T: Sync {}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.1
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.1
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
