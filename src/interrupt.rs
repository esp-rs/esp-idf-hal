use core::cell::{RefCell, RefMut};
use core::ops::{Deref, DerefMut};

use esp_idf_sys::*;

/// Returns true if the currently active core is executing an ISR request
#[inline(always)]
#[link_section = ".iram1.interrupt_active"]
pub fn active() -> bool {
    unsafe { xPortInIsrContext() != 0 }
}

/// A critical section allows the user to disable interrupts
#[cfg(not(any(esp32c3, esp32s2)))]
pub struct CriticalSection(core::cell::UnsafeCell<portMUX_TYPE>);

#[cfg(any(esp32c3, esp32s2))]
pub struct CriticalSection(core::marker::PhantomData<*const ()>);

impl CriticalSection {
    /// Constructs a new `CriticalSection` instance
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_new"]
    pub const fn new() -> Self {
        #[cfg(not(any(esp32c3, esp32s2)))]
        let mux = core::cell::UnsafeCell::new(portMUX_TYPE {
            owner: portMUX_FREE_VAL,
            count: 0,
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedFn: b"(never locked)",
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedLine: -1,
        });

        #[cfg(any(esp32c3, esp32s2))]
        let mux = core::marker::PhantomData;

        Self(mux)
    }

    /// Disables all interrupts for the lifetime of the returned guard instance.
    /// This method supports nesting in that is safe to be called multiple times.
    /// This method is also safe to call from ISR routines.
    ///
    /// NOTE: On dual-core esp32* chips, interrupts will be disabled only on one of
    /// the cores (the one where `CriticalSection::enter` is called), while the other
    /// core will continue its execution. Moreover, if the same `CriticalSection` instance
    /// is shared across multiple threads, where some of these happen to be scheduled on
    /// the second core (which has its interrupts enabled), the second core will then spinlock
    /// (busy-wait) in `CriticalSection::enter`, until the first CPU releases the critical
    /// section and re-enables its interrupts. The second core will then - in turn - disable
    /// its interrupts and own the spinlock.
    ///
    /// For more information, refer to https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#critical-sections
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_enter"]
    pub fn enter(&self) -> CriticalSectionGuard {
        #[cfg(any(esp32c3, esp32s2))]
        unsafe {
            vPortEnterCritical()
        };

        #[cfg(all(esp_idf_version = "4.3", not(any(esp32c3, esp32s2))))]
        unsafe {
            vPortEnterCritical(self.0.get())
        };

        #[cfg(all(not(esp_idf_version = "4.3"), not(any(esp32c3, esp32s2))))]
        unsafe {
            xPortEnterCriticalTimeout(self.0.get(), portMUX_NO_TIMEOUT)
        };

        CriticalSectionGuard(self)
    }
}

impl Default for CriticalSection {
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_default"]
    fn default() -> Self {
        Self::new()
    }
}

unsafe impl Send for CriticalSection {}
unsafe impl Sync for CriticalSection {}

pub struct CriticalSectionGuard<'a>(&'a CriticalSection);

impl<'a> Drop for CriticalSectionGuard<'a> {
    /// Drops the critical section guard thus potentially re-enabling
    /// al interrupts for the currently active core.
    ///
    /// Note that - due to the fact that calling `CriticalSection::enter`
    /// multiple times on the same or multiple critical sections is supported -
    /// interrupts for the core will be re-enabled only when the last guard that
    /// disabled interrupts for the concrete core is dropped.
    #[inline(always)]
    #[link_section = ".iram1.interrupt_csg_drop"]
    fn drop(&mut self) {
        #[cfg(any(esp32c3, esp32s2))]
        unsafe {
            vPortExitCritical()
        };

        #[cfg(not(any(esp32c3, esp32s2)))]
        unsafe {
            vPortExitCritical(self.0 .0.get())
        };
    }
}

/// Executes closure f in an interrupt-free context
#[inline(always)]
#[link_section = ".iram1.interrupt_free"]
pub fn free<R>(f: impl FnOnce() -> R) -> R {
    let cs = CriticalSection::new();
    let _guard = cs.enter();

    f()
}

/// A mutex based on critical sections
pub struct Mutex<T> {
    cs: CriticalSection,
    data: RefCell<T>,
}

impl<T> Mutex<T> {
    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutex_new"]
    pub const fn new(data: T) -> Self {
        Self {
            cs: CriticalSection::new(),
            data: RefCell::new(data),
        }
    }

    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutex_lock"]
    pub fn lock(&self) -> MutexGuard<'_, T> {
        MutexGuard::new(self)
    }
}

unsafe impl<T> Sync for Mutex<T> where T: Send {}
unsafe impl<T> Send for Mutex<T> where T: Send {}

pub struct MutexGuard<'a, T: 'a>(CriticalSectionGuard<'a>, RefMut<'a, T>);

impl<'a, T> MutexGuard<'a, T> {
    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutexg_new"]
    fn new(mutex: &'a Mutex<T>) -> Self {
        Self(mutex.cs.enter(), mutex.data.borrow_mut())
    }
}

unsafe impl<T> Sync for MutexGuard<'_, T> where T: Sync {}

impl<'a, T> Deref for MutexGuard<'a, T> {
    type Target = T;

    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutexg_deref"]
    fn deref(&self) -> &Self::Target {
        &*self.1
    }
}

impl<'a, T> DerefMut for MutexGuard<'a, T> {
    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutexg_derefmut"]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.1
    }
}

#[cfg(feature = "mutex-trait")]
impl<'a, T> mutex_trait::Mutex for &'a Mutex<T> {
    type Data = T;

    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutext_lock"]
    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let mut guard = Mutex::lock(self);

        f(&mut guard)
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
    #[link_section = ".iram1.interrupt_mutexe_new"]
    fn new(data: Self::Data) -> Self {
        Mutex::new(data)
    }

    #[inline(always)]
    #[link_section = ".iram1.interrupt_mutexe_lock"]
    fn lock(&self) -> Self::Guard<'_> {
        Mutex::lock(self)
    }
}
