use core::cell::{RefCell, RefMut};
use core::ops::{Deref, DerefMut};
use core::ptr;
use core::sync::atomic::{AtomicPtr, Ordering};

use esp_idf_sys::*;

/// Returns true if the currently active core is executing an ISR request
#[inline(always)]
#[link_section = ".iram1.interrupt_active"]
pub fn active() -> bool {
    unsafe { xPortInIsrContext() != 0 }
}

static ISR_YIELDER: AtomicPtr<unsafe fn()> = AtomicPtr::new(ptr::null_mut());

#[inline(always)]
#[link_section = ".iram1.interrupt_get_isr_yielder"]
unsafe fn get_isr_yielder() -> Option<unsafe fn()> {
    if active() {
        ISR_YIELDER.load(Ordering::SeqCst).as_ref().copied()
    } else {
        None
    }
}

/// # Safety
///
/// This function should only be called from within an ISR handler, so as to set
/// a custom ISR yield function (e.g. when using the ESP-IDF timer service).
///
/// Thus, if some function further down the ISR call chain invokes `do_yield`,
/// the custom yield function set here will be called.
///
/// Users should not forget to call again `set_isr_yielder` at the end of the
/// ISR handler so as to reastore the yield function which was valid before the
/// ISR handler was invoked.
#[inline(always)]
#[link_section = ".iram1.interrupt_set_isr_yielder"]
pub unsafe fn set_isr_yielder(yielder: Option<unsafe fn()>) -> Option<unsafe fn()> {
    if active() {
        let yielder = if let Some(yielder) = yielder {
            &yielder as *const _ as *mut _
        } else {
            ptr::null_mut()
        };

        ISR_YIELDER
            .swap(yielder, Ordering::SeqCst)
            .as_ref()
            .copied()
    } else {
        None
    }
}

pub mod task {
    use core::ptr;
    use core::time::Duration;

    use esp_idf_sys::*;

    use crate::delay::TickType;

    #[inline(always)]
    #[link_section = ".iram1.interrupt_task_do_yield"]
    pub fn do_yield() {
        if super::active() {
            #[cfg(esp32c3)]
            unsafe {
                if let Some(yielder) = super::get_isr_yielder() {
                    yielder();
                } else {
                    vPortYieldFromISR();
                }
            }

            #[cfg(not(esp32c3))]
            unsafe {
                if let Some(yielder) = super::get_isr_yielder() {
                    yielder();
                } else {
                    vPortEvaluateYieldFromISR(0);
                }
            }
        } else {
            unsafe {
                vPortYield();
            }
        }
    }

    #[inline(always)]
    #[link_section = ".iram1.interrupt_task_current"]
    pub fn current() -> Option<TaskHandle_t> {
        if super::active() {
            None
        } else {
            Some(unsafe { xTaskGetCurrentTaskHandle() })
        }
    }

    pub fn wait_any_notification() {
        loop {
            if let Some(notification) = wait_notification(None) {
                if notification != 0 {
                    break;
                }
            }
        }
    }

    pub fn wait_notification(duration: Option<Duration>) -> Option<u32> {
        let mut notification = 0_u32;

        #[cfg(esp_idf_version = "4.3")]
        let notified = unsafe {
            xTaskNotifyWait(
                0,
                u32::MAX,
                &mut notification as *mut _,
                TickType::from(duration).0,
            )
        } != 0;

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = unsafe {
            xTaskGenericNotifyWait(
                0,
                0,
                u32::MAX,
                &mut notification as *mut _,
                TickType::from(duration).0,
            )
        } != 0;

        if notified {
            Some(notification)
        } else {
            None
        }
    }

    /// # Safety
    ///
    /// When calling this function care should be taken to pass a valid
    /// FreeRTOS task handle. Moreover, the FreeRTOS task should be valid
    /// when this function is being called.
    pub unsafe fn notify(task: TaskHandle_t, notification: u32) -> bool {
        let notified = if super::active() {
            let mut higher_prio_task_woken: BaseType_t = Default::default();

            #[cfg(esp_idf_version = "4.3")]
            let notified = xTaskGenericNotifyFromISR(
                task,
                notification,
                eNotifyAction_eSetBits,
                ptr::null_mut(),
                &mut higher_prio_task_woken as *mut _,
            );

            #[cfg(not(esp_idf_version = "4.3"))]
            let notified = xTaskGenericNotifyFromISR(
                task,
                0,
                notification,
                eNotifyAction_eSetBits,
                ptr::null_mut(),
                &mut higher_prio_task_woken as *mut _,
            );

            if higher_prio_task_woken != 0 {
                do_yield();
            }

            notified
        } else {
            #[cfg(esp_idf_version = "4.3")]
            let notified =
                xTaskGenericNotify(task, notification, eNotifyAction_eSetBits, ptr::null_mut());

            #[cfg(not(esp_idf_version = "4.3"))]
            let notified = xTaskGenericNotify(
                task,
                0,
                notification,
                eNotifyAction_eSetBits,
                ptr::null_mut(),
            );

            notified
        };

        notified != 0
    }
}

/// A critical section allows the user to disable interrupts
#[cfg(not(esp32c3))]
pub struct CriticalSection(core::cell::UnsafeCell<portMUX_TYPE>);

#[cfg(esp32c3)]
pub struct CriticalSection(core::marker::PhantomData<*const ()>);

#[cfg(esp32c3)]
#[inline(always)]
#[link_section = ".iram1.interrupt_enter"]
fn enter(_cs: &CriticalSection) {
    unsafe {
        vPortEnterCritical();
    }
}

#[cfg(not(esp32c3))]
#[inline(always)]
#[link_section = ".iram1.interrupt_enter"]
fn enter(cs: &CriticalSection) {
    #[cfg(esp_idf_version = "4.3")]
    unsafe {
        vPortEnterCritical(cs.0.get());
    }

    #[cfg(not(esp_idf_version = "4.3"))]
    unsafe {
        xPortEnterCriticalTimeout(cs.0.get(), portMUX_NO_TIMEOUT);
    }
}

#[cfg(esp32c3)]
#[inline(always)]
#[link_section = ".iram1.interrupt_exit"]
fn exit(_cs: &CriticalSection) {
    unsafe {
        vPortExitCritical();
    }
}

#[cfg(not(esp32c3))]
#[inline(always)]
#[link_section = ".iram1.interrupt_exit"]
fn exit(cs: &CriticalSection) {
    unsafe {
        vPortExitCritical(cs.0.get());
    }
}

impl CriticalSection {
    /// Constructs a new `CriticalSection` instance
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_new"]
    pub const fn new() -> Self {
        #[cfg(not(esp32c3))]
        let mux = core::cell::UnsafeCell::new(portMUX_TYPE {
            owner: portMUX_FREE_VAL,
            count: 0,
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedFn: b"(never locked)",
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedLine: -1,
        });

        #[cfg(esp32c3)]
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
        enter(self);

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
        exit(self.0);
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

#[cfg(feature = "critical-section")]
mod embassy_cs {
    static CS: super::CriticalSection = super::CriticalSection::new();

    struct EmbassyCriticalSectionImpl {}
    critical_section::custom_impl!(EmbassyCriticalSectionImpl);

    unsafe impl critical_section::Impl for EmbassyCriticalSectionImpl {
        unsafe fn acquire() -> u8 {
            super::enter(&CS);
            return 1;
        }

        unsafe fn release(token: u8) {
            if token != 0 {
                super::exit(&CS);
            }
        }
    }
}

#[cfg(feature = "embassy")]
pub mod embassy {
    pub enum CriticalSectionMutexKind {}
    impl embassy::blocking_mutex::kind::MutexKind for CriticalSectionMutexKind {
        type Mutex<T> = super::Mutex<T>;
    }

    impl<'a, T> embassy::blocking_mutex::Mutex for super::Mutex<T> {
        type Data = T;

        fn new(data: Self::Data) -> Self {
            super::Mutex::new(data)
        }

        #[inline(always)]
        #[link_section = ".iram1.interrupt_embmutex_lock"]
        fn lock<R>(&self, f: impl FnOnce(&Self::Data) -> R) -> R {
            let mut guard = super::Mutex::lock(self);

            f(&mut guard)
        }
    }
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

#[cfg(feature = "embedded-svc")]
pub struct MutexFamily;

#[cfg(feature = "embedded-svc")]
impl embedded_svc::mutex::MutexFamily for MutexFamily {
    type Mutex<T> = Mutex<T>;
}

#[cfg(feature = "embedded-svc")]
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
