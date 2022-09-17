use core::sync::atomic::{AtomicU64, Ordering};

use esp_idf_sys::*;

/// Returns true if the currently active core is executing an ISR request
#[inline(always)]
#[link_section = ".iram1.interrupt_active"]
pub fn active() -> bool {
    unsafe { xPortInIsrContext() != 0 }
}

pub fn with_isr_yield_signal(cb: impl FnOnce()) -> bool {
    if !active() {
        panic!("with_isr_yield_signal() can only be called from an ISR context");
    }

    let mut signaled = false;

    let prev_yielder =
        unsafe { set_isr_yielder(Some((do_yield_signal, &mut signaled as *mut _ as _))) };

    cb();

    unsafe { set_isr_yielder(prev_yielder) };

    signaled
}

unsafe fn do_yield_signal(arg: *mut ()) {
    let signaled = (arg as *mut bool).as_mut().unwrap();

    *signaled = true
}

static ISR_YIELDER: AtomicU64 = AtomicU64::new(0);

#[allow(clippy::type_complexity)]
#[inline(always)]
#[link_section = ".iram1.interrupt_get_isr_yielder"]
pub(crate) unsafe fn get_isr_yielder() -> Option<(unsafe fn(*mut ()), *mut ())> {
    if active() {
        let value = ISR_YIELDER.load(Ordering::SeqCst);
        if value == 0 {
            None
        } else {
            let func = core::mem::transmute((value >> 32) as u32);
            let arg = (value & 0xffffffff) as u32 as *mut ();
            Some((func, arg))
        }
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
#[allow(clippy::type_complexity)]
#[inline(always)]
#[link_section = ".iram1.interrupt_set_isr_yielder"]
pub unsafe fn set_isr_yielder(
    yielder: Option<(unsafe fn(*mut ()), *mut ())>,
) -> Option<(unsafe fn(*mut ()), *mut ())> {
    if active() {
        let value = if let Some((func, arg)) = yielder {
            ((func as usize as u64) << 32) | (arg as u32 as u64)
        } else {
            0
        };

        let value = ISR_YIELDER.swap(value, Ordering::SeqCst);
        if value == 0 {
            None
        } else {
            let func = core::mem::transmute((value >> 32) as u32);
            let arg = (value & 0xffffffff) as u32 as *mut ();
            Some((func, arg))
        }
    } else {
        None
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

#[cfg(feature = "critical-section-interrupt")]
mod critical_section {
    static CS: super::CriticalSection = super::CriticalSection::new();

    struct EspCriticalSection {}

    critical_section::set_impl!(EspCriticalSection);

    unsafe impl critical_section::Impl for EspCriticalSection {
        unsafe fn acquire() {
            super::enter(&CS);
        }

        unsafe fn release(_token: ()) {
            super::exit(&CS);
        }
    }
}
