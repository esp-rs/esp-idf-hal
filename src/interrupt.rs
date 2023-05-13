use core::sync::atomic::{AtomicU64, Ordering};

use enumset::{EnumSet, EnumSetType};
use esp_idf_sys::*;

/// Interrupt allocation flags.
/// These flags can be used to specify which interrupt qualities the code calling esp_intr_alloc* needs.
#[derive(Debug, EnumSetType)]
pub enum IntrFlags {
    // Accept a Level 1 interrupt vector (lowest priority)
    Level1,
    // Accept a Level 2 interrupt vector.
    Level2,
    // Accept a Level 3 interrupt vector.
    Level3,
    // Accept a Level 4 interrupt vector.
    Level4,
    // Accept a Level 5 interrupt vector.
    Level5,
    // Accept a Level 6 interrupt vector.
    Level6,
    // Accept a Level 7 interrupt vector (highest priority)
    Nmi,
    // Interrupt can be shared between ISRs.
    Shared,
    // Edge-triggered interrupt.
    Edge,
    // ISR can be called if cache is disabled.
    // Must be used with a proper option *_ISR_IN_IRAM in SDKCONFIG
    Iram,
    // Return with this interrupt disabled.
    IntrDisabled,
    // Low and medium prio interrupts. These can be handled in C.
    LowMed,
    // High level interrupts. Need to be handled in assembly.
    High,
}

impl IntrFlags {
    pub fn levels(&self) -> EnumSet<Self> {
        Self::Level1
            | Self::Level2
            | Self::Level3
            | Self::Level4
            | Self::Level5
            | Self::Level6
            | Self::Nmi
    }

    pub(crate) fn to_native(flags: EnumSet<Self>) -> u32 {
        let mut uflags: u32 = 0;
        for flag in flags {
            uflags |= u32::from(flag);
        }

        uflags
    }
}

impl From<IntrFlags> for u32 {
    fn from(flag: IntrFlags) -> Self {
        match flag {
            IntrFlags::Level1 => esp_idf_sys::ESP_INTR_FLAG_LEVEL1,
            IntrFlags::Level2 => esp_idf_sys::ESP_INTR_FLAG_LEVEL2,
            IntrFlags::Level3 => esp_idf_sys::ESP_INTR_FLAG_LEVEL3,
            IntrFlags::Level4 => esp_idf_sys::ESP_INTR_FLAG_LEVEL4,
            IntrFlags::Level5 => esp_idf_sys::ESP_INTR_FLAG_LEVEL5,
            IntrFlags::Level6 => esp_idf_sys::ESP_INTR_FLAG_LEVEL6,
            IntrFlags::Nmi => esp_idf_sys::ESP_INTR_FLAG_NMI,
            IntrFlags::Shared => esp_idf_sys::ESP_INTR_FLAG_SHARED,
            IntrFlags::Edge => esp_idf_sys::ESP_INTR_FLAG_EDGE,
            IntrFlags::Iram => esp_idf_sys::ESP_INTR_FLAG_IRAM,
            IntrFlags::IntrDisabled => esp_idf_sys::ESP_INTR_FLAG_INTRDISABLED,
            IntrFlags::LowMed => esp_idf_sys::ESP_INTR_FLAG_LOWMED,
            IntrFlags::High => esp_idf_sys::ESP_INTR_FLAG_HIGH,
        }
    }
}

pub(crate) static CS: IsrCriticalSection = IsrCriticalSection::new();

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
    let signaled = arg.cast::<bool>().as_mut().unwrap();

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
            let func: fn(*mut ()) = core::mem::transmute((value >> 32) as usize);
            let arg = (value & 0xffffffff) as usize as *mut ();
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
            ((func as usize as u64) << 32) | (arg as usize as u64)
        } else {
            0
        };

        let value = ISR_YIELDER.swap(value, Ordering::SeqCst);
        if value == 0 {
            None
        } else {
            let func: fn(*mut ()) = core::mem::transmute((value >> 32) as usize);
            let arg = (value & 0xffffffff) as usize as *mut ();
            Some((func, arg))
        }
    } else {
        None
    }
}

/// A critical section allows the user to disable interrupts
#[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
pub struct IsrCriticalSection(core::cell::UnsafeCell<portMUX_TYPE>);

#[cfg(not(any(esp32, esp32s2, esp32s3, esp32p4)))]
pub struct IsrCriticalSection(core::marker::PhantomData<*const ()>);

#[cfg(not(any(esp32, esp32s2, esp32s3, esp32p4)))]
#[inline(always)]
#[link_section = ".iram1.interrupt_enter"]
fn enter(_cs: &IsrCriticalSection) {
    unsafe {
        vPortEnterCritical();
    }
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
#[inline(always)]
#[link_section = ".iram1.interrupt_enter"]
fn enter(cs: &IsrCriticalSection) {
    #[cfg(esp_idf_version = "4.3")]
    unsafe {
        vPortEnterCritical(cs.0.get());
    }

    #[cfg(not(esp_idf_version = "4.3"))]
    unsafe {
        xPortEnterCriticalTimeout(cs.0.get(), portMUX_NO_TIMEOUT);
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3, esp32p4)))]
#[inline(always)]
#[link_section = ".iram1.interrupt_exit"]
fn exit(_cs: &IsrCriticalSection) {
    unsafe {
        vPortExitCritical();
    }
}

#[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
#[inline(always)]
#[link_section = ".iram1.interrupt_exit"]
fn exit(cs: &IsrCriticalSection) {
    unsafe {
        vPortExitCritical(cs.0.get());
    }
}

impl IsrCriticalSection {
    /// Constructs a new `IsrCriticalSection` instance
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_new"]
    pub const fn new() -> Self {
        #[cfg(any(esp32, esp32s2, esp32s3, esp32p4))]
        let mux = core::cell::UnsafeCell::new(portMUX_TYPE {
            owner: portMUX_FREE_VAL,
            count: 0,
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedFn: b"(never locked)",
            #[cfg(esp_idf_freertos_portmux_debug)]
            lastLockedLine: -1,
        });

        #[cfg(not(any(esp32, esp32s2, esp32s3, esp32p4)))]
        let mux = core::marker::PhantomData;

        Self(mux)
    }

    /// Disables all interrupts for the lifetime of the returned guard instance.
    /// This method supports nesting in that is safe to be called multiple times.
    /// This method is also safe to call from ISR routines.
    ///
    /// NOTE: On dual-core esp32* chips, interrupts will be disabled only on one of
    /// the cores (the one where `IsrCriticalSection::enter` is called), while the other
    /// core will continue its execution. Moreover, if the same `IsrCriticalSection` instance
    /// is shared across multiple threads, where some of these happen to be scheduled on
    /// the second core (which has its interrupts enabled), the second core will then spinlock
    /// (busy-wait) in `IsrCriticalSection::enter`, until the first CPU releases the critical
    /// section and re-enables its interrupts. The second core will then - in turn - disable
    /// its interrupts and own the spinlock.
    ///
    /// For more information, refer to https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html#critical-sections
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_enter"]
    pub fn enter(&self) -> IsrCriticalSectionGuard {
        enter(self);

        IsrCriticalSectionGuard(self)
    }
}

impl Default for IsrCriticalSection {
    #[inline(always)]
    #[link_section = ".iram1.interrupt_cs_default"]
    fn default() -> Self {
        Self::new()
    }
}

unsafe impl Send for IsrCriticalSection {}
unsafe impl Sync for IsrCriticalSection {}

pub struct IsrCriticalSectionGuard<'a>(&'a IsrCriticalSection);

impl<'a> Drop for IsrCriticalSectionGuard<'a> {
    /// Drops the critical section guard thus potentially re-enabling
    /// al interrupts for the currently active core.
    ///
    /// Note that - due to the fact that calling `IsrCriticalSection::enter`
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
    let _guard = CS.enter();

    f()
}

#[cfg(feature = "embassy-sync")]
pub mod embassy_sync {
    use core::marker::PhantomData;

    use embassy_sync::blocking_mutex::raw::RawMutex;

    /// A mutex that allows borrowing data across executors and interrupts.
    ///
    /// # Safety
    ///
    /// This mutex is safe to share between different executors and interrupts.
    pub struct IsrRawMutex {
        _phantom: PhantomData<()>,
    }
    unsafe impl Send for IsrRawMutex {}
    unsafe impl Sync for IsrRawMutex {}

    impl IsrRawMutex {
        /// Create a new `IsrRawMutex`.
        pub const fn new() -> Self {
            Self {
                _phantom: PhantomData,
            }
        }
    }

    unsafe impl RawMutex for IsrRawMutex {
        const INIT: Self = Self::new();

        fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
            super::free(f)
        }
    }
}
