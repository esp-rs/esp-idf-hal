use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

use esp_idf_sys::*;

use crate::interrupt;

pub struct CriticalSection(UnsafeCell<MaybeUninit<StaticQueue_t>>, AtomicBool);

// Not available in the esp-idf-sys bindings
const QUEUE_TYPE_RECURSIVE_MUTEX: u8 = 4;

#[inline(always)]
#[link_section = ".iram1.cs_enter"]
fn enter(cs: &CriticalSection) {
    if !cs.1.load(Ordering::SeqCst) {
        interrupt::free(|| {
            if !cs.1.load(Ordering::SeqCst) {
                unsafe {
                    xQueueCreateMutexStatic(
                        QUEUE_TYPE_RECURSIVE_MUTEX,
                        cs.0.get().as_mut().unwrap().as_mut_ptr(),
                    );
                }
                cs.1.store(true, Ordering::SeqCst);
            }
        });
    }

    unsafe {
        xQueueTakeMutexRecursive(
            cs.0.get().as_mut().unwrap().as_mut_ptr() as *mut _,
            crate::delay::BLOCK,
        );
    }
}

#[inline(always)]
#[link_section = ".iram1.cs_exit"]
fn exit(cs: &CriticalSection) {
    if !cs.1.load(Ordering::SeqCst) {
        panic!("Called exit() without matching enter()");
    }

    unsafe {
        xQueueGiveMutexRecursive(cs.0.get().as_mut().unwrap().as_mut_ptr() as *mut _);
    }
}

impl CriticalSection {
    /// Constructs a new `CriticalSection` instance
    #[inline(always)]
    #[link_section = ".iram1.cs_new"]
    pub const fn new() -> Self {
        Self(
            UnsafeCell::new(MaybeUninit::uninit()),
            AtomicBool::new(false),
        )
    }

    #[inline(always)]
    #[link_section = ".iram1.cs_enter"]
    pub fn enter(&self) -> CriticalSectionGuard {
        enter(self);

        CriticalSectionGuard(self)
    }
}

impl Default for CriticalSection {
    #[inline(always)]
    #[link_section = ".iram1.cs_default"]
    fn default() -> Self {
        Self::new()
    }
}

unsafe impl Send for CriticalSection {}
unsafe impl Sync for CriticalSection {}

pub struct CriticalSectionGuard<'a>(&'a CriticalSection);

impl<'a> Drop for CriticalSectionGuard<'a> {
    #[inline(always)]
    #[link_section = ".iram1.csg_drop"]
    fn drop(&mut self) {
        exit(self.0);
    }
}

#[cfg(feature = "critical-section-mutex")]
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
