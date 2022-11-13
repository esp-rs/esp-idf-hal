use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;

use esp_idf_sys::*;

use crate::delay::TickType;
use crate::interrupt;

#[inline(always)]
#[link_section = ".iram1.interrupt_task_do_yield"]
pub fn do_yield() {
    if interrupt::active() {
        unsafe {
            if let Some((yielder, arg)) = interrupt::get_isr_yielder() {
                yielder(arg);
            } else {
                #[cfg(esp32c3)]
                vPortYieldFromISR();

                #[cfg(all(not(esp32c3), esp_idf_version_major = "4"))]
                vPortEvaluateYieldFromISR(0);

                #[cfg(all(not(esp32c3), not(esp_idf_version_major = "4")))]
                _frxt_setup_switch();
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
    if interrupt::active() {
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
    let notified = if interrupt::active() {
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

#[cfg(esp_idf_comp_pthread_enabled)]
pub mod thread {
    use esp_idf_sys::*;

    use crate::cpu::Core;

    #[derive(Debug)]
    pub struct ThreadSpawnConfiguration {
        pub name: Option<&'static [u8]>,
        pub stack_size: usize,
        pub priority: u8,
        pub inherit: bool,
        pub pin_to_core: Option<Core>,
    }

    impl ThreadSpawnConfiguration {
        pub fn get() -> Option<Self> {
            get_conf()
        }

        pub fn set(&self) -> Result<(), EspError> {
            set_conf(self)
        }
    }

    impl Default for ThreadSpawnConfiguration {
        fn default() -> Self {
            get_default_conf()
        }
    }

    impl From<&ThreadSpawnConfiguration> for esp_pthread_cfg_t {
        fn from(conf: &ThreadSpawnConfiguration) -> Self {
            Self {
                thread_name: conf
                    .name
                    .map(|name| name.as_ptr() as _)
                    .unwrap_or(core::ptr::null()),
                stack_size: conf.stack_size as _,
                prio: conf.priority as _,
                inherit_cfg: conf.inherit,
                pin_to_core: conf
                    .pin_to_core
                    .map(Into::into)
                    .unwrap_or(tskNO_AFFINITY as _),
            }
        }
    }

    impl From<esp_pthread_cfg_t> for ThreadSpawnConfiguration {
        fn from(conf: esp_pthread_cfg_t) -> Self {
            Self {
                name: if conf.thread_name.is_null() {
                    None
                } else {
                    Some(unsafe {
                        core::slice::from_raw_parts(
                            conf.thread_name as _,
                            c_strlen(conf.thread_name) + 1,
                        )
                    })
                },
                stack_size: conf.stack_size as _,
                priority: conf.prio as _,
                inherit: conf.inherit_cfg,
                pin_to_core: if conf.pin_to_core == tskNO_AFFINITY as _ {
                    None
                } else {
                    Some(conf.pin_to_core.into())
                },
            }
        }
    }

    fn get_default_conf() -> ThreadSpawnConfiguration {
        unsafe { esp_pthread_get_default_config() }.into()
    }

    fn get_conf() -> Option<ThreadSpawnConfiguration> {
        let mut conf: esp_pthread_cfg_t = Default::default();

        let res = unsafe { esp_pthread_get_cfg(&mut conf as _) };

        if res == ESP_ERR_NOT_FOUND {
            None
        } else {
            Some(conf.into())
        }
    }

    fn set_conf(conf: &ThreadSpawnConfiguration) -> Result<(), EspError> {
        esp!(unsafe { esp_pthread_set_cfg(&conf.into()) })?;

        Ok(())
    }

    fn c_strlen(c_str: *const i8) -> usize {
        let mut offset = 0;

        loop {
            if *unsafe { c_str.offset(offset).as_ref() }.unwrap() == 0 {
                return offset as _;
            }

            offset += 1;
        }
    }
}

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

#[cfg(feature = "critical-section")]
pub mod critical_section {
    static CS: super::CriticalSection = super::CriticalSection::new();

    pub struct EspCriticalSection {}

    unsafe impl critical_section::Impl for EspCriticalSection {
        unsafe fn acquire() {
            super::enter(&CS);
        }

        unsafe fn release(_token: ()) {
            super::exit(&CS);
        }
    }

    pub type LinkWorkaround = [*mut (); 2];

    static mut __INTERNAL_REFERENCE: LinkWorkaround = [
        _critical_section_1_0_acquire as *mut _,
        _critical_section_1_0_release as *mut _,
    ];

    pub fn link() -> LinkWorkaround {
        unsafe { __INTERNAL_REFERENCE }
    }

    critical_section::set_impl!(EspCriticalSection);
}

#[cfg(feature = "embassy-sync")]
pub mod embassy_sync {
    use embassy_sync::blocking_mutex::raw::RawMutex;

    /// A mutex that allows borrowing data across executors but NOT accross interrupts.
    ///
    /// # Safety
    ///
    /// This mutex is safe to share between different executors.
    pub struct EspRawMutex(super::CriticalSection);

    unsafe impl Send for EspRawMutex {}
    unsafe impl Sync for EspRawMutex {}

    impl EspRawMutex {
        /// Create a new `EspRawMutex`.
        pub const fn new() -> Self {
            Self(super::CriticalSection::new())
        }
    }

    unsafe impl RawMutex for EspRawMutex {
        const INIT: Self = Self::new();

        fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
            let _guard = self.0.enter();

            f()
        }
    }
}

#[cfg(all(
    feature = "edge-executor",
    feature = "alloc",
    target_has_atomic = "ptr"
))]
pub mod executor {
    use core::sync::atomic::{AtomicPtr, Ordering};
    use core::{mem, ptr};

    extern crate alloc;
    use alloc::sync::{Arc, Weak};

    use crate::task;

    pub use edge_executor::*;

    pub type EspExecutor<'a, const C: usize, S> = Executor<'a, C, FreeRtosMonitor, S>;
    pub type EspBlocker = Blocker<FreeRtosMonitor>;

    pub struct FreeRtosMonitor(Arc<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>, *const ());

    impl FreeRtosMonitor {
        pub fn new() -> Self {
            Self(
                Arc::new(AtomicPtr::new(task::current().unwrap())),
                core::ptr::null(),
            )
        }
    }

    impl Default for FreeRtosMonitor {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Drop for FreeRtosMonitor {
        fn drop(&mut self) {
            let mut arc = mem::replace(&mut self.0, Arc::new(AtomicPtr::new(ptr::null_mut())));

            // Busy loop until we can destroy the Arc - which means that nobody is actively holding a strong reference to it
            // and thus trying to notify our FreeRtos task, which will likely be destroyed afterwards
            loop {
                arc = match Arc::try_unwrap(arc) {
                    Ok(_) => break,
                    Err(a) => a,
                }
            }
        }
    }

    impl Monitor for FreeRtosMonitor {
        type Notify = FreeRtosMonitorNotify;

        fn notifier(&self) -> Self::Notify {
            FreeRtosMonitorNotify(Arc::downgrade(&self.0))
        }
    }

    impl Wait for FreeRtosMonitor {
        fn wait(&self) {
            task::wait_any_notification();
        }
    }

    pub struct FreeRtosMonitorNotify(Weak<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>);

    impl Notify for FreeRtosMonitorNotify {
        fn notify(&self) {
            if let Some(notify) = self.0.upgrade() {
                let freertos_task = notify.load(Ordering::SeqCst);

                if !freertos_task.is_null() {
                    unsafe {
                        task::notify(freertos_task, 1);
                    }
                }
            }
        }
    }
}
