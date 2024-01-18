use core::cell::Cell;
#[cfg(feature = "alloc")]
use core::future::Future;
use core::num::NonZeroU32;
#[cfg(feature = "alloc")]
use core::pin::pin;
use core::ptr::{self, NonNull};
use core::sync::atomic::{AtomicBool, Ordering};
#[cfg(feature = "alloc")]
use core::task::{Context, Poll};

#[cfg(feature = "alloc")]
extern crate alloc;

use esp_idf_sys::*;

use crate::cpu::Core;
use crate::interrupt;

#[cfg(not(any(
    esp_idf_version_major = "4",
    esp_idf_version = "5.0",
    esp_idf_version = "5.1"
)))]
const NO_AFFINITY: core::ffi::c_int = CONFIG_FREERTOS_NO_AFFINITY as _;

#[cfg(any(
    esp_idf_version_major = "4",
    esp_idf_version = "5.0",
    esp_idf_version = "5.1"
))]
const NO_AFFINITY: core::ffi::c_uint = tskNO_AFFINITY;

/// Creates a FreeRTOS task.
///
/// This API is to be used only for niche use cases like where the `std` feature is not enabled, or one absolutely
/// needs to create a raw FreeRTOS task.
///
/// In all other cases, the standard, safe Rust `std::thread` API should be utilized, as it is anyway
/// a thin wrapper around the FreeRTOS task API.
///
/// # Safety
///
/// Only marked as unsafe fo symmetry with `destroy` and to discourage users from leaning on it in favor of `std::thread`.
/// Otherwise, this function is actually safe.
pub unsafe fn create(
    task_handler: extern "C" fn(*mut core::ffi::c_void),
    task_name: &core::ffi::CStr,
    stack_size: usize,
    task_arg: *mut core::ffi::c_void,
    priority: u8,
    pin_to_core: Option<Core>,
) -> Result<TaskHandle_t, EspError> {
    let mut task: TaskHandle_t = core::ptr::null_mut();

    let created = xTaskCreatePinnedToCore(
        Some(task_handler),
        task_name.as_ptr(),
        stack_size as _,
        task_arg,
        priority as _,
        &mut task,
        pin_to_core.map(Into::into).unwrap_or(NO_AFFINITY as _),
    );

    if created == 0 {
        Err(EspError::from_infallible::<ESP_FAIL>())
    } else {
        Ok(task)
    }
}

/// Deletes a FreeRTOS task.
///
/// This API is to be used only for niche use cases like where the `std` feature is not enabled, or one absolutely
/// needs to create a raw FreeRTOS task.
///
/// In all other cases, the standard, safe Rust `std::thread` API should be utilized, as it is anyway
/// a thin wrapper around the FreeRTOS task API.
///
/// # Safety
///
/// A valid `TaskHandle_t` instance of an existing task should be provided.
/// Providing a `TaskHandle_t` of a task which was already destroyed is an undefined behavior.
pub unsafe fn destroy(task: TaskHandle_t) {
    vTaskDelete(task)
}

#[inline(always)]
#[link_section = ".iram1.interrupt_task_do_yield"]
pub fn do_yield() {
    if interrupt::active() {
        unsafe {
            if let Some((yielder, arg)) = interrupt::get_isr_yielder() {
                yielder(arg);
            } else {
                #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6))]
                vPortYieldFromISR();

                #[cfg(all(
                    not(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6)),
                    esp_idf_version_major = "4"
                ))]
                vPortEvaluateYieldFromISR(0);

                #[cfg(all(
                    not(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6)),
                    not(esp_idf_version_major = "4")
                ))]
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

pub fn wait_notification(timeout: TickType_t) -> Option<NonZeroU32> {
    let mut notification = 0_u32;

    #[cfg(esp_idf_version = "4.3")]
    #[deprecated(
        note = "Using ESP-IDF 4.3 is untested, please upgrade to 4.4 or newer. Support will be removed in the next major release."
    )]
    let notified = unsafe { xTaskNotifyWait(0, u32::MAX, &mut notification, timeout) } != 0;

    #[cfg(not(esp_idf_version = "4.3"))]
    let notified =
        unsafe { xTaskGenericNotifyWait(0, 0, u32::MAX, &mut notification, timeout) } != 0;

    if notified {
        NonZeroU32::new(notification)
    } else {
        None
    }
}

/// # Safety
///
/// When calling this function care should be taken to pass a valid
/// FreeRTOS task handle. Moreover, the FreeRTOS task should be valid
/// when this function is being called.
pub unsafe fn notify_and_yield(task: TaskHandle_t, notification: NonZeroU32) -> bool {
    let (notified, higher_prio_task_woken) = notify(task, notification);

    if higher_prio_task_woken {
        do_yield();
    }

    notified
}

/// # Safety
///
/// When calling this function care should be taken to pass a valid
/// FreeRTOS task handle. Moreover, the FreeRTOS task should be valid
/// when this function is being called.
pub unsafe fn notify(task: TaskHandle_t, notification: NonZeroU32) -> (bool, bool) {
    let (notified, higher_prio_task_woken) = if interrupt::active() {
        let mut higher_prio_task_woken: BaseType_t = Default::default();

        #[cfg(esp_idf_version = "4.3")]
        #[deprecated(
            note = "Using ESP-IDF 4.3 is untested, please upgrade to 4.4 or newer. Support will be removed in the next major release."
        )]
        let notified = xTaskGenericNotifyFromISR(
            task,
            notification.into(),
            eNotifyAction_eSetBits,
            ptr::null_mut(),
            &mut higher_prio_task_woken,
        );

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = xTaskGenericNotifyFromISR(
            task,
            0,
            notification.into(),
            eNotifyAction_eSetBits,
            ptr::null_mut(),
            &mut higher_prio_task_woken,
        );

        (notified, higher_prio_task_woken)
    } else {
        #[cfg(esp_idf_version = "4.3")]
        #[deprecated(
            note = "Using ESP-IDF 4.3 is untested, please upgrade to 4.4 or newer. Support will be removed in the next major release."
        )]
        let notified = xTaskGenericNotify(
            task,
            notification.into(),
            eNotifyAction_eSetBits,
            ptr::null_mut(),
        );

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = xTaskGenericNotify(
            task,
            0,
            notification.into(),
            eNotifyAction_eSetBits,
            ptr::null_mut(),
        );

        (notified, 0)
    };

    (notified != 0, higher_prio_task_woken != 0)
}

pub fn get_idle_task(core: crate::cpu::Core) -> TaskHandle_t {
    #[cfg(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6))]
    {
        if matches!(core, crate::cpu::Core::Core0) {
            unsafe { xTaskGetIdleTaskHandle() }
        } else {
            unreachable!()
        }
    }

    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6)))]
    #[cfg(any(
        esp_idf_version_major = "4",
        esp_idf_version = "5.0",
        esp_idf_version = "5.1"
    ))]
    unsafe {
        xTaskGetIdleTaskHandleForCPU(core as u32)
    }

    #[cfg(not(any(esp32c3, esp32c2, esp32h2, esp32c5, esp32c6)))]
    #[cfg(not(any(
        esp_idf_version_major = "4",
        esp_idf_version = "5.0",
        esp_idf_version = "5.1"
    )))]
    unsafe {
        xTaskGetIdleTaskHandleForCore(core as i32)
    }
}

/// Executes the supplied future on the current thread, thus blocking it until the future becomes ready.
#[cfg(feature = "alloc")]
pub fn block_on<F>(mut fut: F) -> F::Output
where
    F: Future,
{
    ::log::trace!("block_on(): started");

    let notification = notification::Notification::new();

    let mut fut = pin!(fut);

    let waker = notification.notifier().into();

    let mut cx = Context::from_waker(&waker);

    let res = loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(res) => break res,
            Poll::Pending => notification.wait_any(),
        }
    };

    ::log::trace!("block_on(): finished");

    res
}

#[cfg(esp_idf_comp_pthread_enabled)]
pub mod thread {
    use core::ffi::CStr;

    use esp_idf_sys::*;

    use super::NO_AFFINITY;

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
                pin_to_core: conf.pin_to_core.map(Into::into).unwrap_or(NO_AFFINITY as _),
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
                            c_strlen(conf.thread_name.cast()) + 1,
                        )
                    })
                },
                stack_size: conf.stack_size as _,
                priority: conf.prio as _,
                inherit: conf.inherit_cfg,
                pin_to_core: if conf.pin_to_core == NO_AFFINITY as _ {
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
        if let Some(name) = conf.name {
            let _str = CStr::from_bytes_with_nul(name)
                .map_err(|_e| panic! {"Missing null byte in provided Thread-Name"});
        }

        if conf.priority < 1 || conf.priority as u32 >= configMAX_PRIORITIES {
            panic!("Thread priority {} has to be [1 - 24]", conf.priority);
        }

        esp!(unsafe { esp_pthread_set_cfg(&conf.into()) })?;

        Ok(())
    }

    fn c_strlen(c_str: *const u8) -> usize {
        let mut offset = 0;

        loop {
            if *unsafe { c_str.offset(offset).as_ref() }.unwrap() == 0 {
                return offset as _;
            }

            offset += 1;
        }
    }
}

pub struct CriticalSection(Cell<Option<NonNull<QueueDefinition>>>, AtomicBool);

// Not available in the esp-idf-sys bindings
const QUEUE_TYPE_RECURSIVE_MUTEX: u8 = 4;

#[inline(always)]
#[link_section = ".iram1.cs_enter"]
fn enter(cs: &CriticalSection) {
    if !cs.1.load(Ordering::SeqCst) {
        interrupt::free(|| {
            if !cs.1.load(Ordering::SeqCst) {
                let ptr = unsafe { xQueueCreateMutex(QUEUE_TYPE_RECURSIVE_MUTEX) };
                cs.0.set(NonNull::new(ptr));
                cs.1.store(true, Ordering::SeqCst);
            }
        });
    }

    let res =
        unsafe { xQueueTakeMutexRecursive(cs.0.get().unwrap().as_ptr(), crate::delay::BLOCK) } != 0;

    if !res {
        unreachable!();
    }
}

#[inline(always)]
#[link_section = ".iram1.cs_exit"]
fn exit(cs: &CriticalSection) {
    if !cs.1.load(Ordering::SeqCst) {
        panic!("Called exit() without matching enter()");
    }

    let res = unsafe { xQueueGiveMutexRecursive(cs.0.get().unwrap().as_ptr()) } != 0;

    if !res {
        unreachable!();
    }
}

impl CriticalSection {
    /// Constructs a new `CriticalSection` instance
    #[inline(always)]
    #[link_section = ".iram1.cs_new"]
    pub const fn new() -> Self {
        Self(Cell::new(None), AtomicBool::new(false))
    }

    #[inline(always)]
    #[link_section = ".iram1.cs_enter"]
    pub fn enter(&self) -> CriticalSectionGuard {
        enter(self);

        CriticalSectionGuard(self)
    }
}

impl Drop for CriticalSection {
    fn drop(&mut self) {
        if self.1.load(Ordering::SeqCst) {
            unsafe {
                vQueueDelete(self.0.get().unwrap().as_ptr());
            }
        }
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

#[cfg(all(
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
    any(
        all(
            not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
            esp_idf_esp_task_wdt_en
        ),
        any(esp_idf_version_major = "4", esp_idf_version = "5.0")
    )
))]
pub mod watchdog {
    //! ## Example
    //!
    //! ```rust, ignore
    //! # fn main() -> Result<()> {
    //! let peripherals = Peripherals::take().unwrap();
    //!
    //! let config = TWDTConfig {
    //!     duration: Duration::from_secs(2),
    //!     panic_on_trigger: true,
    //!     subscribed_idle_tasks: enum_set!(Core::Core0)
    //! };
    //! let mut driver = esp_idf_hal::task::watchdog::TWDTDriver::new(
    //!     peripherals.twdt,
    //!     &config,
    //! )?;
    //!
    //! let mut watchdog = driver.watch_current_task()?;
    //!
    //! loop {
    //!     watchdog.feed();
    //!     unsafe { vTaskDelay(1) };
    //! }
    //! # }
    //! ```

    use core::{
        marker::PhantomData,
        sync::atomic::{AtomicUsize, Ordering},
    };

    use esp_idf_sys::*;

    use crate::peripheral::Peripheral;

    pub type TWDTConfig = config::Config;

    pub mod config {

        #[cfg(not(esp_idf_version_major = "4"))]
        use esp_idf_sys::*;

        #[derive(Clone)]
        pub struct Config {
            pub duration: core::time::Duration,
            pub panic_on_trigger: bool,
            pub subscribed_idle_tasks: enumset::EnumSet<crate::cpu::Core>,
        }

        impl Config {
            // Could be const if enumset operations are const
            pub fn new() -> Self {
                #[cfg(esp_idf_esp_task_wdt)]
                let duration = core::time::Duration::from_secs(
                    esp_idf_sys::CONFIG_ESP_TASK_WDT_TIMEOUT_S as u64,
                );
                #[cfg(not(esp_idf_esp_task_wdt))]
                let duration = core::time::Duration::from_secs(5);
                Self {
                    duration,
                    panic_on_trigger: cfg!(esp_idf_esp_task_wdt_panic),
                    subscribed_idle_tasks: {
                        let mut subscribed_idle_tasks = enumset::EnumSet::empty();
                        if cfg!(esp_idf_esp_task_wdt_check_idle_task_cpu0) {
                            subscribed_idle_tasks |= crate::cpu::Core::Core0;
                        }
                        #[cfg(any(esp32, esp32s3))]
                        if cfg!(esp_idf_esp_task_wdt_check_idle_task_cpu1) {
                            subscribed_idle_tasks |= crate::cpu::Core::Core1;
                        }
                        subscribed_idle_tasks
                    },
                }
            }
        }

        impl Default for Config {
            fn default() -> Self {
                Self::new()
            }
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        impl From<&Config> for esp_task_wdt_config_t {
            fn from(config: &Config) -> Self {
                esp_task_wdt_config_t {
                    timeout_ms: config.duration.as_millis() as u32,
                    trigger_panic: config.panic_on_trigger,
                    idle_core_mask: config.subscribed_idle_tasks.as_u32(),
                }
            }
        }
    }

    pub struct TWDTDriver<'d> {
        init_by_idf: bool,
        _marker: PhantomData<&'d mut ()>,
    }

    static TWDT_DRIVER_REF_COUNT: AtomicUsize = AtomicUsize::new(0);

    impl<'d> TWDTDriver<'d> {
        pub fn new(
            _twdt: impl Peripheral<P = TWDT> + 'd,
            config: &config::Config,
        ) -> Result<Self, EspError> {
            TWDT_DRIVER_REF_COUNT.fetch_add(1, Ordering::SeqCst);
            let init_by_idf = Self::watchdog_is_init_by_idf();

            #[cfg(not(esp_idf_version_major = "4"))]
            if !init_by_idf {
                esp!(unsafe { esp_task_wdt_init(&config.into() as *const esp_task_wdt_config_t) })?;
            } else {
                esp!(unsafe {
                    esp_task_wdt_reconfigure(&config.into() as *const esp_task_wdt_config_t)
                })?;
            }

            #[cfg(esp_idf_version_major = "4")]
            esp!(unsafe {
                esp_task_wdt_init(config.duration.as_secs() as u32, config.panic_on_trigger)
            })?;

            #[cfg(esp_idf_version_major = "4")]
            if let Err(e) = Self::subscribe_idle_tasks(config.subscribed_idle_tasks) {
                // error task already subscribed could occur but it's ok (not checking if tasks already subscribed before)
                if e.code() != ESP_ERR_INVALID_ARG {
                    return Err(e);
                }
            }

            Ok(Self {
                init_by_idf,
                _marker: Default::default(),
            })
        }

        pub fn watch_current_task(&mut self) -> Result<WatchdogSubscription<'_>, EspError> {
            esp!(unsafe { esp_task_wdt_add(core::ptr::null_mut()) })?;
            Ok(WatchdogSubscription::new())
        }

        #[cfg(esp_idf_version_major = "4")]
        fn subscribe_idle_tasks(cores: enumset::EnumSet<crate::cpu::Core>) -> Result<(), EspError> {
            for core in cores {
                let task = super::get_idle_task(core);
                esp!(unsafe { esp_task_wdt_add(task) })?;
            }

            Ok(())
        }

        #[cfg(esp_idf_version_major = "4")]
        fn unsubscribe_idle_tasks() -> Result<(), EspError> {
            for core in enumset::EnumSet::<crate::cpu::Core>::all() {
                let task = super::get_idle_task(core);
                esp!(unsafe { esp_task_wdt_delete(task) })?;
            }

            Ok(())
        }

        fn watchdog_is_init_by_idf() -> bool {
            if cfg!(not(any(
                esp_idf_version_major = "4",
                esp_idf_version = "5.0"
            ))) {
                cfg!(esp_idf_esp_task_wdt_init)
            } else {
                !matches!(
                    unsafe { esp_task_wdt_status(core::ptr::null_mut()) },
                    ESP_ERR_INVALID_STATE
                )
            }
        }

        fn deinit(&self) -> Result<(), EspError> {
            if !self.init_by_idf {
                #[cfg(esp_idf_version_major = "4")]
                if let Err(e) = Self::unsubscribe_idle_tasks() {
                    // error task not subscribed could occur but it's ok (not checking if tasks subscribed before)
                    if e.code() != ESP_ERR_INVALID_ARG {
                        return Err(e);
                    }
                }
                esp!(unsafe { esp_task_wdt_deinit() }).unwrap();
            }

            Ok(())
        }
    }

    impl Clone for TWDTDriver<'_> {
        fn clone(&self) -> Self {
            TWDT_DRIVER_REF_COUNT.fetch_add(1, Ordering::SeqCst);
            Self {
                init_by_idf: self.init_by_idf,
                _marker: Default::default(),
            }
        }
    }

    impl Drop for TWDTDriver<'_> {
        fn drop(&mut self) {
            let refcnt = TWDT_DRIVER_REF_COUNT.fetch_sub(1, Ordering::SeqCst);
            match refcnt {
                1 => self.deinit().unwrap(),
                r if r < 1 => unreachable!(), // Bug, should never happen
                _ => (),
            }
        }
    }

    unsafe impl Send for TWDTDriver<'_> {}

    pub struct WatchdogSubscription<'s>(PhantomData<&'s mut ()>);

    impl WatchdogSubscription<'_> {
        fn new() -> Self {
            Self(Default::default())
        }

        pub fn feed(&mut self) -> Result<(), EspError> {
            esp!(unsafe { esp_task_wdt_reset() })
        }
    }

    impl embedded_hal_0_2::watchdog::Watchdog for WatchdogSubscription<'_> {
        fn feed(&mut self) {
            Self::feed(self).unwrap()
        }
    }

    impl Drop for WatchdogSubscription<'_> {
        fn drop(&mut self) {
            esp!(unsafe { esp_task_wdt_delete(core::ptr::null_mut()) }).unwrap();
        }
    }

    crate::impl_peripheral!(TWDT);
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
        #[allow(clippy::declare_interior_mutable_const)]
        const INIT: Self = Self::new();

        fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
            let _guard = self.0.enter();

            f()
        }
    }
}

#[cfg(all(feature = "alloc", target_has_atomic = "ptr"))]
pub mod notification {
    use core::marker::PhantomData;
    use core::num::NonZeroU32;
    use core::sync::atomic::{AtomicPtr, Ordering};

    extern crate alloc;
    use alloc::sync::Arc;
    use alloc::task::Wake;

    use esp_idf_sys::TickType_t;

    use crate::task;

    #[cfg(esp_idf_version_major = "4")]
    type Task = core::ffi::c_void;

    #[cfg(not(esp_idf_version_major = "4"))]
    type Task = esp_idf_sys::tskTaskControlBlock;

    pub struct Notification(Arc<Notifier>, PhantomData<*const ()>);

    impl Notification {
        pub fn new() -> Self {
            Self(
                Arc::new(Notifier(AtomicPtr::new(task::current().unwrap()))),
                PhantomData,
            )
        }

        pub fn notifier(&self) -> Arc<Notifier> {
            self.0.clone()
        }

        pub fn wait_any(&self) {
            loop {
                if task::wait_notification(crate::delay::BLOCK).is_some() {
                    break;
                }
            }
        }

        pub fn wait(&self, timeout: TickType_t) -> Option<NonZeroU32> {
            task::wait_notification(timeout)
        }
    }

    impl Default for Notification {
        fn default() -> Self {
            Self::new()
        }
    }

    pub struct Notifier(AtomicPtr<Task>);

    impl Notifier {
        /// # Safety
        ///
        /// This method is unsafe because it is possible to call `core::mem::forget` on the Monitor instance
        /// that produced this notifier.
        ///
        /// If that happens, the `Drop` dtor of `Monitor` will NOT be called, which - in turn - means that the
        /// `Arc` holding the task reference will stick around even when the actual task where the `Monitor` instance was
        /// created no longer exists. Which - in turn - would mean that the method will be trying to notify a task
        /// which does no longer exist, which would lead to UB and specifically - to memory corruption.
        pub unsafe fn notify(&self, notification: NonZeroU32) -> (bool, bool) {
            let freertos_task = self.0.load(Ordering::SeqCst);

            if !freertos_task.is_null() {
                return unsafe { task::notify(freertos_task, notification) };
            }

            (false, false)
        }

        /// # Safety
        ///
        /// This method is unsafe because it is possible to call `core::mem::forget` on the Monitor instance
        /// that produced this notifier.
        ///
        /// If that happens, the `Drop` dtor of `Monitor` will NOT be called, which - in turn - means that the
        /// `Arc` holding the task reference will stick around even when the actual task where the `Monitor` instance was
        /// created no longer exists. Which - in turn - would mean that the method will be trying to notify a task
        /// which does no longer exist, which would lead to UB and specifically - to memory corruption.
        pub unsafe fn notify_and_yield(&self, notification: NonZeroU32) -> bool {
            let freertos_task = self.0.load(Ordering::SeqCst);

            if !freertos_task.is_null() {
                unsafe { task::notify_and_yield(freertos_task, notification) }
            } else {
                false
            }
        }
    }

    impl Wake for Notifier {
        fn wake(self: Arc<Self>) {
            unsafe {
                self.notify_and_yield(NonZeroU32::new(1).unwrap());
            }
        }
    }
}

pub mod queue {
    use core::{
        marker::PhantomData,
        mem::{size_of, MaybeUninit},
    };

    use esp_idf_sys::{EspError, TickType_t, ESP_FAIL};

    use crate::sys;

    /// Thin wrapper on top of the FreeRTOS queue.
    ///
    /// This may be preferable over a Rust channel
    /// in cases where an ISR needs to send or receive
    /// data as it is safe to use in ISR contexts.
    pub struct Queue<T> {
        ptr: sys::QueueHandle_t,
        is_owned: bool,
        _marker: PhantomData<T>,
    }

    unsafe impl<T> Send for Queue<T> where T: Send + Sync {}
    unsafe impl<T> Sync for Queue<T> where T: Send + Sync {}

    impl<T> Queue<T>
    where
        // ensures the contained elements are not `Drop`
        // might be able to lift restriction in the future
        T: Copy,
    {
        /// Allocate a new queue on the heap.
        pub fn new(size: usize) -> Self {
            Queue {
                ptr: unsafe { sys::xQueueGenericCreate(size as u32, size_of::<T>() as u32, 0) },
                is_owned: true,
                _marker: PhantomData,
            }
        }

        /// Create a new queue which is not deleted on `Drop`, but owned by somebody else.
        ///
        /// # Safety
        ///
        /// Care must be taken that the queue is valid for the constructed
        /// lifetime.
        pub unsafe fn new_borrowed(ptr: sys::QueueHandle_t) -> Self {
            assert!(!ptr.is_null());

            Queue {
                ptr,
                is_owned: false,
                _marker: PhantomData,
            }
        }

        /// Retrieves the underlying FreeRTOS handle.
        #[inline]
        #[link_section = "iram1.queue_as_raw"]
        pub fn as_raw(&self) -> sys::QueueHandle_t {
            self.ptr
        }

        /// Copy item to back of queue, blocking for `timeout` ticks if full.
        ///
        /// # ISR safety
        ///
        /// This function is safe to call in ISR contexts.
        ///
        /// # Parameters
        ///
        /// * `item` the item to push onto the back of the queue
        /// * `timeout` specifies how long to block. Ignored in ISR context.
        ///
        /// # Returns
        ///
        /// Will return an error if queue is full.
        /// If this function is executed in an ISR context,
        /// it will return true if a higher priority task was awoken.
        /// In non-ISR contexts, the function will always return `false`.
        /// In this case the interrupt should call [`crate::task::do_yield`].
        #[inline]
        #[link_section = "iram1.queue_send_back"]
        pub fn send_back(&self, item: T, timeout: TickType_t) -> Result<bool, EspError> {
            self.send_generic(item, timeout, 0)
        }

        /// Copy item to front of queue, blocking for `timeout` ticks if full.
        /// This can be used for hight priority messages which should be processed
        /// sooner.
        ///
        /// # ISR safety
        ///
        /// This function is safe to call in ISR contexts.
        ///
        /// # Parameters
        ///
        /// * `item` the item to push to front of the queue
        /// * `timeout` specifies how long to block. Ignored in ISR context.
        ///
        /// # Returns
        ///
        /// Will return an error if queue is full.
        /// If this function is executed in an ISR context,
        /// it will return true if a higher priority task was awoken.
        /// In non-ISR contexts, the function will always return `false`.
        /// In this case the interrupt should call [`crate::task::do_yield`].
        #[inline]
        #[link_section = "iram1.queue_send_front"]
        pub fn send_front(&self, item: T, timeout: TickType_t) -> Result<bool, EspError> {
            self.send_generic(item, timeout, 1)
        }

        /// Copy item to queue, blocking for `timeout` ticks if full.
        ///
        /// # ISR safety
        ///
        /// This function is safe to call in ISR contexts.
        ///
        /// # Parameters
        ///
        /// * `item` the item to push to the queue
        /// * `timeout` specifies how long to block. Ignored in ISR context.
        /// * `copy_position` 0 to push onto back, 1 to push to front
        ///
        /// # Returns
        ///
        /// Will return an error if queue is full.
        /// If this function is executed in an ISR context,
        /// it will return true if a higher priority task was awoken.
        /// In non-ISR contexts, the function will always return `false`.
        /// In this case the interrupt should call [`crate::task::do_yield`].
        #[inline]
        #[link_section = "iram1.queue_send_generic"]
        fn send_generic(
            &self,
            item: T,
            timeout: TickType_t,
            copy_position: i32,
        ) -> Result<bool, EspError> {
            let mut hp_task_awoken: i32 = false as i32;
            let success = unsafe {
                if crate::interrupt::active() {
                    sys::xQueueGenericSendFromISR(
                        self.ptr,
                        &item as *const T as *const _,
                        &mut hp_task_awoken,
                        copy_position,
                    )
                } else {
                    sys::xQueueGenericSend(
                        self.ptr,
                        &item as *const T as *const _,
                        timeout,
                        copy_position,
                    )
                }
            };
            let success = success == 1;
            let hp_task_awoken = hp_task_awoken == 1;

            match success {
                true => Ok(hp_task_awoken),
                false => Err(EspError::from_infallible::<ESP_FAIL>()),
            }
        }

        /// Receive a message from the queue and remove it.
        ///
        /// # ISR safety
        ///
        /// This function is safe to use in ISR contexts
        ///
        /// # Parameters
        ///
        /// * `timeout` specifies how long to block. Ignored in ISR contexts.
        ///
        /// # Returns
        ///
        /// * `None` if no message could be received in time
        /// * `Some((message, higher_priority_task_awoken))` otherwise
        ///
        /// The boolean is used for ISRs and indicates if a higher priority task was awoken.
        /// In this case the interrupt should call [`crate::task::do_yield`].
        /// In non-ISR contexts, the function will always return `false`.
        #[inline]
        #[link_section = "iram1.queue_recv_front"]
        pub fn recv_front(&self, timeout: TickType_t) -> Option<(T, bool)> {
            let mut buf = MaybeUninit::uninit();
            let mut hp_task_awoken = false as i32;

            unsafe {
                let success = if crate::interrupt::active() {
                    sys::xQueueReceiveFromISR(
                        self.ptr,
                        buf.as_mut_ptr() as *mut _,
                        &mut hp_task_awoken,
                    )
                } else {
                    sys::xQueueReceive(self.ptr, buf.as_mut_ptr() as *mut _, timeout)
                };
                if success == 1 {
                    Some((buf.assume_init(), hp_task_awoken == 1))
                } else {
                    None
                }
            }
        }

        /// Copy the first message from the queue without removing it.
        ///
        /// # ISR safety
        ///
        /// This function is safe to use in ISR contexts
        ///
        /// # Parameters
        ///
        /// * `timeout` specifies how long to block. Ignored in ISR contexts.
        ///
        /// # Returns
        ///
        /// * `None` if no message could be received in time
        /// * `Some(message)` otherwise
        ///
        /// This function does not return a boolean to indicate if
        /// a higher priority task was awoken since we don't free
        /// up space in the queue and thus cannot unblock anyone.
        #[inline]
        #[link_section = "iram1.queue_peek_front"]
        pub fn peek_front(&self, timeout: TickType_t) -> Option<T> {
            let mut buf = MaybeUninit::uninit();

            unsafe {
                let success = if crate::interrupt::active() {
                    sys::xQueuePeekFromISR(self.ptr, buf.as_mut_ptr() as *mut _)
                } else {
                    sys::xQueuePeek(self.ptr, buf.as_mut_ptr() as *mut _, timeout)
                };
                if success == 1 {
                    Some(buf.assume_init())
                } else {
                    None
                }
            }
        }
    }

    impl<T> Drop for Queue<T> {
        fn drop(&mut self) {
            if self.is_owned {
                unsafe { sys::vQueueDelete(self.ptr) }
            }
        }
    }
}

pub mod asynch {
    use core::future::Future;
    use core::num::NonZeroU32;
    use core::sync::atomic::{AtomicU32, Ordering};
    use core::task::{Context, Poll, Waker};

    use atomic_waker::AtomicWaker;

    /// Single-slot lock-free signaling primitive supporting signalling with a `u32` bit-set.
    ///
    /// It is useful for sending data between tasks when the receiver only cares about
    /// the latest data, and therefore it's fine to "lose" messages. This is often the case for "state"
    /// updates.
    ///
    /// The sending part of the primitive is non-blocking, so it is also useful for notifying asynchronous tasks
    /// from contexts where blocking or async wait is not possible.
    ///
    /// Similar in spirit to the ESP-IDF FreeRTOS task notifications in that it is light-weight and operates on bit-sets,
    /// but for synchronization between an asynchronous task, and another one, which might be blocking or asynchronous.
    pub struct Notification {
        waker: AtomicWaker,
        notified: AtomicU32,
    }

    impl Notification {
        /// Creates a new `Notification`.
        pub const fn new() -> Self {
            Self {
                waker: AtomicWaker::new(),
                notified: AtomicU32::new(0),
            }
        }

        /// Marks the least significant bit (bit 0) in this `IsrNotification` as nofified.
        /// Returns `true` if there was a registered waker which got awoken.
        pub fn notify_lsb(&self) -> bool {
            self.notify(NonZeroU32::new(1).unwrap())
        }

        /// Marks the supplied bits in this `Notification` as notified.
        /// Returns `true` if there was a registered waker which got awoken.
        pub fn notify(&self, bits: NonZeroU32) -> bool {
            if let Some(waker) = self.notify_waker(bits) {
                waker.wake();

                true
            } else {
                false
            }
        }

        /// A utility to help in implementing a custom `wait` logic:
        /// Adds the supplied bits as notified in the notification instance and returns the registered waker (if any).
        pub fn notify_waker(&self, bits: NonZeroU32) -> Option<Waker> {
            self.notified.fetch_or(bits.into(), Ordering::SeqCst);

            self.waker.take()
        }

        /// Clears the state of this notification by removing any registered waker and setting all bits to 0.
        pub fn reset(&self) {
            self.waker.take();
            self.notified.store(0, Ordering::SeqCst);
        }

        /// Future that completes when this `Notification` has been notified.
        #[allow(unused)]
        pub fn wait(&self) -> impl Future<Output = NonZeroU32> + '_ {
            core::future::poll_fn(move |cx| self.poll_wait(cx))
        }

        /// Non-blocking method to check whether this notification has been notified.
        pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<NonZeroU32> {
            self.waker.register(cx.waker());

            let bits = self.notified.swap(0, Ordering::SeqCst);

            if let Some(bits) = NonZeroU32::new(bits) {
                Poll::Ready(bits)
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for Notification {
        fn drop(&mut self) {
            self.reset();
        }
    }
}
