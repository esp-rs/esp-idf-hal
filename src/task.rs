use core::cell::Cell;
use core::ptr::{self, NonNull};
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
    let notified =
        unsafe { xTaskNotifyWait(0, u32::MAX, &mut notification, TickType::from(duration).0) } != 0;

    #[cfg(not(esp_idf_version = "4.3"))]
    let notified = unsafe {
        xTaskGenericNotifyWait(
            0,
            0,
            u32::MAX,
            &mut notification,
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
            &mut higher_prio_task_woken,
        );

        #[cfg(not(esp_idf_version = "4.3"))]
        let notified = xTaskGenericNotifyFromISR(
            task,
            0,
            notification,
            eNotifyAction_eSetBits,
            ptr::null_mut(),
            &mut higher_prio_task_woken,
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
    unsafe {
        xTaskGetIdleTaskHandleForCPU(core as u32)
    }
}

#[cfg(esp_idf_comp_pthread_enabled)]
pub mod thread {
    use core::ffi::CStr;

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
                            c_strlen(conf.thread_name.cast()) + 1,
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
    not(feature = "riscv-ulp-hal"),
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
        #[allow(clippy::declare_interior_mutable_const)]
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

    #[cfg(esp_idf_version_major = "4")]
    pub struct FreeRtosMonitor(Arc<AtomicPtr<core::ffi::c_void>>, *const ());

    #[cfg(not(esp_idf_version_major = "4"))]
    pub struct FreeRtosMonitor(Arc<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>, *const ());

    impl FreeRtosMonitor {
        pub fn new() -> Self {
            Self(
                Arc::new(AtomicPtr::new(task::current().unwrap())),
                ptr::null(),
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

    #[cfg(esp_idf_version_major = "4")]
    pub struct FreeRtosMonitorNotify(Weak<AtomicPtr<core::ffi::c_void>>);

    #[cfg(not(esp_idf_version_major = "4"))]
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
