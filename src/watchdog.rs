#[cfg(esp_idf_esp_task_wdt)]
pub mod task {
    //! ## Example
    //!
    //! ```rust, ignore
    //! # fn main() -> Result<()> {
    //! let peripherals = Peripherals::take().unwrap();
    //!
    //! let mut driver = esp_idf_hal::watchdog::task::TWDTDriver::new(
    //!     peripherals.twdt,
    //!     WatchdogConfig {
    //!         duration: Duration::from_secs(2),
    //!         panic_on_trigger: true,
    //!         ..Default::default()
    //!     },
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

    use core::{marker::PhantomData, sync::atomic::AtomicUsize};

    use esp_idf_sys::*;

    use crate::peripheral::Peripheral;

    pub struct WatchdogConfig {
        pub duration: core::time::Duration,
        pub panic_on_trigger: bool,
        #[cfg(not(esp_idf_version_major = "4"))]
        pub subscribed_idle_tasks: heapless::Vec<crate::task::IdleTask, 2>,
    }

    impl Default for WatchdogConfig {
        fn default() -> Self {
            Self {
                duration: core::time::Duration::from_secs(CONFIG_ESP_TASK_WDT_TIMEOUT_S as u64),
                panic_on_trigger: cfg!(esp_idf_esp_task_wdt_panic), // CONFIG_ESP_TASK_WDT_PANIC
                #[cfg(not(esp_idf_version_major = "4"))]
                subscribed_idle_tasks: {
                    let subscribed_idle_tasks = heapless::Vec::new();
                    if cfg!(esp_idf_esp_task_wdt_check_idle_task_cpu0) {
                        subscribed_idle_tasks
                            .push(crate::task::IdleTask::Core0)
                            .unwrap();
                    }
                    #[cfg(any(esp32, esp32s3))]
                    if cfg!(esp_idf_esp_task_wdt_check_idle_task_cpu1) {
                        subscribed_idle_tasks
                            .push(crate::task::IdleTask::Core1)
                            .unwrap();
                    }
                    subscribed_idle_tasks
                },
            }
        }
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    impl From<WatchdogConfig> for esp_task_wdt_config_t {
        fn from(value: WatchdogConfig) -> Self {
            esp_task_wdt_config_t {
                timeout_ms: value.duration.as_millis() as u32,
                trigger_panic: value.panic_on_trigger,
                idle_core_mask: value
                    .subscribed_idle_tasks
                    .iter()
                    .fold(0u32, |mask, core| mask & (1 << (*core as u32))),
            }
        }
    }

    pub trait Twdt {}

    pub struct TWDTDriver<'d>(PhantomData<&'d mut ()>);

    unsafe impl Send for TWDTDriver<'_> {}

    static TWDT_DRIVER_REF_COUNT: AtomicUsize = AtomicUsize::new(0);

    impl Clone for TWDTDriver<'_> {
        fn clone(&self) -> Self {
            TWDT_DRIVER_REF_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            Self(Default::default())
        }
    }

    impl<'d> TWDTDriver<'d> {
        pub fn new(
            _twdt: impl Peripheral<P = TWDT> + 'd,
            config: WatchdogConfig,
        ) -> Result<Self, EspError> {
            init_or_reconfigure(config)?;
            Ok(Self(Default::default()))
        }

        // will fail in isr
        pub fn watch_current_task<'a>(&'a mut self) -> Result<WatchdogSubscription<'a>, EspError> {
            add_task(core::ptr::null_mut())?;
            Ok(WatchdogSubscription::new())
        }
    }

    impl Drop for TWDTDriver<'_> {
        fn drop(&mut self) {
            let drop = TWDT_DRIVER_REF_COUNT
                .fetch_update(
                    core::sync::atomic::Ordering::Relaxed,
                    core::sync::atomic::Ordering::Relaxed,
                    |count| if count == 0 { None } else { Some(count - 1) },
                )
                .is_err();
            if drop {
                // TODO: maybe deinit twdt from idf (only if not initialized by idf)
                println!("TWDTDriver dropped")
            }
        }
    }

    pub struct WatchdogSubscription<'s>(PhantomData<&'s mut ()>);

    impl WatchdogSubscription<'_> {
        fn new() -> Self {
            Self(Default::default())
        }
    }

    impl embedded_hal_0_2::watchdog::Watchdog for WatchdogSubscription<'_> {
        fn feed(&mut self) {
            feed_from_curr_task().unwrap()
        }
    }

    impl Drop for WatchdogSubscription<'_> {
        fn drop(&mut self) {
            let _ = remove_task(core::ptr::null_mut());
        }
    }

    crate::impl_peripheral!(TWDT);
    impl Twdt for TWDT {}

    fn init_or_reconfigure(config: WatchdogConfig) -> Result<(), EspError> {
        unsafe {
            esp!(esp_task_wdt_init(
                config.duration.as_secs() as u32,
                config.panic_on_trigger
            ))
        }
    }

    fn feed_from_curr_task() -> Result<(), EspError> {
        unsafe { esp!(esp_task_wdt_reset()) }
    }

    fn add_task(task: TaskHandle_t) -> Result<(), EspError> {
        unsafe { esp!(esp_task_wdt_add(task)) }
    }

    fn remove_task(task: TaskHandle_t) -> Result<(), EspError> {
        match unsafe { esp_task_wdt_delete(task) } {
            ESP_ERR_INVALID_ARG | ESP_OK => Ok(()),
            e => EspError::convert(e),
        }
    }

    // fn is_enabled_for_task(task: TaskHandle_t) -> Result<bool, EspError> {
    //     let is_enabled = match unsafe { esp_task_wdt_status(task) } {
    //         ESP_ERR_NOT_FOUND => false,
    //         ESP_OK => true,
    //         e => return Err(EspError::from(e).unwrap()),
    //     };

    //     Ok(is_enabled)
    // }

    // fn is_initialized() -> bool {
    //     !matches!(
    //         unsafe { esp_task_wdt_status(core::ptr::null_mut()) },
    //         ESP_ERR_INVALID_STATE
    //     )
    // }
}
