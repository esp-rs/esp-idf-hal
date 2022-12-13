#[cfg(esp_idf_esp_task_wdt)]
pub mod task {
    //! ## Example
    //!
    //! ```rust, ignore
    //! # fn main() -> Result<()> {
    //! let mut watchdog = TaskWatchdog::take()?.unwrap();
    //!
    //! watchdog.reconfigure(WatchdogConfig {
    //!     duration: Duration::from_secs(2),
    //!     panic_on_trigger: true,
    //!     ..Default::default()
    //! })?;
    //!
    //! let task = Task::current();
    //!
    //! task.set_watchdog(&mut watchdog)?;
    //!
    //! loop {
    //!     task.reset_watchdog()?;
    //!     unsafe { vTaskDelay(1) };
    //! }
    //! # }
    //! ```

    use esp_idf_sys::*;

    use crate::cpu::Core;

    pub struct WatchdogConfig {
        pub duration: core::time::Duration,
        pub panic_on_trigger: bool,
        // unused by now
        pub cores: heapless::Vec<Core, 2>,
    }

    impl Default for WatchdogConfig {
        fn default() -> Self {
            let mut cores = heapless::Vec::new();
            if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 == 1 {
                cores.push(Core::Core0).unwrap();
            }
            #[cfg(any(esp32, esp32s3))]
            if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 == 1 {
                cores.push(Core::Core1).unwrap();
            }
            Self {
                duration: core::time::Duration::from_secs(CONFIG_ESP_TASK_WDT_TIMEOUT_S as u64),
                panic_on_trigger: false, // CONFIG_ESP_TASK_WDT_PANIC
                cores,
            }
        }
    }

    #[cfg(esp_idf_version = "master")]
    impl From<WatchdogConfig> for esp_task_wdt_config_t {
        fn from(value: WatchdogConfig) -> Self {
            esp_task_wdt_config_t {
                timeout_ms: value.duration.as_millis() as u32,
                trigger_panic: value.panic_on_trigger,
                idle_core_mask: value
                    .cores
                    .iter()
                    .fold(0u32, |mask, core| mask & (1 << (*core as u32))),
            }
        }
    }

    #[cfg(feature = "riscv-ulp-hal")]
    static mut TAKEN: bool = false;

    #[cfg(not(feature = "riscv-ulp-hal"))]
    static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    #[cfg(not(feature = "riscv-ulp-hal"))]
    static TAKEN_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

    #[derive(Debug, Clone)]
    pub struct TaskWatchdog;

    impl TaskWatchdog {
        #[cfg(feature = "riscv-ulp-hal")]
        pub fn take() -> Result<Option<Self>, EspError> {
            if unsafe { TAKEN } {
                None
            } else {
                unsafe {
                    TAKEN = true;
                }
                Some(Self::init())
            }
        }

        #[cfg(not(feature = "riscv-ulp-hal"))]
        pub fn take() -> Result<Option<Self>, EspError> {
            if TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
                Ok(None)
            } else {
                let _ = TAKEN_CS.enter();

                if !TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
                    TAKEN.store(true, core::sync::atomic::Ordering::SeqCst);

                    Ok(Some(Self::init()?))
                } else {
                    Ok(None)
                }
            }
        }

        fn init() -> Result<Self, EspError> {
            if !is_initialized() {
                init_or_reconfigure(WatchdogConfig::default())?;
            }

            Ok(TaskWatchdog)
        }

        pub fn reconfigure(&mut self, config: WatchdogConfig) -> Result<(), EspError> {
            init_or_reconfigure(config)
        }

        pub(crate) fn add_task(&mut self, task: TaskHandle_t) -> Result<(), EspError> {
            add_task(task)
        }

        pub(crate) fn remove_task(&mut self, task: TaskHandle_t) {
            // can be unwrapped, error occurs only if watchdog not initialized yet and this
            // shouldn't be the case at all
            remove_task(task).unwrap()
        }

        pub(crate) fn is_enabled_for_task(&self, task: TaskHandle_t) -> bool {
            is_enabled_for_task(task).unwrap()
        }

        pub(crate) fn reset_current() -> Result<(), EspError> {
            reset_timer()
        }
    }

    fn init_or_reconfigure(config: WatchdogConfig) -> Result<(), EspError> {
        unsafe {
            esp!(esp_task_wdt_init(
                config.duration.as_secs() as u32,
                config.panic_on_trigger
            ))
        }
    }

    fn reset_timer() -> Result<(), EspError> {
        unsafe { esp!(esp_task_wdt_reset()) }
    }

    fn add_task(task: TaskHandle_t) -> Result<(), EspError> {
        match unsafe { esp_task_wdt_add(task) } {
            ESP_ERR_INVALID_ARG | ESP_OK => Ok(()),
            e => EspError::convert(e),
        }
    }

    fn remove_task(task: TaskHandle_t) -> Result<(), EspError> {
        match unsafe { esp_task_wdt_delete(task) } {
            ESP_ERR_INVALID_ARG | ESP_OK => Ok(()),
            e => EspError::convert(e),
        }
    }

    fn is_enabled_for_task(task: TaskHandle_t) -> Result<bool, EspError> {
        let is_enabled = match unsafe { esp_task_wdt_status(task) } {
            ESP_ERR_NOT_FOUND => false,
            ESP_OK => true,
            e => return Err(EspError::from(e).unwrap()),
        };

        Ok(is_enabled)
    }

    pub fn is_initialized() -> bool {
        !matches!(
            unsafe { esp_task_wdt_status(core::ptr::null_mut()) },
            ESP_ERR_INVALID_STATE
        )
    }
}
