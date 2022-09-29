//! Reset reasons
use esp_idf_sys::*;

/// Reset reasons
#[doc(alias = "esp_reset_reason_t")]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum ResetReason {
    /// Software restart via `esp_restart()`
    Software,
    /// Reset by external pin
    ExternalPin,
    /// Reset due to other watchdogs
    Watchdog,
    /// Reset over SDIO
    Sdio,
    /// Software reset due to exception/panic
    Panic,
    /// Reset (software or hardware) due to interrupt watchdog
    InterruptWatchdog,
    /// Reset due to power-on event
    PowerOn,
    /// Reset reason can not be determined
    Unknown,
    /// Brownout reset (software or hardware)
    Brownout,
    /// Reset due to task watchdog
    TaskWatchdog,
    /// Reset after exiting deep sleep mode
    DeepSleep,
}

impl From<esp_reset_reason_t> for ResetReason {
    #[allow(non_upper_case_globals)]
    fn from(value: esp_reset_reason_t) -> Self {
        match value {
            esp_reset_reason_t_ESP_RST_SW => Self::Software,
            esp_reset_reason_t_ESP_RST_EXT => Self::ExternalPin,
            esp_reset_reason_t_ESP_RST_WDT => Self::Watchdog,
            esp_reset_reason_t_ESP_RST_SDIO => Self::Sdio,
            esp_reset_reason_t_ESP_RST_PANIC => Self::Panic,
            esp_reset_reason_t_ESP_RST_INT_WDT => Self::InterruptWatchdog,
            esp_reset_reason_t_ESP_RST_POWERON => Self::PowerOn,
            esp_reset_reason_t_ESP_RST_UNKNOWN => Self::Unknown,
            esp_reset_reason_t_ESP_RST_BROWNOUT => Self::Brownout,
            esp_reset_reason_t_ESP_RST_TASK_WDT => Self::TaskWatchdog,
            esp_reset_reason_t_ESP_RST_DEEPSLEEP => Self::DeepSleep,
            _ => unreachable!(),
        }
    }
}

impl ResetReason {
    /// Get the reason for the last reset
    #[doc(alias = "esp_reset_reason")]
    pub fn get() -> Self {
        // SAFETY: `esp_reset_reason()` reads from static memory that is initialized when
        // constructors run and never modified after that point. We would only see
        // uninitialized/partially initialized data if we run before main() (and before
        // constructors running).
        let rr = unsafe { esp_reset_reason() };
        rr.into()
    }
}

/// Wakeup reasons
#[doc(alias = "esp_sleep_source_t")]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum WakeupReason {
    Unknown,
    ULP,
    Button,
    Timer,
    Other(u32),
}

impl From<esp_sleep_source_t> for WakeupReason {
    #[allow(non_upper_case_globals)]
    fn from(value: esp_sleep_source_t) -> Self {
        match value {
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_UNDEFINED => Self::Unknown,
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_EXT1 => Self::Button,
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_COCPU => Self::ULP,
            esp_sleep_source_t_ESP_SLEEP_WAKEUP_TIMER => Self::Timer,
            other => Self::Other(other),
        }
    }
}

impl WakeupReason {
    /// Get the reason for the wakeup
    #[doc(alias = "esp_sleep_get_wakeup_cause")]
    pub fn get() -> Self {
        let wr = unsafe { esp_sleep_get_wakeup_cause() };
        wr.into()
    }
}

pub fn restart() {
    unsafe {
        esp_restart();
    }
}
