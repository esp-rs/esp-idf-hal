//! Reset reasons
use esp_idf_sys::*;

/// Reset reasons
#[doc(alias = "esp_reset_reason_t")]
#[derive(Debug)]
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
            esp_reset_reason_t_ESP_RST_SW => ResetReason::Software,
            esp_reset_reason_t_ESP_RST_EXT => ResetReason::ExternalPin,
            esp_reset_reason_t_ESP_RST_WDT => ResetReason::Watchdog,
            esp_reset_reason_t_ESP_RST_SDIO => ResetReason::Sdio,
            esp_reset_reason_t_ESP_RST_PANIC => ResetReason::Panic,
            esp_reset_reason_t_ESP_RST_INT_WDT => ResetReason::InterruptWatchdog,
            esp_reset_reason_t_ESP_RST_POWERON => ResetReason::PowerOn,
            esp_reset_reason_t_ESP_RST_UNKNOWN => ResetReason::Unknown,
            esp_reset_reason_t_ESP_RST_BROWNOUT => ResetReason::Brownout,
            esp_reset_reason_t_ESP_RST_TASK_WDT => ResetReason::TaskWatchdog,
            esp_reset_reason_t_ESP_RST_DEEPSLEEP => ResetReason::DeepSleep,
            _ => unreachable!(),
        }
    }
}

impl ResetReason {
    /// Get the reason for the last reset
    #[doc(alias = "esp_reset_reason")]
    pub fn get() -> ResetReason {
        // SAFETY: `esp_reset_reason()` reads from static memory that is initialized when
        // constructors run and never modified after that point. We would only see
        // uninitialized/partially initialized data if we run before main() (and before
        // constructors running).
        let rr = unsafe { esp_reset_reason() };
        rr.into()
    }
}
