use esp_idf_sys::*;

#[derive(Debug)]
pub enum ResetReason {
    Software,
    ExternalPin,
    Watchdog,
    Sdio,
    Panic,
    InterruptWatchdog,
    PowerOn,
    Unknown,
    Brownout,
    TaskWatchdog,
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
            _ => panic!(),
        }
    }
}

pub fn reset_reason() -> ResetReason {
    let rr = unsafe { esp_reset_reason() };
    rr.into()
}
