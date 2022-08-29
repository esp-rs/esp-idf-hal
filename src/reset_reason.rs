use core::{
    convert::{TryFrom, TryInto},
    fmt,
};
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

#[derive(Debug)]
pub struct InvalidResetReason(esp_reset_reason_t);

#[cfg(feature = "std")]
impl std::error::Error for InvalidResetReason {}

impl fmt::Display for InvalidResetReason {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(fmt, "reset reason {} does not match a known value", self.0)
    }
}

impl TryFrom<esp_reset_reason_t> for ResetReason {
    type Error = InvalidResetReason;

    #[allow(non_upper_case_globals)]
    fn try_from(value: esp_reset_reason_t) -> Result<Self, Self::Error> {
        Ok(match value {
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
            _ => return Err(InvalidResetReason(value)),
        })
    }
}

pub fn reset_reason() -> Result<ResetReason, InvalidResetReason> {
    let rr = unsafe { esp_reset_reason() };
    rr.try_into()
}
