use core::time::Duration;

use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use esp_idf_sys::*;

#[allow(non_upper_case_globals)]
pub const portMAX_DELAY: TickType_t = TickType_t::max_value();

#[allow(non_upper_case_globals)]
pub const portTICK_PERIOD_MS: u32 = 1000 / configTICK_RATE_HZ;

pub struct TickType(pub TickType_t);

impl From<Duration> for TickType {
    fn from(duration: Duration) -> Self {
        TickType(((duration.as_millis() + portTICK_PERIOD_MS as u128 - 1) / portTICK_PERIOD_MS as u128) as TickType_t)
    }
}

impl From<Option<Duration>> for TickType {
    fn from(duration: Option<Duration>) -> Self {
        if let Some(duration) = duration {
            duration.into()
        } else {
            TickType(portMAX_DELAY)
        }
    }
}

impl From<TickType> for Duration {
    fn from(ticks: TickType) -> Self {
        Duration::from_millis(ticks.0 as u64 * portTICK_PERIOD_MS as u64)
    }
}

impl From<TickType> for Option<Duration> {
    fn from(ticks: TickType) -> Self {
        if ticks.0 == portMAX_DELAY {
            None
        } else {
            Some(ticks.into())
        }
    }
}

/// Espressif Task Scheduler-based delay provider
pub struct Ets;

impl DelayUs<u32> for Ets {
    fn delay_us(&mut self, us: u32) {
        unsafe {
            ets_delay_us(us);
        }
    }
}

impl DelayUs<u16> for Ets {
    fn delay_us(&mut self, us: u16) {
        DelayUs::<u32>::delay_us(self, us as u32);
    }
}

/// FreeRTOS-based delay provider
pub struct FreeRtos;

impl DelayMs<u32> for FreeRtos {
    fn delay_ms(&mut self, ms: u32) {
        // divide by tick length, rounding up
        let ticks = (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
        unsafe {
            vTaskDelay(ticks);
        }
    }
}

impl DelayMs<u16> for FreeRtos {
    fn delay_ms(&mut self, ms: u16) {
        DelayMs::<u32>::delay_ms(self, ms as u32);
    }
}
