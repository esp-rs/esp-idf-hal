use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use esp32_sys::{configTICK_RATE_HZ, ets_delay_us, vTaskDelay};

#[allow(non_upper_case_globals)]
const portTICK_PERIOD_MS: u32 = 1000 / configTICK_RATE_HZ;

/// Espressif Task Scheduler-based delay provider
pub struct Ets;

impl DelayUs<u32> for Ets {
    fn delay_us(&mut self, us: u32) {
        unsafe {
            ets_delay_us(us);
        }
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
