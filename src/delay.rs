use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use esp_idf_sys::*;

#[allow(non_upper_case_globals)]
pub const portMAX_DELAY: TickType_t = TickType_t::max_value();

#[allow(non_upper_case_globals)]
pub const portTICK_PERIOD_MS: u32 = 1000 / configTICK_RATE_HZ;

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

impl DelayMs<u32> for Ets {
    fn delay_ms(&mut self, ms: u32) {
        unsafe {
            ets_delay_us(ms * 1000);
        }
    }
}

impl DelayMs<u16> for Ets {
    fn delay_ms(&mut self, ms: u16) {
        DelayMs::<u32>::delay_ms(self, ms as u32);
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
