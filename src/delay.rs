use core::convert::Infallible;
use core::time::Duration;

use esp_idf_sys::*;

#[allow(non_upper_case_globals)]
pub(crate) const portMAX_DELAY: TickType_t = TickType_t::max_value();

#[allow(non_upper_case_globals)]
const portTICK_PERIOD_MS: u32 = 1000 / configTICK_RATE_HZ;

pub struct TickType(pub TickType_t);

impl From<Duration> for TickType {
    fn from(duration: Duration) -> Self {
        TickType(
            ((duration.as_millis() + portTICK_PERIOD_MS as u128 - 1) / portTICK_PERIOD_MS as u128)
                as TickType_t,
        )
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

impl Ets {
    fn delay_us_internal(&mut self, us: u32) {
        unsafe {
            ets_delay_us(us);
        }
    }

    fn delay_ms_internal(&mut self, ms: u32) {
        unsafe {
            ets_delay_us(ms * 1000);
        }
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Ets {
    fn delay_us(&mut self, us: u32) {
        self.delay_us_internal(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Ets {
    fn delay_us(&mut self, us: u16) {
        self.delay_us_internal(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for Ets {
    fn delay_us(&mut self, us: u8) {
        self.delay_us_internal(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Ets {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_ms_internal(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Ets {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms_internal(ms as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for Ets {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms_internal(ms as _);
    }
}

impl embedded_hal::delay::blocking::DelayUs for Ets {
    type Error = Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.delay_us_internal(us);

        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay_ms_internal(ms);

        Ok(())
    }
}

/// FreeRTOS-based delay provider
pub struct FreeRtos;

impl FreeRtos {
    fn delay_us_internal(&mut self, us: u32) {
        let ms = us / 1000;

        Self::delay_ms_internal(self, ms);
    }

    fn delay_ms_internal(&mut self, ms: u32) {
        // divide by tick length, rounding up
        let ticks = (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;

        unsafe {
            vTaskDelay(ticks);
        }
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for FreeRtos {
    fn delay_us(&mut self, us: u32) {
        self.delay_us_internal(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for FreeRtos {
    fn delay_us(&mut self, us: u16) {
        self.delay_us_internal(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for FreeRtos {
    fn delay_us(&mut self, us: u8) {
        self.delay_us_internal(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for FreeRtos {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_ms_internal(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for FreeRtos {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms_internal(ms as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for FreeRtos {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms_internal(ms as _);
    }
}

impl embedded_hal::delay::blocking::DelayUs for FreeRtos {
    type Error = Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        self.delay_us_internal(us);

        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay_ms_internal(ms);

        Ok(())
    }
}
