//! Delay providers.
//!
//! If you don't know how large your delays will be, you'll probably want to
//! use [`Delay`]. Otherwise use [`Ets`] for delays <10ms and
//! [`FreeRtos`] for delays >=10ms.

use core::time::Duration;

use esp_idf_sys::*;

mod general_purpose;

pub use general_purpose::Delay;

#[allow(non_upper_case_globals)]
pub const BLOCK: TickType_t = TickType_t::MAX;

#[allow(non_upper_case_globals)]
pub const NON_BLOCK: TickType_t = TickType_t::MIN;

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
            TickType(BLOCK)
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
        if ticks.0 == BLOCK {
            None
        } else {
            Some(ticks.into())
        }
    }
}

/// Espressif built-in delay provider for small delays
///
/// Use only for very small delays (us or a few ms at most), or else the
/// FreeRTOS IDLE tasks' might starve and the IDLE tasks' watchdog will
/// trigger.
pub struct Ets;

// No longer available in the generated bindings for ESP-IDF 5
#[cfg(not(esp_idf_version_major = "4"))]
extern "C" {
    pub fn ets_delay_us(us: u32);
}

impl Ets {
    pub fn delay_us(us: u32) {
        unsafe {
            ets_delay_us(us);
        }
    }

    pub fn delay_ms(ms: u32) {
        unsafe {
            ets_delay_us(ms * 1000);
        }
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Ets {
    fn delay_us(&mut self, us: u32) {
        Ets::delay_us(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Ets {
    fn delay_us(&mut self, us: u16) {
        Ets::delay_us(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for Ets {
    fn delay_us(&mut self, us: u8) {
        Ets::delay_us(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Ets {
    fn delay_ms(&mut self, ms: u32) {
        Ets::delay_ms(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Ets {
    fn delay_ms(&mut self, ms: u16) {
        Ets::delay_ms(ms as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for Ets {
    fn delay_ms(&mut self, ms: u8) {
        Ets::delay_ms(ms as _);
    }
}

impl embedded_hal::delay::DelayUs for Ets {
    fn delay_us(&mut self, us: u32) {
        Ets::delay_us(us)
    }

    fn delay_ms(&mut self, ms: u32) {
        Ets::delay_ms(ms)
    }
}

/// FreeRTOS-based delay provider for delays larger than 10ms
///
/// Ddelays smaller than 10ms used in a loop would starve the FreeRTOS IDLE
/// tasks' as they are low prio tasks and hence the the IDLE tasks' watchdog
/// will trigger.
pub struct FreeRtos;

impl FreeRtos {
    pub fn delay_us(us: u32) {
        let ms = us / 1000;

        Self::delay_ms(ms);
    }

    pub fn delay_ms(ms: u32) {
        // divide by tick length, rounding up
        let ticks = ms.saturating_add(portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;

        unsafe {
            vTaskDelay(ticks);
        }
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for FreeRtos {
    fn delay_us(&mut self, us: u32) {
        FreeRtos::delay_us(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for FreeRtos {
    fn delay_us(&mut self, us: u16) {
        FreeRtos::delay_us(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for FreeRtos {
    fn delay_us(&mut self, us: u8) {
        FreeRtos::delay_us(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for FreeRtos {
    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for FreeRtos {
    fn delay_ms(&mut self, ms: u16) {
        FreeRtos::delay_ms(ms as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for FreeRtos {
    fn delay_ms(&mut self, ms: u8) {
        FreeRtos::delay_ms(ms as _);
    }
}

impl embedded_hal::delay::DelayUs for FreeRtos {
    fn delay_us(&mut self, us: u32) {
        FreeRtos::delay_us(us)
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms)
    }
}
