//! Delay providers.
//!
//! If you don't know how large your delays will be, you'll probably want to
//! use [`Delay`]. Otherwise use [`Ets`] for delays <10ms and
//! [`FreeRtos`] for delays >=10ms.

use core::{cmp::min, time::Duration};

use esp_idf_sys::*;

#[allow(non_upper_case_globals)]
pub const BLOCK: TickType_t = TickType_t::MAX;

#[allow(non_upper_case_globals)]
pub const NON_BLOCK: TickType_t = 0;

#[allow(non_upper_case_globals)]
pub const TICK_PERIOD_MS: u32 = 1000 / configTICK_RATE_HZ;

#[repr(transparent)]
pub struct TickType(pub TickType_t);

impl TickType {
    pub const fn new(ticks: TickType_t) -> Self {
        Self(ticks)
    }

    pub const fn ticks(&self) -> TickType_t {
        self.0
    }

    pub fn as_millis(&self) -> u64 {
        self.0 as u64 * TICK_PERIOD_MS as u64
    }

    pub fn as_millis_u32(&self) -> u32 {
        min(self.as_millis(), u32::MAX as _) as _
    }
}

impl From<TickType_t> for TickType {
    fn from(value: TickType_t) -> Self {
        Self::new(value)
    }
}

impl From<TickType> for TickType_t {
    fn from(value: TickType) -> Self {
        value.ticks()
    }
}

impl From<Duration> for TickType {
    fn from(duration: Duration) -> Self {
        TickType(
            ((duration.as_millis() + TICK_PERIOD_MS as u128 - 1) / TICK_PERIOD_MS as u128)
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
        Duration::from_millis(ticks.0 as u64 * TICK_PERIOD_MS as u64)
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
    #[deprecated = "Use delay_ns instead"]
    pub fn delay_us(us: u32) {
        Self::delay_ns(us)
    }

    pub fn delay_ns(ns: u32) {
        unsafe {
            ets_delay_us(ns);
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
        Ets::delay_ns(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Ets {
    fn delay_us(&mut self, us: u16) {
        Ets::delay_ns(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for Ets {
    fn delay_us(&mut self, us: u8) {
        Ets::delay_ns(us as _);
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

impl embedded_hal::delay::DelayNs for Ets {
    fn delay_ns(&mut self, ns: u32) {
        Ets::delay_ns(ns)
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
    #[deprecated = "Use delay_ns instead"]
    pub fn delay_us(us: u32) {
        Self::delay_ns(us)
    }

    pub fn delay_ns(ns: u32) {
        let ms = ns / 1000;

        Self::delay_ms(ms);
    }

    pub fn delay_ms(ms: u32) {
        // divide by tick length, rounding up
        let ticks = ms.saturating_add(TICK_PERIOD_MS - 1) / TICK_PERIOD_MS;

        unsafe {
            vTaskDelay(ticks);
        }
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for FreeRtos {
    fn delay_us(&mut self, us: u32) {
        FreeRtos::delay_ns(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for FreeRtos {
    fn delay_us(&mut self, us: u16) {
        FreeRtos::delay_ns(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for FreeRtos {
    fn delay_us(&mut self, us: u8) {
        FreeRtos::delay_ns(us as _);
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

impl embedded_hal::delay::DelayNs for FreeRtos {
    fn delay_ns(&mut self, ns: u32) {
        FreeRtos::delay_ns(ns)
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms)
    }
}

/// A delay provider that uses [`Ets`] for delays below a certain threshold
/// and [`FreeRtos`] for delays equal or above the threshold.
#[derive(Copy, Clone)]
pub struct Delay(u32);

impl Delay {
    /// Create a delay with a default threshold of 1ms
    pub const fn new_default() -> Self {
        Self::new(1000)
    }

    pub const fn new(threshold: u32) -> Self {
        Self(threshold)
    }

    #[deprecated = "Use delay_ns instead"]
    pub fn delay_us(&self, us: u32) {
        self.delay_ns(us)
    }

    pub fn delay_ns(&self, ns: u32) {
        if ns < self.0 {
            Ets::delay_ns(ns);
        } else {
            FreeRtos::delay_ns(ns);
        }
    }

    pub fn delay_ms(&self, ms: u32) {
        if ms * 1000 < self.0 {
            Ets::delay_ms(ms);
        } else {
            FreeRtos::delay_ms(ms);
        }
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, us: u32) {
        Delay::delay_ns(self, us)
    }

    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        Delay::delay_ns(self, us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_ns(self, us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        Delay::delay_ms(self, ms as _)
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}
