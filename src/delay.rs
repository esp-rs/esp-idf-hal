//! Delay providers.
//!
//! If you don't know how large your delays will be, you'll probably want to
//! use [`Delay`]. Otherwise use [`Ets`] for delays <10ms and
//! [`FreeRtos`] for delays >=10ms.

use core::{cmp::min, time::Duration};

use esp_idf_sys::*;

pub use esp_idf_sys::TickType_t;

pub const BLOCK: TickType_t = TickType_t::MAX;
pub const NON_BLOCK: TickType_t = 0;
pub const TICK_RATE_HZ: u32 = configTICK_RATE_HZ;

const MS_PER_S: u64 = 1_000;
const NS_PER_MS: u64 = 1_000_000;
const US_PER_MS: u32 = 1_000;
const NS_PER_US: u32 = 1_000;

#[repr(transparent)]
pub struct TickType(pub TickType_t);

impl TickType {
    pub const fn new(ticks: TickType_t) -> Self {
        Self(ticks)
    }

    pub fn new_millis(ms: u64) -> Self {
        let ticks = ms
            .saturating_mul(TICK_RATE_HZ as u64)
            .saturating_add(MS_PER_S - 1)
            / MS_PER_S;
        Self(min(ticks, TickType_t::MAX as _) as _)
    }

    pub const fn ticks(&self) -> TickType_t {
        self.0
    }

    pub fn as_millis(&self) -> u64 {
        (self.0 as u64)
            .saturating_mul(MS_PER_S)
            .saturating_add(TICK_RATE_HZ as u64 - 1)
            / TICK_RATE_HZ as u64
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
        let sec_ms = duration.as_secs().saturating_mul(MS_PER_S);
        let subsec_ns: u64 = duration.subsec_nanos().into();
        // Convert to ms and round up. Not saturating. Cannot overflow.
        let subsec_ms = (subsec_ns + (NS_PER_MS - 1)) / NS_PER_MS;

        TickType::new_millis(sec_ms.saturating_add(subsec_ms))
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
        Duration::from_millis(ticks.as_millis())
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

// This binding is no longer available in the generated bindings for ESP-IDF 5 or later.
// The function itself is still available. Therefore, we define the binding here.
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
        Self::delay_us(ms.saturating_mul(US_PER_MS));
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Ets {
    fn delay_us(&mut self, us: u32) {
        Ets::delay_us(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Ets {
    fn delay_us(&mut self, us: u16) {
        Ets::delay_us(us.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for Ets {
    fn delay_us(&mut self, us: u8) {
        Ets::delay_us(us.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Ets {
    fn delay_ms(&mut self, ms: u32) {
        Ets::delay_ms(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Ets {
    fn delay_ms(&mut self, ms: u16) {
        Ets::delay_ms(ms.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for Ets {
    fn delay_ms(&mut self, ms: u8) {
        Ets::delay_ms(ms.into());
    }
}

impl embedded_hal::delay::DelayNs for Ets {
    fn delay_ns(&mut self, ns: u32) {
        Ets::delay_us(ns.saturating_add(NS_PER_US - 1) / NS_PER_US);
    }

    fn delay_us(&mut self, us: u32) {
        Ets::delay_us(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        Ets::delay_ms(ms);
    }
}

/// FreeRTOS-based delay provider for delays larger than 10ms
///
/// Ddelays smaller than 10ms used in a loop would starve the FreeRTOS IDLE
/// tasks' as they are low prio tasks and hence the the IDLE tasks' watchdog
/// will trigger.
pub struct FreeRtos;

impl FreeRtos {
    pub fn delay_ms(ms: u32) {
        let ticks = TickType::new_millis(ms.into()).ticks();
        unsafe {
            vTaskDelay(ticks);
        }
    }

    // Internal helper: Round up to ms.
    // This is not supposed to be `pub`, because the user code shall not use this
    // timer for microsecond delay. Only used for trait impl below.
    fn delay_us(us: u32) {
        Self::delay_ms(us.saturating_add(US_PER_MS - 1) / US_PER_MS);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for FreeRtos {
    fn delay_us(&mut self, us: u32) {
        FreeRtos::delay_us(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for FreeRtos {
    fn delay_us(&mut self, us: u16) {
        FreeRtos::delay_us(us.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for FreeRtos {
    fn delay_us(&mut self, us: u8) {
        FreeRtos::delay_us(us.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for FreeRtos {
    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for FreeRtos {
    fn delay_ms(&mut self, ms: u16) {
        FreeRtos::delay_ms(ms.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for FreeRtos {
    fn delay_ms(&mut self, ms: u8) {
        FreeRtos::delay_ms(ms.into());
    }
}

impl embedded_hal::delay::DelayNs for FreeRtos {
    fn delay_ns(&mut self, ns: u32) {
        FreeRtos::delay_us(ns.saturating_add(NS_PER_US - 1) / NS_PER_US);
    }

    fn delay_us(&mut self, us: u32) {
        FreeRtos::delay_us(us);
    }

    fn delay_ms(&mut self, ms: u32) {
        FreeRtos::delay_ms(ms);
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

    /// Create a delay with a threshold of the specified amount of microseconds.
    pub const fn new(threshold_us: u32) -> Self {
        Self(threshold_us)
    }

    pub fn delay_us(&self, us: u32) {
        if us < self.0 {
            Ets::delay_us(us);
        } else {
            FreeRtos::delay_us(us);
        }
    }

    pub fn delay_ms(&self, ms: u32) {
        if ms.saturating_mul(US_PER_MS) < self.0 {
            Ets::delay_ms(ms);
        } else {
            FreeRtos::delay_ms(ms);
        }
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        Delay::delay_us(self, ns.saturating_add(NS_PER_US - 1) / NS_PER_US)
    }

    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us)
    }

    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        Delay::delay_us(self, us.into());
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        Delay::delay_ms(self, ms.into())
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}
