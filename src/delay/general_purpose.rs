use crate::delay::{Ets, FreeRtos};

/// A delay provider that uses [`Ets`] for delays <10ms, and [`FreeRtos`] for
/// delays >=10 ms
#[derive(Copy, Clone)]
pub struct Delay;

impl Delay {
    pub fn delay_us(us: u32) {
        if us < 10_000 {
            Ets::delay_us(us);
        } else {
            FreeRtos::delay_us(us);
        }
    }

    pub fn delay_ms(ms: u32) {
        if ms < 10 {
            Ets::delay_ms(ms);
        } else {
            FreeRtos::delay_ms(ms);
        }
    }
}

impl embedded_hal::delay::DelayUs for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(us)
    }

    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(ms)
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        Delay::delay_us(us as _);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(us);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        Delay::delay_ms(ms as _)
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(ms)
    }
}
