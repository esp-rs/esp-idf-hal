use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use super::sys::*;

/// ESP-IDF busy-loop based delay for the risc-v ULP coprocessor
pub struct Ulp;

impl DelayUs<u32> for Ulp {
    fn delay_us(&mut self, us: u32) {
        delay_cycles(us * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM);
    }
}

impl DelayUs<u16> for Ulp {
    fn delay_us(&mut self, us: u16) {
        DelayUs::<u32>::delay_us(self, us as u32);
    }
}

impl DelayMs<u32> for Ulp {
    fn delay_ms(&mut self, ms: u32) {
        delay_cycles(ms * ULP_RISCV_CYCLES_PER_MS);
    }
}

impl DelayMs<u16> for Ulp {
    fn delay_ms(&mut self, ms: u16) {
        DelayMs::<u32>::delay_ms(self, ms as u32);
    }
}

#[inline(always)]
fn delay_cycles(cycles: u32) {
    let start = get_ccount();

    while get_ccount() - start < cycles {
        /* Wait */
    }
}
