use super::sys::*;

/// Busy-loop based delay for the RiscV ULP coprocessor
pub struct Ulp;

impl embedded_hal_0_2::blocking::delay::DelayUs<u32> for Ulp {
    fn delay_us(&mut self, us: u32) {
        delay_cycles(us * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u16> for Ulp {
    fn delay_us(&mut self, us: u16) {
        delay_cycles(us as u32 * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayUs<u8> for Ulp {
    fn delay_us(&mut self, us: u8) {
        delay_cycles(us as u32 * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Ulp {
    fn delay_ms(&mut self, ms: u32) {
        delay_cycles(ms * ULP_RISCV_CYCLES_PER_MS);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Ulp {
    fn delay_ms(&mut self, ms: u16) {
        delay_cycles(ms as u32 * ULP_RISCV_CYCLES_PER_MS);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u8> for Ulp {
    fn delay_ms(&mut self, ms: u8) {
        delay_cycles(ms as u32 * ULP_RISCV_CYCLES_PER_MS);
    }
}

impl embedded_hal::delay::DelayUs for Ulp {
    fn delay_us(&mut self, us: u32) {
        delay_cycles(us * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM)
    }

    fn delay_ms(&mut self, ms: u32) {
        delay_cycles(ms * ULP_RISCV_CYCLES_PER_MS)
    }
}

#[inline(always)]
fn delay_cycles(cycles: u32) {
    let start = get_ccount();

    while get_ccount() - start < cycles { /* Wait */ }
}
