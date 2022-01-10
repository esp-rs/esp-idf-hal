use embedded_hal::delay::blocking::DelayUs;

use super::sys::*;

/// Busy-loop based delay for the RiscV ULP coprocessor
pub struct Ulp;

impl DelayUs for Ulp {
    type Error = core::convert::Infallible;

    fn delay_us(&mut self, us: u32) -> Result<(), Self::Error> {
        delay_cycles(us * ULP_RISCV_CYCLES_PER_US_NUM / ULP_RISCV_CYCLES_PER_US_DENUM);
        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        delay_cycles(ms * ULP_RISCV_CYCLES_PER_MS);
        Ok(())
    }
}

#[inline(always)]
fn delay_cycles(cycles: u32) {
    let start = get_ccount();

    while get_ccount() - start < cycles { /* Wait */ }
}
