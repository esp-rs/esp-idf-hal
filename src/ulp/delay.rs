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
        DelayUs::<u32>::delay_us(self, us as u32);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u32> for Ulp {
    fn delay_ms(&mut self, ms: u32) {
        delay_cycles(ms * ULP_RISCV_CYCLES_PER_MS);
    }
}

impl embedded_hal_0_2::blocking::delay::DelayMs<u16> for Ulp {
    fn delay_ms(&mut self, ms: u16) {
        DelayMs::<u32>::delay_ms(self, ms as u32);
    }
}

impl embedded_hal::delay::blocking::DelayUs for Ulp {
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
