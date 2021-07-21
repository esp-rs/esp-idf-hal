#![allow(dead_code)]

/// This module is a manual translation of the following C file from current ESP-IDF master:
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/include/ulp_riscv/ulp_utils.h
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/ulp_utils.c

pub const ULP_RISCV_CYCLES_PER_US_NUM: u32 = 80;
pub const ULP_RISCV_CYCLES_PER_US_DENUM: u32 = 5;
pub const ULP_RISCV_CYCLES_PER_MS: u32 = ULP_RISCV_CYCLES_PER_US_NUM * (1000 / ULP_RISCV_CYCLES_PER_US_DENUM);

use crate::ulp::reg::*;
use crate::ulp::pac::*;

#[inline(always)]
pub fn get_ccount() -> u32 {
    #[allow(unused_assignments)]
    let mut ccount = 0;

    unsafe {
        llvm_asm!("rdcycle $0" : "=r"(ccount) : : : "volatile");
    }

    ccount
}

pub unsafe fn wakeup_main_processor() {
    set_peri_reg_mask(RTC_CNTL_STATE0_REG, RTC_CNTL_SW_CPU_INT);
}

pub unsafe fn rescue_from_monitor() {
    // Rescue RISCV from monitor state
    clear_peri_reg_mask(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_DONE | RTC_CNTL_COCPU_SHUT_RESET_EN);
}

pub unsafe fn shutdown() -> ! {
    // Setting the delay time after RISCV recv `DONE` signal, Ensure that action `RESET` can be executed in time.
    reg_set_field(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_SHUT_2_CLK_DIS, RTC_CNTL_COCPU_SHUT_2_CLK_DIS_V, 0x3F);

    // Suspends the ulp operation
    set_peri_reg_mask(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_DONE);

    // Resets the processor
    set_peri_reg_mask(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_SHUT_RESET_EN);

    loop { }
}
