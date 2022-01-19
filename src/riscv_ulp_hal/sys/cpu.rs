#![allow(dead_code)]

/// This module is a manual translation of the following C file from current ESP-IDF master:
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/include/ulp_riscv/ulp_utils.h
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/ulp_utils.c
use core::arch::asm;

use crate::riscv_ulp_hal::pac::*;
use crate::riscv_ulp_hal::reg::*;

pub const ULP_RISCV_CYCLES_PER_US_NUM: u32 = 85;
pub const ULP_RISCV_CYCLES_PER_US_DENUM: u32 = 10;
pub const ULP_RISCV_CYCLES_PER_MS: u32 =
    ULP_RISCV_CYCLES_PER_US_NUM * (1000 / ULP_RISCV_CYCLES_PER_US_DENUM);

#[inline(always)]
pub fn get_ccount() -> u32 {
    #[allow(unused_assignments)]
    let mut ccount = 0;

    unsafe {
        asm!("rdcycle {}", out(reg) ccount);
    }

    ccount
}

pub fn wakeup_main_processor() {
    unsafe { set_peri_reg_mask(RTC_CNTL_STATE0_REG, RTC_CNTL_SW_CPU_INT) };
}

pub fn rescue_from_monitor() {
    // Rescue RISCV from monitor state
    unsafe {
        clear_peri_reg_mask(
            RTC_CNTL_COCPU_CTRL_REG,
            RTC_CNTL_COCPU_DONE | RTC_CNTL_COCPU_SHUT_RESET_EN,
        )
    };
}

pub fn enable_timer(enable: bool) {
    unsafe {
        if enable {
            set_peri_reg_mask(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
        } else {
            clear_peri_reg_mask(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
        }
    }
}

pub fn shutdown() -> ! {
    unsafe {
        // Setting the delay time after RISCV recv `DONE` signal, Ensure that action `RESET` can be executed in time.
        reg_set_field(
            RTC_CNTL_COCPU_CTRL_REG,
            RTC_CNTL_COCPU_SHUT_2_CLK_DIS_S,
            RTC_CNTL_COCPU_SHUT_2_CLK_DIS_V,
            0x3F,
        );

        // Suspends the ulp operation
        set_peri_reg_mask(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_DONE);

        // Resets the processor
        set_peri_reg_mask(RTC_CNTL_COCPU_CTRL_REG, RTC_CNTL_COCPU_SHUT_RESET_EN);
    }

    #[allow(clippy::empty_loop)]
    loop {}
}
