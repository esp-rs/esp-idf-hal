#[cfg(feature = "ulp")]
mod pac;
#[cfg(feature = "ulp")]
mod reg;

#[cfg(feature = "ulp")]
#[macro_use]
pub mod sys;
#[cfg(feature = "ulp")]
pub mod delay;

#[cfg(not(feature = "ulp"))]
#[cfg(any(esp32, esp32s2, esp32s3))]
pub fn enable_timer(enable: bool) {
    use core::ptr::{read_volatile, write_volatile};

    // TODO: Get rid of these hard-codings
    const DR_REG_RTCCNTL_BASE: u32 = 0x3f408000;
    const RTC_CNTL_STATE0_REG: u32 = DR_REG_RTCCNTL_BASE + 0x0018;
    const RTC_CNTL_ULP_CP_SLP_TIMER_EN: u32 = 1 << 31;

    unsafe {
        if enable {
            write_volatile(
                RTC_CNTL_STATE0_REG as *mut u32,
                read_volatile(RTC_CNTL_STATE0_REG as *const u32) | RTC_CNTL_ULP_CP_SLP_TIMER_EN,
            );
        } else {
            write_volatile(
                RTC_CNTL_STATE0_REG as *mut u32,
                read_volatile(RTC_CNTL_STATE0_REG as *const u32) & !RTC_CNTL_ULP_CP_SLP_TIMER_EN,
            );
        }
    }
}
