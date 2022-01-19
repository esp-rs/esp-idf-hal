#![allow(dead_code)]

/// This module is a manual translation of a bunch of C files from current ESP-IDF master (currently ESP32-S2 specific):
/// - https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s2/include/soc/soc.h (a subset)
/// - https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s2/include/soc/sens_reg.h (a subset)
/// - https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s2/include/soc/rtc_io_reg.h (a subset)
use super::reg::bit;

pub const DR_REG_SENS_BASE: u32 = 0x3f408800;
pub const DR_REG_RTCIO_BASE: u32 = 0x3ff48400;
pub const DR_REG_RTCCNTL_BASE: u32 = 0x3f408000;

pub const RTC_CNTL_COCPU_CTRL_REG: u32 = DR_REG_RTCCNTL_BASE + 0x0100;
pub const RTC_CNTL_COCPU_DONE: u32 = bit(25);
pub const RTC_CNTL_COCPU_SHUT_RESET_EN: u32 = bit(22);
pub const RTC_CNTL_COCPU_SHUT_2_CLK_DIS: u32 = 0x000000FF;
pub const RTC_CNTL_COCPU_SHUT_2_CLK_DIS_V: u32 = 0xFF;
pub const RTC_CNTL_COCPU_SHUT_2_CLK_DIS_S: u32 = 14;

pub const RTC_CNTL_STATE0_REG: u32 = DR_REG_RTCCNTL_BASE + 0x0018;
pub const RTC_CNTL_SW_CPU_INT: u32 = bit(0);
pub const RTC_CNTL_ULP_CP_SLP_TIMER_EN: u32 = bit(31);
pub const RTC_CNTL_ULP_CP_SLP_TIMER_EN_V: u32 = 0x1;
pub const RTC_CNTL_ULP_CP_SLP_TIMER_EN_S: u32 = 31;

pub const SENS_SAR_IO_MUX_CONF_REG: u32 = DR_REG_SENS_BASE + 0x0144;
pub const SENS_IOMUX_CLK_GATE_EN_M: u32 = bit(31);

pub const RTC_IO_TOUCH_PAD0_REG: u32 = DR_REG_RTCIO_BASE + 0x84;
pub const RTC_IO_TOUCH_PAD0_DRV: u32 = 0x00000003;
pub const RTC_IO_TOUCH_PAD0_DRV_V: u32 = 0x3;
pub const RTC_IO_TOUCH_PAD0_DRV_S: u32 = 29;
pub const RTC_IO_TOUCH_PAD0_MUX_SEL: u32 = bit(19);
pub const RTC_IO_TOUCH_PAD0_FUN_SEL: u32 = 0x00000003;
pub const RTC_IO_TOUCH_PAD0_FUN_SEL_V: u32 = 0x3;
pub const RTC_IO_TOUCH_PAD0_FUN_SEL_S: u32 = 17;
pub const RTC_IO_TOUCH_PAD0_FUN_IE: u32 = bit(13);
pub const RTC_IO_TOUCH_PAD0_FUN_IE_V: u32 = 0x01;
pub const RTC_IO_TOUCH_PAD0_FUN_IE_S: u32 = 13;
pub const RTC_IO_TOUCH_PAD0_RUE: u32 = bit(27);
pub const RTC_IO_TOUCH_PAD0_RDE: u32 = bit(28);

pub const RTC_GPIO_ENABLE_W1TS_REG: u32 = DR_REG_RTCIO_BASE + 0x10;
pub const RTC_GPIO_ENABLE_W1TS: u32 = 0x0003FFFF;
pub const RTC_GPIO_ENABLE_W1TS_V: u32 = 0x3FFFF;
pub const RTC_GPIO_ENABLE_W1TS_S: u32 = 10;

pub const RTC_GPIO_ENABLE_W1TC_REG: u32 = DR_REG_RTCIO_BASE + 0x14;
pub const RTC_GPIO_ENABLE_W1TC: u32 = 0x0003FFFF;
pub const RTC_GPIO_ENABLE_W1TC_V: u32 = 0x3FFFF;
pub const RTC_GPIO_ENABLE_W1TC_S: u32 = 10;

pub const RTC_GPIO_IN_REG: u32 = DR_REG_RTCIO_BASE + 0x24;
pub const RTC_GPIO_IN_NEXT: u32 = 0x0003FFFF;
pub const RTC_GPIO_IN_NEXT_V: u32 = 0x3FFFF;
pub const RTC_GPIO_IN_NEXT_S: u32 = 10;

pub const RTC_GPIO_OUT_W1TS_REG: u32 = DR_REG_RTCIO_BASE + 0x4;
pub const RTC_GPIO_OUT_DATA_W1TS: u32 = 0x0003FFFF;
pub const RTC_GPIO_OUT_DATA_W1TS_V: u32 = 0x3FFFF;
pub const RTC_GPIO_OUT_DATA_W1TS_S: u32 = 10;

pub const RTC_GPIO_OUT_W1TC_REG: u32 = DR_REG_RTCIO_BASE + 0x8;
pub const RTC_GPIO_OUT_DATA_W1TC: u32 = 0x0003FFFF;
pub const RTC_GPIO_OUT_DATA_W1TC_V: u32 = 0x3FFFF;
pub const RTC_GPIO_OUT_DATA_W1TC_S: u32 = 10;
