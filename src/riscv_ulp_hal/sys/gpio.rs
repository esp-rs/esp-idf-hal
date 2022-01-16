use crate::ulp::pac::*;
/// A mini "esp-idf-ulp-sys" module exposing stuff on top of which the ULP HAL support is implemented
/// (currently, only GPIO)
/// Implemented as a manual transation of a few C fields from current ESP-IDF S2 master:
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv_gpio.h
use crate::ulp::reg::*;

#[allow(non_camel_case_types)]
pub type adc_unit_t = i32;
#[allow(non_camel_case_types)]
pub type adc_channel_t = i32;
#[allow(non_camel_case_types)]
pub type dac_channel_t = i32;
#[allow(non_camel_case_types)]
pub type touch_pad_t = i32;
#[allow(non_camel_case_types)]
pub type adc_atten_t = i32;

#[allow(non_upper_case_globals)]
pub const adc_unit_t_ADC_UNIT_1: adc_unit_t = 0;
#[allow(non_upper_case_globals)]
pub const adc_unit_t_ADC_UNIT_2: adc_unit_t = 1;

#[allow(non_upper_case_globals)]
pub const adc_atten_t_ADC_ATTEN_DB_0: adc_atten_t = 0;
#[allow(non_upper_case_globals)]
pub const adc_atten_t_ADC_ATTEN_DB_2_5: adc_atten_t = 1;
#[allow(non_upper_case_globals)]
pub const adc_atten_t_ADC_ATTEN_DB_6: adc_atten_t = 2;
#[allow(non_upper_case_globals)]
pub const adc_atten_t_ADC_ATTEN_DB_11: adc_atten_t = 3;

#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_DISABLE: u8 = 0;
#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_INPUT: u8 = 1;
#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_OUTPUT: u8 = 2;
#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_INPUT_OUTPUT: u8 = 3;
#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_OUTPUT_OD: u8 = 4;
#[allow(non_upper_case_globals)]
pub const gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD: u8 = 5;

#[allow(non_upper_case_globals)]
pub const gpio_pull_mode_t_GPIO_PULLUP_ONLY: u8 = 0;
#[allow(non_upper_case_globals)]
pub const gpio_pull_mode_t_GPIO_PULLDOWN_ONLY: u8 = 1;
#[allow(non_upper_case_globals)]
pub const gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN: u8 = 2;
#[allow(non_upper_case_globals)]
pub const gpio_pull_mode_t_GPIO_FLOATING: u8 = 3;

#[inline(always)]
pub unsafe fn gpio_set_direction(gpio_num: i32, direction: u8) {
    if direction == gpio_mode_t_GPIO_MODE_DISABLE {
        // Deinit
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_MUX_SEL,
        );
        return;
    } else {
        // Init
        set_peri_reg_mask(SENS_SAR_IO_MUX_CONF_REG, SENS_IOMUX_CLK_GATE_EN_M);
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_MUX_SEL,
        );
        reg_set_field(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_FUN_SEL_S,
            RTC_IO_TOUCH_PAD0_FUN_SEL_V,
            0,
        );
    }

    let input = direction == gpio_mode_t_GPIO_MODE_INPUT
        || direction == gpio_mode_t_GPIO_MODE_INPUT_OUTPUT
        || direction == gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD;
    let output = direction == gpio_mode_t_GPIO_MODE_OUTPUT
        || direction == gpio_mode_t_GPIO_MODE_OUTPUT_OD
        || direction == gpio_mode_t_GPIO_MODE_INPUT_OUTPUT
        || direction == gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD;
    let od = direction == gpio_mode_t_GPIO_MODE_OUTPUT_OD
        || direction == gpio_mode_t_GPIO_MODE_INPUT_OUTPUT_OD;

    if input {
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_FUN_IE,
        );
    } else {
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_FUN_IE,
        );
    }

    if output {
        reg_set_field(
            RTC_GPIO_ENABLE_W1TS_REG,
            RTC_GPIO_ENABLE_W1TS_S,
            RTC_GPIO_ENABLE_W1TS_V,
            bit(gpio_num as u32),
        );
        reg_set_field(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_DRV_S,
            RTC_IO_TOUCH_PAD0_DRV_V,
            if od { 1 } else { 0 },
        );
    } else {
        reg_set_field(
            RTC_GPIO_ENABLE_W1TC_REG,
            RTC_GPIO_ENABLE_W1TC_S,
            RTC_GPIO_ENABLE_W1TC_V,
            bit(gpio_num as u32),
        );
    }
}

#[inline(always)]
pub unsafe fn gpio_set_pull_mode(gpio_num: i32, mode: u8) {
    let pullup =
        mode == gpio_pull_mode_t_GPIO_PULLUP_ONLY || mode == gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN;
    let pulldown = mode == gpio_pull_mode_t_GPIO_PULLDOWN_ONLY
        || mode == gpio_pull_mode_t_GPIO_PULLUP_PULLDOWN;

    if pullup {
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_RUE,
        );
    } else {
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_RUE,
        );
    }

    if pulldown {
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_RDE,
        );
    } else {
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num as u32 * 4,
            RTC_IO_TOUCH_PAD0_RDE,
        );
    }
}

#[inline(always)]
pub unsafe fn gpio_get_level(gpio_num: i32) -> u8 {
    if (reg_get_field(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, RTC_GPIO_IN_NEXT_V)
        & bit(gpio_num as u32))
        != 0
    {
        1
    } else {
        0
    }
}

#[inline(always)]
pub unsafe fn gpio_set_level(gpio_num: i32, level: u8) {
    if level != 0 {
        reg_set_field(
            RTC_GPIO_OUT_W1TS_REG,
            RTC_GPIO_OUT_DATA_W1TS_S,
            RTC_GPIO_OUT_DATA_W1TS_V,
            bit(gpio_num as u32),
        );
    } else {
        reg_set_field(
            RTC_GPIO_OUT_W1TC_REG,
            RTC_GPIO_OUT_DATA_W1TC_S,
            RTC_GPIO_OUT_DATA_W1TC_V,
            bit(gpio_num as u32),
        );
    }
}

#[inline(always)]
pub unsafe fn gpio_get_output_level(gpio_num: i32) -> u8 {
    if (reg_get_field(
        RTC_GPIO_OUT_W1TS_REG,
        RTC_GPIO_OUT_DATA_W1TS_S,
        RTC_GPIO_OUT_DATA_W1TS_V,
    ) & bit(gpio_num as u32))
        != 0
    {
        1
    } else {
        0
    }
}
