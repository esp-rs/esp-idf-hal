/// A mini "esp-idf-ulp-sys" module exposing stuff on top of which the ULP HAL support is implemented
/// (currently, only GPIO)
/// Implemented as a manual transation of a few C fields from current ESP-IDF S2 master:
/// - https://github.com/espressif/esp-idf/blob/master/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv_gpio.h
use crate::riscv_ulp_hal::pac::*;
use crate::riscv_ulp_hal::reg::*;

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct adc_unit_t(pub i32);

impl adc_unit_t {
    pub const ADC_UNIT_1: Self = Self(0);
    pub const ADC_UNIT_2: Self = Self(1);
}

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct adc_channel_t(pub i32);

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct dac_channel_t(pub i32);

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct touch_pad_t(pub i32);

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct adc_atten_t(pub i32);

impl adc_atten_t {
    pub const ADC_ATTEN_DB_0: Self = Self(0);
    pub const ADC_ATTEN_DB_2_5: Self = Self(1);
    pub const ADC_ATTEN_DB_6: Self = Self(2);
    pub const ADC_ATTEN_DB_11: Self = Self(3);
    pub const ADC_ATTEN_MAX: Self = Self(4);
}

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct gpio_num_t(pub i32);

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct gpio_mode_t(pub u8);

impl gpio_mode_t {
    pub const GPIO_MODE_DISABLE: Self = Self(0);
    pub const GPIO_MODE_INPUT: Self = Self(1);
    pub const GPIO_MODE_OUTPUT: Self = Self(2);
    pub const GPIO_MODE_INPUT_OUTPUT: Self = Self(3);
    pub const GPIO_MODE_OUTPUT_OD: Self = Self(4);
    pub const GPIO_MODE_INPUT_OUTPUT_OD: Self = Self(5);
}

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub struct gpio_pull_mode_t(pub u8);

impl gpio_pull_mode_t {
    pub const GPIO_PULLUP_ONLY: Self = Self(0);
    pub const GPIO_PULLDOWN_ONLY: Self = Self(1);
    pub const GPIO_PULLUP_PULLDOWN: Self = Self(2);
    pub const GPIO_FLOATING: Self = Self(3);
}

#[inline(always)]
pub unsafe fn gpio_set_direction(gpio_num: gpio_num_t, direction: gpio_mode_t) {
    let gpio_num = gpio_num.0 as u32;

    if direction == gpio_mode_t::GPIO_MODE_DISABLE {
        // Deinit
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_MUX_SEL,
        );
        return;
    } else {
        // Init
        set_peri_reg_mask(SENS_SAR_IO_MUX_CONF_REG, SENS_IOMUX_CLK_GATE_EN_M);
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_MUX_SEL,
        );
        reg_set_field(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_FUN_SEL_S,
            RTC_IO_TOUCH_PAD0_FUN_SEL_V,
            0,
        );
    }

    let input = direction == gpio_mode_t::GPIO_MODE_INPUT
        || direction == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT
        || direction == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT_OD;
    let output = direction == gpio_mode_t::GPIO_MODE_OUTPUT
        || direction == gpio_mode_t::GPIO_MODE_OUTPUT_OD
        || direction == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT
        || direction == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT_OD;
    let od = direction == gpio_mode_t::GPIO_MODE_OUTPUT_OD
        || direction == gpio_mode_t::GPIO_MODE_INPUT_OUTPUT_OD;

    if input {
        set_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_FUN_IE,
        );
    } else {
        clear_peri_reg_mask(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_FUN_IE,
        );
    }

    if output {
        reg_set_field(
            RTC_GPIO_ENABLE_W1TS_REG,
            RTC_GPIO_ENABLE_W1TS_S,
            RTC_GPIO_ENABLE_W1TS_V,
            bit(gpio_num),
        );
        reg_set_field(
            RTC_IO_TOUCH_PAD0_REG + gpio_num * 4,
            RTC_IO_TOUCH_PAD0_DRV_S,
            RTC_IO_TOUCH_PAD0_DRV_V,
            if od { 1 } else { 0 },
        );
    } else {
        reg_set_field(
            RTC_GPIO_ENABLE_W1TC_REG,
            RTC_GPIO_ENABLE_W1TC_S,
            RTC_GPIO_ENABLE_W1TC_V,
            bit(gpio_num),
        );
    }
}

#[inline(always)]
pub unsafe fn gpio_set_pull_mode(gpio_num: gpio_num_t, mode: gpio_pull_mode_t) {
    let gpio_num = gpio_num.0 as u32;

    let pullup = mode == gpio_pull_mode_t::GPIO_PULLUP_ONLY
        || mode == gpio_pull_mode_t::GPIO_PULLUP_PULLDOWN;
    let pulldown = mode == gpio_pull_mode_t::GPIO_PULLDOWN_ONLY
        || mode == gpio_pull_mode_t::GPIO_PULLUP_PULLDOWN;

    if pullup {
        set_peri_reg_mask(RTC_IO_TOUCH_PAD0_REG + gpio_num * 4, RTC_IO_TOUCH_PAD0_RUE);
    } else {
        clear_peri_reg_mask(RTC_IO_TOUCH_PAD0_REG + gpio_num * 4, RTC_IO_TOUCH_PAD0_RUE);
    }

    if pulldown {
        set_peri_reg_mask(RTC_IO_TOUCH_PAD0_REG + gpio_num * 4, RTC_IO_TOUCH_PAD0_RDE);
    } else {
        clear_peri_reg_mask(RTC_IO_TOUCH_PAD0_REG + gpio_num * 4, RTC_IO_TOUCH_PAD0_RDE);
    }
}

#[inline(always)]
pub unsafe fn gpio_get_level(gpio_num: gpio_num_t) -> u8 {
    if (reg_get_field(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, RTC_GPIO_IN_NEXT_V)
        & bit(gpio_num.0 as u32))
        != 0
    {
        1
    } else {
        0
    }
}

#[inline(always)]
pub unsafe fn gpio_set_level(gpio_num: gpio_num_t, level: u8) {
    if level != 0 {
        reg_set_field(
            RTC_GPIO_OUT_W1TS_REG,
            RTC_GPIO_OUT_DATA_W1TS_S,
            RTC_GPIO_OUT_DATA_W1TS_V,
            bit(gpio_num.0 as u32),
        );
    } else {
        reg_set_field(
            RTC_GPIO_OUT_W1TC_REG,
            RTC_GPIO_OUT_DATA_W1TC_S,
            RTC_GPIO_OUT_DATA_W1TC_V,
            bit(gpio_num.0 as u32),
        );
    }
}

#[inline(always)]
pub unsafe fn gpio_get_output_level(gpio_num: gpio_num_t) -> u8 {
    if (reg_get_field(
        RTC_GPIO_OUT_W1TS_REG,
        RTC_GPIO_OUT_DATA_W1TS_S,
        RTC_GPIO_OUT_DATA_W1TS_V,
    ) & bit(gpio_num.0 as u32))
        != 0
    {
        1
    } else {
        0
    }
}
