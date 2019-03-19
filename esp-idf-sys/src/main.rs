#![no_std]
#![no_main]

extern crate esp32_sys;

use core::panic::PanicInfo;
use esp32_sys::*;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const BLINK_GPIO: gpio_num_t = gpio_num_t_GPIO_NUM_2;

#[no_mangle]
pub fn app_main() {
    unsafe {
        rust_blink();
    }
}

unsafe fn rust_blink() {
    gpio_pad_select_gpio(BLINK_GPIO);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, gpio_mode_t_GPIO_MODE_OUTPUT);

    loop {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        ets_delay_us(1_000_000);

        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        ets_delay_us(1_000_000);
    }
}

