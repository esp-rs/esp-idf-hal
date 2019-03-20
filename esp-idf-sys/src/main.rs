#![no_std]
#![no_main]

extern crate esp32_sys;

use core::panic::PanicInfo;
use core::ptr;
use esp32_sys::*;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const BLINK_GPIO: gpio_num_t = gpio_num_t_GPIO_NUM_2;
const UART_NUM: uart_port_t = uart_port_t_UART_NUM_1;
const ECHO_TEST_TXD: i32 = gpio_num_t_GPIO_NUM_17 as i32;
const ECHO_TEST_RXD: i32 = gpio_num_t_GPIO_NUM_16 as i32;
const ECHO_TEST_RTS: i32 = UART_PIN_NO_CHANGE;
const ECHO_TEST_CTS: i32 = UART_PIN_NO_CHANGE;

const BUF_SIZE: i32 = 1024;

#[no_mangle]
pub fn app_main() {
    unsafe {
        rust_blink_and_write();
    }
}

unsafe fn rust_blink_and_write() {
    gpio_pad_select_gpio(BLINK_GPIO as u8);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, gpio_mode_t_GPIO_MODE_OUTPUT);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    let uart_config = uart_config_t {
        baud_rate: 115200,
        data_bits: uart_word_length_t_UART_DATA_8_BITS,
        parity: uart_parity_t_UART_PARITY_DISABLE,
        stop_bits: uart_stop_bits_t_UART_STOP_BITS_1,
        flow_ctrl: uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
        rx_flow_ctrl_thresh: 0,
        use_ref_tick: false,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, ptr::null_mut(), 0);

    loop {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        ets_delay_us(1_000_000);

        // Write data to UART.
        let test_str = "This is a test string.\n";
        uart_write_bytes(UART_NUM, test_str.as_ptr() as *const _, test_str.len());

        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        ets_delay_us(1_000_000);
    }
}
