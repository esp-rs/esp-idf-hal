use core::ptr;
use embedded_hal::serial::{Read, Write};
use esp32_sys::{
    uart_config_t, uart_driver_install, uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
    uart_param_config, uart_parity_t_UART_PARITY_DISABLE, uart_port_t_UART_NUM_0, uart_read_bytes,
    uart_set_pin, uart_stop_bits_t_UART_STOP_BITS_1, uart_wait_tx_done,
    uart_word_length_t_UART_DATA_8_BITS, uart_write_bytes, ESP_ERR_TIMEOUT, ESP_OK,
    UART_PIN_NO_CHANGE,
};

const BUF_SIZE: i32 = 1024;

#[derive(Clone, Debug)]
pub enum Error {}

pub struct Uart0;

impl Uart0 {
    pub unsafe fn new(tx_pin: i32, rx_pin: i32) -> Self {
        let uart_config = uart_config_t {
            baud_rate: 115200,
            data_bits: uart_word_length_t_UART_DATA_8_BITS,
            parity: uart_parity_t_UART_PARITY_DISABLE,
            stop_bits: uart_stop_bits_t_UART_STOP_BITS_1,
            flow_ctrl: uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
            rx_flow_ctrl_thresh: 0,
            use_ref_tick: false,
        };

        unsafe {
            uart_param_config(uart_port_t_UART_NUM_0, &uart_config);
            uart_set_pin(
                uart_port_t_UART_NUM_0,
                tx_pin,
                rx_pin,
                UART_PIN_NO_CHANGE, // RTS
                UART_PIN_NO_CHANGE, // CTS
            );
            uart_driver_install(
                uart_port_t_UART_NUM_0,
                BUF_SIZE * 2,
                0,
                0,
                ptr::null_mut(),
                0,
            );
        }

        Self
    }
}

impl Write<u8> for Uart0 {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        unsafe {
            uart_write_bytes(uart_port_t_UART_NUM_0, &word as *const u8 as *const _, 1);
        }

        Ok(())
    }

    /// Ensures that none of the previously written words are still buffered
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        let result = unsafe { uart_wait_tx_done(uart_port_t_UART_NUM_0, 0) };
        match result as u32 {
            ESP_OK => Ok(()),
            ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            _ => unreachable!(),
        }
    }
}

impl Read<u8> for Uart0 {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut c = 0;
        let n =
            unsafe { uart_read_bytes(uart_port_t_UART_NUM_0, &mut c as *mut u8 as *mut _, 1, 0) };
        match n {
            -1 => Err(nb::Error::WouldBlock),
            1 => Ok(c),
            _ => unreachable!(),
        }
    }
}
