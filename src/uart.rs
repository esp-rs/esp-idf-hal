//! UART peripheral control
//!
//! Controls UART peripherals (UART0, UART1, UART2).
//! Notice that UART0 is typically already used for loading firmware and logging.
//! Therefore use UART1 and UART2 in your application.
//! Any pin can be used for `rx` and `tx`.
//!
//! # Example
//!
//! Create a serial peripheral and write to serial port.
//! ```
//! use std::fmt::Write;
//! use esp_idf_hal::prelude::*;
//! use esp_idf_hal::serial;
//!
//! let peripherals = Peripherals::take().unwrap();
//! let pins = peripherals.pins;
//!
//! let config = uart::config::Config::default().baudrate(Hertz(115_200));
//!
//! let mut uart: uart::UartDriver = uart::UartDriver::new(
//!     peripherals.uart1,
//!     pins.gpio1,
//!     pins.gpio3,
//!     None,
//!     None,
//!     &config
//! ).unwrap();
//!
//! for i in 0..10 {
//!     writeln!(uart, "{:}", format!("count {:}", i)).unwrap();
//! }
//! ```
//!
//! # TODO
//! - Add all extra features esp32 supports (eg rs485, etc. etc.)
//! - Free APB lock when TX is idle (and no RX used)
//! - Address errata 3.17: UART fifo_cnt is inconsistent with FIFO pointer

use core::marker::PhantomData;
use core::ptr;

use crate::delay::NON_BLOCK;
use crate::gpio::*;
use crate::units::*;

use esp_idf_sys::*;

use crate::peripheral::{Peripheral, PeripheralRef};

const UART_FIFO_SIZE: i32 = 128;

pub type UartConfig = config::Config;

/// UART configuration
pub mod config {
    use crate::units::*;
    use esp_idf_sys::*;

    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum DataBits {
        DataBits5,
        DataBits6,
        DataBits7,
        DataBits8,
    }

    impl From<DataBits> for uart_word_length_t {
        fn from(data_bits: DataBits) -> Self {
            match data_bits {
                DataBits::DataBits5 => uart_word_length_t_UART_DATA_5_BITS,
                DataBits::DataBits6 => uart_word_length_t_UART_DATA_6_BITS,
                DataBits::DataBits7 => uart_word_length_t_UART_DATA_7_BITS,
                DataBits::DataBits8 => uart_word_length_t_UART_DATA_8_BITS,
            }
        }
    }

    impl From<uart_word_length_t> for DataBits {
        #[allow(non_upper_case_globals)]
        fn from(word_length: uart_word_length_t) -> Self {
            match word_length {
                uart_word_length_t_UART_DATA_5_BITS => DataBits::DataBits5,
                uart_word_length_t_UART_DATA_6_BITS => DataBits::DataBits6,
                uart_word_length_t_UART_DATA_7_BITS => DataBits::DataBits7,
                uart_word_length_t_UART_DATA_8_BITS => DataBits::DataBits8,
                _ => unreachable!(),
            }
        }
    }

    /// Number of data bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum FlowControl {
        None,
        RTS,
        CTS,
        CTSRTS,
        MAX,
    }

    impl From<FlowControl> for uart_hw_flowcontrol_t {
        fn from(flow_control: FlowControl) -> Self {
            match flow_control {
                FlowControl::None => uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
                FlowControl::RTS => uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_RTS,
                FlowControl::CTS => uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_CTS,
                FlowControl::CTSRTS => uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_CTS_RTS,
                FlowControl::MAX => uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_MAX,
            }
        }
    }

    impl From<uart_hw_flowcontrol_t> for FlowControl {
        #[allow(non_upper_case_globals)]
        fn from(flow_control: uart_hw_flowcontrol_t) -> Self {
            match flow_control {
                uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE => FlowControl::None,
                uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_RTS => FlowControl::RTS,
                uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_CTS => FlowControl::CTS,
                uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_CTS_RTS => FlowControl::CTSRTS,
                uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_MAX => FlowControl::MAX,
                _ => unreachable!(),
            }
        }
    }

    /// Parity check
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    impl From<Parity> for uart_parity_t {
        fn from(parity: Parity) -> Self {
            match parity {
                Parity::ParityNone => uart_parity_t_UART_PARITY_DISABLE,
                Parity::ParityEven => uart_parity_t_UART_PARITY_EVEN,
                Parity::ParityOdd => uart_parity_t_UART_PARITY_ODD,
            }
        }
    }

    impl From<uart_parity_t> for Parity {
        #[allow(non_upper_case_globals)]
        fn from(parity: uart_parity_t) -> Self {
            match parity {
                uart_parity_t_UART_PARITY_DISABLE => Parity::ParityNone,
                uart_parity_t_UART_PARITY_EVEN => Parity::ParityEven,
                uart_parity_t_UART_PARITY_ODD => Parity::ParityOdd,
                _ => unreachable!(),
            }
        }
    }

    /// Number of stop bits
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum StopBits {
        /// 1 stop bit
        STOP1,
        /// 1.5 stop bits
        STOP1P5,
        /// 2 stop bits
        STOP2,
    }

    impl From<StopBits> for uart_stop_bits_t {
        fn from(stop_bits: StopBits) -> Self {
            match stop_bits {
                StopBits::STOP1 => uart_stop_bits_t_UART_STOP_BITS_1,
                StopBits::STOP1P5 => uart_stop_bits_t_UART_STOP_BITS_1_5,
                StopBits::STOP2 => uart_stop_bits_t_UART_STOP_BITS_2,
            }
        }
    }

    impl From<uart_stop_bits_t> for StopBits {
        #[allow(non_upper_case_globals)]
        fn from(stop_bits: uart_stop_bits_t) -> Self {
            match stop_bits {
                uart_stop_bits_t_UART_STOP_BITS_1 => StopBits::STOP1,
                uart_stop_bits_t_UART_STOP_BITS_1_5 => StopBits::STOP1P5,
                uart_stop_bits_t_UART_STOP_BITS_2 => StopBits::STOP2,
                _ => unreachable!(),
            }
        }
    }

    /// UART configuration
    #[derive(Debug, Copy, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
        pub flow_control: FlowControl,
        pub flow_control_rts_threshold: u8,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }

        #[must_use]
        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        #[must_use]
        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        #[must_use]
        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        #[must_use]
        pub fn data_bits(mut self, data_bits: DataBits) -> Self {
            self.data_bits = data_bits;
            self
        }

        #[must_use]
        pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
            self.stop_bits = stop_bits;
            self
        }

        #[must_use]
        pub fn flow_control(mut self, flow_control: FlowControl) -> Self {
            self.flow_control = flow_control;
            self
        }

        #[must_use]
        /// This setting only has effect if flow control is enabled.
        /// It determines how many bytes must be received before `RTS` line is asserted.
        /// Notice that count starts from `0` which means that `RTS` is asserted after every received byte.
        pub fn flow_control_rts_threshold(mut self, flow_control_rts_threshold: u8) -> Self {
            self.flow_control_rts_threshold = flow_control_rts_threshold;
            self
        }
    }

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: Hertz(19_200),
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
                flow_control: FlowControl::None,
                flow_control_rts_threshold: 122,
            }
        }
    }
}

pub trait Uart {
    fn port() -> uart_port_t;
}

crate::embedded_hal_error!(
    SerialError,
    embedded_hal::serial::Error,
    embedded_hal::serial::ErrorKind
);

/// Serial abstraction
///
pub struct UartDriver<'d, UART: Uart> {
    _uart: PeripheralRef<'d, UART>,
    rx: UartRxDriver<'d, UART>,
    tx: UartTxDriver<'d, UART>,
}

/// Serial receiver
pub struct UartRxDriver<'d, UART: Uart> {
    _uart: PhantomData<&'d UART>,
}

/// Serial transmitter
pub struct UartTxDriver<'d, UART: Uart> {
    _uart: PhantomData<&'d UART>,
}

impl<'d, UART: Uart> UartDriver<'d, UART> {
    /// Create a new serial driver
    pub fn new(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        crate::into_ref!(uart, tx, rx);

        let cts = cts.map(|cts| cts.into_ref());
        let rts = rts.map(|rts| rts.into_ref());

        let uart_config = uart_config_t {
            baud_rate: config.baudrate.0 as i32,
            data_bits: config.data_bits.into(),
            parity: config.parity.into(),
            stop_bits: config.stop_bits.into(),
            flow_ctrl: config.flow_control.into(),
            rx_flow_ctrl_thresh: config.flow_control_rts_threshold,
            ..Default::default()
        };

        esp!(unsafe { uart_param_config(UART::port(), &uart_config) })?;

        esp!(unsafe {
            uart_set_pin(
                UART::port(),
                tx.pin(),
                rx.pin(),
                rts.as_ref().map_or(-1, |p| p.pin()),
                cts.as_ref().map_or(-1, |p| p.pin()),
            )
        })?;

        esp!(unsafe {
            uart_driver_install(
                UART::port(),
                UART_FIFO_SIZE * 2,
                UART_FIFO_SIZE * 2,
                0,
                ptr::null_mut(),
                0,
            )
        })?;

        Ok(Self {
            _uart: uart,
            rx: UartRxDriver { _uart: PhantomData },
            tx: UartTxDriver { _uart: PhantomData },
        })
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&mut self, stop_bits: config::StopBits) -> Result<&mut Self, EspError> {
        esp_result!(
            unsafe { uart_set_stop_bits(UART::port(), stop_bits.into()) },
            self
        )
    }

    /// Retruns the current number of stop bits
    pub fn stop_bits(&self) -> Result<config::StopBits, EspError> {
        let mut stop_bits: uart_stop_bits_t = 0;
        esp_result!(
            unsafe { uart_get_stop_bits(UART::port(), &mut stop_bits) },
            stop_bits.into()
        )
    }

    /// Change the number of data bits
    pub fn change_data_bits(&mut self, data_bits: config::DataBits) -> Result<&mut Self, EspError> {
        esp_result!(
            unsafe { uart_set_word_length(UART::port(), data_bits.into()) },
            self
        )
    }

    /// Return the current number of data bits
    pub fn data_bits(&self) -> Result<config::DataBits, EspError> {
        let mut data_bits: uart_word_length_t = 0;
        esp_result!(
            unsafe { uart_get_word_length(UART::port(), &mut data_bits) },
            data_bits.into()
        )
    }

    /// Change the type of parity checking
    pub fn change_parity(&mut self, parity: config::Parity) -> Result<&mut Self, EspError> {
        esp_result!(
            unsafe { uart_set_parity(UART::port(), parity.into()) },
            self
        )
    }

    /// Returns the current type of parity checking
    pub fn parity(&self) -> Result<config::Parity, EspError> {
        let mut parity: uart_parity_t = 0;
        esp_result!(
            unsafe { uart_get_parity(UART::port(), &mut parity) },
            parity.into()
        )
    }

    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference clock (1MHz) will
    /// be used, because this is constant when the clock source/frequency changes.
    /// However if one of the clock frequencies is below 10MHz or if the baudrate is above
    /// the reference clock or if the baudrate cannot be set within 1.5%
    /// then use the APB clock.
    pub fn change_baudrate<T: Into<Hertz> + Copy>(
        &mut self,
        baudrate: T,
    ) -> Result<&mut Self, EspError> {
        esp_result!(
            unsafe { uart_set_baudrate(UART::port(), baudrate.into().into()) },
            self
        )
    }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<Hertz, EspError> {
        let mut baudrate: u32 = 0;
        esp_result!(
            unsafe { uart_get_baudrate(UART::port(), &mut baudrate) },
            baudrate.into()
        )
    }

    /// Split the serial driver in separate TX and RX drivers
    pub fn split(&mut self) -> (&mut UartTxDriver<'d, UART>, &mut UartRxDriver<'d, UART>) {
        (&mut self.tx, &mut self.rx)
    }

    /// Read multiple bytes into a slice
    pub fn read(&mut self, buf: &mut [u8], delay: TickType_t) -> Result<usize, EspError> {
        self.rx.read(buf, delay)
    }

    /// Write multiple bytes from a slice
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspError> {
        self.tx.write(buf)
    }

    pub fn flush_read(&mut self) -> Result<(), EspError> {
        self.rx.flush()
    }

    pub fn flush_write(&mut self) -> Result<(), EspError> {
        self.tx.flush()
    }
}

impl<'d, UART: Uart> Drop for UartDriver<'d, UART> {
    fn drop(&mut self) {
        esp!(unsafe { uart_driver_delete(UART::port()) }).unwrap();
    }
}

unsafe impl<'d, UART: Uart> Send for UartDriver<'d, UART> {}

impl<'d, UART: Uart> embedded_hal::serial::ErrorType for UartDriver<'d, UART> {
    type Error = SerialError;
}

impl<'d, UART: Uart> embedded_hal_0_2::serial::Read<u8> for UartDriver<'d, UART> {
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_0_2::serial::Read::read(&mut self.rx)
    }
}

impl<'d, UART: Uart> embedded_hal::serial::nb::Read<u8> for UartDriver<'d, UART> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal::serial::nb::Read::read(&mut self.rx)
    }
}

impl<'d, UART: Uart> embedded_hal_0_2::serial::Write<u8> for UartDriver<'d, UART> {
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_0_2::serial::Write::flush(&mut self.tx)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_0_2::serial::Write::write(&mut self.tx, byte)
    }
}

impl<'d, UART: Uart> embedded_hal::serial::nb::Write<u8> for UartDriver<'d, UART> {
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal::serial::nb::Write::flush(&mut self.tx)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal::serial::nb::Write::write(&mut self.tx, byte)
    }
}

impl<'d, UART: Uart> core::fmt::Write for UartDriver<'d, UART> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<'d, UART: Uart> embedded_hal::serial::ErrorType for UartRxDriver<'d, UART> {
    type Error = SerialError;
}

impl<'d, UART: Uart> UartRxDriver<'d, UART> {
    /// Get count of bytes in the receive FIFO
    pub fn count(&self) -> Result<u8, EspError> {
        let mut size = 0_u32;
        esp_result!(
            unsafe { uart_get_buffered_data_len(UART::port(), &mut size) },
            size as u8
        )
    }

    /// Read multiple bytes into a slice; block until specified timeout
    pub fn read(&mut self, buf: &mut [u8], delay: TickType_t) -> Result<usize, EspError> {
        // uart_read_bytes() returns error (-1) or how many bytes were read out
        // 0 means timeout and nothing is yet read out
        let len = unsafe {
            uart_read_bytes(
                UART::port(),
                buf.as_mut_ptr() as *mut _,
                buf.len() as u32,
                delay,
            )
        };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(EspError::from(ESP_ERR_INVALID_STATE).unwrap())
        }
    }

    pub fn flush(&self) -> Result<(), EspError> {
        esp!(unsafe { uart_flush_input(UART::port()) })?;

        Ok(())
    }
}

impl<'d, UART: Uart> embedded_hal_0_2::serial::Read<u8> for UartRxDriver<'d, UART> {
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf = [0_u8];

        let result = self.read(&mut buf, NON_BLOCK);

        check_nb(result, buf[0])
    }
}

impl<'d, UART: Uart> embedded_hal::serial::nb::Read<u8> for UartRxDriver<'d, UART> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf = [0_u8];

        let result = self.read(&mut buf, NON_BLOCK);

        check_nb(result, buf[0])
    }
}

impl<'d, UART: Uart> UartTxDriver<'d, UART> {
    /// Write multiple bytes from a slice
    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        // `uart_write_bytes()` returns error (-1) or how many bytes were written
        let len = unsafe {
            uart_write_bytes(UART::port(), bytes.as_ptr() as *const _, bytes.len() as u32)
        };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(EspError::from(ESP_ERR_INVALID_STATE).unwrap())
        }
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        esp!(unsafe { uart_wait_tx_done(UART::port(), 0) })?;

        Ok(())
    }
}

impl<'d, UART: Uart> embedded_hal::serial::ErrorType for UartTxDriver<'d, UART> {
    type Error = SerialError;
}

impl<'d, UART: Uart> embedded_hal_0_2::serial::Write<u8> for UartTxDriver<'d, UART> {
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        UartTxDriver::flush(self).map_err(to_nb_err)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        check_nb(self.write(&[byte]), ())
    }
}

impl<'d, UART: Uart> embedded_hal::serial::nb::Write<u8> for UartTxDriver<'d, UART> {
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        UartTxDriver::flush(self).map_err(to_nb_err)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        check_nb(self.write(&[byte]), ())
    }
}

impl<'d, UART: Uart> core::fmt::Write for UartTxDriver<'d, UART> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = s.as_bytes();
        let mut offset = 0;

        while offset < buf.len() {
            offset += self.write(buf).map_err(|_| core::fmt::Error)?
        }

        Ok(())
    }
}

macro_rules! impl_uart {
    ($uart:ident: $port:expr) => {
        crate::impl_peripheral!($uart);

        impl Uart for $uart {
            fn port() -> uart_port_t {
                $port
            }
        }
    };
}

fn to_nb_err(err: EspError) -> nb::Error<SerialError> {
    if err.code() == ESP_ERR_TIMEOUT as i32 {
        nb::Error::WouldBlock
    } else {
        nb::Error::Other(SerialError::from(err))
    }
}

fn check_nb<T>(result: Result<usize, EspError>, value: T) -> nb::Result<T, SerialError> {
    match result {
        Ok(1) => Ok(value),
        Ok(0) => Err(nb::Error::WouldBlock),
        Ok(_) => unreachable!(),
        Err(err) => Err(nb::Error::Other(SerialError::other(err))),
    }
}

impl_uart!(UART0: 0);
impl_uart!(UART1: 1);
#[cfg(esp32)]
impl_uart!(UART2: 2);
