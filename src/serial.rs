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
//! let config = serial::config::Config::default().baudrate(Hertz(115_200));
//!
//! let mut serial: serial::Serial<serial::UART1, _, _> = serial::Serial::new(
//!     peripherals.uart1,
//!     serial::Pins {
//!         tx: pins.gpio1,
//!         rx: pins.gpio3,
//!         cts: None,
//!         rts: None,
//!     },
//!     config
//! ).unwrap();
//!
//! for i in 0..10 {
//!     writeln!(serial, "{:}", format!("count {:}", i)).unwrap();
//! }
//! ```
//!
//! # TODO
//! - Add all extra features esp32 supports (eg rs485, etc. etc.)
//! - Free APB lock when TX is idle (and no RX used)
//! - Address errata 3.17: UART fifo_cnt is inconsistent with FIFO pointer

use core::marker::PhantomData;
use core::ptr;

use crate::gpio::*;
use crate::units::*;

use esp_idf_sys::*;

const UART_FIFO_SIZE: i32 = 128;

// /// Interrupt event
// pub enum Event {
//     /// New data has been received
//     Rxne,
//     /// New data can be sent
//     Txe,
//     /// Idle line state detected
//     Idle,
// }

/// UART configuration
pub mod config {
    use crate::units::*;
    use esp_idf_sys::*;

    /// Number of data bits
    #[derive(PartialEq, Eq, Hash, Copy, Clone, Debug)]
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
    #[derive(PartialEq, Eq, Hash, Copy, Clone, Debug)]
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
    #[derive(PartialEq, Eq, Hash, Copy, Clone, Debug)]
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

/// Pins used by the UART interface
///
/// Note that any two pins may be used
pub struct Pins<
    TX: OutputPin,
    RX: InputPin,
    // default pins to allow type inference
    CTS: InputPin = crate::gpio::Gpio1<crate::gpio::Input>,
    RTS: OutputPin = crate::gpio::Gpio2<crate::gpio::Output>,
> {
    pub tx: TX,
    pub rx: RX,
    pub cts: Option<CTS>,
    pub rts: Option<RTS>,
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
pub struct Serial<
    UART: Uart,
    TX: OutputPin,
    RX: InputPin,
    // default pins to allow type inference
    CTS: InputPin = crate::gpio::Gpio1<crate::gpio::Input>,
    RTS: OutputPin = crate::gpio::Gpio2<crate::gpio::Output>,
> {
    uart: UART,
    pins: Pins<TX, RX, CTS, RTS>,
    rx: Rx<UART>,
    tx: Tx<UART>,
}

/// Serial receiver
pub struct Rx<UART: Uart> {
    _uart: PhantomData<UART>,
}

/// Serial transmitter
pub struct Tx<UART: Uart> {
    _uart: PhantomData<UART>,
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    Serial<UART, TX, RX, CTS, RTS>
{
    /// Create a new serial driver
    pub fn new(
        uart: UART,
        pins: Pins<TX, RX, CTS, RTS>,
        config: config::Config,
    ) -> Result<Self, EspError> {
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
                pins.tx.pin(),
                pins.rx.pin(),
                pins.rts.as_ref().map_or(-1, |p| p.pin()),
                pins.cts.as_ref().map_or(-1, |p| p.pin()),
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
            uart,
            pins,
            rx: Rx { _uart: PhantomData },
            tx: Tx { _uart: PhantomData },
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

    // /// Returns if the reference or APB clock is used
    // pub fn is_clock_apb(&self) -> bool {
    //     self.uart.conf0.read().tick_ref_always_on().bit_is_set()
    // }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<Hertz, EspError> {
        let mut baudrate: u32 = 0;
        esp_result!(
            unsafe { uart_get_baudrate(UART::port(), &mut baudrate) },
            baudrate.into()
        )
    }

    /// Read multiple bytes into a slice
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> nb::Result<usize, SerialError> {
        self.rx.read_bytes(buf)
    }

    /// Write multiple bytes from a slice
    pub fn write_bytes(&mut self, buf: &[u8]) -> nb::Result<usize, SerialError> {
        self.tx.write_bytes(buf)
    }

    // /// Stop listening for an interrupt event
    // pub fn unlisten(&mut self, _event: Event) {
    //     unimplemented!();
    // }

    // /// Return true if the receiver is idle
    // pub fn is_rx_idle(&self) -> bool {
    //     self.uart.status.read().st_urx_out().is_rx_idle()
    // }

    // /// Return true if the transmitter is idle
    // pub fn is_tx_idle(&self) -> bool {
    //     self.uart.status.read().st_utx_out().is_tx_idle()
    // }

    /// Split the serial driver in separate TX and RX drivers
    pub fn split(self) -> (Tx<UART>, Rx<UART>) {
        (self.tx, self.rx)
    }

    /// Release the UART and GPIO resources
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> Result<(UART, Pins<TX, RX, CTS, RTS>), EspError> {
        esp!(unsafe { uart_driver_delete(UART::port()) })?;

        // self.pins.tx.reset()?;
        // self.pins.rx.reset()?;

        Ok((self.uart, self.pins))
    }

    // pub fn reset_rx_fifo(&mut self) {
    //     // Hardware issue: rxfifo_rst does not work properly;
    //     while self.uart.status.read().rxfifo_cnt().bits() != 0
    //         || (self.uart.mem_rx_status.read().mem_rx_rd_addr().bits()
    //             != self.uart.mem_rx_status.read().mem_rx_wr_addr().bits())
    //     {
    //         self.rx.read().unwrap();
    //     }
    // }

    // pub fn reset_tx_fifo(&self) {
    //     self.uart.conf0.write(|w| w.txfifo_rst().set_bit());
    //     self.uart.conf0.write(|w| w.txfifo_rst().clear_bit());
    // }
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    embedded_hal::serial::ErrorType for Serial<UART, TX, RX, CTS, RTS>
{
    type Error = SerialError;
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    embedded_hal_0_2::serial::Read<u8> for Serial<UART, TX, RX, CTS, RTS>
{
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    embedded_hal::serial::nb::Read<u8> for Serial<UART, TX, RX, CTS, RTS>
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    embedded_hal_0_2::serial::Write<u8> for Serial<UART, TX, RX, CTS, RTS>
{
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin>
    embedded_hal::serial::nb::Write<u8> for Serial<UART, TX, RX, CTS, RTS>
{
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }
}

impl<UART: Uart, TX: OutputPin, RX: InputPin, CTS: InputPin, RTS: OutputPin> core::fmt::Write
    for Serial<UART, TX, RX, CTS, RTS>
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        use embedded_hal::serial::nb::Write;
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

impl<UART: Uart> embedded_hal::serial::ErrorType for Rx<UART> {
    type Error = SerialError;
}

impl<UART: Uart> Rx<UART> {
    /// Get count of bytes in the receive FIFO
    pub fn count(&self) -> Result<u8, EspError> {
        let mut size = 0_u32;
        esp_result!(
            unsafe { uart_get_buffered_data_len(UART::port(), &mut size) },
            size as u8
        )
    }

    /// Read multiple bytes into a slice
    pub fn read_bytes(&mut self, buf: &mut [u8]) -> nb::Result<usize, SerialError> {
        // uart_read_bytes() returns error (-1) or how many bytes were read out
        // 0 means timeout and nothing is yet read out
        match unsafe {
            uart_read_bytes(
                UART::port(),
                buf.as_mut_ptr() as *mut _,
                buf.len() as u32,
                0,
            )
        } {
            len if len > 0 => Ok(len as usize),
            0 => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }

    // /// Check if the receivers is idle
    // pub fn is_idle(&self) -> bool {
    //     unsafe { (*UART::ptr()).status.read().st_urx_out().is_rx_idle() }
    // }
}

impl<UART: Uart> embedded_hal_0_2::serial::Read<u8> for Rx<UART> {
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf: u8 = 0;

        // uart_read_bytes() returns error (-1) or how many bytes were read out
        // 0 means timeout and nothing is yet read out
        match unsafe { uart_read_bytes(UART::port(), &mut buf as *mut u8 as *mut _, 1, 0) } {
            1 => Ok(buf),
            0 => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }
}

impl<UART: Uart> embedded_hal::serial::nb::Read<u8> for Rx<UART> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf: u8 = 0;

        // uart_read_bytes() returns error (-1) or how many bytes were read out
        // 0 means timeout and nothing is yet read out
        match unsafe { uart_read_bytes(UART::port(), &mut buf as *mut u8 as *mut _, 1, 0) } {
            1 => Ok(buf),
            0 => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }
}

impl<UART: Uart> Tx<UART> {
    /// Write multiple bytes from a slice
    pub fn write_bytes(&mut self, bytes: &[u8]) -> nb::Result<usize, SerialError> {
        // `uart_write_bytes()` returns error (-1) or how many bytes were written
        match unsafe {
            uart_write_bytes(UART::port(), bytes.as_ptr() as *const _, bytes.len() as u32)
        } {
            len if len > 0 => Ok(len as usize),
            0 => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }

    //     /// Get count of bytes in the transmitter FIFO
    //     pub fn count(&self) -> u8 {
    //         unsafe { (*UART::ptr()).status.read().txfifo_cnt().bits() }
    //     }

    //     /// Check if the transmitter is idle
    //     pub fn is_idle(&self) -> bool {
    //         unsafe { (*UART::ptr()).status.read().st_utx_out().is_tx_idle() }
    //     }
}

impl<UART: Uart> embedded_hal::serial::ErrorType for Tx<UART> {
    type Error = SerialError;
}

impl<UART: Uart> embedded_hal_0_2::serial::Write<u8> for Tx<UART> {
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match unsafe { uart_wait_tx_done(UART::port(), 0) } {
            ESP_OK => Ok(()),
            ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // `uart_write_bytes()` returns error (-1) or how many bytes were written
        match unsafe { uart_write_bytes(UART::port(), &byte as *const u8 as *const _, 1) } {
            1 => Ok(()),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }
}

impl<UART: Uart> embedded_hal::serial::nb::Write<u8> for Tx<UART> {
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match unsafe { uart_wait_tx_done(UART::port(), 0) } {
            ESP_OK => Ok(()),
            ESP_ERR_TIMEOUT => Err(nb::Error::WouldBlock),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        // `uart_write_bytes()` returns error (-1) or how many bytes were written
        match unsafe { uart_write_bytes(UART::port(), &byte as *const u8 as *const _, 1) } {
            1 => Ok(()),
            _ => Err(nb::Error::Other(SerialError::other(
                EspError::from(ESP_ERR_INVALID_STATE).unwrap(),
            ))),
        }
    }
}

impl<UART: Uart> core::fmt::Write for Tx<UART>
where
    Tx<UART>: embedded_hal::serial::nb::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        use embedded_hal::serial::nb::Write;
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

macro_rules! impl_uart {
    ($uart:ident: $port:expr) => {
        pub struct $uart;

        impl $uart {
            /// # Safety
            ///
            /// Care should be taken not to instantiate this UART instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $uart {}
            }
        }

        impl Uart for $uart {
            fn port() -> uart_port_t {
                $port
            }
        }
    };
}

impl_uart!(UART0: 0);
impl_uart!(UART1: 1);

#[cfg(esp32)]
impl_uart!(UART2: 2);
