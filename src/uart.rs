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
use core::mem::ManuallyDrop;
use core::ptr;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::delay::NON_BLOCK;
use crate::gpio::*;
use crate::interrupt::IntrFlags;
use crate::units::*;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;

const UART_FIFO_SIZE: i32 = SOC_UART_FIFO_LEN as i32;

pub type UartConfig = config::Config;

/// UART configuration
pub mod config {
    use crate::{interrupt::IntrFlags, units::*};
    use enumset::EnumSet;
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

    /// UART source clock
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum SourceClock {
        /// UART source clock from `APB`
        #[cfg(any(
            esp_idf_soc_uart_support_apb_clk,
            esp_idf_soc_uart_support_pll_f40m_clk,
            esp_idf_version_major = "4",
        ))]
        APB,
        /// UART source clock from `RTC`
        #[cfg(esp_idf_soc_uart_support_rtc_clk)]
        RTC,
        /// UART source clock from `XTAL`
        #[cfg(esp_idf_soc_uart_support_xtal_clk)]
        Crystal,
        /// UART source clock from `REF_TICK`
        #[cfg(esp_idf_soc_uart_support_ref_tick)]
        RefTick,
    }

    impl SourceClock {
        #[cfg(not(esp_idf_version_major = "4"))]
        pub fn frequency(self) -> Result<Hertz, EspError> {
            let mut frequency: u32 = 0;
            esp_result! {
                unsafe { uart_get_sclk_freq(self.into(), &mut frequency) },
                Hertz(frequency)
            }
        }
    }

    #[cfg(all(not(esp_idf_version_major = "4"), esp_idf_soc_uart_support_apb_clk))]
    const APB_SCLK: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_APB;
    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(esp_idf_soc_uart_support_apb_clk),
        esp_idf_soc_uart_support_pll_f40m_clk,
    ))]
    const APB_SCLK: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_PLL_F40M;
    #[cfg(esp_idf_version_major = "4")]
    const APB_SCLK: uart_sclk_t = uart_sclk_t_UART_SCLK_APB;

    #[cfg(all(not(esp_idf_version_major = "4"), esp_idf_soc_uart_support_rtc_clk))]
    const RTC_SCLK: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_RTC;
    #[cfg(all(esp_idf_version_major = "4", esp_idf_soc_uart_support_rtc_clk))]
    const RTC_SCLK: uart_sclk_t = uart_sclk_t_UART_SCLK_RTC;

    #[cfg(all(not(esp_idf_version_major = "4"), esp_idf_soc_uart_support_xtal_clk))]
    const XTAL_SCLK: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_XTAL;
    #[cfg(all(esp_idf_version_major = "4", esp_idf_soc_uart_support_xtal_clk))]
    const XTAL_SCLK: uart_sclk_t = uart_sclk_t_UART_SCLK_XTAL;

    #[cfg(all(not(esp_idf_version_major = "4"), esp_idf_soc_uart_support_ref_tick))]
    const REF_TICK_SCLK: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_REF_TICK;
    #[cfg(all(esp_idf_version_major = "4", esp_idf_soc_uart_support_ref_tick))]
    const REF_TICK_SCLK: uart_sclk_t = uart_sclk_t_UART_SCLK_REF_TICK;

    impl Default for SourceClock {
        fn default() -> Self {
            #[cfg(not(esp_idf_version_major = "4"))]
            const DEFAULT: uart_sclk_t = soc_periph_uart_clk_src_legacy_t_UART_SCLK_DEFAULT;
            #[cfg(esp_idf_version_major = "4")]
            const DEFAULT: uart_sclk_t = uart_sclk_t_UART_SCLK_APB;
            Self::from(DEFAULT)
        }
    }

    impl From<SourceClock> for uart_sclk_t {
        fn from(source_clock: SourceClock) -> Self {
            match source_clock {
                #[cfg(any(
                    esp_idf_soc_uart_support_apb_clk,
                    esp_idf_soc_uart_support_pll_f40m_clk,
                    esp_idf_version_major = "4",
                ))]
                SourceClock::APB => APB_SCLK,
                #[cfg(esp_idf_soc_uart_support_rtc_clk)]
                SourceClock::RTC => RTC_SCLK,
                #[cfg(esp_idf_soc_uart_support_xtal_clk)]
                SourceClock::Crystal => XTAL_SCLK,
                #[cfg(esp_idf_soc_uart_support_ref_tick)]
                SourceClock::RefTick => REF_TICK_SCLK,
            }
        }
    }

    impl From<uart_sclk_t> for SourceClock {
        fn from(source_clock: uart_sclk_t) -> Self {
            match source_clock {
                #[cfg(any(
                    esp_idf_soc_uart_support_apb_clk,
                    esp_idf_soc_uart_support_pll_f40m_clk,
                    esp_idf_version_major = "4",
                ))]
                APB_SCLK => SourceClock::APB,
                #[cfg(esp_idf_soc_uart_support_rtc_clk)]
                RTC_SCLK => SourceClock::RTC,
                #[cfg(esp_idf_soc_uart_support_xtal_clk)]
                XTAL_SCLK => SourceClock::Crystal,
                #[cfg(esp_idf_soc_uart_support_ref_tick)]
                REF_TICK_SCLK => SourceClock::RefTick,
                _ => unreachable!(),
            }
        }
    }

    /// UART configuration
    #[derive(Debug, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_bits: DataBits,
        pub parity: Parity,
        pub stop_bits: StopBits,
        pub flow_control: FlowControl,
        pub flow_control_rts_threshold: u8,
        pub source_clock: SourceClock,
        pub intr_flags: EnumSet<IntrFlags>,
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

        #[must_use]
        pub fn source_clock(mut self, source_clock: SourceClock) -> Self {
            self.source_clock = source_clock;
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
                source_clock: SourceClock::default(),
                intr_flags: EnumSet::<IntrFlags>::empty(),
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
pub struct UartDriver<'d> {
    port: u8,
    _p: PhantomData<&'d mut ()>,
}

/// Serial receiver
pub struct UartRxDriver<'d> {
    port: u8,
    owner: Owner,
    _p: PhantomData<&'d mut ()>,
}

/// Serial transmitter
pub struct UartTxDriver<'d> {
    port: u8,
    owner: Owner,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> UartDriver<'d> {
    /// Create a new serial driver
    pub fn new<UART: Uart>(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        new_common(uart, Some(tx), Some(rx), cts, rts, config)?;

        Ok(Self {
            port: UART::port() as _,
            _p: PhantomData,
        })
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&self, stop_bits: config::StopBits) -> Result<&Self, EspError> {
        change_stop_bits(self.port(), stop_bits).map(|_| self)
    }

    /// Returns the current number of stop bits
    pub fn stop_bits(&self) -> Result<config::StopBits, EspError> {
        stop_bits(self.port())
    }

    /// Change the number of data bits
    pub fn change_data_bits(&self, data_bits: config::DataBits) -> Result<&Self, EspError> {
        change_data_bits(self.port(), data_bits).map(|_| self)
    }

    /// Return the current number of data bits
    pub fn data_bits(&self) -> Result<config::DataBits, EspError> {
        data_bits(self.port())
    }

    /// Change the type of parity checking
    pub fn change_parity(&self, parity: config::Parity) -> Result<&Self, EspError> {
        change_parity(self.port(), parity).map(|_| self)
    }

    /// Returns the current type of parity checking
    pub fn parity(&self) -> Result<config::Parity, EspError> {
        parity(self.port())
    }

    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference clock (1MHz) will
    /// be used, because this is constant when the clock source/frequency changes.
    /// However if one of the clock frequencies is below 10MHz or if the baudrate is above
    /// the reference clock or if the baudrate cannot be set within 1.5%
    /// then use the APB clock.
    pub fn change_baudrate<T: Into<Hertz> + Copy>(&self, baudrate: T) -> Result<&Self, EspError> {
        change_baudrate(self.port(), baudrate).map(|_| self)
    }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<Hertz, EspError> {
        baudrate(self.port())
    }

    /// Split the serial driver in separate TX and RX drivers
    pub fn split(&self) -> (UartTxDriver<'_>, UartRxDriver<'_>) {
        (
            UartTxDriver {
                port: self.port,
                owner: Owner::Borrowed,
                _p: PhantomData,
            },
            UartRxDriver {
                port: self.port,
                owner: Owner::Borrowed,
                _p: PhantomData,
            },
        )
    }

    /// Split the serial driver in separate TX and RX drivers.
    ///
    /// Unlike [`split`], the halves are owned and reference counted.
    pub fn into_split(self) -> (UartTxDriver<'d>, UartRxDriver<'d>) {
        let port = self.port;
        let _ = ManuallyDrop::new(self);
        REFS[port as usize].fetch_add(2, Ordering::SeqCst);
        (
            UartTxDriver {
                port,
                owner: Owner::Shared,
                _p: PhantomData,
            },
            UartRxDriver {
                port,
                owner: Owner::Shared,
                _p: PhantomData,
            },
        )
    }

    /// Read multiple bytes into a slice
    pub fn read(&self, buf: &mut [u8], delay: TickType_t) -> Result<usize, EspError> {
        self.rx().read(buf, delay)
    }

    /// Write multiple bytes from a slice
    pub fn write(&self, buf: &[u8]) -> Result<usize, EspError> {
        self.tx().write(buf)
    }

    pub fn flush_read(&self) -> Result<(), EspError> {
        self.rx().flush()
    }

    pub fn flush_write(&self) -> Result<(), EspError> {
        self.tx().flush()
    }

    pub fn port(&self) -> uart_port_t {
        self.port as _
    }

    /// Get count of remaining bytes in the receive ring buffer
    pub fn remaining_read(&self) -> Result<usize, EspError> {
        remaining_unread_bytes(self.port())
    }

    /// Get count of remaining capacity in the transmit ring buffer
    #[cfg(any(
        not(esp_idf_version_major = "4"),
        all(
            esp_idf_version_minor = "4",
            not(any(esp_idf_version_patch = "0", esp_idf_version_patch = "1")),
        ),
    ))]
    pub fn remaining_write(&self) -> Result<usize, EspError> {
        remaining_write_capacity(self.port())
    }

    fn rx(&self) -> ManuallyDrop<UartRxDriver<'_>> {
        ManuallyDrop::new(UartRxDriver {
            port: self.port,
            owner: Owner::Borrowed,
            _p: PhantomData,
        })
    }

    fn tx(&self) -> ManuallyDrop<UartTxDriver<'_>> {
        ManuallyDrop::new(UartTxDriver {
            port: self.port,
            owner: Owner::Borrowed,
            _p: PhantomData,
        })
    }
}

impl<'d> Drop for UartDriver<'d> {
    fn drop(&mut self) {
        delete_driver(self.port()).unwrap();
    }
}

impl<'d> embedded_hal::serial::ErrorType for UartDriver<'d> {
    type Error = SerialError;
}

impl<'d> embedded_hal_0_2::serial::Read<u8> for UartDriver<'d> {
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_0_2::serial::Read::read(&mut *self.rx())
    }
}

impl<'d> embedded_hal_nb::serial::Read<u8> for UartDriver<'d> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_nb::serial::Read::read(&mut *self.rx())
    }
}

impl<'d> embedded_hal_0_2::serial::Write<u8> for UartDriver<'d> {
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_0_2::serial::Write::flush(&mut *self.tx())
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_0_2::serial::Write::write(&mut *self.tx(), byte)
    }
}

impl<'d> embedded_hal_nb::serial::Write<u8> for UartDriver<'d> {
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut *self.tx())
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut *self.tx(), byte)
    }
}

impl<'d> core::fmt::Write for UartDriver<'d> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx().write_str(s)
    }
}

impl<'d> embedded_hal::serial::ErrorType for UartRxDriver<'d> {
    type Error = SerialError;
}

impl<'d> UartRxDriver<'d> {
    /// Create a new serial receiver
    pub fn new<UART: Uart>(
        uart: impl Peripheral<P = UART> + 'd,
        rx: impl Peripheral<P = impl InputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        new_common(uart, None::<AnyOutputPin>, Some(rx), cts, rts, config)?;

        Ok(Self {
            port: UART::port() as _,
            owner: Owner::Owned,
            _p: PhantomData,
        })
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&self, stop_bits: config::StopBits) -> Result<&Self, EspError> {
        change_stop_bits(self.port(), stop_bits).map(|_| self)
    }

    /// Returns the current number of stop bits
    pub fn stop_bits(&self) -> Result<config::StopBits, EspError> {
        stop_bits(self.port())
    }

    /// Change the number of data bits
    pub fn change_data_bits(&self, data_bits: config::DataBits) -> Result<&Self, EspError> {
        change_data_bits(self.port(), data_bits).map(|_| self)
    }

    /// Return the current number of data bits
    pub fn data_bits(&self) -> Result<config::DataBits, EspError> {
        data_bits(self.port())
    }

    /// Change the type of parity checking
    pub fn change_parity(&self, parity: config::Parity) -> Result<&Self, EspError> {
        change_parity(self.port(), parity).map(|_| self)
    }

    /// Returns the current type of parity checking
    pub fn parity(&self) -> Result<config::Parity, EspError> {
        parity(self.port())
    }

    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference clock (1MHz) will
    /// be used, because this is constant when the clock source/frequency changes.
    /// However if one of the clock frequencies is below 10MHz or if the baudrate is above
    /// the reference clock or if the baudrate cannot be set within 1.5%
    /// then use the APB clock.
    pub fn change_baudrate<T: Into<Hertz> + Copy>(&self, baudrate: T) -> Result<&Self, EspError> {
        change_baudrate(self.port(), baudrate).map(|_| self)
    }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<Hertz, EspError> {
        baudrate(self.port())
    }

    /// Read multiple bytes into a slice; block until specified timeout
    pub fn read(&self, buf: &mut [u8], delay: TickType_t) -> Result<usize, EspError> {
        // uart_read_bytes() returns error (-1) or how many bytes were read out
        // 0 means timeout and nothing is yet read out
        let len = unsafe {
            uart_read_bytes(
                self.port(),
                buf.as_mut_ptr().cast(),
                buf.len() as u32,
                delay,
            )
        };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
        }
    }

    pub fn flush(&self) -> Result<(), EspError> {
        esp!(unsafe { uart_flush_input(self.port()) })?;

        Ok(())
    }

    pub fn port(&self) -> uart_port_t {
        self.port as _
    }

    /// Get count of remaining bytes in the receive ring buffer
    pub fn count(&self) -> Result<usize, EspError> {
        remaining_unread_bytes(self.port())
    }
}

impl<'d> Drop for UartRxDriver<'d> {
    fn drop(&mut self) {
        self.owner.drop_impl(self.port()).unwrap()
    }
}

impl<'d> embedded_hal_0_2::serial::Read<u8> for UartRxDriver<'d> {
    type Error = SerialError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf = [0_u8];

        let result = UartRxDriver::read(self, &mut buf, NON_BLOCK);

        check_nb(result, buf[0])
    }
}

impl<'d> embedded_hal_nb::serial::Read<u8> for UartRxDriver<'d> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf = [0_u8];

        let result = UartRxDriver::read(self, &mut buf, NON_BLOCK);

        check_nb(result, buf[0])
    }
}

impl<'d> UartTxDriver<'d> {
    /// Create a new serial transmitter
    pub fn new<UART: Uart>(
        uart: impl Peripheral<P = UART> + 'd,
        tx: impl Peripheral<P = impl OutputPin> + 'd,
        cts: Option<impl Peripheral<P = impl InputPin> + 'd>,
        rts: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        new_common(uart, Some(tx), None::<AnyInputPin>, cts, rts, config)?;

        Ok(Self {
            port: UART::port() as _,
            owner: Owner::Owned,
            _p: PhantomData,
        })
    }

    /// Change the number of stop bits
    pub fn change_stop_bits(&self, stop_bits: config::StopBits) -> Result<&Self, EspError> {
        change_stop_bits(self.port(), stop_bits).map(|_| self)
    }

    /// Returns the current number of stop bits
    pub fn stop_bits(&self) -> Result<config::StopBits, EspError> {
        stop_bits(self.port())
    }

    /// Change the number of data bits
    pub fn change_data_bits(&self, data_bits: config::DataBits) -> Result<&Self, EspError> {
        change_data_bits(self.port(), data_bits).map(|_| self)
    }

    /// Return the current number of data bits
    pub fn data_bits(&self) -> Result<config::DataBits, EspError> {
        data_bits(self.port())
    }

    /// Change the type of parity checking
    pub fn change_parity(&self, parity: config::Parity) -> Result<&Self, EspError> {
        change_parity(self.port(), parity).map(|_| self)
    }

    /// Returns the current type of parity checking
    pub fn parity(&self) -> Result<config::Parity, EspError> {
        parity(self.port())
    }

    /// Change the baudrate.
    ///
    /// Will automatically select the clock source. When possible the reference clock (1MHz) will
    /// be used, because this is constant when the clock source/frequency changes.
    /// However if one of the clock frequencies is below 10MHz or if the baudrate is above
    /// the reference clock or if the baudrate cannot be set within 1.5%
    /// then use the APB clock.
    pub fn change_baudrate<T: Into<Hertz> + Copy>(&self, baudrate: T) -> Result<&Self, EspError> {
        change_baudrate(self.port(), baudrate).map(|_| self)
    }

    /// Returns the current baudrate
    pub fn baudrate(&self) -> Result<Hertz, EspError> {
        baudrate(self.port())
    }

    /// Write multiple bytes from a slice
    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        // `uart_write_bytes()` returns error (-1) or how many bytes were written
        let len = unsafe { uart_write_bytes(self.port(), bytes.as_ptr().cast(), bytes.len()) };

        if len >= 0 {
            Ok(len as usize)
        } else {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
        }
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        esp!(unsafe { uart_wait_tx_done(self.port(), 0) })?;

        Ok(())
    }

    pub fn port(&self) -> uart_port_t {
        self.port as _
    }

    /// Get count of remaining capacity in the transmit ring buffer
    #[cfg(any(
        not(esp_idf_version_major = "4"),
        all(
            esp_idf_version_minor = "4",
            not(any(esp_idf_version_patch = "0", esp_idf_version_patch = "1")),
        ),
    ))]
    pub fn count(&self) -> Result<usize, EspError> {
        remaining_write_capacity(self.port())
    }
}

impl<'d> Drop for UartTxDriver<'d> {
    fn drop(&mut self) {
        self.owner.drop_impl(self.port()).unwrap()
    }
}

impl<'d> embedded_hal::serial::ErrorType for UartTxDriver<'d> {
    type Error = SerialError;
}

impl<'d> embedded_hal_0_2::serial::Write<u8> for UartTxDriver<'d> {
    type Error = SerialError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        UartTxDriver::flush(self).map_err(to_nb_err)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        check_nb(UartTxDriver::write(self, &[byte]), ())
    }
}

impl<'d> embedded_hal_nb::serial::Write<u8> for UartTxDriver<'d> {
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        UartTxDriver::flush(self).map_err(to_nb_err)
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        check_nb(UartTxDriver::write(self, &[byte]), ())
    }
}

impl<'d> core::fmt::Write for UartTxDriver<'d> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = s.as_bytes();
        let mut offset = 0;

        while offset < buf.len() {
            offset += self.write(buf).map_err(|_| core::fmt::Error)?
        }

        Ok(())
    }
}

fn new_common<UART: Uart>(
    _uart: impl Peripheral<P = UART>,
    tx: Option<impl Peripheral<P = impl OutputPin>>,
    rx: Option<impl Peripheral<P = impl InputPin>>,
    cts: Option<impl Peripheral<P = impl InputPin>>,
    rts: Option<impl Peripheral<P = impl OutputPin>>,
    config: &config::Config,
) -> Result<(), EspError> {
    let tx = tx.map(|tx| tx.into_ref());
    let rx = rx.map(|rx| rx.into_ref());
    let cts = cts.map(|cts| cts.into_ref());
    let rts = rts.map(|rts| rts.into_ref());

    #[allow(clippy::needless_update)]
    let uart_config = uart_config_t {
        baud_rate: config.baudrate.0 as i32,
        data_bits: config.data_bits.into(),
        parity: config.parity.into(),
        stop_bits: config.stop_bits.into(),
        flow_ctrl: config.flow_control.into(),
        rx_flow_ctrl_thresh: config.flow_control_rts_threshold,
        #[cfg(not(esp_idf_version_major = "4"))]
        source_clk: config.source_clock.into(),
        #[cfg(esp_idf_version_major = "4")]
        __bindgen_anon_1: uart_config_t__bindgen_ty_1 {
            source_clk: config.source_clock.into(),
        },
        ..Default::default()
    };

    esp!(unsafe { uart_param_config(UART::port(), &uart_config) })?;
    esp!(unsafe { uart_intr_config(UART::port(), IntrFlags::to_native(config.intr_flags) as _) })?;

    esp!(unsafe {
        uart_set_pin(
            UART::port(),
            tx.as_ref().map_or(-1, |p| p.pin()),
            rx.as_ref().map_or(-1, |p| p.pin()),
            rts.as_ref().map_or(-1, |p| p.pin()),
            cts.as_ref().map_or(-1, |p| p.pin()),
        )
    })?;

    esp!(unsafe {
        uart_driver_install(
            UART::port(),
            UART_FIFO_SIZE * 2,
            if tx.is_some() { UART_FIFO_SIZE * 2 } else { 0 },
            0,
            ptr::null_mut(),
            0,
        )
    })?;

    Ok(())
}

fn stop_bits(port: uart_port_t) -> Result<config::StopBits, EspError> {
    let mut stop_bits: uart_stop_bits_t = 0;
    esp_result!(
        unsafe { uart_get_stop_bits(port, &mut stop_bits) },
        stop_bits.into()
    )
}

fn change_stop_bits(port: uart_port_t, stop_bits: config::StopBits) -> Result<(), EspError> {
    esp!(unsafe { uart_set_stop_bits(port, stop_bits.into()) })
}

fn data_bits(port: uart_port_t) -> Result<config::DataBits, EspError> {
    let mut data_bits: uart_word_length_t = 0;
    esp_result!(
        unsafe { uart_get_word_length(port, &mut data_bits) },
        data_bits.into()
    )
}

fn change_data_bits(port: uart_port_t, data_bits: config::DataBits) -> Result<(), EspError> {
    esp!(unsafe { uart_set_word_length(port, data_bits.into()) })
}

fn parity(port: uart_port_t) -> Result<config::Parity, EspError> {
    let mut parity: uart_parity_t = 0;
    esp_result!(unsafe { uart_get_parity(port, &mut parity) }, parity.into())
}

fn change_parity(port: uart_port_t, parity: config::Parity) -> Result<(), EspError> {
    esp!(unsafe { uart_set_parity(port, parity.into()) })
}

fn baudrate(port: uart_port_t) -> Result<Hertz, EspError> {
    let mut baudrate: u32 = 0;
    esp_result!(
        unsafe { uart_get_baudrate(port, &mut baudrate) },
        baudrate.into()
    )
}

fn change_baudrate<T: Into<Hertz> + Copy>(port: uart_port_t, baudrate: T) -> Result<(), EspError> {
    esp!(unsafe { uart_set_baudrate(port, baudrate.into().into()) })
}

fn delete_driver(port: uart_port_t) -> Result<(), EspError> {
    esp!(unsafe { uart_driver_delete(port) })
}

pub fn remaining_unread_bytes(port: uart_port_t) -> Result<usize, EspError> {
    let mut size = 0;
    esp_result!(unsafe { uart_get_buffered_data_len(port, &mut size) }, size)
}

#[cfg(any(
    not(esp_idf_version_major = "4"),
    all(
        esp_idf_version_minor = "4",
        not(any(esp_idf_version_patch = "0", esp_idf_version_patch = "1")),
    ),
))]
pub fn remaining_write_capacity(port: uart_port_t) -> Result<usize, EspError> {
    let mut size = 0;
    esp_result!(
        unsafe { uart_get_tx_buffer_free_size(port, &mut size) },
        size
    )
}

enum Owner {
    Owned,
    Borrowed,
    Shared,
}

impl Owner {
    fn drop_impl(&self, port: uart_port_t) -> Result<(), EspError> {
        let needs_drop = match self {
            Owner::Owned => true,
            Owner::Borrowed => false,
            Owner::Shared => REFS[port as usize].fetch_sub(1, Ordering::SeqCst) == 0,
        };
        needs_drop.then(|| delete_driver(port)).unwrap_or(Ok(()))
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
    if err.code() == ESP_ERR_TIMEOUT {
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
#[cfg(any(esp32, esp32s3))]
impl_uart!(UART2: 2);

#[allow(clippy::declare_interior_mutable_const)]
const NO_REFS: AtomicU8 = AtomicU8::new(0);
static REFS: [AtomicU8; UART_NUM_MAX as usize] = [NO_REFS; UART_NUM_MAX as usize];
