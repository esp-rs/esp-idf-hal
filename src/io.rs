//! Error types

use core::fmt::{self, Display, Formatter};
use core::marker::PhantomData;

use crate::peripheral::Peripheral;
use crate::sys::{EspError, ESP_ERR_INVALID_STATE};

pub use embedded_io::*;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct EspIOError(pub EspError);

impl Error for EspIOError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl From<EspError> for EspIOError {
    fn from(e: EspError) -> Self {
        Self(e)
    }
}

impl Display for EspIOError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EspIOError {}

pub mod asynch {
    pub use embedded_io_async::*;
}

/// A driver for bufferd / blocking communication ontop of Standard IO Streams.
///
/// By default all communication via std::io:stdin / stdout is non-blocking.
/// That means if a user wants to read from stdin, they have to constantly poll the driver, since
/// the respective hardware FIFO buffers are relative small. Also users would have to handle WouldBlock
/// errors on every call, thouse make it unergonomic to handle.
///
/// A BlockingIoDriver will instruct the VFS(Virtual File System) to use the drivers interrupt driven,
/// blocking read and write functions instead.
pub struct BlockingIoDriver<T> {
    phantom: PhantomData<T>,
}

pub struct BlockingUartIo<T: Uart> {
    _driver: BlockingIoDriver<T>,
}

#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
use crate::usb_serial::UsbSerial;
#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
pub struct BlockingSerialIo<T: UsbSerial> {
    _driver: BlockingIoDriver<T>,
}

use crate::sys::esp;
use crate::sys::{
    esp_vfs_dev_uart_use_driver, esp_vfs_dev_uart_use_nonblocking, uart_driver_delete,
    uart_driver_install, uart_is_driver_installed,
};
use crate::uart::Uart;

impl<T: Uart> BlockingIoDriver<T> {
    /// Create a BlockingIoDriver using UART
    pub fn uart<UART: Uart>(
        // we could also take the gpio pins to make sure they are not used?
        uart: impl Peripheral<P = UART>,
        // Size of the receive buffer the driver will allocate internally.
        //
        // For performance reasons make it at least the size of crate::sys::SOC_UART_FIFO_LEN.
        rx_buffer_size: u32,
        // Size of the transmit buffer the driver will allocate internally.
        //
        // For performance reasons make it either 0 or at least the size of crate::sys::SOC_UART_FIFO_LEN.
        tx_buffer_size: u32,
    ) -> Result<BlockingUartIo<UART>, EspError> {
        // future improvement: expose queue and event handler

        // sanity check if somehow driver was already installed somewhere else
        let is_installed = unsafe { uart_is_driver_installed(UART::port()) };

        if is_installed {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        let _ = uart;
        esp!(unsafe {
            uart_driver_install(
                UART::port(),
                rx_buffer_size as _,
                tx_buffer_size as _,
                10,
                core::ptr::null_mut(),
                0,
            )
        })?;

        unsafe { esp_vfs_dev_uart_use_driver(UART::port()) }

        Ok(BlockingUartIo {
            _driver: BlockingIoDriver {
                phantom: PhantomData,
            },
        })
    }
}

impl<T: Uart> Drop for BlockingUartIo<T> {
    fn drop(&mut self) {
        unsafe { esp_vfs_dev_uart_use_nonblocking(T::port()) }
        let _ = unsafe { uart_driver_delete(T::port()) };
    }
}

use crate::sys::{
    esp_vfs_usb_serial_jtag_use_driver, esp_vfs_usb_serial_jtag_use_nonblocking,
    usb_serial_jtag_driver_config_t, usb_serial_jtag_driver_install,
    usb_serial_jtag_driver_uninstall,
};

#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
impl<T: UsbSerial> BlockingIoDriver<T> {
    /// Create a BlockingIoDriver using Serial / JTAG interface over USB-CDC
    ///
    /// By default the esp console input is routed to console slot1.
    /// Only slot1 is capable of reading from stdin.
    /// Thouse slot1 needs to be set to usb_serial_jtat.
    /// This can be done by adding CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y to your sdkconfig.defaults file
    pub fn serial<SERIAL: UsbSerial>(
        // we could also take the gpio pins to make sure they are not used?
        serial: impl Peripheral<P = SERIAL>,
        // Size of the receive buffer the driver will allocate internally.
        rx_buffer_size: u32,
        // Size of the transmit buffer the driver will allocate internally.
        tx_buffer_size: u32,
    ) -> Result<BlockingSerialIo<SERIAL>, EspError> {
        let _ = serial;

        let mut config = usb_serial_jtag_driver_config_t {
            tx_buffer_size,
            rx_buffer_size,
        };
        esp!(unsafe { usb_serial_jtag_driver_install(&mut config) })?;

        unsafe { esp_vfs_usb_serial_jtag_use_driver() }

        Ok(BlockingSerialIo {
            _driver: BlockingIoDriver {
                phantom: PhantomData,
            },
        })
    }
}

#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
impl<T: UsbSerial> Drop for BlockingSerialIo<T> {
    fn drop(&mut self) {
        unsafe { esp_vfs_usb_serial_jtag_use_nonblocking() }
        let _ = unsafe { usb_serial_jtag_driver_uninstall() };
    }
}
