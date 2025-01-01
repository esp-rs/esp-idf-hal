//! USB Serial / JTAG peripheral and driver
//!
//! Communication through a virtualized UART-like USB-CDC interface.
#![allow(non_camel_case_types)]

use crate::io::EspIOError;
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::sys::{
    esp, usb_serial_jtag_driver_config_t, usb_serial_jtag_driver_install,
    usb_serial_jtag_driver_uninstall, usb_serial_jtag_is_connected, usb_serial_jtag_read_bytes,
    usb_serial_jtag_write_bytes, EspError, TickType_t,
};
use crate::{delay, gpio};

/// A type alias for the USB Serial driver configuration
pub type UsbSerialConfig = config::Config;

/// USB D- GPIO pin
#[cfg(esp32c3)]
pub type UsbDMinGpio = gpio::Gpio18;
/// USB D+ GPIO pin
#[cfg(esp32c3)]
pub type UsbDPlusGpio = gpio::Gpio19;
/// USB D- GPIO pin
#[cfg(esp32c5)]
pub type UsbDMinGpio = gpio::Gpio13;
/// USB D+ GPIO pin
#[cfg(esp32c5)]
pub type UsbDPlusGpio = gpio::Gpio14;
/// USB D- GPIO pin
#[cfg(esp32c6)]
pub type UsbDMinGpio = gpio::Gpio12;
/// USB D+ GPIO pin
#[cfg(esp32c6)]
pub type UsbDPlusGpio = gpio::Gpio13;
/// USB D- GPIO pin
#[cfg(esp32h2)]
pub type UsbDMinGpio = gpio::Gpio26;
/// USB D+ GPIO pin
#[cfg(esp32h2)]
pub type UsbDPlusGpio = gpio::Gpio27;
/// USB D- GPIO pin
#[cfg(esp32p4)]
pub type UsbDMinGpio = gpio::Gpio24;
/// USB D+ GPIO pin
#[cfg(esp32p4)]
pub type UsbDPlusGpio = gpio::Gpio25;
// TODO
// #[cfg(esp32p4)]
// pub type UsbDMinGpio2 = gpio::Gpio26;
// #[cfg(esp32p4)]
// pub type UsbDPlusGpio2 = gpio::Gpio27;
/// USB D- GPIO pin
#[cfg(esp32s3)]
pub type UsbDMinGpio = gpio::Gpio19;
/// USB D+ GPIO pin
#[cfg(esp32s3)]
pub type UsbDPlusGpio = gpio::Gpio20;

/// USB Serial driver configuration
pub mod config {
    /// USB Serial driver configuration
    #[derive(Debug, Clone)]
    #[non_exhaustive]
    pub struct Config {
        pub tx_buffer_size: usize,
        pub rx_buffer_size: usize,
    }

    impl Config {
        /// Create a new configuration with default values
        pub const fn new() -> Self {
            Self {
                tx_buffer_size: 256,
                rx_buffer_size: 256,
            }
        }

        /// Set the transmit buffer size
        #[must_use]
        pub fn tx_buffer_size(mut self, tx_buffer_size: usize) -> Self {
            self.tx_buffer_size = tx_buffer_size;
            self
        }

        /// Set the receive buffer size
        #[must_use]
        pub fn rx_buffer_size(mut self, rx_buffer_size: usize) -> Self {
            self.rx_buffer_size = rx_buffer_size;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// USB-SERIAL driver
pub struct UsbSerialDriver<'d>(PeripheralRef<'d, USB_SERIAL>);

impl<'d> UsbSerialDriver<'d> {
    /// Create a new USB Serial driver
    ///
    /// # Arguments
    /// - `usb_serial`: The USB Serial peripheral
    /// - `config`: The driver configuration
    /// - `usb_d_min`: The USB D- GPIO pin
    /// - `usb_d_plus`: The USB D+ GPIO pin
    pub fn new(
        usb_serial: impl Peripheral<P = USB_SERIAL> + 'd,
        _usb_d_min: impl Peripheral<P = UsbDMinGpio>,
        _usb_d_plus: impl Peripheral<P = UsbDPlusGpio>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        crate::into_ref!(usb_serial);

        let mut config = usb_serial_jtag_driver_config_t {
            tx_buffer_size: config.tx_buffer_size as _,
            rx_buffer_size: config.rx_buffer_size as _,
        };

        esp!(unsafe { usb_serial_jtag_driver_install(&mut config) })?;

        Ok(Self(usb_serial))
    }

    /// Check if the USB Serial is connected
    pub fn is_connected(&self) -> bool {
        unsafe { usb_serial_jtag_is_connected() }
    }

    /// Read bytes into a slice
    ///
    /// # Arguments
    /// - `buf`: The buffer to read into
    /// - `timeout`: The timeout in ticks
    ///
    /// # Returns
    /// The number of bytes read or an error if the operation failed or the timeout was reached
    pub fn read(&mut self, buf: &mut [u8], timeout: TickType_t) -> Result<usize, EspError> {
        let len = unsafe {
            usb_serial_jtag_read_bytes(buf.as_mut_ptr() as *mut _, buf.len() as _, timeout)
        };

        Ok(len as _)
    }

    /// Write bytes from a slice
    ///
    /// # Arguments
    /// - `bytes`: The bytes to write
    /// - `timeout`: The timeout in ticks
    ///
    /// # Returns
    /// The number of bytes written or an error if the operation failed or the timeout was reached
    pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<usize, EspError> {
        let len = unsafe {
            usb_serial_jtag_write_bytes(bytes.as_ptr() as *const _, bytes.len() as _, timeout)
        };

        Ok(len as _)
    }
}

impl Drop for UsbSerialDriver<'_> {
    fn drop(&mut self) {
        esp!(unsafe { usb_serial_jtag_driver_uninstall() }).unwrap();
    }
}

unsafe impl Send for UsbSerialDriver<'_> {}

impl embedded_io::ErrorType for UsbSerialDriver<'_> {
    type Error = EspIOError;
}

impl embedded_io::Read for UsbSerialDriver<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        UsbSerialDriver::read(self, buf, delay::BLOCK).map_err(EspIOError)
    }
}

impl embedded_io::Write for UsbSerialDriver<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        UsbSerialDriver::write(self, buf, delay::BLOCK).map_err(EspIOError)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal_0_2::serial::Write<u8> for UsbSerialDriver<'_> {
    type Error = EspError;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Ok(())
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        UsbSerialDriver::write(self, &[byte], delay::BLOCK)?;

        Ok(())
    }
}

impl core::fmt::Write for UsbSerialDriver<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let buf = s.as_bytes();
        let mut offset = 0;

        while offset < buf.len() {
            offset += self
                .write(buf, delay::BLOCK)
                .map_err(|_| core::fmt::Error)?
        }

        Ok(())
    }
}

crate::impl_peripheral!(USB_SERIAL);
