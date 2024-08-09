#![allow(non_camel_case_types)]
//! USB Serial / JTAG peripheral
//!
//! Communication through a virtualised Uart like USB-CDC interface
//!
//! By default println! and log! output will be redirected to it if no Uart
//! connection is established to a HOST PC. The peripheral is initialized at startup
//! and is using the esp console slot 2 by default.
//!
//! Esp console slot 2 cannot be used to read from the HOST, only writing is supported.
//! If reading from the HOST is nececarry reconfigure esp console by setting
//! the following into your projects sdkconfig.default file
//!
//! CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y

// TODO: impl usb_serial driver

/// USB Serial / JTAG peripheral
pub trait UsbSerial {}

macro_rules! impl_usb_serial {
    ($usb_serial:ident: $port:expr) => {
        crate::impl_peripheral!($usb_serial);

        impl UsbSerial for $usb_serial {}
    };
}

impl_usb_serial!(USB_SERIAL);
