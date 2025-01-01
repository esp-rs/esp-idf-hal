#![allow(non_camel_case_types)]
//! USB Serial / JTAG peripheral
//!
//! Communication through a virtualized UART-like USB-CDC interface.
//!
//! By default, `println!` and `log!` output will be redirected to it if no UART
//! connection is established to a HOST PC. The peripheral is initialized at startup
//! and is using the ESP console slot 2 by default.
//!
//! ESP console slot 2 cannot be used to read from the HOST, only writing is supported.
//! If reading from the HOST is necessary, reconfigure the ESP console by setting
//! the following into your projects sdkconfig.default file:
//! ```
//! CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
//! ```

// TODO: impl usb_serial driver

crate::impl_peripheral!(USB_SERIAL);
