use core::borrow::Borrow;
use core::ffi::c_void;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::AtomicBool;
use core::time::Duration;
use std::sync::Condvar;

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

use esp_idf_sys::*;

use crate::delay::*;
use crate::gpio::*;
use crate::interrupt::asynch::HalIsrNotification;
use crate::peripheral::Peripheral;
use crate::units::*;

pub use embedded_hal::i2c::Operation;

use super::I2c;

crate::embedded_hal_error!(
    I2cError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

pub type I2cConfig = config::Config;
#[cfg(not(esp32c2))]
pub type I2cSlaveConfig = config::SlaveDeviceConfig;

/// I2C configuration
pub mod config {
    use esp_idf_sys::*;

    use crate::units::*;

    // TODO: in bindings its XTAL called and in doc its APB
    const APB_SCLK: soc_periph_i2c_clk_src_t = soc_periph_i2c_clk_src_t_I2C_CLK_SRC_XTAL;
    const FAST_SCLK: soc_periph_i2c_clk_src_t = soc_periph_i2c_clk_src_t_I2C_CLK_SRC_RC_FAST;

    /// i2c source clock
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    #[allow(non_camel_case_types)]
    pub enum SourceClock {
        APB,
        RC_FAST,
    }

    impl SourceClock {
        pub const fn default() -> Self {
            Self::from_raw(soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT)
        }

        pub const fn from_raw(source_clock: soc_periph_i2c_clk_src_t) -> Self {
            match source_clock {
                APB_SCLK => SourceClock::APB,
                FAST_SCLK => SourceClock::RC_FAST,
                _ => unreachable!(),
            }
        }
    }

    impl Default for SourceClock {
        fn default() -> Self {
            SourceClock::default()
        }
    }

    impl From<SourceClock> for i2c_clock_source_t {
        fn from(source_clock: SourceClock) -> Self {
            match source_clock {
                SourceClock::RC_FAST => FAST_SCLK,
                SourceClock::APB => APB_SCLK,
            }
        }
    }

    impl From<uart_sclk_t> for SourceClock {
        fn from(source_clock: i2c_clock_source_t) -> Self {
            Self::from_raw(source_clock)
        }
    }

    /// I2C Master configuration
    #[derive(Debug, Clone)]
    pub struct Config {
        pub pullup_enabled: bool,
        pub source_clock: SourceClock,
        pub glitch_ignore_cnt: u8,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn enable_pullup(mut self, enable: bool) -> Self {
            self.pullup_enabled = enable;
            self
        }

        #[must_use]
        pub fn source_clock(mut self, source_clock: SourceClock) -> Self {
            self.source_clock = source_clock;
            self
        }

        #[must_use]
        pub fn glitch_ignore_count(mut self, count: u8) -> Self {
            self.glitch_ignore_cnt = count;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                pullup_enabled: true,
                source_clock: SourceClock::default(),
                glitch_ignore_cnt: 7,
            }
        }
    }

    #[derive(Debug, Clone)]
    pub enum DeviceAddress {
        SevenBit(u8),
        TenBit(u16),
    }

    impl DeviceAddress {
        pub(super) fn address(&self) -> u16 {
            match self {
                DeviceAddress::SevenBit(addr) => *addr as u16,
                // TODO: if cfg allows 10 bit address
                DeviceAddress::TenBit(addr) => *addr,
            }
        }
    }

    impl From<DeviceAddress> for i2c_addr_bit_len_t {
        fn from(address: DeviceAddress) -> Self {
            match address {
                DeviceAddress::SevenBit(_) => i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
                DeviceAddress::TenBit(_) => i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_10,
            }
        }
    }

    #[derive(Debug, Clone)]
    pub struct DeviceConfig {
        pub address: DeviceAddress,
        pub baudrate: Hertz,
    }

    impl DeviceConfig {
        pub const fn new(address: DeviceAddress) -> Self {
            Self {
                address,
                baudrate: Hertz(1_000_000),
            }
        }

        #[must_use]
        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }
    }

    /// I2C Slave configuration
    #[cfg(not(esp32c2))]
    #[derive(Debug, Clone)]
    pub struct SlaveDeviceConfig {
        pub source_clock: SourceClock,
        pub broadcast_enable: bool,
        pub send_buffer_depth: u32,
    }

    #[cfg(not(esp32c2))]
    impl SlaveDeviceConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn source_clock(mut self, source_clock: SourceClock) -> Self {
            self.source_clock = source_clock;
            self
        }

        #[must_use]
        pub fn enable_broadcast(mut self, enable: bool) -> Self {
            self.broadcast_enable = enable;
            self
        }

        #[must_use]
        pub fn set_send_buffer_depth(mut self, depth: u32) -> Self {
            self.send_buffer_depth = depth;
            self
        }
    }

    #[cfg(not(esp32c2))]
    impl Default for SlaveDeviceConfig {
        fn default() -> Self {
            Self {
                source_clock: SourceClock::default(),
                broadcast_enable: false,
                send_buffer_depth: 0,
            }
        }
    }
}

#[cfg(any(esp32c3, esp32c2, esp32c6))]
static ASYNC_DEVICE_IN_USE: [AtomicBool; 1] = [AtomicBool::new(false)];

#[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
static ASYNC_DEVICE_IN_USE: [AtomicBool; 2] = [AtomicBool::new(false), AtomicBool::new(false)];

mod private {
    pub trait Sealed {}
}

// Should allow to construct a `I2cDeviceDriver` from a `I2cDriver` or `AsyncI2cDriver`.
pub trait BorrowI2cDriver<'d>: private::Sealed {
    fn borrowed_bus_handle(&self) -> i2c_master_bus_handle_t;
    fn borrowed_port(&self) -> u8;
}

macro_rules! impl_borrow_trait {
    ($( $accessor:ty ),*) => {
        $(
            impl<'d> BorrowI2cDriver<'d> for $accessor {
                fn borrowed_bus_handle(&self) -> i2c_master_bus_handle_t {
                    self.bus_handle()
                }

                fn borrowed_port(&self) -> u8 {
                    self.port()
                }
            }

            impl<'d> private::Sealed for $accessor {}
        )*
    };
}

impl_borrow_trait!(
    I2cDriver<'d>,
    &I2cDriver<'d>,
    &mut I2cDriver<'d>,
    std::rc::Rc<I2cDriver<'d>>,
    std::sync::Arc<I2cDriver<'d>>,
    std::boxed::Box<I2cDriver<'d>>,
    AsyncI2cDriver<'d>,
    &AsyncI2cDriver<'d>,
    &mut AsyncI2cDriver<'d>,
    std::rc::Rc<AsyncI2cDriver<'d>>,
    std::sync::Arc<AsyncI2cDriver<'d>>,
    std::boxed::Box<AsyncI2cDriver<'d>>
);

/// I2C Driver controlling the I2C bus.
///
/// # Example
/// ```no_run
/// let driver_config = config::Config::new();
/// let driver = I2cDriver::new(..., &device_config).unwrap();
/// ```
pub struct I2cDriver<'d> {
    port: u8,
    handle: i2c_master_bus_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> I2cDriver<'d> {
    pub fn new<I2C: I2c>(
        _i2c: impl Peripheral<P = I2C> + 'd,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        super::check_and_set_modern_driver();

        let handle = init_master_bus(_i2c, sda, scl, config, 0)?;

        Ok(Self {
            port: I2C::port() as u8,
            handle,
            _p: PhantomData,
        })
    }

    pub fn port(&self) -> u8 {
        self.port
    }

    fn bus_handle(&self) -> i2c_master_bus_handle_t {
        self.handle
    }
}

unsafe impl<'d> Send for I2cDriver<'d> {}

impl<'d> Drop for I2cDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
    }
}

/// The `async` version of the I2C driver.
///
/// This driver allows to have one `AsyncI2cDeviceDriver` per bus with `async` support.
///
/// # Important
/// I2C master asynchronous transaction is still an experimental feature. (The issue is when
/// asynchronous transaction is very large, it will cause memory problem.)
pub struct AsyncI2cDriver<'d> {
    port: u8,
    handle: i2c_master_bus_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> AsyncI2cDriver<'d> {
    /// Creates a new i2c driver with `async` support.
    pub fn new<I2C: I2c>(
        _i2c: impl Peripheral<P = I2C> + 'd,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        super::check_and_set_modern_driver();

        let handle = init_master_bus(_i2c, sda, scl, config, 1)?;

        Ok(Self {
            port: I2C::port() as u8,
            handle,
            _p: PhantomData,
        })
    }

    pub fn port(&self) -> u8 {
        self.port
    }

    fn bus_handle(&self) -> i2c_master_bus_handle_t {
        self.handle
    }
}

unsafe impl<'d> Send for AsyncI2cDriver<'d> {}

impl<'d> Drop for AsyncI2cDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
    }
}

macro_rules! impl_driver_blocking_functions {
    ($driver:ident) => {
        impl<'d> $driver<'d> {
            /// Probe device on the bus.
            pub fn probe_device(
                &mut self,
                address: config::DeviceAddress,
                timeout: TickType_t,
            ) -> Result<(), EspError> {
                esp!(unsafe {
                    i2c_master_probe(self.bus_handle(), address.address(), timeout as i32)
                })
            }

            // Helper to use the embedded_hal traits.
            fn read(
                &self,
                addr: u8,
                buffer: &mut [u8],
                timeout: TickType_t,
            ) -> Result<(), EspError> {
                let device = init_device(
                    self.bus_handle(),
                    &config::DeviceConfig::new(config::DeviceAddress::SevenBit(addr)),
                )?;

                esp!(unsafe {
                    i2c_master_receive(
                        device,
                        buffer.as_mut_ptr().cast(),
                        buffer.len(),
                        timeout as i32,
                    )
                })
            }

            // Helper to use the embedded_hal traits.
            fn write(
                &mut self,
                addr: u8,
                bytes: &[u8],
                timeout: TickType_t,
            ) -> Result<(), EspError> {
                let device = init_device(
                    self.bus_handle(),
                    &config::DeviceConfig::new(config::DeviceAddress::SevenBit(addr)),
                )?;

                esp!(unsafe {
                    i2c_master_transmit(device, bytes.as_ptr().cast(), bytes.len(), timeout as i32)
                })
            }

            // Helper to use the embedded_hal traits.
            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
                timeout: TickType_t,
            ) -> Result<(), EspError> {
                let device = init_device(
                    self.bus_handle(),
                    &config::DeviceConfig::new(config::DeviceAddress::SevenBit(addr)),
                )?;

                esp!(unsafe {
                    i2c_master_transmit_receive(
                        device,
                        bytes.as_ptr().cast(),
                        bytes.len(),
                        buffer.as_mut_ptr().cast(),
                        buffer.len(),
                        timeout as i32,
                    )
                })
            }
        }

        impl<'d> embedded_hal_0_2::blocking::i2c::Read for $driver<'d> {
            type Error = I2cError;

            fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                Self::read(self, addr, buffer, BLOCK).map_err(to_i2c_err)
            }
        }

        impl<'d> embedded_hal_0_2::blocking::i2c::Write for $driver<'d> {
            type Error = I2cError;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                Self::write(self, addr, bytes, BLOCK).map_err(to_i2c_err)
            }
        }

        impl<'d> embedded_hal_0_2::blocking::i2c::WriteRead for $driver<'d> {
            type Error = I2cError;

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                Self::write_read(self, addr, bytes, buffer, BLOCK).map_err(to_i2c_err)
            }
        }

        impl<'d> embedded_hal::i2c::ErrorType for $driver<'d> {
            type Error = I2cError;
        }

        impl<'d> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for $driver<'d> {
            fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
                Self::read(self, addr, buffer, BLOCK).map_err(to_i2c_err)
            }

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                Self::write(self, addr, bytes, BLOCK).map_err(to_i2c_err)
            }

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                Self::write_read(self, addr, bytes, buffer, BLOCK).map_err(to_i2c_err)
            }

            fn transaction(
                &mut self,
                _addr: u8,
                _operations: &mut [embedded_hal::i2c::Operation<'_>],
            ) -> Result<(), Self::Error> {
                unimplemented!("transactional not implemented")
            }
        }
    };
}

impl_driver_blocking_functions!(I2cDriver);
impl_driver_blocking_functions!(AsyncI2cDriver);

pub struct I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    _driver: T,
    handle: i2c_master_dev_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d, T> I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    pub fn new(driver: T, config: &config::DeviceConfig) -> Result<Self, EspError> {
        let handle = init_device(driver.borrowed_bus_handle(), &config)?;

        Ok(I2cDeviceDriver {
            _driver: driver,
            handle,
            _p: PhantomData,
        })
    }

    fn device_handle(&self) -> i2c_master_dev_handle_t {
        self.handle
    }
}

impl<'d, T> I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_receive(
                self.device_handle(),
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })
    }

    pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit(
                self.device_handle(),
                bytes.as_ptr().cast(),
                bytes.len(),
                timeout as i32,
            )
        })
    }

    pub fn write_read(
        &mut self,
        bytes: &[u8],
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit_receive(
                self.device_handle(),
                bytes.as_ptr().cast(),
                bytes.len(),
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })
    }
}

impl<'d, T> Drop for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d>,
{
    fn drop(&mut self) {
        esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::Read for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    type Error = I2cError;

    fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::Write for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    type Error = I2cError;

    fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, bytes, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::WriteRead for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    type Error = I2cError;

    fn write_read(
        &mut self,
        _addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal::i2c::ErrorType for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    type Error = I2cError;
}

impl<'d, T> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDeviceDriver<'d, T>
where
    T: BorrowI2cDriver<'d> + 'd,
{
    fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, bytes, BLOCK).map_err(to_i2c_err)
    }

    fn write_read(
        &mut self,
        _addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn transaction(
        &mut self,
        _addr: u8,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        unimplemented!("transactional not implemented")
    }
}

pub struct AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    driver: T,
    handle: i2c_master_dev_handle_t,
    _isr_callback_handle: IsrCallbackHandle,
    _p: PhantomData<&'d mut ()>,
}

impl<'d, T> AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    pub fn new(driver: T, config: &config::DeviceConfig) -> Result<Self, EspError> {
        // As documented here https://docs.espressif.com/projects/esp-idf/en/v5.2.3/esp32/api-reference/peripherals/i2c.html?highlight=i2c#_CPPv435i2c_master_register_event_callbacks23i2c_master_dev_handle_tPK28i2c_master_event_callbacks_tPv
        if ASYNC_DEVICE_IN_USE[driver.borrow().port() as usize]
            .swap(true, core::sync::atomic::Ordering::SeqCst)
        {
            // Should indicate that another async device is already in use
            return Err(EspError::from(ESP_ERR_INVALID_STATE).unwrap());
        }

        let handle = init_device(driver.borrow().bus_handle(), &config)?;
        let isr_callback_handle = enable_master_dev_isr_callback(handle, driver.borrow().port())?;

        Ok(Self {
            driver,
            handle,
            _isr_callback_handle: isr_callback_handle,
            _p: PhantomData,
        })
    }

    /// Reads data from the device.
    ///
    /// # Safety
    /// The buffer must be valid until the future returned by this function is resolved. Don't drop
    /// the future before it's resolved. This can lead to corruption of the data.
    pub async fn async_read(
        &mut self,
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        let handle = self.handle;
        let driver = self.driver.borrow();
        let port = driver.port();

        esp!(unsafe {
            i2c_master_receive(
                handle,
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })?;

        NOTIFIER[port as usize].wait().await;
        Ok(())
    }

    /// Writes data to the device.
    ///
    /// # Safety
    /// The buffer must be valid until the future returned by this function is resolved. Don't drop
    /// the future before it's resolved. This can lead to corruption of the data.
    pub async fn async_write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
        let handle = self.handle;
        let driver = self.driver.borrow();
        let port = driver.port();

        esp!(unsafe {
            i2c_master_transmit(handle, bytes.as_ptr().cast(), bytes.len(), timeout as i32)
        })?;

        NOTIFIER[port as usize].wait().await;
        Ok(())
    }

    /// Writes data to and reads data from the device.
    ///
    /// # Safety
    /// The buffer must be valid until the future returned by this function is resolved. Don't drop
    /// the future before it's resolved. This can lead to corruption of the data.
    pub async fn async_write_read(
        &mut self,
        bytes: &[u8],
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        let handle = self.handle;
        let driver = self.driver.borrow();
        let port = driver.port();

        esp!(unsafe {
            i2c_master_transmit_receive(
                handle,
                bytes.as_ptr().cast(),
                bytes.len(),
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })?;

        NOTIFIER[port as usize].wait().await;
        Ok(())
    }

    fn device_handle(&self) -> i2c_master_dev_handle_t {
        self.handle
    }
}

impl<'d, T> AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_receive(
                self.device_handle(),
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })
    }

    pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit(
                self.device_handle(),
                bytes.as_ptr().cast(),
                bytes.len(),
                timeout as i32,
            )
        })
    }

    pub fn write_read(
        &mut self,
        bytes: &[u8],
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit_receive(
                self.device_handle(),
                bytes.as_ptr().cast(),
                bytes.len(),
                buffer.as_mut_ptr().cast(),
                buffer.len(),
                timeout as i32,
            )
        })
    }
}

impl<'d, T> Drop for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>>,
{
    fn drop(&mut self) {
        esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
        ASYNC_DEVICE_IN_USE[self.driver.borrow().port() as usize]
            .store(false, core::sync::atomic::Ordering::SeqCst);
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::Read for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    type Error = I2cError;

    fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::Write for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    type Error = I2cError;

    fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, bytes, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::i2c::WriteRead for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    type Error = I2cError;

    fn write_read(
        &mut self,
        _addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d, T> embedded_hal::i2c::ErrorType for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    type Error = I2cError;
}

impl<'d, T> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress>
    for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>> + 'd,
{
    fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, bytes, BLOCK).map_err(to_i2c_err)
    }

    fn write_read(
        &mut self,
        _addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn transaction(
        &mut self,
        _addr: u8,
        _operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        unimplemented!("transactional not implemented")
    }
}

#[cfg(not(esp_idf_i2c_isr_iram_safe))]
impl<'d, T> embedded_hal_async::i2c::I2c<embedded_hal::i2c::SevenBitAddress>
    for AsyncI2cDeviceDriver<'d, T>
where
    T: Borrow<AsyncI2cDriver<'d>>,
{
    async fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        Self::async_read(self, buffer, BLOCK)
            .await
            .map_err(to_i2c_err)
    }

    async fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Self::async_write(self, bytes, BLOCK)
            .await
            .map_err(to_i2c_err)
    }

    async fn write_read(
        &mut self,
        _address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        Self::async_write_read(self, bytes, buffer, BLOCK)
            .await
            .map_err(to_i2c_err)
    }

    async fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        unimplemented!("transactional not implemented")
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------- Slave ----------------------------------------------------
// ------------------------------------------------------------------------------------------------

static I2C_BLOCKING_READ_LOCK: (std::sync::Mutex<bool>, Condvar) =
    (std::sync::Mutex::new(false), Condvar::new());

#[cfg(not(esp32c2))]
pub struct I2cSlaveDriver<'d> {
    i2c: u8,
    handle: i2c_slave_dev_handle_t,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(not(esp32c2))]
unsafe impl<'d> Send for I2cSlaveDriver<'d> {}

#[cfg(not(esp32c2))]
impl<'d> I2cSlaveDriver<'d> {
    pub fn new<I2C: I2c>(
        _i2c: impl Peripheral<P = I2C> + 'd,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        address: config::DeviceAddress,
        config: &config::SlaveDeviceConfig,
    ) -> Result<Self, EspError> {
        super::check_and_set_modern_driver();

        let handle = init_slave_device(_i2c, sda, scl, address, config)?;

        enable_slave_isr_callback(handle, I2C::port() as _)?;

        Ok(Self {
            i2c: I2C::port() as _,
            handle,
            _p: PhantomData,
        })
    }

    pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<usize, EspError> {
        esp!(unsafe { i2c_slave_receive(self.handle, buffer.as_mut_ptr(), buffer.len()) })?;
        let mut read_pending = I2C_BLOCKING_READ_LOCK.0.lock().unwrap();
        *read_pending = true;
        let (_guard, e) = Condvar::new()
            .wait_timeout_while(
                read_pending,
                Duration::from_millis(timeout as u64),
                |pending| *pending,
            )
            .unwrap();

        if e.timed_out() {
            Err(EspError::from(ESP_ERR_TIMEOUT).unwrap())
        } else {
            Ok(buffer.len())
        }
    }

    pub async fn async_read(&mut self, buffer: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe { i2c_slave_receive(self.handle, buffer.as_mut_ptr(), buffer.len()) })?;

        NOTIFIER[self.port() as usize].wait().await;
        Ok(())
    }

    pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_slave_transmit(
                self.handle,
                bytes.as_ptr(),
                bytes.len() as i32,
                timeout as i32,
            )
        })
    }

    pub fn port(&self) -> i2c_port_t {
        self.i2c as _
    }
}

#[cfg(not(esp32c2))]
impl<'d> Drop for I2cSlaveDriver<'d> {
    fn drop(&mut self) {
        disable_slave_isr_callback(self.handle).unwrap();
        esp!(unsafe { i2c_del_slave_device(self.handle) }).unwrap();
    }
}

fn init_master_bus<'d, I2C: I2c>(
    _i2c: impl Peripheral<P = I2C> + 'd,
    sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    config: &config::Config,
    trans_queue_depth: usize,
) -> Result<i2c_master_bus_handle_t, EspError> {
    crate::into_ref!(sda, scl);

    let config = i2c_master_bus_config_t {
        sda_io_num: sda.pin(),
        scl_io_num: scl.pin(),
        clk_source: config.source_clock.into(),
        flags: {
            let mut flags = i2c_master_bus_config_t__bindgen_ty_1::default();
            flags.set_enable_internal_pullup(config.pullup_enabled as _);
            flags
        },
        glitch_ignore_cnt: config.glitch_ignore_cnt,
        i2c_port: I2C::port() as i32,
        intr_priority: 0,
        trans_queue_depth,
    };

    let mut handle: i2c_master_bus_handle_t = ptr::null_mut();

    esp!(unsafe { i2c_new_master_bus(&config, &mut handle as _) })?;

    Ok(handle)
}

fn init_device(
    bus_handle: i2c_master_bus_handle_t,
    config: &config::DeviceConfig,
) -> Result<i2c_master_dev_handle_t, EspError> {
    // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
    if config.baudrate > 1.MHz().into() {
        return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
    }

    let config = i2c_device_config_t {
        device_address: config.address.address(),
        dev_addr_length: config.address.clone().into(),
        scl_speed_hz: config.baudrate.into(),
    };

    let mut handle: i2c_master_dev_handle_t = ptr::null_mut();

    esp!(unsafe { i2c_master_bus_add_device(bus_handle, &config, &mut handle as _) })?;

    Ok(handle)
}

#[cfg(not(esp32c2))]
fn init_slave_device<'d, I2C: I2c>(
    _i2c: impl Peripheral<P = I2C> + 'd,
    sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    address: config::DeviceAddress,
    config: &config::SlaveDeviceConfig,
) -> Result<i2c_slave_dev_handle_t, EspError> {
    crate::into_ref!(sda, scl);

    let config = i2c_slave_config_t {
        sda_io_num: sda.pin(),
        scl_io_num: scl.pin(),
        clk_source: config.source_clock.into(),
        flags: {
            let mut flags = i2c_slave_config_t__bindgen_ty_1::default();
            flags.set_stretch_en(0);
            flags.set_broadcast_en(config.broadcast_enable as _);
            flags
        },
        i2c_port: I2C::port() as i32,
        intr_priority: 0,
        slave_addr: address.address(),
        addr_bit_len: address.into(),
        send_buf_depth: config.send_buffer_depth,
    };

    let mut handle: i2c_slave_dev_handle_t = ptr::null_mut();

    esp!(unsafe { i2c_new_slave_device(&config, &mut handle as _) })?;

    Ok(handle)
}

fn to_i2c_err(err: EspError) -> I2cError {
    if err.code() == ESP_FAIL {
        I2cError::new(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown), err)
    } else {
        I2cError::other(err)
    }
}

struct IsrCallbackHandle {
    handle: i2c_master_dev_handle_t,
}

impl Drop for IsrCallbackHandle {
    fn drop(&mut self) {
        disable_master_dev_isr_callback(self.handle).unwrap();
    }
}

#[cfg(not(esp_idf_i2c_isr_iram_safe))]
fn enable_master_dev_isr_callback(
    handle: i2c_master_dev_handle_t,
    host: u8,
) -> Result<IsrCallbackHandle, EspError> {
    esp!(unsafe {
        i2c_master_register_event_callbacks(
            handle,
            &i2c_master_event_callbacks_t {
                on_trans_done: Some(master_isr),
            },
            &NOTIFIER[host as usize] as *const _ as *mut _,
        )
    })?;

    Ok(IsrCallbackHandle { handle })
}

#[cfg(not(esp_idf_i2c_isr_iram_safe))]
fn disable_master_dev_isr_callback(handle: i2c_master_dev_handle_t) -> Result<(), EspError> {
    esp!(unsafe {
        i2c_master_register_event_callbacks(
            handle,
            &i2c_master_event_callbacks_t::default(),
            ptr::null_mut(),
        )
    })
}

#[cfg(not(esp_idf_i2c_isr_iram_safe))]
extern "C" fn master_isr(
    _handle: i2c_master_dev_handle_t,
    _data: *const i2c_master_event_data_t,
    user_data: *mut c_void,
) -> bool {
    let notifier: &HalIsrNotification =
        unsafe { (user_data as *const HalIsrNotification).as_ref() }.unwrap();

    notifier.notify_lsb()
}

#[cfg(all(not(esp32c2), not(esp_idf_i2c_isr_iram_safe)))]
fn enable_slave_isr_callback(handle: i2c_slave_dev_handle_t, host: u8) -> Result<(), EspError> {
    esp!(unsafe {
        i2c_slave_register_event_callbacks(
            handle,
            &i2c_slave_event_callbacks_t {
                on_recv_done: Some(slave_isr),
                on_stretch_occur: None,
            },
            &NOTIFIER[host as usize] as *const _ as *mut _,
        )
    })
}

#[cfg(all(not(esp32c2), not(esp_idf_i2c_isr_iram_safe)))]
fn disable_slave_isr_callback(handle: i2c_slave_dev_handle_t) -> Result<(), EspError> {
    esp!(unsafe {
        i2c_slave_register_event_callbacks(
            handle,
            &i2c_slave_event_callbacks_t::default(),
            ptr::null_mut(),
        )
    })
}

#[cfg(all(not(esp32c2), not(esp_idf_i2c_isr_iram_safe)))]
extern "C" fn slave_isr(
    _handle: i2c_slave_dev_handle_t,
    _data: *const i2c_slave_rx_done_event_data_t,
    user_data: *mut c_void,
) -> bool {
    // Notify for sync read
    {
        let mut read_pending = I2C_BLOCKING_READ_LOCK.0.lock().unwrap();
        I2C_BLOCKING_READ_LOCK.1.notify_one();
        *read_pending = false;
    }

    // Notify for async read
    let notifier: &HalIsrNotification =
        unsafe { (user_data as *const HalIsrNotification).as_ref() }.unwrap();
    notifier.notify_lsb()
}

#[cfg(any(esp32c3, esp32c2, esp32c6))]
static NOTIFIER: [HalIsrNotification; 1] = [HalIsrNotification::new()];

#[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
static NOTIFIER: [HalIsrNotification; 2] = [HalIsrNotification::new(), HalIsrNotification::new()];
