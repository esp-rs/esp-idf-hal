use core::borrow::Borrow;
use core::ffi::c_void;
use core::marker::PhantomData;
use core::ptr;

use alloc::boxed::Box;
#[cfg(not(esp_idf_version_at_least_6_0_0))]
use alloc::vec;

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

use esp_idf_sys::*;

use crate::gpio::*;
use crate::interrupt;

pub use embedded_hal::i2c::Operation;

crate::embedded_hal_error!(
    I2cError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

pub type I2cBusConfig = config::BusConfig;
pub type I2cDeviceConfig = config::DeviceConfig;
#[cfg(not(esp32c2))]
pub type I2cSlaveConfig = config::SlaveConfig;

pub mod config {
    /// Configuration for the I2C bus (driver/i2c_master.h)
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct BusConfig {
        pub glitch_ignore_cnt: u8,
        pub enable_internal_pullup: bool,
    }

    impl BusConfig {
        pub const fn new() -> Self {
            Self {
                glitch_ignore_cnt: 7,
                enable_internal_pullup: true,
            }
        }

        #[must_use]
        pub fn glitch_ignore_cnt(mut self, cnt: u8) -> Self {
            self.glitch_ignore_cnt = cnt;
            self
        }

        #[must_use]
        pub fn enable_internal_pullup(mut self, enable: bool) -> Self {
            self.enable_internal_pullup = enable;
            self
        }
    }

    impl Default for BusConfig {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Configuration for an I2C device on the bus
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct DeviceConfig {
        pub scl_speed_hz: u32,
        pub scl_wait_us: u32,
        pub timeout_ms: i32,
    }

    impl DeviceConfig {
        pub const fn new() -> Self {
            Self {
                scl_speed_hz: 100_000,
                scl_wait_us: 0,
                timeout_ms: -1,
            }
        }

        #[must_use]
        pub fn scl_speed_hz(mut self, hz: u32) -> Self {
            self.scl_speed_hz = hz;
            self
        }

        #[must_use]
        pub fn scl_wait_us(mut self, us: u32) -> Self {
            self.scl_wait_us = us;
            self
        }

        /// Transfer timeout in milliseconds. Use -1 for blocking (default).
        #[must_use]
        pub fn timeout_ms(mut self, ms: i32) -> Self {
            self.timeout_ms = ms;
            self
        }
    }

    impl Default for DeviceConfig {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Configuration for the new I2C slave device (driver/i2c_slave.h)
    #[cfg(not(esp32c2))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct SlaveConfig {
        pub send_buf_depth: u32,
        #[cfg(esp_idf_version_at_least_6_0_0)]
        pub recv_buf_depth: u32,
        pub timeout_ms: i32,
    }

    #[cfg(not(esp32c2))]
    impl SlaveConfig {
        pub const fn new() -> Self {
            Self {
                send_buf_depth: 256,
                #[cfg(esp_idf_version_at_least_6_0_0)]
                recv_buf_depth: 256,
                timeout_ms: -1,
            }
        }

        #[must_use]
        pub fn send_buf_depth(mut self, depth: u32) -> Self {
            self.send_buf_depth = depth;
            self
        }

        /// Depth of the internal receive ring buffer (ESP-IDF v6.0+ only).
        #[cfg(esp_idf_version_at_least_6_0_0)]
        #[must_use]
        pub fn recv_buf_depth(mut self, depth: u32) -> Self {
            self.recv_buf_depth = depth;
            self
        }

        /// Transmit timeout in milliseconds. Use -1 for blocking (default).
        #[must_use]
        pub fn timeout_ms(mut self, ms: i32) -> Self {
            self.timeout_ms = ms;
            self
        }
    }

    #[cfg(not(esp32c2))]
    impl Default for SlaveConfig {
        fn default() -> Self {
            Self::new()
        }
    }
}

pub trait I2c: Send {
    fn port() -> i2c_port_t;
}

/// I2C bus using the new ESP-IDF driver (`driver/i2c_master.h`).
///
/// This replaces the legacy command-link based API with the bus/device model
/// introduced in ESP-IDF v5.0. Create a bus, then add devices to it via
/// [`I2cDriver::new`].
pub struct I2cBusDriver<'d> {
    handle: i2c_master_bus_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> I2cBusDriver<'d> {
    pub fn new<I2C: I2c + 'd>(
        _i2c: I2C,
        sda: impl InputPin + OutputPin + 'd,
        scl: impl InputPin + OutputPin + 'd,
        config: &config::BusConfig,
    ) -> Result<Self, EspError> {
        let mut bus_config = i2c_master_bus_config_t {
            i2c_port: I2C::port() as _,
            sda_io_num: sda.pin() as _,
            scl_io_num: scl.pin() as _,
            __bindgen_anon_1: i2c_master_bus_config_t__bindgen_ty_1 {
                clk_source: soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT,
            },
            glitch_ignore_cnt: config.glitch_ignore_cnt,
            intr_priority: 0,
            trans_queue_depth: 0,
            ..Default::default()
        };
        bus_config
            .flags
            .set_enable_internal_pullup(config.enable_internal_pullup as u32);

        let mut handle: i2c_master_bus_handle_t = core::ptr::null_mut();
        esp!(unsafe { i2c_new_master_bus(&bus_config, &mut handle) })?;

        Ok(Self {
            handle,
            _p: PhantomData,
        })
    }

    /// Probe for a device at the given 7-bit address.
    /// Returns `Ok(())` if a device acknowledges, or an error otherwise.
    pub fn probe(&self, address: u8, timeout_ms: i32) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_probe(self.handle, address as u16, timeout_ms) })
    }

    /// Reset the I2C bus (recover from stuck SDA/SCL).
    pub fn reset(&self) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_bus_reset(self.handle) })
    }

    pub fn handle(&self) -> i2c_master_bus_handle_t {
        self.handle
    }
}

impl Drop for I2cBusDriver<'_> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
    }
}

// SAFETY: The bus handle is not tied to a specific thread
unsafe impl Send for I2cBusDriver<'_> {}
// SAFETY: All data transfer operations (transmit, receive, probe) are serialized
// by an internal bus_lock_mux semaphore in the ESP-IDF driver
unsafe impl Sync for I2cBusDriver<'_> {}

/// I2C device on a bus, wrapping `i2c_master_dev_handle_t`.
///
/// Created from an [`I2cBusDriver`] with a device address and speed.
/// Implements [`embedded_hal::i2c::I2c`] so it works with ecosystem device
/// drivers (e.g. `ina228`, `ssd1306`).
pub struct I2cDriver<'d, T = I2cBusDriver<'d>>
where
    T: Borrow<I2cBusDriver<'d>> + 'd,
{
    handle: i2c_master_dev_handle_t,
    address: u8,
    timeout_ms: i32,
    _bus: T,
    _p: PhantomData<&'d ()>,
}

impl<'d> I2cDriver<'d, I2cBusDriver<'d>> {
    /// Create a device driver that owns the bus. Convenient when there is
    /// only a single device on the bus.
    pub fn new_single<I2C: I2c + 'd>(
        i2c: I2C,
        sda: impl InputPin + OutputPin + 'd,
        scl: impl InputPin + OutputPin + 'd,
        address: u8,
        bus_config: &config::BusConfig,
        dev_config: &config::DeviceConfig,
    ) -> Result<Self, EspError> {
        Self::new(
            I2cBusDriver::new(i2c, sda, scl, bus_config)?,
            address,
            dev_config,
        )
    }
}

impl<'d, T> I2cDriver<'d, T>
where
    T: Borrow<I2cBusDriver<'d>> + 'd,
{
    pub fn new(bus: T, address: u8, config: &config::DeviceConfig) -> Result<Self, EspError> {
        let dev_config = i2c_device_config_t {
            dev_addr_length: i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
            device_address: address as u16,
            scl_speed_hz: config.scl_speed_hz,
            scl_wait_us: config.scl_wait_us,
            ..Default::default()
        };

        let mut handle: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(unsafe {
            i2c_master_bus_add_device(bus.borrow().handle(), &dev_config, &mut handle)
        })?;

        Ok(Self {
            handle,
            address,
            timeout_ms: config.timeout_ms,
            _bus: bus,
            _p: PhantomData,
        })
    }

    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_receive(
                self.handle,
                buffer.as_mut_ptr(),
                buffer.len(),
                self.timeout_ms,
            )
        })
    }

    pub fn transmit(&mut self, bytes: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit(self.handle, bytes.as_ptr(), bytes.len(), self.timeout_ms)
        })
    }

    pub fn transmit_receive(&mut self, bytes: &[u8], buffer: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit_receive(
                self.handle,
                bytes.as_ptr(),
                bytes.len(),
                buffer.as_mut_ptr(),
                buffer.len(),
                self.timeout_ms,
            )
        })
    }

    pub fn transaction(&mut self, operations: &mut [Operation<'_>]) -> Result<(), EspError> {
        let mut iter = operations.iter_mut().peekable();

        while let Some(op) = iter.next() {
            match op {
                Operation::Write(bytes) => {
                    if let Some(Operation::Read(read_buf)) = iter.peek_mut() {
                        self.transmit_receive(bytes, read_buf)?;
                        iter.next();
                    } else {
                        self.transmit(bytes)?;
                    }
                }
                Operation::Read(buf) => {
                    self.receive(buf)?;
                }
            }
        }

        Ok(())
    }

    pub fn handle(&self) -> i2c_master_dev_handle_t {
        self.handle
    }

    fn check_address(&self, addr: u8) -> Result<(), I2cError> {
        if addr != self.address {
            Err(I2cError::other(EspError::from_infallible::<
                ESP_ERR_INVALID_ARG,
            >()))
        } else {
            Ok(())
        }
    }
}

impl<'d, T> Drop for I2cDriver<'d, T>
where
    T: Borrow<I2cBusDriver<'d>> + 'd,
{
    fn drop(&mut self) {
        esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
    }
}

unsafe impl<'d, T> Send for I2cDriver<'d, T> where T: Borrow<I2cBusDriver<'d>> + 'd {}

impl<'d, T> embedded_hal::i2c::ErrorType for I2cDriver<'d, T>
where
    T: Borrow<I2cBusDriver<'d>> + 'd,
{
    type Error = I2cError;
}

impl<'d, T> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDriver<'d, T>
where
    T: Borrow<I2cBusDriver<'d>> + 'd,
{
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.check_address(addr)?;
        I2cDriver::receive(self, buffer).map_err(to_i2c_err)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.check_address(addr)?;
        I2cDriver::transmit(self, bytes).map_err(to_i2c_err)
    }

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.check_address(addr)?;
        I2cDriver::transmit_receive(self, bytes, buffer).map_err(to_i2c_err)
    }

    fn transaction(
        &mut self,
        addr: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.check_address(addr)?;
        I2cDriver::transaction(self, operations).map_err(to_i2c_err)
    }
}

#[cfg(not(esp32c2))]
#[allow(clippy::type_complexity)]
struct I2cSlaveRecvUserData {
    on_recv: Box<dyn FnMut(&[u8]) + Send + 'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    recv_buf: alloc::vec::Vec<u8>,
}

/// I2C slave driver using the new ESP-IDF driver (`driver/i2c_slave.h`).
///
/// Wraps `i2c_slave_dev_handle_t`. The slave listens on a configured address
/// and communicates with an external master.
#[cfg(not(esp32c2))]
pub struct I2cSlaveDriver<'d> {
    handle: i2c_slave_dev_handle_t,
    timeout_ms: i32,
    on_recv: Option<Box<I2cSlaveRecvUserData>>,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(not(esp32c2))]
impl<'d> I2cSlaveDriver<'d> {
    pub fn new<I2C: I2c + 'd>(
        _i2c: I2C,
        sda: impl InputPin + OutputPin + 'd,
        scl: impl InputPin + OutputPin + 'd,
        slave_addr: u8,
        config: &config::SlaveConfig,
    ) -> Result<Self, EspError> {
        #[allow(unused_mut)]
        let mut slave_config = i2c_slave_config_t {
            i2c_port: I2C::port() as _,
            sda_io_num: sda.pin() as _,
            scl_io_num: scl.pin() as _,
            clk_source: soc_periph_i2c_clk_src_t_I2C_CLK_SRC_DEFAULT,
            send_buf_depth: config.send_buf_depth,
            slave_addr: slave_addr as u16,
            addr_bit_len: i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
            intr_priority: 0,
            ..Default::default()
        };
        #[cfg(esp_idf_version_at_least_6_0_0)]
        {
            slave_config.receive_buf_depth = config.recv_buf_depth;
        }

        let mut handle: i2c_slave_dev_handle_t = core::ptr::null_mut();
        esp!(unsafe { i2c_new_slave_device(&slave_config, &mut handle) })?;

        Ok(Self {
            handle,
            timeout_ms: config.timeout_ms,
            on_recv: None,
            _p: PhantomData,
        })
    }

    /// Register a continuous callback for data received from the master.
    ///
    /// The callback runs in ISR context and fires for every master write
    /// transaction. The `&[u8]` slice is borrowed from ESP-IDF's internal
    /// ring buffer — copy any data you need before returning.
    ///
    /// Only available on ESP-IDF v6.0+. On earlier versions, use
    /// [`receive`](Self::receive).
    #[cfg(esp_idf_version_at_least_6_0_0)]
    pub fn subscribe(
        &mut self,
        on_recv: impl FnMut(&[u8]) + Send + 'static,
    ) -> Result<(), EspError> {
        self.unsubscribe()?;

        let mut user_data = Box::new(I2cSlaveRecvUserData {
            on_recv: Box::new(on_recv),
        });

        let cbs = i2c_slave_event_callbacks_t {
            on_receive: Some(Self::handle_recv_isr),
            ..Default::default()
        };

        esp!(unsafe {
            i2c_slave_register_event_callbacks(
                self.handle,
                &cbs,
                (&raw mut *user_data) as *mut c_void,
            )
        })?;

        self.on_recv = Some(user_data);
        Ok(())
    }

    /// Unregister the previously registered receive callback.
    #[cfg(esp_idf_version_at_least_6_0_0)]
    pub fn unsubscribe(&mut self) -> Result<(), EspError> {
        if self.on_recv.is_some() {
            esp!(unsafe {
                i2c_slave_register_event_callbacks(self.handle, ptr::null(), ptr::null_mut())
            })?;
            self.on_recv = None;
        }
        Ok(())
    }

    /// Initiate a one-shot receive with a callback.
    ///
    /// Allocates a buffer of `buf_size` bytes and arms the slave to receive.
    /// When data arrives, `on_recv` fires in ISR context with a `&[u8]`.
    ///
    /// Returns `ESP_ERR_INVALID_STATE` if a receive is already pending.
    /// Call [`cancel_receive`](Self::cancel_receive) after the callback fires,
    /// then call `receive` again for the next transfer.
    ///
    /// Only available on ESP-IDF < 6.0. On v6.0+, use
    /// [`subscribe`](Self::subscribe).
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    pub fn receive(
        &mut self,
        buf_size: usize,
        on_recv: impl FnMut(&[u8]) + Send + 'static,
    ) -> Result<(), EspError> {
        if self.on_recv.is_some() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        let mut user_data = Box::new(I2cSlaveRecvUserData {
            on_recv: Box::new(on_recv),
            recv_buf: vec![0u8; buf_size],
        });

        let cbs = i2c_slave_event_callbacks_t {
            on_recv_done: Some(Self::handle_recv_isr),
            ..Default::default()
        };

        esp!(unsafe {
            i2c_slave_register_event_callbacks(
                self.handle,
                &cbs,
                (&raw mut *user_data) as *mut c_void,
            )
        })?;

        if let Err(e) = esp!(unsafe {
            i2c_slave_receive(
                self.handle,
                user_data.recv_buf.as_mut_ptr(),
                user_data.recv_buf.len(),
            )
        }) {
            let _ = esp!(unsafe {
                i2c_slave_register_event_callbacks(self.handle, ptr::null(), ptr::null_mut())
            });
            return Err(e);
        }

        self.on_recv = Some(user_data);
        Ok(())
    }

    /// Cancel the pending receive, allowing a new [`receive`](Self::receive)
    /// call. Deregisters the callback and frees the internal buffer.
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    pub fn cancel_receive(&mut self) -> Result<(), EspError> {
        if self.on_recv.is_some() {
            // Deregister callback first — this disables the RX interrupt,
            // ensuring ESP-IDF no longer references the buffer
            esp!(unsafe {
                i2c_slave_register_event_callbacks(self.handle, ptr::null(), ptr::null_mut())
            })?;
            self.on_recv = None;
        }
        Ok(())
    }

    /// Write data to the internal TX buffer. The hardware FIFO is filled
    /// from this buffer when the master clocks in a read.
    pub fn transmit(&mut self, bytes: &[u8]) -> Result<(), EspError> {
        #[cfg(not(esp_idf_version_at_least_6_0_0))]
        {
            esp!(unsafe {
                i2c_slave_transmit(
                    self.handle,
                    bytes.as_ptr(),
                    bytes.len() as _,
                    self.timeout_ms,
                )
            })
        }
        #[cfg(esp_idf_version_at_least_6_0_0)]
        {
            let mut write_len: u32 = 0;
            esp!(unsafe {
                i2c_slave_write(
                    self.handle,
                    bytes.as_ptr(),
                    bytes.len() as _,
                    &mut write_len,
                    self.timeout_ms,
                )
            })
        }
    }

    pub fn handle(&self) -> i2c_slave_dev_handle_t {
        self.handle
    }

    unsafe extern "C" fn handle_recv_isr(
        _i2c_slave: i2c_slave_dev_handle_t,
        evt_data: *const i2c_slave_rx_done_event_data_t,
        arg: *mut c_void,
    ) -> bool {
        let user_data = &mut *(arg as *mut I2cSlaveRecvUserData);
        let data = &*evt_data;

        #[cfg(not(esp_idf_version_at_least_6_0_0))]
        let slice = core::slice::from_raw_parts(data.buffer, user_data.recv_buf.len());
        #[cfg(esp_idf_version_at_least_6_0_0)]
        let slice = core::slice::from_raw_parts(data.buffer, data.length as usize);

        interrupt::with_isr_yield_signal(|| {
            (user_data.on_recv)(slice);
        })
    }
}

#[cfg(not(esp32c2))]
impl Drop for I2cSlaveDriver<'_> {
    fn drop(&mut self) {
        if self.on_recv.is_some() {
            let _ = esp!(unsafe {
                i2c_slave_register_event_callbacks(self.handle, ptr::null(), ptr::null_mut())
            });
            self.on_recv = None;
        }
        esp!(unsafe { i2c_del_slave_device(self.handle) }).unwrap();
    }
}

#[cfg(not(esp32c2))]
unsafe impl Send for I2cSlaveDriver<'_> {}

fn to_i2c_err(err: EspError) -> I2cError {
    if err.code() == ESP_FAIL {
        I2cError::new(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown), err)
    } else {
        I2cError::other(err)
    }
}

macro_rules! impl_i2c {
    ($i2c:ident: $port:expr) => {
        crate::impl_peripheral!($i2c);

        impl I2c for $i2c<'_> {
            #[inline(always)]
            fn port() -> i2c_port_t {
                $port
            }
        }
    };
}

impl_i2c!(I2C0: 0);
#[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
impl_i2c!(I2C1: 1);
