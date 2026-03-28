use core::marker::PhantomData;

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

use esp_idf_sys::*;

use crate::gpio::*;

pub use embedded_hal::i2c::Operation;

crate::embedded_hal_error!(
    I2cError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

pub type I2cBusConfig = config::BusConfig;
pub type I2cDeviceConfig = config::DeviceConfig;
#[cfg(not(esp32c2))]
pub type I2cSlaveDeviceConfig = config::SlaveDeviceConfig;

pub mod config {
    /// Configuration for the I2C bus (driver/i2c_master.h)
    #[derive(Debug, Clone)]
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
    #[derive(Debug, Clone)]
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
    #[derive(Debug, Clone)]
    pub struct SlaveDeviceConfig {
        pub send_buf_depth: u32,
        pub timeout_ms: i32,
    }

    #[cfg(not(esp32c2))]
    impl SlaveDeviceConfig {
        pub const fn new() -> Self {
            Self {
                send_buf_depth: 256,
                timeout_ms: -1,
            }
        }

        #[must_use]
        pub fn send_buf_depth(mut self, depth: u32) -> Self {
            self.send_buf_depth = depth;
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
    impl Default for SlaveDeviceConfig {
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
/// [`I2cDevice::new`].
pub struct I2cBus<'d> {
    handle: i2c_master_bus_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> I2cBus<'d> {
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

impl Drop for I2cBus<'_> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
    }
}

// SAFETY: The bus handle is not tied to a specific thread
unsafe impl Send for I2cBus<'_> {}
// SAFETY: All data transfer operations (transmit, receive, probe) are serialized
// by an internal bus_lock_mux semaphore in the ESP-IDF driver
unsafe impl Sync for I2cBus<'_> {}

/// I2C device on a bus, wrapping `i2c_master_dev_handle_t`.
///
/// Created from an [`I2cBus`] with a device address and speed.
/// Implements [`embedded_hal::i2c::I2c`] so it works with ecosystem device
/// drivers (e.g. `ina228`, `ssd1306`).
///
pub struct I2cDevice<'d> {
    handle: i2c_master_dev_handle_t,
    address: u8,
    timeout_ms: i32,
    _p: PhantomData<&'d ()>,
}

impl<'d> I2cDevice<'d> {
    pub fn new(
        bus: &'d I2cBus<'_>,
        address: u8,
        config: &config::DeviceConfig,
    ) -> Result<Self, EspError> {
        let dev_config = i2c_device_config_t {
            dev_addr_length: i2c_addr_bit_len_t_I2C_ADDR_BIT_LEN_7,
            device_address: address as u16,
            scl_speed_hz: config.scl_speed_hz,
            scl_wait_us: config.scl_wait_us,
            ..Default::default()
        };

        let mut handle: i2c_master_dev_handle_t = core::ptr::null_mut();
        esp!(unsafe { i2c_master_bus_add_device(bus.handle(), &dev_config, &mut handle) })?;

        Ok(Self {
            handle,
            address,
            timeout_ms: config.timeout_ms,
            _p: PhantomData,
        })
    }

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_receive(
                self.handle,
                buffer.as_mut_ptr(),
                buffer.len(),
                self.timeout_ms,
            )
        })
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_transmit(self.handle, bytes.as_ptr(), bytes.len(), self.timeout_ms)
        })
    }

    pub fn write_read(&mut self, bytes: &[u8], buffer: &mut [u8]) -> Result<(), EspError> {
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
                    // If the next operation is a Read, use transmit_receive for repeated start
                    if let Some(Operation::Read(read_buf)) = iter.peek_mut() {
                        self.write_read(bytes, read_buf)?;
                        iter.next();
                    } else {
                        self.write(bytes)?;
                    }
                }
                Operation::Read(buf) => {
                    self.read(buf)?;
                }
            }
        }

        Ok(())
    }

    pub fn handle(&self) -> i2c_master_dev_handle_t {
        self.handle
    }
}

impl Drop for I2cDevice<'_> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
    }
}

unsafe impl Send for I2cDevice<'_> {}

impl embedded_hal::i2c::ErrorType for I2cDevice<'_> {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDevice<'_> {
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        assert_eq!(addr, self.address, "I2C address mismatch: trait called with {addr:#04x} but device is configured for {:#04x}", self.address);
        I2cDevice::read(self, buffer).map_err(to_i2c_err)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        assert_eq!(addr, self.address, "I2C address mismatch: trait called with {addr:#04x} but device is configured for {:#04x}", self.address);
        I2cDevice::write(self, bytes).map_err(to_i2c_err)
    }

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        assert_eq!(addr, self.address, "I2C address mismatch: trait called with {addr:#04x} but device is configured for {:#04x}", self.address);
        I2cDevice::write_read(self, bytes, buffer).map_err(to_i2c_err)
    }

    fn transaction(
        &mut self,
        addr: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        assert_eq!(addr, self.address, "I2C address mismatch: trait called with {addr:#04x} but device is configured for {:#04x}", self.address);
        I2cDevice::transaction(self, operations).map_err(to_i2c_err)
    }
}

/// I2C slave device using the new ESP-IDF driver (`driver/i2c_slave.h`).
///
/// Wraps `i2c_slave_dev_handle_t`. The slave listens on a configured address
/// and communicates with an external master.
///
/// Note: [`I2cSlaveDevice::receive`] is non-blocking — it initiates a receive
/// job. Use the `on_recv_done` callback (via [`i2c_slave_register_event_callbacks`])
/// to be notified when data arrives.
#[cfg(not(esp32c2))]
pub struct I2cSlaveDevice<'d> {
    handle: i2c_slave_dev_handle_t,
    timeout_ms: i32,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(not(esp32c2))]
impl<'d> I2cSlaveDevice<'d> {
    pub fn new<I2C: I2c + 'd>(
        _i2c: I2C,
        sda: impl InputPin + OutputPin + 'd,
        scl: impl InputPin + OutputPin + 'd,
        slave_addr: u8,
        config: &config::SlaveDeviceConfig,
    ) -> Result<Self, EspError> {
        let slave_config = i2c_slave_config_t {
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

        let mut handle: i2c_slave_dev_handle_t = core::ptr::null_mut();
        esp!(unsafe { i2c_new_slave_device(&slave_config, &mut handle) })?;

        Ok(Self {
            handle,
            timeout_ms: config.timeout_ms,
            _p: PhantomData,
        })
    }

    /// Initiate a non-blocking receive job.
    ///
    /// The buffer must remain valid until the `on_recv_done` callback fires.
    /// Register callbacks via the raw `i2c_slave_register_event_callbacks` FFI.
    pub fn receive(&mut self, buffer: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe { i2c_slave_receive(self.handle, buffer.as_mut_ptr(), buffer.len()) })
    }

    /// Write data to the internal TX ringbuffer. The hardware FIFO is filled
    /// from this buffer when the master clocks in a read.
    pub fn transmit(&mut self, bytes: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_slave_transmit(
                self.handle,
                bytes.as_ptr(),
                bytes.len() as _,
                self.timeout_ms,
            )
        })
    }

    pub fn handle(&self) -> i2c_slave_dev_handle_t {
        self.handle
    }
}

#[cfg(not(esp32c2))]
impl Drop for I2cSlaveDevice<'_> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_del_slave_device(self.handle) }).unwrap();
    }
}

#[cfg(not(esp32c2))]
unsafe impl Send for I2cSlaveDevice<'_> {}

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
