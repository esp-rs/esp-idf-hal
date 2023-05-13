use core::marker::PhantomData;
use core::time::Duration;

use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

use esp_idf_sys::*;

use crate::delay::*;
use crate::gpio::*;
use crate::interrupt::IntrFlags;
use crate::peripheral::Peripheral;
use crate::units::*;

pub use embedded_hal::i2c::Operation;

crate::embedded_hal_error!(
    I2cError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

const APB_TICK_PERIOD_NS: u32 = 1_000_000_000 / 80_000_000;
#[derive(Copy, Clone, Debug)]
pub struct APBTickType(::core::ffi::c_int);
impl From<Duration> for APBTickType {
    fn from(duration: Duration) -> Self {
        APBTickType(
            ((duration.as_nanos() + APB_TICK_PERIOD_NS as u128 - 1) / APB_TICK_PERIOD_NS as u128)
                as ::core::ffi::c_int,
        )
    }
}

pub type I2cConfig = config::Config;
#[cfg(not(esp32c2))]
pub type I2cSlaveConfig = config::SlaveConfig;

/// I2C configuration
pub mod config {
    use enumset::EnumSet;

    use super::APBTickType;
    use crate::{interrupt::IntrFlags, units::*};

    /// I2C Master configuration
    #[derive(Debug, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub sda_pullup_enabled: bool,
        pub scl_pullup_enabled: bool,
        pub timeout: Option<APBTickType>,
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
        pub fn sda_enable_pullup(mut self, enable: bool) -> Self {
            self.sda_pullup_enabled = enable;
            self
        }

        #[must_use]
        pub fn scl_enable_pullup(mut self, enable: bool) -> Self {
            self.scl_pullup_enabled = enable;
            self
        }

        #[must_use]
        pub fn timeout(mut self, timeout: APBTickType) -> Self {
            self.timeout = Some(timeout);
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = flags;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                sda_pullup_enabled: true,
                scl_pullup_enabled: true,
                timeout: None,
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }
    }

    /// I2C Slave configuration
    #[cfg(not(esp32c2))]
    #[derive(Debug, Clone)]
    pub struct SlaveConfig {
        pub sda_pullup_enabled: bool,
        pub scl_pullup_enabled: bool,
        pub rx_buf_len: usize,
        pub tx_buf_len: usize,
        pub intr_flags: EnumSet<IntrFlags>,
    }

    #[cfg(not(esp32c2))]
    impl SlaveConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn sda_enable_pullup(mut self, enable: bool) -> Self {
            self.sda_pullup_enabled = enable;
            self
        }

        #[must_use]
        pub fn scl_enable_pullup(mut self, enable: bool) -> Self {
            self.scl_pullup_enabled = enable;
            self
        }

        #[must_use]
        pub fn rx_buffer_length(mut self, len: usize) -> Self {
            self.rx_buf_len = len;
            self
        }

        #[must_use]
        pub fn tx_buffer_length(mut self, len: usize) -> Self {
            self.tx_buf_len = len;
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = flags;
            self
        }
    }

    #[cfg(not(esp32c2))]
    impl Default for SlaveConfig {
        fn default() -> Self {
            Self {
                sda_pullup_enabled: true,
                scl_pullup_enabled: true,
                rx_buf_len: 0,
                tx_buf_len: 0,
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }
    }
}

pub trait I2c: Send {
    fn port() -> i2c_port_t;
}

pub struct I2cDriver<'d> {
    i2c: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> I2cDriver<'d> {
    pub fn new<I2C: I2c>(
        _i2c: impl Peripheral<P = I2C> + 'd,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        if config.baudrate > 1.MHz().into() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
        }

        crate::into_ref!(sda, scl);

        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_MASTER,
            sda_io_num: sda.pin(),
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: scl.pin(),
            scl_pullup_en: config.scl_pullup_enabled,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 {
                    clk_speed: config.baudrate.into(),
                },
            },
            ..Default::default()
        };

        esp!(unsafe { i2c_param_config(I2C::port(), &sys_config) })?;

        esp!(unsafe {
            i2c_driver_install(
                I2C::port(),
                i2c_mode_t_I2C_MODE_MASTER,
                0, // Not used in master mode
                0, // Not used in master mode
                IntrFlags::to_native(config.intr_flags) as _,
            )
        })?;

        if let Some(timeout) = config.timeout {
            esp!(unsafe { i2c_set_timeout(I2C::port(), timeout.0) })?;
        }

        Ok(I2cDriver {
            i2c: I2C::port() as _,
            _p: PhantomData,
        })
    }

    pub fn read(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        let mut command_link = CommandLink::new()?;

        command_link.master_start()?;
        command_link.master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)?;

        if !buffer.is_empty() {
            command_link.master_read(buffer, AckType::LastNack)?;
        }

        command_link.master_stop()?;

        self.cmd_begin(&command_link, timeout)
    }

    pub fn write(&mut self, addr: u8, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
        let mut command_link = CommandLink::new()?;

        command_link.master_start()?;
        command_link.master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;

        if !bytes.is_empty() {
            command_link.master_write(bytes, true)?;
        }

        command_link.master_stop()?;

        self.cmd_begin(&command_link, timeout)
    }

    pub fn write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        let mut command_link = CommandLink::new()?;

        command_link.master_start()?;
        command_link.master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;

        if !bytes.is_empty() {
            command_link.master_write(bytes, true)?;
        }

        command_link.master_start()?;
        command_link.master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)?;

        if !buffer.is_empty() {
            command_link.master_read(buffer, AckType::LastNack)?;
        }

        command_link.master_stop()?;

        self.cmd_begin(&command_link, timeout)
    }

    pub fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        let mut command_link = CommandLink::new()?;

        let last_op_index = operations.len() - 1;
        let mut prev_was_read = None;

        for (i, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buf) => {
                    if Some(true) != prev_was_read {
                        command_link.master_start()?;
                        command_link.master_write_byte(
                            (address << 1) | (i2c_rw_t_I2C_MASTER_READ as u8),
                            true,
                        )?;
                    }
                    prev_was_read = Some(true);

                    if !buf.is_empty() {
                        let ack = if i == last_op_index {
                            AckType::LastNack
                        } else {
                            AckType::Ack
                        };

                        command_link.master_read(buf, ack)?;
                    }
                }
                Operation::Write(buf) => {
                    if Some(false) != prev_was_read {
                        command_link.master_start()?;
                        command_link.master_write_byte(
                            (address << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8),
                            true,
                        )?;
                    }
                    prev_was_read = Some(false);

                    if !buf.is_empty() {
                        command_link.master_write(buf, true)?;
                    }
                }
            }
        }

        command_link.master_stop()?;

        self.cmd_begin(&command_link, timeout)
    }

    fn cmd_begin(
        &mut self,
        command_link: &CommandLink,
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_cmd_begin(self.port(), command_link.0, timeout) })
    }

    pub fn port(&self) -> i2c_port_t {
        self.i2c as _
    }
}

impl<'d> Drop for I2cDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_driver_delete(self.port()) }).unwrap();
    }
}

unsafe impl<'d> Send for I2cDriver<'d> {}

impl<'d> embedded_hal_0_2::blocking::i2c::Read for I2cDriver<'d> {
    type Error = I2cError;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        I2cDriver::read(self, addr, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d> embedded_hal_0_2::blocking::i2c::Write for I2cDriver<'d> {
    type Error = I2cError;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        I2cDriver::write(self, addr, bytes, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d> embedded_hal_0_2::blocking::i2c::WriteRead for I2cDriver<'d> {
    type Error = I2cError;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        I2cDriver::write_read(self, addr, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }
}

impl<'d> embedded_hal::i2c::ErrorType for I2cDriver<'d> {
    type Error = I2cError;
}

impl<'d> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDriver<'d> {
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        I2cDriver::read(self, addr, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        I2cDriver::write(self, addr, bytes, BLOCK).map_err(to_i2c_err)
    }

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        I2cDriver::write_read(self, addr, bytes, buffer, BLOCK).map_err(to_i2c_err)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        I2cDriver::transaction(self, address, operations, BLOCK).map_err(to_i2c_err)
    }
}

fn to_i2c_err(err: EspError) -> I2cError {
    if err.code() == ESP_FAIL {
        I2cError::new(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown), err)
    } else {
        I2cError::other(err)
    }
}

#[cfg(not(esp32c2))]
pub struct I2cSlaveDriver<'d> {
    i2c: u8,
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
        slave_addr: u8,
        config: &config::SlaveConfig,
    ) -> Result<Self, EspError> {
        crate::into_ref!(sda, scl);

        #[cfg(not(esp_idf_version = "4.3"))]
        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_SLAVE,
            sda_io_num: sda.pin(),
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: scl.pin(),
            scl_pullup_en: config.scl_pullup_enabled,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                slave: i2c_config_t__bindgen_ty_1__bindgen_ty_2 {
                    slave_addr: slave_addr as u16,
                    addr_10bit_en: 0, // For now; to become configurable with embedded-hal V1.0
                    maximum_speed: 0,
                },
            },
            ..Default::default()
        };

        #[cfg(esp_idf_version = "4.3")]
        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_SLAVE,
            sda_io_num: pins.sda.pin(),
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: pins.scl.pin(),
            scl_pullup_en: config.scl_pullup_enabled,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                slave: i2c_config_t__bindgen_ty_1__bindgen_ty_2 {
                    slave_addr: slave_addr as u16,
                    addr_10bit_en: 0, // For now; to become configurable with embedded-hal V1.0
                },
            },
            ..Default::default()
        };

        esp!(unsafe { i2c_param_config(I2C::port(), &sys_config) })?;

        esp!(unsafe {
            i2c_driver_install(
                I2C::port(),
                i2c_mode_t_I2C_MODE_SLAVE,
                config.rx_buf_len,
                config.tx_buf_len,
                IntrFlags::to_native(config.intr_flags) as _,
            )
        })?;

        Ok(Self {
            i2c: I2C::port() as _,
            _p: PhantomData,
        })
    }

    pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<usize, EspError> {
        let n = unsafe {
            i2c_slave_read_buffer(self.port(), buffer.as_mut_ptr(), buffer.len(), timeout)
        };

        if n > 0 {
            Ok(n as usize)
        } else {
            Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>())
        }
    }

    pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<usize, EspError> {
        let n = unsafe {
            i2c_slave_write_buffer(self.port(), bytes.as_ptr(), bytes.len() as i32, timeout)
        };

        if n > 0 {
            Ok(n as usize)
        } else {
            Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>())
        }
    }

    pub fn port(&self) -> i2c_port_t {
        self.i2c as _
    }
}

#[cfg(not(esp32c2))]
impl<'d> Drop for I2cSlaveDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { i2c_driver_delete(self.port()) }).unwrap();
    }
}

#[repr(u32)]
enum AckType {
    Ack = i2c_ack_type_t_I2C_MASTER_ACK,
    #[allow(dead_code)]
    Nack = i2c_ack_type_t_I2C_MASTER_NACK,
    LastNack = i2c_ack_type_t_I2C_MASTER_LAST_NACK,
}

struct CommandLink<'buffers>(i2c_cmd_handle_t, PhantomData<&'buffers u8>);

impl<'buffers> CommandLink<'buffers> {
    fn new() -> Result<Self, EspError> {
        let handle = unsafe { i2c_cmd_link_create() };

        if handle.is_null() {
            return Err(EspError::from_infallible::<ESP_ERR_NO_MEM>());
        }

        Ok(CommandLink(handle, PhantomData))
    }

    fn master_start(&mut self) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_start(self.0) })
    }

    fn master_stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_stop(self.0) })
    }

    fn master_write_byte(&mut self, data: u8, ack_en: bool) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_write_byte(self.0, data, ack_en) })
    }

    fn master_write(&mut self, buf: &'buffers [u8], ack_en: bool) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_write(self.0, buf.as_ptr(), buf.len(), ack_en,) })
    }

    fn master_read(&mut self, buf: &'buffers mut [u8], ack: AckType) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_read(self.0, buf.as_mut_ptr().cast(), buf.len(), ack as u32,) })
    }
}

impl<'buffers> Drop for CommandLink<'buffers> {
    fn drop(&mut self) {
        unsafe {
            i2c_cmd_link_delete(self.0);
        }
    }
}

macro_rules! impl_i2c {
    ($i2c:ident: $port:expr) => {
        crate::impl_peripheral!($i2c);

        impl I2c for $i2c {
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
