use core::marker::PhantomData;
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
use esp_idf_sys::*;

use crate::{delay::*, gpio::*, units::*};

crate::embedded_hal_error!(
    I2cError,
    embedded_hal::i2c::Error,
    embedded_hal::i2c::ErrorKind
);

pub struct MasterPins<SDA: OutputPin + InputPin, SCL: OutputPin> {
    pub sda: SDA,
    pub scl: SCL,
}

pub struct SlavePins<SDA: OutputPin + InputPin, SCL: InputPin> {
    pub sda: SDA,
    pub scl: SCL,
}

/// I2C configuration
pub mod config {
    use crate::units::*;
    use core::time::Duration;

    /// I2C Master configuration
    #[derive(Copy, Clone)]
    pub struct MasterConfig {
        pub baudrate: Hertz,
        pub timeout: Option<Duration>,
        pub sda_pullup_enabled: bool,
        pub scl_pullup_enabled: bool,
    }

    impl MasterConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }

        #[must_use]
        pub fn timeout(mut self, timeout: Option<Duration>) -> Self {
            self.timeout = timeout;
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
    }

    impl Default for MasterConfig {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                timeout: None,
                sda_pullup_enabled: true,
                scl_pullup_enabled: true,
            }
        }
    }

    /// I2C Slave configuration
    #[derive(Copy, Clone)]
    pub struct SlaveConfig {
        pub timeout: Option<Duration>,
        pub sda_pullup_enabled: bool,
        pub scl_pullup_enabled: bool,
        pub rx_buf_len: usize,
        pub tx_buf_len: usize,
    }

    impl SlaveConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn timeout(mut self, timeout: Option<Duration>) -> Self {
            self.timeout = timeout;
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
        pub fn rx_buffer_length(mut self, len: usize) -> Self {
            self.rx_buf_len = len;
            self
        }

        #[must_use]
        pub fn tx_buffer_length(mut self, len: usize) -> Self {
            self.tx_buf_len = len;
            self
        }
    }

    impl Default for SlaveConfig {
        fn default() -> Self {
            Self {
                timeout: None,
                sda_pullup_enabled: true,
                scl_pullup_enabled: true,
                rx_buf_len: 0,
                tx_buf_len: 0,
            }
        }
    }
}

pub trait I2c: Send {
    fn port() -> i2c_port_t;
}

unsafe impl<I2C: I2c, SDA: OutputPin + InputPin, SCL: OutputPin> Send for Master<I2C, SDA, SCL> {}

pub struct Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    i2c: I2C,
    pins: MasterPins<SDA, SCL>,
    timeout: TickType_t,
}

pub struct Slave<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: InputPin,
{
    i2c: I2C,
    pins: SlavePins<SDA, SCL>,
    timeout: TickType_t,
}

unsafe impl<I2C: I2c, SDA: OutputPin + InputPin, SCL: InputPin> Send for Slave<I2C, SDA, SCL> {}

impl<I2C, SDA, SCL> Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    pub fn new(
        i2c: I2C,
        pins: MasterPins<SDA, SCL>,
        config: config::MasterConfig,
    ) -> Result<Master<I2C, SDA, SCL>, EspError> {
        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        if config.baudrate > 1.MHz().into() {
            return Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap());
        }

        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_MASTER,
            sda_io_num: pins.sda.pin(),
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: pins.scl.pin(),
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
                0,
            ) // TODO: set flags
        })?;

        Ok(Master {
            i2c,
            pins,
            timeout: TickType::from(config.timeout).0,
        })
    }

    pub fn release(self) -> Result<(I2C, MasterPins<SDA, SCL>), EspError> {
        esp!(unsafe { i2c_driver_delete(I2C::port()) })?;

        //self.pins.sda.reset()?;
        //self.pins.scl.reset()?;

        Ok((self.i2c, self.pins))
    }

    fn cmd_begin(
        &mut self,
        command_link: &CommandLink,
        timeout: TickType_t,
    ) -> Result<(), EspError> {
        esp!(unsafe { i2c_master_cmd_begin(I2C::port(), command_link.0, timeout) })
    }

    fn submit(&mut self, command_link: &CommandLink) -> Result<(), I2cError> {
        if let Err(err) = self.cmd_begin(command_link, self.timeout) {
            let err = if err.code() == ESP_FAIL {
                I2cError::new(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown), err)
            } else {
                I2cError::other(err)
            };
            Err(err)
        } else {
            Ok(())
        }
    }
}

impl<I2C, SDA, SCL> embedded_hal_0_2::blocking::i2c::Read for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = I2cError;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        embedded_hal::i2c::blocking::I2c::read(self, addr, buffer)
    }
}

impl<I2C, SDA, SCL> embedded_hal_0_2::blocking::i2c::Write for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = I2cError;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_hal::i2c::blocking::I2c::write(self, addr, bytes)
    }
}

impl<I2C, SDA, SCL> embedded_hal_0_2::blocking::i2c::WriteRead for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = I2cError;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        embedded_hal::i2c::blocking::I2c::write_read(self, addr, bytes, buffer)
    }
}

impl<I2C, SDA, SCL> embedded_hal::i2c::ErrorType for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = I2cError;
}

impl<I2C, SDA, SCL> embedded_hal::i2c::blocking::I2c<embedded_hal::i2c::SevenBitAddress>
    for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut command_link = CommandLink::new().map_err(I2cError::other)?;

        command_link.master_start().map_err(I2cError::other)?;
        command_link
            .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)
            .map_err(I2cError::other)?;
        if !buffer.is_empty() {
            command_link
                .master_read(buffer, AckType::LastNack)
                .map_err(I2cError::other)?;
        }
        command_link.master_stop().map_err(I2cError::other)?;

        self.submit(&command_link)
    }

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut command_link = CommandLink::new().map_err(I2cError::other)?;

        command_link.master_start().map_err(I2cError::other)?;

        command_link
            .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)
            .map_err(I2cError::other)?;

        if !bytes.is_empty() {
            command_link
                .master_write(bytes, true)
                .map_err(I2cError::other)?;
        }
        command_link.master_stop().map_err(I2cError::other)?;

        self.submit(&command_link)
    }

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut command_link = CommandLink::new().map_err(I2cError::other)?;

        command_link.master_start().map_err(I2cError::other)?;
        command_link
            .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)
            .map_err(I2cError::other)?;
        if !bytes.is_empty() {
            command_link
                .master_write(bytes, true)
                .map_err(I2cError::other)?;
        }

        command_link.master_start().map_err(I2cError::other)?;
        command_link
            .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)
            .map_err(I2cError::other)?;
        if !buffer.is_empty() {
            command_link
                .master_read(buffer, AckType::LastNack)
                .map_err(I2cError::other)?;
        }

        command_link.master_stop().map_err(I2cError::other)?;

        self.submit(&command_link)
    }

    fn write_iter<B>(&mut self, _address: u8, _bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }

    fn write_iter_read<B>(
        &mut self,
        _address: u8,
        _bytes: B,
        _buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        todo!()
    }

    fn transaction<'a>(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::blocking::Operation<'a>],
    ) -> Result<(), Self::Error> {
        use embedded_hal::i2c::blocking::Operation;

        let mut command_link = CommandLink::new().map_err(I2cError::other)?;

        command_link.master_start().map_err(I2cError::other)?;

        let last_op_index = operations.len() - 1;
        let mut prev_was_read = None;

        for (i, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buf) => {
                    if let Some(false) = prev_was_read {
                        command_link.master_start().map_err(I2cError::other)?;
                    }
                    prev_was_read = Some(true);

                    command_link
                        .master_write_byte((address << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)
                        .map_err(I2cError::other)?;
                    if !buf.is_empty() {
                        let ack = if i == last_op_index {
                            AckType::LastNack
                        } else {
                            AckType::Ack
                        };
                        command_link
                            .master_read(*buf, ack)
                            .map_err(I2cError::other)?;
                    }
                }
                Operation::Write(buf) => {
                    if let Some(true) = prev_was_read {
                        command_link.master_start().map_err(I2cError::other)?;
                    }
                    prev_was_read = Some(false);

                    command_link
                        .master_write_byte((address << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)
                        .map_err(I2cError::other)?;
                    if !buf.is_empty() {
                        command_link
                            .master_write(buf, true)
                            .map_err(I2cError::other)?;
                    }
                }
            }
        }

        command_link.master_stop().map_err(I2cError::other)?;

        self.submit(&command_link)
    }

    fn transaction_iter<'a, O>(&mut self, _address: u8, _operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = embedded_hal::i2c::blocking::Operation<'a>>,
    {
        todo!()
    }
}

impl<I2C, SDA, SCL> Slave<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: InputPin,
{
    pub fn new(
        i2c: I2C,
        pins: SlavePins<SDA, SCL>,
        slave_addr: u8,
        config: config::SlaveConfig,
    ) -> Result<Self, EspError> {
        #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
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
                    maximum_speed: 0,
                },
            },
            ..Default::default()
        };

        #[cfg(not(any(esp_idf_version = "4.4", esp_idf_version_major = "5")))]
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
                config.rx_buf_len as u32,
                config.tx_buf_len as u32,
                0, // TODO: set flags
            )
        })?;

        Ok(Self {
            i2c,
            pins,
            timeout: TickType::from(config.timeout).0,
        })
    }

    pub fn release(self) -> Result<(I2C, SlavePins<SDA, SCL>), EspError> {
        esp!(unsafe { i2c_driver_delete(I2C::port()) })?;

        //self.pins.sda.reset()?;
        //self.pins.scl.reset()?;

        Ok((self.i2c, self.pins))
    }

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, EspError> {
        let n = unsafe {
            i2c_slave_read_buffer(
                I2C::port(),
                buffer.as_mut_ptr(),
                buffer.len() as u32,
                self.timeout,
            )
        };

        if n > 0 {
            Ok(n as usize)
        } else {
            Err(EspError::from(ESP_ERR_TIMEOUT as i32).unwrap())
        }
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        let n = unsafe {
            i2c_slave_write_buffer(
                I2C::port(),
                bytes.as_ptr() as *const u8 as *mut u8,
                bytes.len() as i32,
                self.timeout,
            )
        };

        if n > 0 {
            Ok(n as usize)
        } else {
            Err(EspError::from(ESP_ERR_TIMEOUT as i32).unwrap())
        }
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
            return Err(EspError::from(ESP_ERR_NO_MEM as i32).unwrap());
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
        esp!(unsafe {
            i2c_master_write(
                self.0,
                buf.as_ptr() as *const u8 as *mut u8,
                buf.len() as u32,
                ack_en,
            )
        })
    }

    fn master_read(&mut self, buf: &'buffers mut [u8], ack: AckType) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_read(
                self.0,
                buf.as_ptr() as *const u8 as *mut u8,
                buf.len() as u32,
                ack as u32,
            )
        })
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
        pub struct $i2c(::core::marker::PhantomData<*const ()>);

        impl $i2c {
            /// # Safety
            ///
            /// Care should be taken not to instnatiate this I2C instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $i2c(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $i2c {}

        impl I2c for $i2c {
            #[inline(always)]
            fn port() -> i2c_port_t {
                $port
            }
        }
    };
}

impl_i2c!(I2C0: 0);

#[cfg(not(esp32c3))]
impl_i2c!(I2C1: 1);
