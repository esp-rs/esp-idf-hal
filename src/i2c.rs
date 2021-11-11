use crate::{delay::*, gpio::*, units::*};

use esp_idf_sys::*;

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
        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn timeout(mut self, timeout: Option<Duration>) -> Self {
            self.timeout = timeout;
            self
        }

        pub fn sda_enable_pullup(mut self, enable: bool) -> Self {
            self.sda_pullup_enabled = enable;
            self
        }

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
        pub fn timeout(mut self, timeout: Option<Duration>) -> Self {
            self.timeout = timeout;
            self
        }

        pub fn sda_enable_pullup(mut self, enable: bool) -> Self {
            self.sda_pullup_enabled = enable;
            self
        }

        pub fn scl_enable_pullup(mut self, enable: bool) -> Self {
            self.scl_pullup_enabled = enable;
            self
        }

        pub fn rx_buffer_length(mut self, len: usize) -> Self {
            self.rx_buf_len = len;
            self
        }

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

pub trait I2c {
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
            sda_io_num: SDA::pin() as i32,
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: SCL::pin() as i32,
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
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::Read for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = EspError;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let command_link = CommandLink::new()?;

        unsafe {
            esp!(i2c_master_start(command_link.0))?;
            esp!(i2c_master_write_byte(
                command_link.0,
                (addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8),
                true
            ))?;
            esp!(i2c_master_read(
                command_link.0,
                buffer.as_ptr() as *const u8 as *mut u8,
                buffer.len() as u32,
                i2c_ack_type_t_I2C_MASTER_LAST_NACK
            ))?;
            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(
                i2c_master_cmd_begin(I2C::port(), command_link.0, self.timeout),
                ()
            )
        }
    }
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::Write for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = EspError;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        unsafe {
            let command_link = CommandLink::new()?;

            esp!(i2c_master_start(command_link.0))?;
            esp!(i2c_master_write_byte(
                command_link.0,
                (addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8),
                true
            ))?;
            esp!(i2c_master_write(
                command_link.0,
                bytes.as_ptr() as *const u8 as *mut u8,
                bytes.len() as u32,
                true
            ))?;
            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(
                i2c_master_cmd_begin(I2C::port(), command_link.0, self.timeout),
                ()
            )
        }
    }
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::WriteRead for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin,
{
    type Error = EspError;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        let command_link = CommandLink::new()?;

        unsafe {
            esp!(i2c_master_start(command_link.0))?;
            esp!(i2c_master_write_byte(
                command_link.0,
                (addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8),
                true
            ))?;
            esp!(i2c_master_write(
                command_link.0,
                bytes.as_ptr() as *const u8 as *mut u8,
                bytes.len() as u32,
                true
            ))?;

            esp!(i2c_master_start(command_link.0))?;
            esp!(i2c_master_write_byte(
                command_link.0,
                (addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8),
                true
            ))?;
            esp!(i2c_master_read(
                command_link.0,
                buffer.as_ptr() as *const u8 as *mut u8,
                buffer.len() as u32,
                i2c_ack_type_t_I2C_MASTER_LAST_NACK
            ))?;

            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(
                i2c_master_cmd_begin(I2C::port(), command_link.0, self.timeout),
                ()
            )
        }
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
        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_SLAVE,
            sda_io_num: SDA::pin() as i32,
            sda_pullup_en: config.sda_pullup_enabled,
            scl_io_num: SCL::pin() as i32,
            scl_pullup_en: config.scl_pullup_enabled,
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                slave: i2c_config_t__bindgen_ty_1__bindgen_ty_2 {
                    slave_addr: slave_addr as u16,
                    addr_10bit_en: 0, // For now; to become configurable with embedded-hal V1.0
                    ..Default::default()
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

struct CommandLink(i2c_cmd_handle_t);

impl CommandLink {
    fn new() -> Result<Self, EspError> {
        let handle = unsafe { i2c_cmd_link_create() };

        if handle.is_null() {
            return Err(EspError::from(ESP_ERR_NO_MEM as i32).unwrap());
        }

        Ok(CommandLink(handle))
    }
}

impl Drop for CommandLink {
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
