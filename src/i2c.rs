use crate::{delay::*, gpio::*};

use esp_idf_sys::*;

pub struct Pins<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin> {
    pub sda: SDA,
    pub scl: SCL,
}

pub trait I2c {
    fn port() -> i2c_port_t;
}

pub struct Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    i2c: I2C,
    pins: Pins<SDA, SCL>,
}

pub struct Slave<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    i2c: I2C,
    pins: Pins<SDA, SCL>,
    timeout: Option<u32>,
}

impl<I2C, SDA, SCL> Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    pub fn new(
            i2c: I2C,
            pins: Pins<SDA, SCL>,
            clk_speed: u32) -> Result<Master<I2C, SDA, SCL>, EspError> {
        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        // if clk_speed > 1_000_000 {
        //     return Err(Error::InvalidArg);
        // }

        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_MASTER,
            sda_io_num: SDA::pin() as i32,
            sda_pullup_en: true, //sda.pullup_enable(),
            scl_io_num: SCL::pin() as i32,
            scl_pullup_en: true, //scl.pullup_enable(),
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed },
            },
        };

        esp!(unsafe {i2c_param_config(I2C::port(), &sys_config)})?;

        esp!(unsafe {i2c_driver_install(
            I2C::port(),
            i2c_mode_t_I2C_MODE_MASTER,
            0, // rx_buf_len,
            0, // tx_buf_len,
            0) // TODO: set flags
        })?;

        Ok(Master {
            i2c,
            pins,
        })
    }

    pub fn release(self) -> Result<(I2C, Pins<SDA, SCL>), EspError> {
        esp!(unsafe {i2c_driver_delete(I2C::port())})?;

        //self.pins.sda.reset()?;
        //self.pins.scl.reset()?;

        Ok((self.i2c, self.pins))
    }
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::Read for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    type Error = EspError;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let command_link = CommandLink::new()?;

        unsafe {
            esp!(i2c_master_write_byte(command_link.0, (addr << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true))?;
            esp!(i2c_master_read(command_link.0, buffer.as_ptr() as *const u8 as *mut u8, buffer.len() as u32, i2c_ack_type_t_I2C_MASTER_LAST_NACK))?;
            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(i2c_master_cmd_begin(I2C::port(), command_link.0, portMAX_DELAY), ())
        }
    }
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::Write for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    type Error = EspError;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        unsafe {
            let command_link = CommandLink::new()?;

            esp!(i2c_master_write_byte(command_link.0, (addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true))?;
            esp!(i2c_master_write(command_link.0, bytes.as_ptr() as *const u8 as *mut u8, bytes.len() as u32, true))?;
            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(i2c_master_cmd_begin(I2C::port(), command_link.0, portMAX_DELAY), ())
        }
    }
}

impl<I2C, SDA, SCL> embedded_hal::blocking::i2c::WriteRead for Master<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    type Error = EspError;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        let command_link = CommandLink::new()?;

        unsafe {
            esp!(i2c_master_write_byte(command_link.0, (addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true))?;
            esp!(i2c_master_write(command_link.0, bytes.as_ptr() as *const u8 as *mut u8, bytes.len() as u32, true))?;
            esp!(i2c_master_read(command_link.0, buffer.as_ptr() as *const u8 as *mut u8, buffer.len() as u32, i2c_ack_type_t_I2C_MASTER_LAST_NACK))?;
            esp!(i2c_master_stop(command_link.0))?;

            esp_result!(i2c_master_cmd_begin(I2C::port(), command_link.0, portMAX_DELAY), ())
        }
    }
}

impl<I2C, SDA, SCL> Slave<I2C, SDA, SCL>
where
    I2C: I2c,
    SDA: OutputPin + InputPin,
    SCL: OutputPin + InputPin
{
    pub fn new(
            i2c: I2C,
            pins: Pins<SDA, SCL>,
            slave_addr: u16,
            addr10bits: bool,
            rx_buf_len: usize,
            tx_buf_len: usize) -> Result<Self, EspError> {
        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_SLAVE,
            sda_io_num: SDA::pin() as i32,
            sda_pullup_en: true, //sda.pullup_enable(),
            scl_io_num: SCL::pin() as i32,
            scl_pullup_en: true, //scl.pullup_enable(),
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                slave: i2c_config_t__bindgen_ty_1__bindgen_ty_2 {
                    slave_addr,
                    addr_10bit_en: if addr10bits {1} else {0} as u8,
                },
            },
        };

        esp!(unsafe {i2c_param_config(I2C::port(), &sys_config)})?;

        esp!(unsafe {i2c_driver_install(
            I2C::port(),
            i2c_mode_t_I2C_MODE_SLAVE,
            rx_buf_len as u32,
            tx_buf_len as u32,
            0, // TODO: set flags
        )})?;

        Ok(Self {
            i2c,
            pins,
            timeout: None, // TODO
        })
    }

    pub fn release(self) -> Result<(I2C, Pins<SDA, SCL>), EspError> {
        esp!(unsafe {i2c_driver_delete(I2C::port())})?;

        //self.pins.sda.reset()?;
        //self.pins.scl.reset()?;

        Ok((self.i2c, self.pins))
    }

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, EspError> {
        let n = unsafe {i2c_slave_read_buffer(
            I2C::port(),
            buffer.as_mut_ptr(),
            buffer.len() as u32,
            self.timeout.unwrap_or(portMAX_DELAY))};

        // if n > 0 {
            Ok(n as usize)
        // } else if n == 0 {
        //     Err(Error::Timeout)
        // } else {
        //     EspError(n).into_result().map(|()| unreachable!())
        // }
    }

    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError> {
        let n = unsafe {i2c_slave_write_buffer(
            I2C::port(),
            bytes.as_ptr() as *const u8 as *mut u8,
            bytes.len() as i32,
            self.timeout.unwrap_or(portMAX_DELAY))};

        // if n > 0 {
            Ok(n as usize)
        // } else if n == 0 {
        //     Err(Error::Timeout)
        // } else {
        //     EspError(n).into_result().map(|()| unreachable!())
        // }
    }
}

struct CommandLink(i2c_cmd_handle_t);

impl CommandLink {
    fn new() -> Result<Self, EspError> {
        let handle = unsafe {i2c_cmd_link_create()};

        // if handle == null_mut() {
        //     return Err(Error::NoMem);
        // }

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

struct I2cPrivateField;

macro_rules! impl_i2c {
    ($i2c:ident: $port:expr) => {
        pub struct $i2c(I2cPrivateField);

        impl $i2c {
            pub unsafe fn new() -> Self {
                $i2c(I2cPrivateField)
            }
        }

        impl I2c for $i2c {
            #[inline(always)]
            fn port() -> i2c_port_t {$port}
        }
    };
}

impl_i2c!(I2C0: 0);
impl_i2c!(I2C1: 1);
