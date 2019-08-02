use crate::{
    delay::portMAX_DELAY,
    errors::{Error, EspError, Result},
};
use core::{convert::TryInto, ptr::null_mut};
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use esp_idf_sys::{
    gpio_pullup_t, gpio_pullup_t_GPIO_PULLUP_DISABLE, gpio_pullup_t_GPIO_PULLUP_ENABLE,
    i2c_ack_type_t, i2c_ack_type_t_I2C_MASTER_LAST_NACK, i2c_cmd_handle_t, i2c_cmd_link_create,
    i2c_cmd_link_delete, i2c_config_t, i2c_config_t__bindgen_ty_1,
    i2c_config_t__bindgen_ty_1__bindgen_ty_1, i2c_config_t__bindgen_ty_1__bindgen_ty_2,
    i2c_driver_delete, i2c_driver_install, i2c_master_cmd_begin, i2c_master_read, i2c_master_start,
    i2c_master_stop, i2c_master_write, i2c_master_write_byte, i2c_mode_t_I2C_MODE_MASTER,
    i2c_mode_t_I2C_MODE_SLAVE, i2c_param_config, i2c_port_t, i2c_port_t_I2C_NUM_0,
    i2c_port_t_I2C_NUM_1, i2c_rw_t_I2C_MASTER_READ, i2c_rw_t_I2C_MASTER_WRITE,
    i2c_slave_read_buffer, i2c_slave_write_buffer, TickType_t,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Port0,
    Port1,
}

impl From<Port> for i2c_port_t {
    fn from(value: Port) -> Self {
        match value {
            Port::Port0 => i2c_port_t_I2C_NUM_0,
            Port::Port1 => i2c_port_t_I2C_NUM_1,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PinConfig {
    pub pin_num: u32,
    pub pullup: bool,
}

impl PinConfig {
    fn pullup_enable(&self) -> gpio_pullup_t {
        match self.pullup {
            true => gpio_pullup_t_GPIO_PULLUP_ENABLE,
            false => gpio_pullup_t_GPIO_PULLUP_DISABLE,
        }
    }
}

struct CmdHandle(i2c_cmd_handle_t);

impl CmdHandle {
    fn new() -> Result<Self> {
        let cmd = unsafe { i2c_cmd_link_create() };

        if cmd == null_mut() {
            return Err(Error::NoMem);
        }

        Ok(CmdHandle(cmd))
    }
}

impl Drop for CmdHandle {
    fn drop(&mut self) {
        unsafe {
            i2c_cmd_link_delete(self.0);
        }
    }
}

struct MasterCmd(CmdHandle);

impl MasterCmd {
    fn new() -> Result<Self> {
        let cmd = CmdHandle::new()?;

        unsafe {
            EspError(i2c_master_start(cmd.0)).into_result()?;
        }

        Ok(MasterCmd(cmd))
    }

    fn write_byte(&mut self, b: u8, ack_check: bool) -> Result<()> {
        unsafe { EspError(i2c_master_write_byte((self.0).0, b, ack_check)).into_result() }
    }

    fn write(&mut self, buf: &[u8], ack_check: bool) -> Result<()> {
        unsafe {
            EspError(i2c_master_write(
                (self.0).0,
                buf.as_ptr() as *const u8 as *mut u8,
                buf.len(),
                ack_check,
            ))
            .into_result()
        }
    }

    fn read(&mut self, buf: &mut [u8], ack: i2c_ack_type_t) -> Result<()> {
        unsafe {
            EspError(i2c_master_read(
                (self.0).0,
                buf.as_mut_ptr(),
                buf.len(),
                ack,
            ))
            .into_result()
        }
    }

    fn stop(&mut self) -> Result<()> {
        unsafe { EspError(i2c_master_stop((self.0).0)).into_result() }
    }
}

pub struct Master {
    port: Port,
}

impl Master {
    pub unsafe fn new(port: Port, sda: PinConfig, scl: PinConfig, clk_speed: u32) -> Result<Self> {
        // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
        if clk_speed > 1_000_000 {
            return Err(Error::InvalidArg);
        }

        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_MASTER,
            sda_io_num: sda.pin_num,
            sda_pullup_en: sda.pullup_enable(),
            scl_io_num: scl.pin_num,
            scl_pullup_en: scl.pullup_enable(),
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 { clk_speed },
            },
        };

        EspError(i2c_param_config(port.into(), &sys_config)).into_result()?;

        // TODO: set flags
        let intr_alloc_flags = 0;
        EspError(i2c_driver_install(
            port.into(),
            i2c_mode_t_I2C_MODE_MASTER,
            0, // rx_buf_len,
            0, // tx_buf_len,
            intr_alloc_flags,
        ))
        .into_result()?;

        Ok(Self { port })
    }

    fn begin_cmd(&mut self, cmd: MasterCmd, ticks_to_wait: TickType_t) -> Result<()> {
        unsafe {
            EspError(i2c_master_cmd_begin(
                self.port.into(),
                (cmd.0).0,
                ticks_to_wait,
            ))
            .into_result()
        }
    }
}

impl Drop for Master {
    fn drop(&mut self) {
        unsafe {
            EspError(i2c_driver_delete(self.port.into()))
                .into_result()
                .unwrap();
        }
    }
}

impl Write for Master {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;
        cmd.write(bytes, true)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

impl Read for Master {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((address << 1) | (i2c_rw_t_I2C_MASTER_READ as u8), true)?;
        cmd.read(buffer, i2c_ack_type_t_I2C_MASTER_LAST_NACK)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

impl WriteRead for Master {
    type Error = Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<()> {
        let mut cmd = MasterCmd::new()?;
        cmd.write_byte((address << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;
        cmd.write(bytes, true)?;
        cmd.read(buffer, i2c_ack_type_t_I2C_MASTER_LAST_NACK)?;
        cmd.stop()?;

        self.begin_cmd(cmd, portMAX_DELAY)?;

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddrMode {
    Addr7Bit,
    Addr10Bit,
}

pub struct Slave {
    port: Port,
}

impl Slave {
    pub unsafe fn new(
        port: Port,
        sda: PinConfig,
        scl: PinConfig,
        slave_addr: u16,
        addr_mode: AddrMode,
        rx_buf_len: usize,
        tx_buf_len: usize,
    ) -> Result<Self> {
        let sys_config = i2c_config_t {
            mode: i2c_mode_t_I2C_MODE_SLAVE,
            sda_io_num: sda.pin_num,
            sda_pullup_en: sda.pullup_enable(),
            scl_io_num: scl.pin_num,
            scl_pullup_en: scl.pullup_enable(),
            __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
                slave: i2c_config_t__bindgen_ty_1__bindgen_ty_2 {
                    slave_addr,
                    addr_10bit_en: (addr_mode == AddrMode::Addr10Bit).into(),
                },
            },
        };

        EspError(i2c_param_config(port.into(), &sys_config)).into_result()?;

        // TODO: set flags
        let intr_alloc_flags = 0;
        EspError(i2c_driver_install(
            port.into(),
            i2c_mode_t_I2C_MODE_SLAVE,
            rx_buf_len,
            tx_buf_len,
            intr_alloc_flags,
        ))
        .into_result()?;

        Ok(Self { port })
    }

    pub fn write(&mut self, buf: &[u8], ticks_to_wait: TickType_t) -> Result<usize> {
        let n = unsafe {
            i2c_slave_write_buffer(
                self.port.into(),
                buf.as_ptr() as *const u8 as *mut u8,
                buf.len().try_into().unwrap(),
                ticks_to_wait,
            )
        };

        if n > 0 {
            Ok(n.try_into().unwrap())
        } else if n == 0 {
            Err(Error::Timeout)
        } else {
            EspError(n).into_result().map(|()| unreachable!())
        }
    }

    pub fn read(&mut self, buf: &mut [u8], ticks_to_wait: TickType_t) -> Result<usize> {
        let n = unsafe {
            i2c_slave_read_buffer(self.port.into(), buf.as_mut_ptr(), buf.len(), ticks_to_wait)
        };

        if n > 0 {
            Ok(n.try_into().unwrap())
        } else if n == 0 {
            Err(Error::Timeout)
        } else {
            EspError(n).into_result().map(|()| unreachable!())
        }
    }
}

impl Drop for Slave {
    fn drop(&mut self) {
        unsafe {
            EspError(i2c_driver_delete(self.port.into()))
                .into_result()
                .unwrap();
        }
    }
}
