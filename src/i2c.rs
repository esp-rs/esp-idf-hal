pub use i2c as beta;
pub use legacy::*;

use esp_idf_sys::*;

pub trait I2c: Send {
    fn port() -> i2c_port_t;
}

#[repr(u8)]
enum UsedDriver {
    None = 0,
    Legacy = 1,
    Beta = 2,
}

// 0 -> no driver, 1 -> legacy driver, 2 -> beta driver
static DRIVER_IN_USE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(UsedDriver::None);

fn check_and_set_beta_driver() {
    if let Err(super::UsedDriver::Legacy) = DRIVER_IN_USE.compare_exchange(
        super::UsedDriver::None,
        super::UsedDriver::Beta,
        core::sync::atomic::Ordering::Relaxed,
        core::sync::atomic::Ordering::Relaxed,
    ) {
        panic!("Legacy I2C driver is already in use. Either legacy driver or beta driver can be used at a time.");
    }
}

fn check_and_set_legacy_driver() {
    if let Err(super::UsedDriver::Beta) = DRIVER_IN_USE.compare_exchange(
        super::UsedDriver::None,
        super::UsedDriver::Legacy,
        core::sync::atomic::Ordering::Relaxed,
        core::sync::atomic::Ordering::Relaxed,
    ) {
        panic!("Beta I2C driver is already in use. Either legacy driver or beta driver can be used at a time.");
    }
}

mod i2c {

    use core::borrow::Borrow;
    use core::ffi::c_void;
    use core::marker::PhantomData;
    use core::ptr;

    use embassy_sync::mutex::Mutex;
    use embassy_sync::mutex::MutexGuard;

    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

    use esp_idf_sys::*;

    use crate::delay::*;
    use crate::gpio::*;
    use crate::interrupt::asynch::HalIsrNotification;
    use crate::peripheral::Peripheral;
    use crate::task::embassy_sync::EspRawMutex;
    use crate::units::*;

    pub use embedded_hal::i2c::Operation;

    use super::I2c;
    use super::BETA_DRIVER_IN_USE;
    use super::DRIVER_IN_USE;

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
            super::check_and_set_beta_driver();

            let handle = init_master_bus(_i2c, sda, scl, config, 0)?;

            Ok(I2cDriver {
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

        /// Probe device on the bus.
        pub fn probe_device(
            &mut self,
            address: config::DeviceAddress,
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            esp!(unsafe { i2c_master_probe(self.handle, address.address(), timeout as i32) })
        }

        pub fn device(
            &mut self,
            config: &config::DeviceConfig,
        ) -> Result<I2cDeviceDriver<'d, &mut I2cDriver<'d>>, EspError> {
            I2cDeviceDriver::new(self, config)
        }

        // Helper to use the embedded_hal traits.
        fn read(
            &mut self,
            addr: u8,
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                addr,
            )))?
            .read(buffer, timeout)
        }

        // Helper to use the embedded_hal traits.
        fn write(&mut self, addr: u8, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                addr,
            )))?
            .write(bytes, timeout)
        }

        // Helper to use the embedded_hal traits.
        fn write_read(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                addr,
            )))?
            .write_read(bytes, buffer, timeout)
        }
    }

    unsafe impl<'d> Send for I2cDriver<'d> {}

    impl<'d> Drop for I2cDriver<'d> {
        fn drop(&mut self) {
            esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
        }
    }

    impl<'d> embedded_hal_0_2::blocking::i2c::Read for I2cDriver<'d> {
        type Error = I2cError;

        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            Self::read(self, addr, buffer, BLOCK).map_err(to_i2c_err)
        }
    }

    impl<'d> embedded_hal_0_2::blocking::i2c::Write for I2cDriver<'d> {
        type Error = I2cError;

        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            Self::write(self, addr, bytes, BLOCK).map_err(to_i2c_err)
        }
    }

    impl<'d> embedded_hal_0_2::blocking::i2c::WriteRead for I2cDriver<'d> {
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

    impl<'d> embedded_hal::i2c::ErrorType for I2cDriver<'d> {
        type Error = I2cError;
    }

    impl<'d> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDriver<'d> {
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

    pub struct I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        _driver: T,
        handle: i2c_master_dev_handle_t,
        _p: PhantomData<&'d mut ()>,
    }

    impl<'d, T> I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        pub fn new(driver: T, config: &config::DeviceConfig) -> Result<Self, EspError> {
            let handle = init_device(driver.borrow().bus_handle(), &config)?;

            Ok(I2cDeviceDriver {
                _driver: driver,
                handle,
                _p: PhantomData,
            })
        }

        pub fn read(&mut self, buffer: &mut [u8], timeout: TickType_t) -> Result<(), EspError> {
            esp!(unsafe {
                i2c_master_receive(
                    self.handle,
                    buffer.as_mut_ptr().cast(),
                    buffer.len(),
                    timeout as i32,
                )
            })
        }

        pub fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
            esp!(unsafe {
                i2c_master_transmit(
                    self.handle,
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
                    self.handle,
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
        T: Borrow<I2cDriver<'d>>,
    {
        fn drop(&mut self) {
            esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
        }
    }

    impl<'d, T> embedded_hal_0_2::blocking::i2c::Read for I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        type Error = I2cError;

        fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            I2cDeviceDriver::read(self, buffer, BLOCK).map_err(to_i2c_err)
        }
    }

    impl<'d, T> embedded_hal_0_2::blocking::i2c::Write for I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        type Error = I2cError;

        fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            I2cDeviceDriver::write(self, bytes, BLOCK).map_err(to_i2c_err)
        }
    }

    impl<'d, T> embedded_hal_0_2::blocking::i2c::WriteRead for I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        type Error = I2cError;

        fn write_read(
            &mut self,
            _addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            I2cDeviceDriver::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
        }
    }

    impl<'d, T> embedded_hal::i2c::ErrorType for I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        type Error = I2cError;
    }

    impl<'d, T> embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2cDeviceDriver<'d, T>
    where
        T: Borrow<I2cDriver<'d>>,
    {
        fn read(&mut self, _addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            I2cDeviceDriver::read(self, buffer, BLOCK).map_err(to_i2c_err)
        }

        fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            I2cDeviceDriver::write(self, bytes, BLOCK).map_err(to_i2c_err)
        }

        fn write_read(
            &mut self,
            _addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            I2cDeviceDriver::write_read(self, bytes, buffer, BLOCK).map_err(to_i2c_err)
        }

        fn transaction(
            &mut self,
            _addr: u8,
            _operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            unimplemented!("transactional not implemented")
        }
    }

    // ------------------------------------------------------------------------------------------------
    // ------------------------------------- Async ----------------------------------------------------
    // ------------------------------------------------------------------------------------------------

    pub struct AsyncI2cDriver<'d> {
        bus_lock: Mutex<EspRawMutex, ()>,
        handle: i2c_master_bus_handle_t,
        port: u8,
        _p: PhantomData<&'d mut ()>,
    }

    impl<'d> AsyncI2cDriver<'d> {
        pub fn new<I2C: I2c>(
            _i2c: impl Peripheral<P = I2C> + 'd,
            sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
            scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
            config: &config::Config,
        ) -> Result<Self, EspError> {
            super::check_and_set_beta_driver();

            let handle = init_master_bus(_i2c, sda, scl, config, 1)?;

            Ok(AsyncI2cDriver {
                bus_lock: Mutex::new(()),
                handle,
                port: I2C::port() as _,
                _p: PhantomData,
            })
        }

        pub fn port(&self) -> u8 {
            self.port
        }

        fn bus_handle(&self) -> i2c_master_bus_handle_t {
            self.handle
        }

        async fn acquire_bus<'a>(&'a self) -> MutexGuard<'a, EspRawMutex, ()> {
            self.bus_lock.lock().await
        }

        pub fn device(
            &self,
            config: &config::DeviceConfig,
        ) -> Result<AsyncI2cDeviceDriver<'d, &AsyncI2cDriver<'d>>, EspError> {
            AsyncI2cDeviceDriver::new(self, config)
        }

        pub fn owned_device(
            self,
            config: &config::DeviceConfig,
        ) -> Result<OwnedAsyncI2cDeviceDriver<'d>, EspError> {
            OwnedAsyncI2cDeviceDriver::wrap(self, config)
        }

        async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                address,
            )))?
            .read(buffer, BLOCK)
            .await
        }

        async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                address,
            )))?
            .write(bytes, BLOCK)
            .await
        }

        async fn write_read(
            &mut self,
            address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), EspError> {
            self.device(&config::DeviceConfig::new(config::DeviceAddress::SevenBit(
                address,
            )))?
            .write_read(bytes, buffer, BLOCK)
            .await
        }
    }

    impl<'d> embedded_hal::i2c::ErrorType for AsyncI2cDriver<'d> {
        type Error = I2cError;
    }

    impl<'d> embedded_hal_async::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for AsyncI2cDriver<'d> {
        async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.read(address, buffer).await.map_err(to_i2c_err)
        }

        async fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.write(address, bytes).await.map_err(to_i2c_err)
        }

        async fn write_read(
            &mut self,
            address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write_read(address, bytes, buffer)
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

    unsafe impl<'d> Send for AsyncI2cDriver<'d> {}

    impl<'d> Drop for AsyncI2cDriver<'d> {
        fn drop(&mut self) {
            loop {
                if let Ok(_lock_guard) = self.bus_lock.try_lock() {
                    esp!(unsafe { i2c_del_master_bus(self.handle) }).unwrap();
                    break;
                }
            }
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    pub struct AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        driver: T,
        handle: i2c_master_dev_handle_t,
        _p: PhantomData<&'d mut ()>,
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d, T> AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        fn new(driver: T, config: &config::DeviceConfig) -> Result<Self, EspError> {
            let handle = init_device(driver.borrow().bus_handle(), config)?;

            Ok(Self {
                driver,
                handle,
                _p: PhantomData,
            })
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d, T> AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        pub async fn read(
            &mut self,
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            let handle = self.handle;
            let driver = self.driver.borrow();
            let port = driver.port();

            let _lock_guard = driver.acquire_bus().await;
            enable_master_dev_isr_callback(handle, port)?;
            esp!(unsafe {
                i2c_master_receive(
                    handle,
                    buffer.as_mut_ptr().cast(),
                    buffer.len(),
                    timeout as i32,
                )
            })?;

            NOTIFIER[port as usize].wait().await;
            disable_master_dev_isr_callback(handle)?;
            Ok(())
        }

        pub async fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
            let handle = self.handle;
            let driver = self.driver.borrow();
            let port = driver.port();

            let _lock_guard = driver.acquire_bus().await;
            enable_master_dev_isr_callback(handle, port)?;
            esp!(unsafe {
                i2c_master_transmit(handle, bytes.as_ptr().cast(), bytes.len(), timeout as i32)
            })?;

            NOTIFIER[port as usize].wait().await;
            disable_master_dev_isr_callback(handle)?;
            Ok(())
        }

        pub async fn write_read(
            &mut self,
            bytes: &[u8],
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            let handle = self.handle;
            let driver = self.driver.borrow();
            let port = driver.port();

            let _lock_guard = driver.acquire_bus().await;
            enable_master_dev_isr_callback(handle, port)?;
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
            disable_master_dev_isr_callback(handle)?;
            Ok(())
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    unsafe impl<'d, T> Send for AsyncI2cDeviceDriver<'d, T> where
        T: Send + Borrow<AsyncI2cDriver<'d>> + 'd
    {
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d, T> embedded_hal::i2c::ErrorType for AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        type Error = I2cError;
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d, T> embedded_hal_async::i2c::I2c<embedded_hal::i2c::SevenBitAddress>
        for AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        async fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            Self::read(self, buffer, BLOCK).await.map_err(to_i2c_err)
        }

        async fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            Self::write(self, bytes, BLOCK).await.map_err(to_i2c_err)
        }

        async fn write_read(
            &mut self,
            _address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            Self::write_read(self, bytes, buffer, BLOCK)
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

    impl<'d, T> Drop for AsyncI2cDeviceDriver<'d, T>
    where
        T: Borrow<AsyncI2cDriver<'d>>,
    {
        fn drop(&mut self) {
            loop {
                if let Ok(_lock_guard) = self.driver.borrow().bus_lock.try_lock() {
                    esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
                    break;
                }
            }
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    pub struct OwnedAsyncI2cDeviceDriver<'d> {
        driver: Option<AsyncI2cDriver<'d>>,
        handle: i2c_master_dev_handle_t,
        _p: PhantomData<&'d mut ()>,
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d> OwnedAsyncI2cDeviceDriver<'d> {
        pub fn new<I2C: I2c>(
            _i2c: impl Peripheral<P = I2C> + 'd,
            sda: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
            scl: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
            bus_config: &config::Config,
            device_config: &config::DeviceConfig,
        ) -> Result<Self, EspError> {
            let driver = AsyncI2cDriver::new(_i2c, sda, scl, bus_config)?;
            Self::wrap(driver, device_config)
        }

        pub fn wrap(
            driver: AsyncI2cDriver<'d>,
            device_config: &config::DeviceConfig,
        ) -> Result<Self, EspError> {
            let handle = init_device(driver.bus_handle(), device_config)?;

            enable_master_dev_isr_callback(handle, driver.port())?;

            Ok(Self {
                driver: Some(driver),
                handle,
                _p: PhantomData,
            })
        }

        pub fn release(mut self) -> Result<AsyncI2cDriver<'d>, EspError> {
            let driver = self.driver.take().unwrap();
            drop(self);
            Ok(driver)
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d> OwnedAsyncI2cDeviceDriver<'d> {
        pub async fn read(
            &mut self,
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            esp!(unsafe {
                i2c_master_receive(
                    self.handle,
                    buffer.as_mut_ptr().cast(),
                    buffer.len(),
                    timeout as i32,
                )
            })?;

            let port = self.driver.as_ref().unwrap().port() as usize;
            NOTIFIER[port].wait().await;
            Ok(())
        }

        pub async fn write(&mut self, bytes: &[u8], timeout: TickType_t) -> Result<(), EspError> {
            esp!(unsafe {
                i2c_master_transmit(
                    self.handle,
                    bytes.as_ptr().cast(),
                    bytes.len(),
                    timeout as i32,
                )
            })?;

            let port = self.driver.as_ref().unwrap().port() as usize;
            NOTIFIER[port].wait().await;
            Ok(())
        }

        pub async fn write_read(
            &mut self,
            bytes: &[u8],
            buffer: &mut [u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            esp!(unsafe {
                i2c_master_transmit_receive(
                    self.handle,
                    bytes.as_ptr().cast(),
                    bytes.len(),
                    buffer.as_mut_ptr().cast(),
                    buffer.len(),
                    timeout as i32,
                )
            })?;

            let port = self.driver.as_ref().unwrap().port() as usize;
            NOTIFIER[port].wait().await;
            Ok(())
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    unsafe impl<'d> Send for OwnedAsyncI2cDeviceDriver<'d> {}

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d> Drop for OwnedAsyncI2cDeviceDriver<'d> {
        fn drop(&mut self) {
            disable_master_dev_isr_callback(self.handle).unwrap();
            esp!(unsafe { i2c_master_bus_rm_device(self.handle) }).unwrap();
        }
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d> embedded_hal::i2c::ErrorType for OwnedAsyncI2cDeviceDriver<'d> {
        type Error = I2cError;
    }

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    impl<'d> embedded_hal_async::i2c::I2c<embedded_hal::i2c::SevenBitAddress>
        for OwnedAsyncI2cDeviceDriver<'d>
    {
        async fn read(&mut self, _address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            Self::read(self, buffer, BLOCK).await.map_err(to_i2c_err)
        }

        async fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            Self::write(self, bytes, BLOCK).await.map_err(to_i2c_err)
        }

        async fn write_read(
            &mut self,
            _address: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            Self::write_read(self, bytes, buffer, BLOCK)
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
            super::check_and_set_beta_driver();

            let handle = init_slave_device(_i2c, sda, scl, address, config)?;

            enable_slave_isr_callback(handle, I2C::port() as _)?;

            Ok(Self {
                i2c: I2C::port() as _,
                handle,
                _p: PhantomData,
            })
        }

        pub fn read(&mut self, buffer: &mut [u8], _timeout: TickType_t) -> Result<usize, EspError> {
            esp!(unsafe { i2c_slave_receive(self.handle, buffer.as_mut_ptr(), buffer.len()) })?;

            todo!("How to block?");
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
        max_device_count: usize,
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
            trans_queue_depth: max_device_count,
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

    #[cfg(not(esp_idf_i2c_isr_iram_safe))]
    fn enable_master_dev_isr_callback(
        handle: i2c_master_dev_handle_t,
        host: u8,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            i2c_master_register_event_callbacks(
                handle,
                &i2c_master_event_callbacks_t {
                    on_trans_done: Some(master_isr),
                },
                &NOTIFIER[host as usize] as *const _ as *mut _,
            )
        })
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
        let notifier: &HalIsrNotification =
            unsafe { (user_data as *const HalIsrNotification).as_ref() }.unwrap();

        notifier.notify_lsb()
    }

    #[cfg(any(esp32c3, esp32c2, esp32c6))]
    static NOTIFIER: [HalIsrNotification; 1] = [HalIsrNotification::new()];

    #[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
    static NOTIFIER: [HalIsrNotification; 2] =
        [HalIsrNotification::new(), HalIsrNotification::new()];
}

mod legacy {

    use core::marker::PhantomData;
    use core::time::Duration;

    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

    use esp_idf_sys::*;

    use crate::delay::*;
    use crate::gpio::*;
    use crate::interrupt::InterruptType;
    use crate::peripheral::Peripheral;
    use crate::units::*;

    pub use embedded_hal::i2c::Operation;

    use super::I2c;

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
                ((duration.as_nanos() + APB_TICK_PERIOD_NS as u128 - 1)
                    / APB_TICK_PERIOD_NS as u128) as ::core::ffi::c_int,
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
        use crate::{interrupt::InterruptType, units::*};

        /// I2C Master configuration
        #[derive(Debug, Clone)]
        pub struct Config {
            pub baudrate: Hertz,
            pub sda_pullup_enabled: bool,
            pub scl_pullup_enabled: bool,
            pub timeout: Option<APBTickType>,
            pub intr_flags: EnumSet<InterruptType>,
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
            pub fn intr_flags(mut self, flags: EnumSet<InterruptType>) -> Self {
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
                    intr_flags: EnumSet::<InterruptType>::empty(),
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
            pub intr_flags: EnumSet<InterruptType>,
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
            pub fn intr_flags(mut self, flags: EnumSet<InterruptType>) -> Self {
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
                    intr_flags: EnumSet::<InterruptType>::empty(),
                }
            }
        }
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
            super::check_and_set_legacy_driver();

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
                    InterruptType::to_native(config.intr_flags) as _,
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

        pub fn write(
            &mut self,
            addr: u8,
            bytes: &[u8],
            timeout: TickType_t,
        ) -> Result<(), EspError> {
            let mut command_link = CommandLink::new()?;

            command_link.master_start()?;
            command_link
                .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;

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
            command_link
                .master_write_byte((addr << 1) | (i2c_rw_t_I2C_MASTER_WRITE as u8), true)?;

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

        fn write_read(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
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

        fn write_read(
            &mut self,
            addr: u8,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
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
            super::check_and_set_legacy_driver();

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
            #[deprecated(
                note = "Using ESP-IDF 4.3 is untested, please upgrade to 4.4 or newer. Support will be removed in the next major release."
            )]
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
                    InterruptType::to_native(config.intr_flags) as _,
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
            esp!(unsafe { i2c_master_read(self.0, buf.as_mut_ptr().cast(), buf.len(), ack as u32) })
        }
    }

    impl<'buffers> Drop for CommandLink<'buffers> {
        fn drop(&mut self) {
            unsafe {
                i2c_cmd_link_delete(self.0);
            }
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
