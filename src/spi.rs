//! SPI peripheral control
//!
//! SPI0 is reserved for accessing flash and sram and therefore not usable for other purposes.
//! SPI1 shares its external pins with SPI0 and therefore has severe restrictions in use.
//!
//! SPI2 & 3 can be used freely.
//!
//! The CS pin can be controlled by hardware on esp32 variants (contrary to the description of embedded_hal).
//!
//! Look at the following table to determine which driver best suits your requirements:
//!
//! |   |                  | SpiDeviceDriver::new | SpiDeviceDriver::new (no CS) | SpiSoftCsDeviceDriver::new | SpiBusDriver::new |
//! |---|------------------|----------------------|------------------------------|----------------------------|-------------------|
//! |   | Managed CS       |       Hardware       |              N               |     Software triggered     |         N         |
//! |   | 1 device         |           Y          |              Y               |              Y             |         Y         |
//! |   | 1-3 devices      |           Y          |              N               |              Y             |         N         |
//! |   | 4-6 devices      |    Only on esp32CX   |              N               |              Y             |         N         |
//! |   | More than 6      |           N          |              N               |              Y             |         N         |
//! |   | DMA              |           N          |              N               |              N             |         N         |
//! |   | Polling transmit |           Y          |              Y               |              Y             |         Y         |
//! |   | ISR transmit     |           Y          |              Y               |              Y             |         Y         |
//! |   | Async support*   |           Y          |              Y               |              Y             |         Y         |
//!
//! * True non-blocking async possible only when all devices attached to the SPI bus are used in async mode (i.e. calling methods `xxx_async()`
//!   instead of their blocking `xxx()` counterparts)
//!
//! The [Transfer::transfer], [Write::write] and [WriteIter::write_iter] functions lock the
//! APB frequency and therefore the requests are always run at the requested baudrate.
//! The primitive [FullDuplex::read] and [FullDuplex::send] do not lock the APB frequency and
//! therefore may run at a different frequency.
//!
//! # TODO
//! - Quad SPI
//! - Slave SPI

use core::borrow::{Borrow, BorrowMut};
use core::cell::UnsafeCell;
use core::cmp::{max, Ordering};
use core::iter::Peekable;
use core::marker::PhantomData;
use core::{ptr, u8};

use embassy_sync::mutex::Mutex;
use embedded_hal::spi::{
    Operation, SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite, SpiDevice, SpiDeviceRead,
    SpiDeviceWrite,
};

use esp_idf_sys::*;

use crate::delay::{self, Ets, BLOCK};
use crate::gpio::{AnyOutputPin, InputPin, Level, Output, OutputMode, OutputPin, PinDriver};
use crate::interrupt::IntrFlags;
use crate::peripheral::Peripheral;
use crate::private::completion::with_completion;
use crate::private::notification::Notification;
use crate::task::embassy_sync::EspRawMutex;
use crate::task::CriticalSection;

crate::embedded_hal_error!(
    SpiError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);

pub trait Spi: Send {
    fn device() -> spi_host_device_t;
}

/// A marker interface implemented by all SPI peripherals except SPI1 which
/// should use a fixed set of pins
pub trait SpiAnyPins: Spi {}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Dma {
    Disabled,
    Channel1(usize),
    Channel2(usize),
    Auto(usize),
}

impl From<Dma> for spi_dma_chan_t {
    fn from(dma: Dma) -> Self {
        match dma {
            Dma::Channel1(_) => 1,
            Dma::Channel2(_) => 2,
            Dma::Auto(_) => 3,
            _ => 0,
        }
    }
}

impl Dma {
    pub const fn max_transfer_size(&self) -> usize {
        let max_transfer_size = match self {
            Dma::Disabled => TRANS_LEN,
            Dma::Channel1(size) | Dma::Channel2(size) | Dma::Auto(size) => *size,
        };
        match max_transfer_size {
            0 => panic!("The max transfer size must be greater than 0"),
            x if x % 4 != 0 => panic!("The max transfer size must be a multiple of 4"),
            x if x > 4096 => panic!("The max transfer size must be less than or equal to 4096"),
            _ => max_transfer_size,
        }
    }
}

pub type SpiDriverConfig = config::DriverConfig;
pub type SpiConfig = config::Config;

/// SPI configuration
pub mod config {
    use crate::{interrupt::IntrFlags, units::*};
    use enumset::EnumSet;
    use esp_idf_sys::*;

    use super::Dma;

    pub struct V02Type<T>(pub T);

    impl From<V02Type<embedded_hal_0_2::spi::Polarity>> for embedded_hal::spi::Polarity {
        fn from(polarity: V02Type<embedded_hal_0_2::spi::Polarity>) -> Self {
            match polarity.0 {
                embedded_hal_0_2::spi::Polarity::IdleHigh => embedded_hal::spi::Polarity::IdleHigh,
                embedded_hal_0_2::spi::Polarity::IdleLow => embedded_hal::spi::Polarity::IdleLow,
            }
        }
    }

    impl From<V02Type<embedded_hal_0_2::spi::Phase>> for embedded_hal::spi::Phase {
        fn from(phase: V02Type<embedded_hal_0_2::spi::Phase>) -> Self {
            match phase.0 {
                embedded_hal_0_2::spi::Phase::CaptureOnFirstTransition => {
                    embedded_hal::spi::Phase::CaptureOnFirstTransition
                }
                embedded_hal_0_2::spi::Phase::CaptureOnSecondTransition => {
                    embedded_hal::spi::Phase::CaptureOnSecondTransition
                }
            }
        }
    }

    impl From<V02Type<embedded_hal_0_2::spi::Mode>> for embedded_hal::spi::Mode {
        fn from(mode: V02Type<embedded_hal_0_2::spi::Mode>) -> Self {
            Self {
                polarity: V02Type(mode.0.polarity).into(),
                phase: V02Type(mode.0.phase).into(),
            }
        }
    }

    /// Specify the communication mode with the device
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum Duplex {
        /// Full duplex is the default
        Full,
        /// Half duplex in some cases
        Half,
        /// Use MOSI (=spid) for both sending and receiving data (implies half duplex)
        Half3Wire,
    }

    impl Duplex {
        pub fn as_flags(&self) -> u32 {
            match self {
                Duplex::Full => 0,
                Duplex::Half => SPI_DEVICE_HALFDUPLEX,
                Duplex::Half3Wire => SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE,
            }
        }
    }

    /// Specifies the order in which the bits of data should be transfered/received
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum BitOrder {
        /// Most significant bit first (default)
        MsbFirst,
        /// Least significant bit first
        LsbFirst,
        /// Least significant bit first, when sending
        TxLsbFirst,
        /// Least significant bit first, when receiving
        RxLsbFirst,
    }

    impl BitOrder {
        pub fn as_flags(&self) -> u32 {
            match self {
                Self::MsbFirst => 0,
                Self::LsbFirst => SPI_DEVICE_BIT_LSBFIRST,
                Self::TxLsbFirst => SPI_DEVICE_TXBIT_LSBFIRST,
                Self::RxLsbFirst => SPI_DEVICE_RXBIT_LSBFIRST,
            }
        }
    }

    /// SPI Driver configuration
    #[derive(Debug, Clone)]
    pub struct DriverConfig {
        pub dma: Dma,
        pub intr_flags: EnumSet<IntrFlags>,
    }

    impl DriverConfig {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn dma(mut self, dma: Dma) -> Self {
            self.dma = dma;
            self
        }

        #[must_use]
        pub fn intr_flags(mut self, intr_flags: EnumSet<IntrFlags>) -> Self {
            self.intr_flags = intr_flags;
            self
        }
    }

    impl Default for DriverConfig {
        fn default() -> Self {
            Self {
                dma: Dma::Disabled,
                intr_flags: EnumSet::<IntrFlags>::empty(),
            }
        }
    }

    /// SPI Device configuration
    #[derive(Debug, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_mode: embedded_hal::spi::Mode,
        /// This property can be set to configure a SPI Device for being write only
        /// Thus the flag SPI_DEVICE_NO_DUMMY will be passed on initialization and
        /// it will unlock the possibility of using 80Mhz as the bus freq
        /// See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#timing-considerations
        pub write_only: bool,
        pub duplex: Duplex,
        pub bit_order: BitOrder,
        pub cs_active_high: bool,
        pub input_delay_ns: i32,
        pub polling: bool,
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
        pub fn data_mode(mut self, data_mode: embedded_hal::spi::Mode) -> Self {
            self.data_mode = data_mode;
            self
        }

        #[must_use]
        pub fn write_only(mut self, write_only: bool) -> Self {
            self.write_only = write_only;
            self
        }

        #[must_use]
        pub fn duplex(mut self, duplex: Duplex) -> Self {
            self.duplex = duplex;
            self
        }

        #[must_use]
        pub fn bit_order(mut self, bit_order: BitOrder) -> Self {
            self.bit_order = bit_order;
            self
        }

        #[must_use]
        pub fn cs_active_high(mut self) -> Self {
            self.cs_active_high = true;
            self
        }

        #[must_use]
        pub fn input_delay_ns(mut self, input_delay_ns: i32) -> Self {
            self.input_delay_ns = input_delay_ns;
            self
        }

        #[must_use]
        pub fn polling(mut self, polling: bool) -> Self {
            self.polling = polling;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                data_mode: embedded_hal::spi::MODE_0,
                write_only: false,
                cs_active_high: false,
                duplex: Duplex::Full,
                bit_order: BitOrder::MsbFirst,
                input_delay_ns: 0,
                polling: true,
            }
        }
    }
}

pub struct SpiDriver<'d> {
    host: u8,
    max_transfer_size: usize,
    bus_async_lock: Mutex<EspRawMutex, ()>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> SpiDriver<'d> {
    /// Create new instance of SPI controller for SPI1
    ///
    /// SPI1 can only use fixed pin for SCLK, SDO and SDI as they are shared with SPI0.
    #[cfg(esp32)]
    pub fn new_spi1(
        _spi: impl Peripheral<P = SPI1> + 'd,
        sclk: impl Peripheral<P = crate::gpio::Gpio6> + 'd,
        sdo: impl Peripheral<P = crate::gpio::Gpio7> + 'd,
        sdi: Option<impl Peripheral<P = crate::gpio::Gpio8> + 'd>,
        config: &config::DriverConfig,
    ) -> Result<Self, EspError> {
        let max_transfer_size = Self::new_internal(SPI1::device(), sclk, sdo, sdi, config)?;

        Ok(Self {
            host: SPI1::device() as _,
            max_transfer_size,
            bus_async_lock: Mutex::new(()),
            _p: PhantomData,
        })
    }

    /// Create new instance of SPI controller for all others
    pub fn new<SPI: SpiAnyPins>(
        _spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        config: &config::DriverConfig,
    ) -> Result<Self, EspError> {
        let max_transfer_size = Self::new_internal(SPI::device(), sclk, sdo, sdi, config)?;

        Ok(Self {
            host: SPI::device() as _,
            max_transfer_size,
            bus_async_lock: Mutex::new(()),
            _p: PhantomData,
        })
    }

    pub fn host(&self) -> spi_host_device_t {
        self.host as _
    }

    fn new_internal(
        host: spi_host_device_t,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        config: &config::DriverConfig,
    ) -> Result<usize, EspError> {
        crate::into_ref!(sclk, sdo);
        let sdi = sdi.map(|sdi| sdi.into_ref());

        let max_transfer_sz = config.dma.max_transfer_size();
        let dma_chan: spi_dma_chan_t = config.dma.into();

        #[allow(clippy::needless_update)]
        #[cfg(not(esp_idf_version = "4.3"))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk.pin(),

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: sdo.pin(),
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: sdi.as_ref().map_or(-1, |p| p.pin()),
                //data1_io_num: -1,
            },
            __bindgen_anon_3: spi_bus_config_t__bindgen_ty_3 {
                quadwp_io_num: -1,
                //data2_io_num: -1,
            },
            __bindgen_anon_4: spi_bus_config_t__bindgen_ty_4 {
                quadhd_io_num: -1,
                //data3_io_num: -1,
            },
            max_transfer_sz: max_transfer_sz as i32,
            intr_flags: IntrFlags::to_native(config.intr_flags) as _,
            ..Default::default()
        };

        #[allow(clippy::needless_update)]
        #[cfg(esp_idf_version = "4.3")]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk.pin(),

            mosi_io_num: sdo.pin(),
            miso_io_num: sdi.as_ref().map_or(-1, |p| p.pin()),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            max_transfer_sz: max_transfer_sz as i32,
            intr_flags: config.intr_flags.into() as _,
            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(host, &bus_config, dma_chan) })?;

        Ok(max_transfer_sz)
    }
}

impl<'d> Drop for SpiDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { spi_bus_free(self.host()) }).unwrap();
    }
}

unsafe impl<'d> Send for SpiDriver<'d> {}

pub struct SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    _lock: BusLock,
    handle: spi_device_handle_t,
    driver: T,
    polling: bool,
    _d: PhantomData<&'d ()>,
}

impl<'d, T> SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    pub fn new(driver: T, config: &config::Config) -> Result<Self, EspError> {
        let conf = spi_device_interface_config_t {
            spics_io_num: -1,
            clock_speed_hz: config.baudrate.0 as i32,
            mode: data_mode_to_u8(config.data_mode),
            queue_size: 1,
            flags: if config.write_only {
                SPI_DEVICE_NO_DUMMY
            } else {
                0_u32
            } | config.duplex.as_flags()
                | config.bit_order.as_flags(),
            post_cb: Some(spi_notify),
            ..Default::default()
        };

        let mut handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(driver.borrow().host(), &conf, &mut handle as *mut _) })?;

        let lock = BusLock::new(handle)?;

        Ok(Self {
            _lock: lock,
            handle,
            driver,
            polling: config.polling,
            _d: PhantomData,
        })
    }

    pub fn read(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        // Full-Duplex Mode:
        // The internal hardware 16*4 u8 FIFO buffer (shared for read/write) is not cleared
        // between transactions (read/write/transfer)
        // This can lead to rewriting the internal buffer to MOSI on a read call

        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_read_transactions(words, chunk_size) {
            spi_transmit(self.handle, &mut transaction, self.polling, false)?;
        }

        Ok(())
    }

    pub async fn read_async(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_read_transactions(words, chunk_size) {
            spi_transmit_async(self.handle, &mut transaction, false).await?;
        }

        Ok(())
    }

    pub fn write(&mut self, words: &[u8]) -> Result<(), EspError> {
        // Full-Duplex Mode:
        // The internal hardware 16*4 u8 FIFO buffer (shared for read/write) is not cleared
        // between transactions ( read/write/transfer)
        // This can lead to re-reading the last internal buffer MOSI msg, in case the Slave fails to send a msg

        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_write_transactions(words, chunk_size) {
            spi_transmit(self.handle, &mut transaction, self.polling, false)?;
        }

        Ok(())
    }

    pub async fn write_async(&mut self, words: &[u8]) -> Result<(), EspError> {
        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_write_transactions(words, chunk_size) {
            spi_transmit_async(self.handle, &mut transaction, false).await?;
        }

        Ok(())
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        // In non-DMA mode, it will internally split the transfers every 64 bytes (max_transf_len).
        // - If the read and write buffers are not of the same length, it will first transfer the common buffer length
        // and then (separately aligned) the remaining buffer.
        // - Expect a delay time between every internally split (64-byte or remainder) package.

        // Half-Duplex & Half-3-Duplex Mode:
        // Data will be split into 64-byte write/read sections.
        // Example: write: [u8;96] - read [u8; 160]
        // Package 1: write 64, read 64 -> Package 2: write 32, read 32 -> Package 3: write 0, read 64.
        // Note that the first "package" is a 128-byte clock out while the later are respectively 64 bytes.

        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_transfer_transactions(read, write, chunk_size) {
            spi_transmit(self.handle, &mut transaction, self.polling, false)?;
        }

        Ok(())
    }

    pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_transfer_transactions(read, write, chunk_size) {
            spi_transmit_async(self.handle, &mut transaction, false).await?;
        }

        Ok(())
    }

    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_transfer_in_place_transactions(words, chunk_size) {
            spi_transmit(self.handle, &mut transaction, self.polling, false)?;
        }

        Ok(())
    }

    pub async fn transfer_in_place_async(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        let chunk_size = self.driver.borrow().max_transfer_size;

        for mut transaction in spi_transfer_in_place_transactions(words, chunk_size) {
            spi_transmit_async(self.handle, &mut transaction, false).await?;
        }

        Ok(())
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        Ok(())
    }
}

impl<'d, T> Drop for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    fn drop(&mut self) {
        esp!(unsafe { spi_bus_remove_device(self.handle) }).unwrap();
    }
}

impl<'d, T> embedded_hal::spi::ErrorType for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    type Error = SpiError;
}

impl<'d, T> SpiBusFlush for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    fn flush(&mut self) -> Result<(), Self::Error> {
        SpiBusDriver::flush(self).map_err(to_spi_err)
    }
}

impl<'d, T> SpiBusRead for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusDriver::read(self, words).map_err(to_spi_err)
    }
}

impl<'d, T> SpiBusWrite for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        SpiBusDriver::write(self, words).map_err(to_spi_err)
    }
}

impl<'d, T> SpiBus for SpiBusDriver<'d, T>
where
    T: BorrowMut<SpiDriver<'d>>,
{
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        SpiBusDriver::transfer(self, read, write).map_err(to_spi_err)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusDriver::transfer_in_place(self, words).map_err(to_spi_err)
    }
}

pub type SpiSingleDeviceDriver<'d> = SpiDeviceDriver<'d, SpiDriver<'d>>;

pub struct SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    handle: spi_device_handle_t,
    driver: T,
    cs_pin_configured: bool,
    polling: bool,
    _d: PhantomData<&'d ()>,
}

impl<'d> SpiDeviceDriver<'d, SpiDriver<'d>> {
    #[cfg(esp32)]
    pub fn new_single_spi1(
        spi: impl Peripheral<P = SPI1> + 'd,
        sclk: impl Peripheral<P = crate::gpio::Gpio6> + 'd,
        sdo: impl Peripheral<P = crate::gpio::Gpio7> + 'd,
        sdi: Option<impl Peripheral<P = crate::gpio::Gpio8> + 'd>,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        bus_config: &config::DriverConfig,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        Self::new(
            SpiDriver::new_spi1(spi, sclk, sdo, sdi, bus_config)?,
            cs,
            config,
        )
    }

    pub fn new_single<SPI: SpiAnyPins>(
        spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        bus_config: &config::DriverConfig,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        Self::new(SpiDriver::new(spi, sclk, sdo, sdi, bus_config)?, cs, config)
    }
}

impl<'d, T> SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(
        driver: T,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        let cs = cs.map(|cs| cs.into_ref().pin()).unwrap_or(-1);

        let conf = spi_device_interface_config_t {
            spics_io_num: cs,
            clock_speed_hz: config.baudrate.0 as i32,
            mode: data_mode_to_u8(config.data_mode),
            queue_size: 64,
            input_delay_ns: config.input_delay_ns,
            flags: if config.write_only {
                SPI_DEVICE_NO_DUMMY
            } else {
                0_u32
            } | if config.cs_active_high {
                SPI_DEVICE_POSITIVE_CS
            } else {
                0_u32
            } | config.duplex.as_flags()
                | config.bit_order.as_flags(),
            post_cb: Some(spi_notify),
            ..Default::default()
        };

        let mut handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(driver.borrow().host(), &conf, &mut handle as *mut _) })?;

        Ok(Self {
            handle,
            driver,
            cs_pin_configured: cs >= 0,
            polling: config.polling,
            _d: PhantomData,
        })
    }

    pub fn device(&self) -> spi_device_handle_t {
        self.handle
    }

    pub fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), EspError> {
        self.run(
            &mut operations.iter_mut().map(copy_operation),
            SoftCsPin::none(),
        )
    }

    pub async fn transaction_async(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), EspError> {
        self.run_async(operations.iter_mut().map(copy_operation), SoftCsPin::none())
            .await
    }

    pub fn read(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Read(read)])
    }

    pub async fn read_async(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Read(read)]).await
    }

    pub fn read_transaction(&mut self, operations: &mut [&mut [u8]]) -> Result<(), EspError> {
        self.run(
            &mut operations.iter_mut().map(|slice| Operation::Read(slice)),
            SoftCsPin::none(),
        )
    }

    pub async fn read_transaction_async(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run_async(
            &mut operations.iter_mut().map(|slice| Operation::Read(slice)),
            SoftCsPin::none(),
        )
        .await
    }

    pub fn write(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Write(write)])
    }

    pub async fn write_async(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Write(write)]).await
    }

    pub fn write_transaction(&mut self, operations: &[&[u8]]) -> Result<(), EspError> {
        self.run(
            &mut operations.iter().map(|slice| Operation::Write(slice)),
            SoftCsPin::none(),
        )
    }

    pub async fn write_transaction_async(&mut self, operations: &[&[u8]]) -> Result<(), EspError> {
        self.run_async(
            &mut operations.iter().map(|slice| Operation::Write(slice)),
            SoftCsPin::none(),
        )
        .await
    }

    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::TransferInPlace(buf)])
    }

    pub async fn transfer_in_place_async(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::TransferInPlace(buf)])
            .await
    }

    pub fn transfer_in_place_transaction(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run(
            &mut operations
                .iter_mut()
                .map(|slice| Operation::TransferInPlace(slice)),
            SoftCsPin::none(),
        )
    }

    pub async fn transfer_in_place_transaction_async(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run_async(
            &mut operations
                .iter_mut()
                .map(|slice| Operation::TransferInPlace(slice)),
            SoftCsPin::none(),
        )
        .await
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Transfer(read, write)])
    }

    pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Transfer(read, write)])
            .await
    }

    pub fn transfer_transaction(
        &mut self,
        operations: &mut [(&mut [u8], &[u8])],
    ) -> Result<(), EspError> {
        self.run(
            &mut operations
                .iter_mut()
                .map(|(read_slice, write_slice)| Operation::Transfer(read_slice, write_slice)),
            SoftCsPin::none(),
        )
    }

    pub async fn transfer_transaction_async(
        &mut self,
        operations: &mut [(&mut [u8], &[u8])],
    ) -> Result<(), EspError> {
        self.run_async(
            &mut operations
                .iter_mut()
                .map(|(read_slice, write_slice)| Operation::Transfer(read_slice, write_slice)),
            SoftCsPin::none(),
        )
        .await
    }

    fn run<'a, 'c, 'p, P, M>(
        &self,
        operations: impl Iterator<Item = Operation<'a, u8>> + 'a,
        mut soft_cs_pin: Option<SoftCsPin<'c, 'p, P, M>>,
    ) -> Result<(), EspError>
    where
        P: OutputPin,
        M: OutputMode,
    {
        let mut transactions = mark_last(self.spi_transactions(operations)).peekable();

        let needs_bus_lock =
            soft_cs_pin.is_some() || !transactions.peek().map(|(_, last)| *last).unwrap_or(true);

        let _lock = if needs_bus_lock {
            Some(BusLock::new(self.device())?)
        } else {
            None
        };

        if let Some(soft_cs_pin) = soft_cs_pin.as_mut() {
            soft_cs_pin.raise()?;
        }

        for (mut transaction, last) in transactions {
            spi_transmit(
                self.handle,
                &mut transaction,
                self.polling,
                soft_cs_pin.is_none() && self.cs_pin_configured && !last,
            )?;
        }

        if let Some(mut soft_cs_pin) = soft_cs_pin {
            soft_cs_pin.lower()?;
        }

        Ok(())
    }

    async fn run_async<'a, 'c, 'p, P, M>(
        &self,
        operations: impl Iterator<Item = Operation<'a, u8>> + 'a,
        mut soft_cs_pin: Option<SoftCsPin<'c, 'p, P, M>>,
    ) -> Result<(), EspError>
    where
        P: OutputPin,
        M: OutputMode,
    {
        let mut transactions = mark_last(self.spi_transactions(operations)).peekable();

        let needs_bus_lock =
            soft_cs_pin.is_some() || !transactions.peek().map(|(_, last)| *last).unwrap_or(true);

        let _async_bus_lock = if needs_bus_lock {
            Some(self.driver.borrow().bus_async_lock.lock().await)
        } else {
            None
        };

        let _lock = if needs_bus_lock {
            Some(BusLock::new(self.device())?)
        } else {
            None
        };

        if let Some(soft_cs_pin) = soft_cs_pin.as_mut() {
            soft_cs_pin.raise()?;
        }

        for (mut transaction, last) in transactions {
            spi_transmit_async(
                self.handle,
                &mut transaction,
                soft_cs_pin.is_none() && self.cs_pin_configured && !last,
            )
            .await?;
        }

        if let Some(mut soft_cs_pin) = soft_cs_pin {
            soft_cs_pin.lower()?;
        }

        Ok(())
    }

    fn spi_transactions<'a>(
        &self,
        operations: impl Iterator<Item = Operation<'a, u8>> + 'a,
    ) -> impl Iterator<Item = spi_transaction_t> + 'a {
        enum OperationsIter<R, W, T, I> {
            Read(R),
            Write(W),
            Transfer(T),
            TransferInPlace(I),
        }

        impl<R, W, T, I> Iterator for OperationsIter<R, W, T, I>
        where
            R: Iterator<Item = spi_transaction_t>,
            W: Iterator<Item = spi_transaction_t>,
            T: Iterator<Item = spi_transaction_t>,
            I: Iterator<Item = spi_transaction_t>,
        {
            type Item = spi_transaction_t;

            fn next(&mut self) -> Option<Self::Item> {
                match self {
                    Self::Read(iter) => iter.next(),
                    Self::Write(iter) => iter.next(),
                    Self::Transfer(iter) => iter.next(),
                    Self::TransferInPlace(iter) => iter.next(),
                }
            }
        }

        let chunk_size = self.driver.borrow().max_transfer_size;

        operations.flat_map(move |op| match op {
            Operation::Read(words) => {
                OperationsIter::Read(spi_read_transactions(words, chunk_size))
            }
            Operation::Write(words) => {
                OperationsIter::Write(spi_write_transactions(words, chunk_size))
            }
            Operation::Transfer(read, write) => {
                OperationsIter::Transfer(spi_transfer_transactions(read, write, chunk_size))
            }
            Operation::TransferInPlace(words) => OperationsIter::TransferInPlace(
                spi_transfer_in_place_transactions(words, chunk_size),
            ),
        })
    }
}

impl<'d, T> Drop for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    fn drop(&mut self) {
        esp!(unsafe { spi_bus_remove_device(self.handle) }).unwrap();
    }
}

unsafe impl<'d, T> Send for SpiDeviceDriver<'d, T> where T: Send + Borrow<SpiDriver<'d>> + 'd {}

impl<'d, T> embedded_hal::spi::ErrorType for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;
}

impl<'d, T> SpiDeviceRead for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    fn read_transaction(&mut self, operations: &mut [&mut [u8]]) -> Result<(), Self::Error> {
        Self::read_transaction(self, operations).map_err(to_spi_err)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buf).map_err(to_spi_err)
    }
}

impl<'d, T> SpiDeviceWrite for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    fn write_transaction(&mut self, operations: &[&[u8]]) -> Result<(), Self::Error> {
        Self::write_transaction(self, operations).map_err(to_spi_err)
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, buf).map_err(to_spi_err)
    }
}

impl<'d, T> SpiDevice for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        Self::transaction(self, operations).map_err(to_spi_err)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::Transfer<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.transfer_in_place(words)?;

        Ok(words)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::Write<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write(words).map_err(to_spi_err)
    }
}

/// All data is chunked into max(iter.len(), 64)
impl<'d, T> embedded_hal_0_2::blocking::spi::WriteIter<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u8>,
    {
        let mut lock = None;

        let mut words = words.into_iter().peekable();
        let mut buf = [0_u8; TRANS_LEN];

        loop {
            let mut offset = 0_usize;

            while offset < buf.len() {
                if let Some(word) = words.next() {
                    buf[offset] = word;
                    offset += 1;
                } else {
                    break;
                }
            }

            if offset == 0 {
                break;
            }

            let mut transaction =
                spi_create_transaction(core::ptr::null_mut(), buf[..offset].as_ptr(), offset, 0);

            if lock.is_none() && words.peek().is_some() {
                lock = Some(BusLock::new(self.handle)?);
            }

            spi_transmit(
                self.handle,
                &mut transaction,
                self.polling,
                self.cs_pin_configured && words.peek().is_some(),
            )?;
        }

        Ok(())
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::Transactional<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn exec(
        &mut self,
        operations: &mut [embedded_hal_0_2::blocking::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.run(
            operations.iter_mut().map(|op| match op {
                embedded_hal_0_2::blocking::spi::Operation::Write(words) => Operation::Write(words),
                embedded_hal_0_2::blocking::spi::Operation::Transfer(words) => {
                    Operation::TransferInPlace(words)
                }
            }),
            SoftCsPin::none(),
        )
        .map_err(to_spi_err)
    }
}

pub struct SpiSharedDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    driver: UnsafeCell<SpiDeviceDriver<'d, T>>,
    lock: CriticalSection,
    async_lock: Mutex<EspRawMutex, ()>,
}

impl<'d, T> SpiSharedDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(driver: T, config: &config::Config) -> Result<Self, EspError> {
        Ok(Self::wrap(SpiDeviceDriver::new(
            driver,
            Option::<AnyOutputPin>::None,
            config,
        )?))
    }

    pub const fn wrap(device: SpiDeviceDriver<'d, T>) -> Self {
        Self {
            driver: UnsafeCell::new(device),
            lock: CriticalSection::new(),
            async_lock: Mutex::new(()),
        }
    }

    pub fn lock<R>(&self, f: impl FnOnce(&mut SpiDeviceDriver<'d, T>) -> R) -> R {
        let _guard = self.lock.enter();

        let device = unsafe { self.driver_mut() };

        f(device)
    }

    pub fn release(self) -> SpiDeviceDriver<'d, T> {
        self.driver.into_inner()
    }

    #[allow(clippy::mut_from_ref)]
    unsafe fn driver_mut(&self) -> &mut SpiDeviceDriver<'d, T> {
        &mut *self.driver.get()
    }
}

pub struct SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER> {
    shared_device: DEVICE,
    cs_pin: PinDriver<'d, AnyOutputPin, Output>,
    pre_delay_us: Option<u32>,
    post_delay_us: Option<u32>,
    _p: PhantomData<fn() -> DRIVER>,
}

impl<'d, DEVICE, DRIVER> SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>>,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    pub fn new(
        shared_device: DEVICE,
        cs: impl Peripheral<P = impl OutputPin> + 'd,
        cs_level: Level,
    ) -> Result<Self, EspError> {
        crate::into_ref!(cs);

        let mut cs_pin: PinDriver<AnyOutputPin, Output> = PinDriver::output(cs.map_into())?;

        cs_pin.set_level(cs_level)?;

        Ok(Self {
            shared_device,
            cs_pin,
            pre_delay_us: None,
            post_delay_us: None,
            _p: PhantomData,
        })
    }

    /// Add an aditional delay of x in uSeconds before transaction
    /// between chip select and first clk out
    pub fn cs_pre_delay_us(&mut self, delay_us: u32) -> &mut Self {
        self.pre_delay_us = Some(delay_us);

        self
    }

    /// Add an aditional delay of x in uSeconds after transaction
    /// between last clk out and chip select
    pub fn cs_post_delay_us(&mut self, delay_us: u32) -> &mut Self {
        self.post_delay_us = Some(delay_us);

        self
    }

    pub fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), EspError> {
        self.run(operations.iter_mut().map(copy_operation))
    }

    pub async fn transaction_async(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), EspError> {
        self.run_async(operations.iter_mut().map(copy_operation))
            .await
    }

    pub fn read(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Read(read)])
    }

    pub async fn read_async(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Read(read)]).await
    }

    pub fn read_transaction(&mut self, operations: &mut [&mut [u8]]) -> Result<(), EspError> {
        self.run(&mut operations.iter_mut().map(|slice| Operation::Read(slice)))
    }

    pub async fn read_transaction_async(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run_async(&mut operations.iter_mut().map(|slice| Operation::Read(slice)))
            .await
    }

    pub fn write(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Write(write)])
    }

    pub async fn write_async(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Write(write)]).await
    }

    pub fn write_transaction(&mut self, operations: &[&[u8]]) -> Result<(), EspError> {
        self.run(&mut operations.iter().map(|slice| Operation::Write(slice)))
    }

    pub async fn write_transaction_async(&mut self, operations: &[&[u8]]) -> Result<(), EspError> {
        self.run_async(&mut operations.iter().map(|slice| Operation::Write(slice)))
            .await
    }

    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::TransferInPlace(buf)])
    }

    pub async fn transfer_in_place_async(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::TransferInPlace(buf)])
            .await
    }

    pub fn transfer_in_place_transaction(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run(
            &mut operations
                .iter_mut()
                .map(|slice| Operation::TransferInPlace(slice)),
        )
    }

    pub async fn transfer_in_place_transaction_async(
        &mut self,
        operations: &mut [&mut [u8]],
    ) -> Result<(), EspError> {
        self.run_async(
            &mut operations
                .iter_mut()
                .map(|slice| Operation::TransferInPlace(slice)),
        )
        .await
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction(&mut [Operation::Transfer(read, write)])
    }

    pub async fn transfer_async(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction_async(&mut [Operation::Transfer(read, write)])
            .await
    }

    pub fn transfer_transaction(
        &mut self,
        operations: &mut [(&mut [u8], &[u8])],
    ) -> Result<(), EspError> {
        self.run(
            &mut operations
                .iter_mut()
                .map(|(read_slice, write_slice)| Operation::Transfer(read_slice, write_slice)),
        )
    }

    pub async fn transfer_transaction_async(
        &mut self,
        operations: &mut [(&mut [u8], &[u8])],
    ) -> Result<(), EspError> {
        self.run_async(
            &mut operations
                .iter_mut()
                .map(|(read_slice, write_slice)| Operation::Transfer(read_slice, write_slice)),
        )
        .await
    }

    fn run<'a>(
        &mut self,
        operations: impl Iterator<Item = Operation<'a, u8>> + 'a,
    ) -> Result<(), EspError> {
        let soft_cs_pin = SoftCsPin {
            cs: &mut self.cs_pin,
            pre_delay: self.pre_delay_us,
            post_delay: self.post_delay_us,
        };

        self.shared_device
            .borrow()
            .lock(move |device| device.run(operations, Some(soft_cs_pin)))
    }

    async fn run_async<'a>(
        &mut self,
        operations: impl Iterator<Item = Operation<'a, u8>> + 'a,
    ) -> Result<(), EspError> {
        let soft_cs_pin = SoftCsPin {
            cs: &mut self.cs_pin,
            pre_delay: self.pre_delay_us,
            post_delay: self.post_delay_us,
        };

        let device = self.shared_device.borrow();

        let _async_guard = device.async_lock.lock().await;
        let _guard = device.lock.enter();

        unsafe { device.driver_mut() }.run(operations, Some(soft_cs_pin))
    }
}

impl<'d, DEVICE, DRIVER> embedded_hal::spi::ErrorType for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;
}

impl<'d, DEVICE, DRIVER> SpiDeviceRead for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    fn read_transaction(&mut self, operations: &mut [&mut [u8]]) -> Result<(), Self::Error> {
        Self::read_transaction(self, operations).map_err(to_spi_err)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        Self::read(self, buf).map_err(to_spi_err)
    }
}

impl<'d, DEVICE, DRIVER> SpiDeviceWrite for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    fn write_transaction(&mut self, operations: &[&[u8]]) -> Result<(), Self::Error> {
        Self::write_transaction(self, operations).map_err(to_spi_err)
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        Self::write(self, buf).map_err(to_spi_err)
    }
}

impl<'d, DEVICE, DRIVER> SpiDevice for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        Self::transaction(self, operations).map_err(to_spi_err)
    }
}

fn to_spi_err(err: EspError) -> SpiError {
    SpiError::other(err)
}

// Limit to 64, as we sometimes allocate a buffer of size TRANS_LEN on the stack, so we have to keep it small
// SOC_SPI_MAXIMUM_BUFFER_SIZE equals 64 or 72 (esp32s2) anyway
const TRANS_LEN: usize = if SOC_SPI_MAXIMUM_BUFFER_SIZE < 64_u32 {
    SOC_SPI_MAXIMUM_BUFFER_SIZE as _
} else {
    64_usize
};

struct BusLock(spi_device_handle_t);

impl BusLock {
    fn new(device: spi_device_handle_t) -> Result<Self, EspError> {
        esp!(unsafe { spi_device_acquire_bus(device, BLOCK) })?;

        Ok(Self(device))
    }
}

impl Drop for BusLock {
    fn drop(&mut self) {
        unsafe {
            spi_device_release_bus(self.0);
        }
    }
}

struct SoftCsPin<'c, 'p, P, M>
where
    P: OutputPin,
    M: OutputMode,
{
    cs: &'c mut PinDriver<'p, P, M>,
    pre_delay: Option<u32>,
    post_delay: Option<u32>,
}

impl<'c, 'p, P, M> SoftCsPin<'c, 'p, P, M>
where
    P: OutputPin,
    M: OutputMode,
{
    fn raise(&mut self) -> Result<(), EspError> {
        self.cs.toggle()?;

        if let Some(delay) = self.pre_delay {
            Ets::delay_us(delay);
        }

        Ok(())
    }

    fn lower(&mut self) -> Result<(), EspError> {
        if let Some(delay) = self.post_delay {
            Ets::delay_us(delay);
        }

        self.cs.toggle()
    }
}

impl SoftCsPin<'static, 'static, AnyOutputPin, Output> {
    fn none() -> Option<Self> {
        None
    }
}

fn spi_read_transactions(
    words: &mut [u8],
    chunk_size: usize,
) -> impl Iterator<Item = spi_transaction_t> + '_ {
    words.chunks_mut(chunk_size).map(|chunk| {
        spi_create_transaction(
            chunk.as_mut_ptr(),
            core::ptr::null(),
            chunk.len(),
            chunk.len(),
        )
    })
}

fn spi_write_transactions(
    words: &[u8],
    chunk_size: usize,
) -> impl Iterator<Item = spi_transaction_t> + '_ {
    words
        .chunks(chunk_size)
        .map(|chunk| spi_create_transaction(core::ptr::null_mut(), chunk.as_ptr(), chunk.len(), 0))
}

fn spi_transfer_in_place_transactions(
    words: &mut [u8],
    chunk_size: usize,
) -> impl Iterator<Item = spi_transaction_t> + '_ {
    words.chunks_mut(chunk_size).map(|chunk| {
        spi_create_transaction(
            chunk.as_mut_ptr(),
            chunk.as_mut_ptr(),
            chunk.len(),
            chunk.len(),
        )
    })
}

fn spi_transfer_transactions<'a>(
    read: &'a mut [u8],
    write: &'a [u8],
    chunk_size: usize,
) -> impl Iterator<Item = spi_transaction_t> + 'a {
    enum OperationsIter<E, R, W> {
        Equal(E),
        ReadLonger(R),
        WriteLonger(W),
    }

    impl<E, R, W> Iterator for OperationsIter<E, R, W>
    where
        E: Iterator<Item = spi_transaction_t>,
        R: Iterator<Item = spi_transaction_t>,
        W: Iterator<Item = spi_transaction_t>,
    {
        type Item = spi_transaction_t;

        fn next(&mut self) -> Option<Self::Item> {
            match self {
                Self::Equal(iter) => iter.next(),
                Self::ReadLonger(iter) => iter.next(),
                Self::WriteLonger(iter) => iter.next(),
            }
        }
    }

    match read.len().cmp(&write.len()) {
        Ordering::Equal => {
            OperationsIter::Equal(spi_transfer_equal_transactions(read, write, chunk_size))
        }
        Ordering::Greater => {
            let (read, read_trail) = read.split_at_mut(write.len());

            OperationsIter::ReadLonger(
                spi_transfer_equal_transactions(read, write, chunk_size)
                    .chain(spi_read_transactions(read_trail, chunk_size)),
            )
        }
        Ordering::Less => {
            let (write, write_trail) = write.split_at(read.len());

            OperationsIter::WriteLonger(
                spi_transfer_equal_transactions(read, write, chunk_size)
                    .chain(spi_write_transactions(write_trail, chunk_size)),
            )
        }
    }
}

fn spi_transfer_equal_transactions<'a>(
    read: &'a mut [u8],
    write: &'a [u8],
    chunk_size: usize,
) -> impl Iterator<Item = spi_transaction_t> + 'a {
    read.chunks_mut(chunk_size)
        .zip(write.chunks(chunk_size))
        .map(|(read_chunk, write_chunk)| {
            spi_create_transaction(
                read_chunk.as_mut_ptr(),
                write_chunk.as_ptr(),
                max(read_chunk.len(), write_chunk.len()),
                read_chunk.len(),
            )
        })
}

// These parameters assume full duplex.
fn spi_create_transaction(
    read: *mut u8,
    write: *const u8,
    transaction_length: usize,
    rx_length: usize,
) -> spi_transaction_t {
    spi_transaction_t {
        flags: 0,
        __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
            tx_buffer: write as *const _,
        },
        __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
            rx_buffer: read as *mut _,
        },
        length: (transaction_length * 8) as _,
        rxlength: (rx_length * 8) as _,
        ..Default::default()
    }
}

fn spi_transmit(
    handle: spi_device_handle_t,
    transaction: &mut spi_transaction_t,
    polling: bool,
    _keep_cs_active: bool,
) -> Result<(), EspError> {
    // This unfortunately means that this implementation is incorrect for esp-idf < 4.4.
    // The CS pin should be kept active through transactions.
    #[cfg(not(esp_idf_version = "4.3"))]
    if _keep_cs_active {
        transaction.flags |= SPI_TRANS_CS_KEEP_ACTIVE
    }

    if polling {
        esp!(unsafe { spi_device_polling_transmit(handle, transaction as *mut _) })
    } else {
        esp!(unsafe { spi_device_transmit(handle, transaction as *mut _) })
    }
}

async fn spi_transmit_async(
    handle: spi_device_handle_t,
    transaction: &mut spi_transaction_t,
    _keep_cs_active: bool,
) -> Result<(), EspError> {
    // This unfortunately means that this implementation is incorrect for esp-idf < 4.4.
    // The CS pin should be kept active through transactions.
    #[cfg(not(esp_idf_version = "4.3"))]
    if _keep_cs_active {
        transaction.flags |= SPI_TRANS_CS_KEEP_ACTIVE
    }

    let notification = Notification::new();

    transaction.user = ptr::addr_of!(notification) as *mut _;

    match esp!(unsafe { spi_device_queue_trans(handle, transaction as *mut _, delay::NON_BLOCK) }) {
        Err(e) if e.code() == ESP_ERR_TIMEOUT => unreachable!(),
        other => other,
    }?;

    with_completion(
        async {
            notification.wait().await;

            match esp!(unsafe {
                spi_device_get_trans_result(handle, core::ptr::null_mut(), delay::NON_BLOCK)
            }) {
                Err(e) if e.code() == ESP_ERR_TIMEOUT => unreachable!(),
                other => other,
            }
        },
        |completed| {
            if !completed {
                esp!(unsafe {
                    spi_device_get_trans_result(handle, core::ptr::null_mut(), delay::BLOCK)
                })
                .unwrap();
            }
        },
    )
    .await
}

extern "C" fn spi_notify(transaction: *mut spi_transaction_t) {
    if let Some(transaction) = unsafe { transaction.as_ref() } {
        if let Some(notification) =
            unsafe { (transaction.user as *mut Notification as *const Notification).as_ref() }
        {
            notification.notify();
        }
    }
}

fn mark_last<I>(iter: I) -> impl Iterator<Item = (I::Item, bool)>
where
    I: Iterator,
{
    let iter = iter.peekable();

    struct Marked<I>(Peekable<I>)
    where
        I: Iterator;

    impl<I> Iterator for Marked<I>
    where
        I: Iterator,
    {
        type Item = (I::Item, bool);

        fn next(&mut self) -> Option<Self::Item> {
            if let Some(next) = self.0.next() {
                Some((next, self.0.peek().is_none()))
            } else {
                None
            }
        }
    }

    Marked(iter)
}

fn copy_operation<'b>(operation: &'b mut Operation<'_, u8>) -> Operation<'b, u8> {
    match operation {
        Operation::Read(read) => Operation::Read(read),
        Operation::Write(write) => Operation::Write(write),
        Operation::Transfer(read, write) => Operation::Transfer(read, write),
        Operation::TransferInPlace(write) => Operation::TransferInPlace(write),
    }
}

fn data_mode_to_u8(data_mode: embedded_hal::spi::Mode) -> u8 {
    (((data_mode.polarity == embedded_hal::spi::Polarity::IdleHigh) as u8) << 1)
        | ((data_mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition) as u8)
}

macro_rules! impl_spi {
    ($spi:ident: $device:expr) => {
        crate::impl_peripheral!($spi);

        impl Spi for $spi {
            #[inline(always)]
            fn device() -> spi_host_device_t {
                $device
            }
        }
    };
}

macro_rules! impl_spi_any_pins {
    ($spi:ident) => {
        impl SpiAnyPins for $spi {}
    };
}

impl_spi!(SPI1: spi_host_device_t_SPI1_HOST);
impl_spi!(SPI2: spi_host_device_t_SPI2_HOST);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_spi!(SPI3: spi_host_device_t_SPI3_HOST);

impl_spi_any_pins!(SPI2);
#[cfg(any(esp32, esp32s2, esp32s3))]
impl_spi_any_pins!(SPI3);
