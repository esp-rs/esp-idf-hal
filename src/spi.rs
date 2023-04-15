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
//! |   | managed cs       |       hardware       |              -               |     software triggered     |         -         |
//! |   | 1 device         |           x          |              x               |              x             |         x         |
//! |   | 1-3 devices      |           x          |              -               |              x             |         -         |
//! |   | 4-6 devices      |    only on esp32c*   |              -               |              x             |         -         |
//! |   | more than 6      |           -          |              -               |              x             |         -         |
//! |   | Dma              |           -          |              -               |              -             |         -         |
//! |   | polling transmit |           x          |              x               |              x             |         x         |
//! |   | isr transmit     |           -          |              -               |              -             |         -         |
//! |   | async ready      |           -          |              -               |              -             |         -         |
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
use core::cmp::{max, min, Ordering};
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite, SpiDevice};

use esp_idf_sys::*;

use crate::delay::{Ets, BLOCK};
use crate::gpio::{AnyOutputPin, InputPin, Level, Output, OutputPin, PinDriver};
use crate::peripheral::Peripheral;
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
            x if x % 4 != 0 => panic!("The max transfer size must be a multiple of 4"),
            x if x == 0 => panic!("The max transfer size must be greater than 0"),
            x if x > 4096 => panic!("The max transfer size must be less than or equal to 4096"),
            _ => max_transfer_size,
        }
    }
}

pub type SpiConfig = config::Config;

/// SPI configuration
pub mod config {
    use crate::units::*;
    use esp_idf_sys::*;

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
    #[derive(Copy, Clone)]
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
    #[derive(Copy, Clone)]
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

    /// SPI Device configuration
    #[derive(Copy, Clone)]
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

        pub fn write_only(mut self, write_only: bool) -> Self {
            self.write_only = write_only;
            self
        }

        pub fn duplex(mut self, duplex: Duplex) -> Self {
            self.duplex = duplex;
            self
        }

        pub fn bit_order(mut self, bit_order: BitOrder) -> Self {
            self.bit_order = bit_order;
            self
        }

        pub fn cs_active_high(mut self) -> Self {
            self.cs_active_high = true;
            self
        }

        pub fn input_delay_ns(mut self, input_delay_ns: i32) -> Self {
            self.input_delay_ns = input_delay_ns;
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
            }
        }
    }
}

pub struct SpiBusDriver<T> {
    _lock: Lock,
    handle: Device,
    _driver: T,
    trans_len: usize,
    hardware_cs: bool,
}

impl<T> SpiBusDriver<T> {
    pub fn new<'d>(driver: T, config: &config::Config) -> Result<Self, EspError>
    where
        T: BorrowMut<SpiDriver<'d>>,
    {
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
            ..Default::default()
        };

        let mut handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(driver.borrow().host(), &conf, &mut handle as *mut _) })?;

        let device = Device(handle, true);

        let lock = Lock::new(handle)?;

        let trans_len = driver.borrow().max_transfer_size;

        Ok(Self {
            _lock: lock,
            handle: device,
            _driver: driver,
            trans_len,
            hardware_cs: false,
        })
    }

    pub fn read(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        for chunk in words.chunks_mut(self.trans_len) {
            self.polling_transmit(chunk.as_mut_ptr(), ptr::null(), chunk.len(), chunk.len())?;
        }

        Ok(())
    }

    pub fn write(&mut self, words: &[u8]) -> Result<(), EspError> {
        for chunk in words.chunks(self.trans_len) {
            self.polling_transmit(ptr::null_mut(), chunk.as_ptr(), chunk.len(), 0)?;
        }

        Ok(())
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        let common_length = min(read.len(), write.len());
        let common_read = read[0..common_length].chunks_mut(self.trans_len);
        let common_write = write[0..common_length].chunks(self.trans_len);

        for (read_chunk, write_chunk) in common_read.zip(common_write) {
            self.polling_transmit(
                read_chunk.as_mut_ptr(),
                write_chunk.as_ptr(),
                max(read_chunk.len(), write_chunk.len()),
                read_chunk.len(),
            )?;
        }

        match read.len().cmp(&write.len()) {
            Ordering::Equal => { /* Nothing left to do */ }
            Ordering::Greater => {
                // Read remainder
                self.read(&mut read[write.len()..])?;
            }
            Ordering::Less => {
                // Write remainder
                self.write(&write[read.len()..])?;
            }
        }

        Ok(())
    }

    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), EspError> {
        for chunk in words.chunks_mut(self.trans_len) {
            let ptr = chunk.as_mut_ptr();
            let len = chunk.len();
            self.polling_transmit(ptr, ptr, len, len)?;
        }

        Ok(())
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        // Since we use polling transactions, flushing isn't required.
        // In future, when DMA is available spi_device_get_trans_result
        // will be called here.
        Ok(())
    }

    fn polling_transmit(
        &mut self,
        read: *mut u8,
        write: *const u8,
        transaction_length: usize,
        rx_length: usize,
    ) -> Result<(), EspError> {
        polling_transmit(
            self.handle.0,
            read,
            write,
            transaction_length,
            rx_length,
            self.hardware_cs,
        )
    }

    /// Empty transaction to de-assert CS.
    fn finish(&mut self) -> Result<(), EspError> {
        polling_transmit(self.handle.0, ptr::null_mut(), ptr::null(), 0, 0, false)
    }
}

impl<T> embedded_hal::spi::ErrorType for SpiBusDriver<T> {
    type Error = SpiError;
}

impl<T> SpiBusFlush for SpiBusDriver<T> {
    fn flush(&mut self) -> Result<(), Self::Error> {
        SpiBusDriver::flush(self).map_err(to_spi_err)
    }
}

impl<T> SpiBusRead for SpiBusDriver<T> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusDriver::read(self, words).map_err(to_spi_err)
    }
}

impl<T> SpiBusWrite for SpiBusDriver<T> {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        SpiBusDriver::write(self, words).map_err(to_spi_err)
    }
}

impl<T> SpiBus for SpiBusDriver<T> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        SpiBusDriver::transfer(self, read, write).map_err(to_spi_err)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusDriver::transfer_in_place(self, words).map_err(to_spi_err)
    }
}

struct Device(spi_device_handle_t, bool);

impl Drop for Device {
    fn drop(&mut self) {
        if self.1 {
            esp!(unsafe { spi_bus_remove_device(self.0) }).unwrap();
        }
    }
}

pub struct SpiDriver<'d> {
    host: u8,
    max_transfer_size: usize,
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
        dma: Dma,
    ) -> Result<Self, EspError> {
        let max_transfer_size = Self::new_internal(SPI1::device(), sclk, sdo, sdi, dma)?;

        Ok(Self {
            host: SPI1::device() as _,
            max_transfer_size,
            _p: PhantomData,
        })
    }

    /// Create new instance of SPI controller for all others
    pub fn new<SPI: SpiAnyPins>(
        _spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        dma: Dma,
    ) -> Result<Self, EspError> {
        let max_transfer_size = Self::new_internal(SPI::device(), sclk, sdo, sdi, dma)?;

        Ok(Self {
            host: SPI::device() as _,
            max_transfer_size,
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
        dma: Dma,
    ) -> Result<usize, EspError> {
        crate::into_ref!(sclk, sdo);
        let sdi = sdi.map(|sdi| sdi.into_ref());

        let max_transfer_sz = dma.max_transfer_size();
        let dma_chan: spi_dma_chan_t = dma.into();

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
            ..Default::default()
        };

        #[cfg(esp_idf_version = "4.3")]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk.pin(),

            mosi_io_num: sdo.pin(),
            miso_io_num: sdi.as_ref().map_or(-1, |p| p.pin()),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            max_transfer_sz: max_transfer_sz as i32,
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

pub type SpiSingleDeviceDriver<'d> = SpiDeviceDriver<'d, SpiDriver<'d>>;

pub struct SpiDeviceDriver<'d, T> {
    handle: Device,
    driver: T,
    with_cs_pin: bool,
    _p: PhantomData<&'d ()>,
}

impl<'d> SpiDeviceDriver<'d, SpiDriver<'d>> {
    #[cfg(esp32)]
    pub fn new_single_spi1(
        spi: impl Peripheral<P = SPI1> + 'd,
        sclk: impl Peripheral<P = crate::gpio::Gpio6> + 'd,
        sdo: impl Peripheral<P = crate::gpio::Gpio7> + 'd,
        sdi: Option<impl Peripheral<P = crate::gpio::Gpio8> + 'd>,
        dma: Dma,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        Self::new(SpiDriver::new_spi1(spi, sclk, sdo, sdi, dma)?, cs, config)
    }

    pub fn new_single<SPI: SpiAnyPins>(
        spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        dma: Dma,
        cs: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        Self::new(SpiDriver::new(spi, sclk, sdo, sdi, dma)?, cs, config)
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
            ..Default::default()
        };

        let mut handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(driver.borrow().host(), &conf, &mut handle as *mut _) })?;

        Ok(Self {
            handle: Device(handle, true),
            driver,
            with_cs_pin: cs >= 0,
            _p: PhantomData,
        })
    }

    pub fn device(&self) -> spi_device_handle_t {
        self.handle.0
    }

    pub fn transaction<R, E>(
        &mut self,
        f: impl FnOnce(&mut SpiBusDriver<()>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
    {
        // if DMA used -> get trans length info from driver
        let trans_len = self.driver.borrow().max_transfer_size;

        let mut bus = SpiBusDriver {
            _lock: self.lock_bus()?,
            handle: Device(self.handle.0, false),
            _driver: (),
            trans_len,
            hardware_cs: self.with_cs_pin,
        };

        let trans_result = f(&mut bus);

        // #99 is partially resolved by allowing software CS to ignore this bus.finish() work around
        let finish_result = if self.with_cs_pin {
            bus.finish()
        } else {
            Ok(())
        };

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        let flush_result = bus.flush();

        drop(bus);

        let result = trans_result?;
        finish_result?;
        flush_result?;

        Ok(result)
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer(read, write))
    }

    pub fn write(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.write(write))
    }

    pub fn read(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.read(read))
    }

    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer_in_place(buf))
    }

    fn lock_bus(&self) -> Result<Lock, EspError> {
        Lock::new(self.handle.0)
    }
}

unsafe impl<'d, T> Send for SpiDeviceDriver<'d, T> where T: Send {}

impl<'d, T> embedded_hal::spi::ErrorType for SpiDeviceDriver<'d, T> {
    type Error = SpiError;
}

impl<'d, T> SpiDevice for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Bus = SpiBusDriver<()>;

    fn transaction<R>(
        &mut self,
        f: impl FnOnce(&mut Self::Bus) -> Result<R, <Self::Bus as embedded_hal::spi::ErrorType>::Error>,
    ) -> Result<R, Self::Error> {
        Self::transaction(self, f)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::Transfer<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let _lock = self.lock_bus()?;
        let mut chunks = words
            .chunks_mut(self.driver.borrow().max_transfer_size)
            .peekable();

        while let Some(chunk) = chunks.next() {
            let ptr = chunk.as_mut_ptr();
            let len = chunk.len();
            polling_transmit(self.handle.0, ptr, ptr, len, len, chunks.peek().is_some())?;
        }

        Ok(words)
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::Write<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let _lock = self.lock_bus()?;
        let mut chunks = words
            .chunks(self.driver.borrow().max_transfer_size)
            .peekable();

        while let Some(chunk) = chunks.next() {
            polling_transmit(
                self.handle.0,
                ptr::null_mut(),
                chunk.as_ptr(),
                chunk.len(),
                0,
                chunks.peek().is_some(),
            )?;
        }

        Ok(())
    }
}

impl<'d, T> embedded_hal_0_2::blocking::spi::WriteIter<u8> for SpiDeviceDriver<'d, T>
where
    T: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u8>,
    {
        let mut words = words.into_iter();
        let mut buf = [0_u8; TRANS_LEN];

        self.transaction(|bus| {
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

                bus.write(&buf[..offset])?;
            }

            Ok(())
        })
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
        self.transaction(|bus| {
            for operation in operations {
                match operation {
                    embedded_hal_0_2::blocking::spi::Operation::Write(write) => bus.write(write),
                    embedded_hal_0_2::blocking::spi::Operation::Transfer(words) => {
                        bus.transfer_in_place(words)
                    }
                }?;
            }

            Ok(())
        })
    }
}

pub struct SpiSharedDeviceDriver<'d, T> {
    driver: UnsafeCell<SpiDeviceDriver<'d, T>>,
    cs: CriticalSection,
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
            cs: CriticalSection::new(),
        }
    }

    pub fn lock<R>(&self, f: impl FnOnce(&mut SpiDeviceDriver<'d, T>) -> R) -> R {
        let _guard = self.cs.enter();

        let device = unsafe { &mut *self.driver.get() };

        f(device)
    }

    pub fn release(self) -> SpiDeviceDriver<'d, T> {
        self.driver.into_inner()
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

    pub fn transaction<R, E>(
        &mut self,
        f: impl FnOnce(&mut SpiBusDriver<()>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
    {
        let cs_pin = &mut self.cs_pin;
        let pre_delay_us = self.pre_delay_us;
        let post_delay_us = self.post_delay_us;

        self.shared_device.borrow().lock(|device| {
            cs_pin.toggle()?;

            if let Some(delay) = pre_delay_us {
                Ets::delay_us(delay);
            }

            let trans_result = device.transaction(f);

            if let Some(delay) = post_delay_us {
                Ets::delay_us(delay);
            }

            cs_pin.toggle()?;

            trans_result
        })
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer(read, write))
    }

    pub fn write(&mut self, write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.write(write))
    }

    pub fn read(&mut self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.read(read))
    }

    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer_in_place(buf))
    }
}

impl<'d, DEVICE, DRIVER> embedded_hal::spi::ErrorType for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    type Error = SpiError;
}

impl<'d, DEVICE, DRIVER> SpiDevice for SpiSoftCsDeviceDriver<'d, DEVICE, DRIVER>
where
    DEVICE: Borrow<SpiSharedDeviceDriver<'d, DRIVER>> + 'd,
    DRIVER: Borrow<SpiDriver<'d>> + 'd,
{
    type Bus = SpiBusDriver<()>;

    fn transaction<R>(
        &mut self,
        f: impl FnOnce(&mut Self::Bus) -> Result<R, <Self::Bus as embedded_hal::spi::ErrorType>::Error>,
    ) -> Result<R, Self::Error> {
        Self::transaction(self, f)
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

struct Lock(spi_device_handle_t);

impl Lock {
    fn new(device: spi_device_handle_t) -> Result<Self, EspError> {
        esp!(unsafe { spi_device_acquire_bus(device, BLOCK) })?;

        Ok(Self(device))
    }
}

impl Drop for Lock {
    fn drop(&mut self) {
        unsafe {
            spi_device_release_bus(self.0);
        }
    }
}

// These parameters assume full duplex.
fn polling_transmit(
    handle: spi_device_handle_t,
    read: *mut u8,
    write: *const u8,
    transaction_length: usize,
    rx_length: usize,
    _keep_cs_active: bool,
) -> Result<(), EspError> {
    #[cfg(esp_idf_version = "4.3")]
    let flags = 0;

    // This unfortunately means that this implementation is incorrect for esp-idf < 4.4.
    // The CS pin should be kept active through transactions.
    #[cfg(not(esp_idf_version = "4.3"))]
    let flags = if _keep_cs_active {
        SPI_TRANS_CS_KEEP_ACTIVE
    } else {
        0
    };

    let mut transaction = spi_transaction_t {
        flags,
        __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
            tx_buffer: write as *const _,
        },
        __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
            rx_buffer: read as *mut _,
        },
        length: (transaction_length * 8) as _,
        rxlength: (rx_length * 8) as _,
        ..Default::default()
    };

    esp!(unsafe { spi_device_polling_transmit(handle, &mut transaction as *mut _) })
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
#[cfg(not(esp32c3))]
impl_spi!(SPI3: spi_host_device_t_SPI3_HOST);

impl_spi_any_pins!(SPI2);
#[cfg(not(esp32c3))]
impl_spi_any_pins!(SPI3);
