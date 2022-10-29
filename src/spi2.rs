//! SPI peripheral control
//!
//! Currently only implements full duplex controller mode support.
//!
//! SPI0 is reserved for accessing flash and sram and therefore not usable for other purposes.
//! SPI1 shares its external pins with SPI0 and therefore has severe restrictions in use.
//!
//! SPI2 & 3 can be used freely.
//!
//! The CS pin is controlled by hardware on esp32 (contrary to the description of embedded_hal).
//!
//! The [Transfer::transfer], [Write::write] and [WriteIter::write_iter] functions lock the
//! APB frequency and therefore the requests are always run at the requested baudrate.
//! The primitive [FullDuplex::read] and [FullDuplex::send] do not lock the APB frequency and
//! therefore may run at a different frequency.
//!
//! # TODO
//! - Quad SPI
//! - DMA
//! - Multiple CS pins
//! - Slave

// TODO -> transfer size & DMA
// update usage example
// add update device config Fn

use core::cmp::{max, min, Ordering};
use core::marker::PhantomData;
use core::ptr;

use std::borrow::Borrow;
use std::cell::RefCell;
use std::sync::Mutex;

use embedded_hal::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite, SpiDevice};

use esp_idf_sys::*;

use crate::delay::BLOCK;
use crate::gpio::{self, InputPin, OutputPin};
use crate::peripheral::{Peripheral, PeripheralRef};

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
    const fn max_transfer_size(&self) -> usize {
        let max_transfer_size = match self {
            Dma::Disabled => TRANS_LEN,
            Dma::Channel1(size) | Dma::Channel2(size) | Dma::Auto(size) => *size,
        };
        if max_transfer_size % 4 != 0 {
            panic!("The max transfer size must a multiple of 4")
        } else if max_transfer_size > 4096 {
            4096
        } else {
            max_transfer_size
        }
    }
}

pub type SpiMasterConfig = config::Config;

/// SPI configuration
pub mod config {
    use crate::spi::Dma;
    use crate::units::*;

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

    /// SPI configuration
    #[derive(Copy, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_mode: embedded_hal::spi::Mode,
        /// This property can be set to configure a SPI Device for being write only
        /// Thus the flag SPI_DEVICE_NO_DUMMY will be passed on initialization and
        /// it will unlock the possibility of using 80Mhz as the bus freq
        /// See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html#timing-considerations
        pub write_only: bool,
        pub dma: Dma,
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

        pub fn dma(mut self, dma: Dma) -> Self {
            self.dma = dma;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                data_mode: embedded_hal::spi::MODE_0,
                write_only: false,
                dma: Dma::Disabled,
            }
        }
    }
}

pub struct SpiBusMasterDriver<'d> {
    handle: spi_device_handle_t,
    trans_len: usize,
    _p: PhantomData<&'d ()>,
}

impl<'d> SpiBusMasterDriver<'d> {
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
            self.handle,
            read,
            write,
            transaction_length,
            rx_length,
            true,
        )
    }

    /// Empty transaction to de-assert CS.
    fn finish(&mut self) -> Result<(), EspError> {
        polling_transmit(self.handle, ptr::null_mut(), ptr::null(), 0, 0, false)
    }
}

impl<'d> embedded_hal::spi::ErrorType for SpiBusMasterDriver<'d> {
    type Error = SpiError;
}

impl<'d> SpiBusFlush for SpiBusMasterDriver<'d> {
    fn flush(&mut self) -> Result<(), Self::Error> {
        SpiBusMasterDriver::flush(self).map_err(to_spi_err)
    }
}

impl<'d> SpiBusRead for SpiBusMasterDriver<'d> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::read(self, words).map_err(to_spi_err)
    }
}

impl<'d> SpiBusWrite for SpiBusMasterDriver<'d> {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::write(self, words).map_err(to_spi_err)
    }
}

impl<'d> SpiBus for SpiBusMasterDriver<'d> {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::transfer(self, read, write).map_err(to_spi_err)
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::transfer_in_place(self, words).map_err(to_spi_err)
    }
}

pub struct SpiMasterDriver<'d> {
    handle: Mutex<spi_host_device_t>,
    _p: PhantomData<&'d ()>,
}

impl<'d> SpiMasterDriver<'d> {
    pub fn new_spi1<SPI: Spi>(
        _spi: impl Peripheral<P = SPI1> + 'd,
        sclk: impl Peripheral<P = gpio::Gpio6> + 'd,
        sdo: impl Peripheral<P = gpio::Gpio7> + 'd,
        sdi: Option<impl Peripheral<P = gpio::Gpio8> + 'd>,
    ) -> Result<Self, EspError> {
        Self::new_internal_bus::<SPI>(sclk, sdo, sdi)?;
        Ok(Self {
            handle: Mutex::new(SPI::device()),
            _p: PhantomData,
        })
    }

    pub fn new<SPI: SpiAnyPins>(
        _spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
    ) -> Result<Self, EspError> {
        Self::new_internal_bus::<SPI>(sclk, sdo, sdi)?;
        Ok(Self {
            handle: Mutex::new(SPI::device()),
            _p: PhantomData,
        })
    }
    fn new_internal_bus<SPI: Spi>(
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
    ) -> Result<(), EspError> {
        crate::into_ref!(sclk, sdo);
        let sdi = sdi.map(|sdi| sdi.into_ref());

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

            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(SPI::device(), &bus_config, Dma::Disabled.into()) })?;
        Ok(())
    }
}

impl<'d> Drop for SpiMasterDriver<'d> {
    fn drop(&mut self) {
        if let Ok(lock) = self.handle.lock() {
            esp!(unsafe { spi_bus_free(*lock) }).unwrap();
        }
    }
}

pub struct EspSpiDevice<'d, T> {
    handle: RefCell<Option<spi_device_handle_t>>,
    id: i32,
    config: config::Config,
    con: spi_device_interface_config_t,
    //driver: Weak<&'b SpiMasterDriver<'d>>,
    driver: T,
    _p: PhantomData<&'d ()>,
}

impl<'d, T> EspSpiDevice<'d, T> {
    pub fn new(
        driver: T,
        cs: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        config: config::Config,
    ) -> Result<EspSpiDevice<'d, T>, EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        let cs = cs.into_ref().pin();
        let con = Self::create_conf(cs, &config);
        let me = Self {
            handle: RefCell::new(None),
            id: cs,
            config,
            con,
            driver,
            _p: PhantomData,
        };
        me.add_to_bus()?;
        Ok(me)
    }

    fn create_conf(
        cs: i32,
        config: &config::Config,
    ) -> spi_device_interface_config_t {
        spi_device_interface_config_t {
            spics_io_num: cs,
            clock_speed_hz: config.baudrate.0 as i32,
            mode: (((config.data_mode.polarity == embedded_hal::spi::Polarity::IdleHigh) as u8)
                << 1)
                | ((config.data_mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition)
                    as u8),
            queue_size: 64,
            flags: if config.write_only {
                SPI_DEVICE_NO_DUMMY
            } else {
                0_u32
            },
            ..Default::default()
        }
    }

    pub fn get_id(&self) -> i32 {
        self.id
    }

    pub fn add_to_bus(&self) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        let master: &SpiMasterDriver = self.driver.borrow();
        if let Ok(lock) = master.handle.lock() {
            let mut device_handle: spi_device_handle_t = ptr::null_mut();
            esp!(unsafe { spi_bus_add_device(*lock, &self.con, &mut device_handle as *mut _) })?;
            self.handle.replace(Some(device_handle));
        }
        Ok(())
    }

    pub fn rm_from_bus(&self) -> Result<(), EspError> {
        let handle = self.handle.replace(None);
        match handle {
            Some(handle) => {
                esp!(unsafe { spi_bus_remove_device(handle) })?;
            }
            None => {
                println!("SPI MASTER ERROR: Removed handle from slave who had already no handle")
            }
        }
        Ok(())
    }

    pub fn transaction<R, E>(
        &self,
        f: impl FnOnce(&mut SpiBusMasterDriver) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        // lock the bus through the mutex in SpiMasterDriver
        let master: &SpiMasterDriver = self.driver.borrow();
        // should we check for poison here?
        let driver_lock = master.handle.lock().unwrap();

        let handle = self.handle.clone().into_inner().unwrap();
        let mut bus = SpiBusMasterDriver {
            handle,
            trans_len: SOC_SPI_MAXIMUM_BUFFER_SIZE as usize,
            _p: PhantomData,
        };

        let lock = Self::lock_bus(self.handle.clone().into_inner().unwrap())?;

        let trans_result = f(&mut bus);

        let finish_result = bus.finish();

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        let flush_result = bus.flush();

        core::mem::drop(lock);
        drop(driver_lock);
        let result = trans_result?;
        finish_result?;
        flush_result?;

        Ok(result)
    }

    pub fn transfer(&self, read: &mut [u8], write: &[u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.transfer(read, write))
    }

    pub fn write(&self, write: &[u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.write(write))
    }

    pub fn read(&self, read: &mut [u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.read(read))
    }

    pub fn transfer_in_place(&self, buf: &mut [u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.transfer_in_place(buf))
    }

    fn lock_bus(handle: spi_device_handle_t) -> Result<Lock, EspError> {
        Lock::new(handle)
    }
}

impl<'d, T> Drop for EspSpiDevice<'d, T> {
    fn drop(&mut self) {
        self.rm_from_bus().unwrap();
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
