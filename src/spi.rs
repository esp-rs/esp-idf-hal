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

use core::cmp::{max, min, Ordering};
use core::ptr;
use embedded_hal::spi::blocking::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite, SpiDevice};

use crate::delay::portMAX_DELAY;
use crate::gpio::{self, InputPin, OutputPin};

use esp_idf_sys::*;

crate::embedded_hal_error!(
    SpiError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);

pub trait Spi: Send {
    fn device() -> spi_host_device_t;
}

// Limit to 64, as we sometimes allocate a buffer of size TRANS_LEN on the stack, so we have to keep it small
// SOC_SPI_MAXIMUM_BUFFER_SIZE equals 64 or 72 (esp32s2) anyway
const TRANS_LEN: usize = if SOC_SPI_MAXIMUM_BUFFER_SIZE < 64_u32 {
    SOC_SPI_MAXIMUM_BUFFER_SIZE as _
} else {
    64_usize
};

#[derive(Copy, Clone)]
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

/// Pins used by the SPI interface
pub struct Pins<
    SCLK: OutputPin,
    SDO: OutputPin,
    // default pins to allow type inference
    SDI: InputPin + OutputPin = crate::gpio::Gpio1<crate::gpio::Input>,
    CS: OutputPin = crate::gpio::Gpio2<crate::gpio::Output>,
> {
    pub sclk: SCLK,
    pub sdo: SDO,
    pub sdi: Option<SDI>,
    pub cs: Option<CS>,
}

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

/// Master SPI abstraction
pub struct Master<
    SPI: Spi,
    SCLK: OutputPin,
    SDO: OutputPin,
    // default pins to allow type inference
    SDI: InputPin + OutputPin = crate::gpio::Gpio1<crate::gpio::Input>,
    CS: OutputPin = crate::gpio::Gpio2<crate::gpio::Output>,
> {
    spi: SPI,
    pins: Pins<SCLK, SDO, SDI, CS>,
    device: spi_device_handle_t,
    max_transfer_size: usize,
}

unsafe impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Send for Master<SPI, SCLK, SDO, SDI, CS>
{
}

impl<CS: OutputPin>
    Master<SPI1, gpio::Gpio6<gpio::Output>, gpio::Gpio7<gpio::Output>, gpio::Gpio8<gpio::Input>, CS>
{
    /// Create new instance of SPI controller for SPI1
    ///
    /// SPI1 can only use fixed pin for SCLK, SDO and SDI as they are shared with SPI0.
    pub fn new(
        spi: SPI1,
        pins: Pins<
            gpio::Gpio6<gpio::Output>,
            gpio::Gpio7<gpio::Output>,
            gpio::Gpio8<gpio::Input>,
            CS,
        >,
        config: config::Config,
    ) -> Result<Self, EspError> {
        Master::new_internal(spi, pins, config)
    }
}

impl<SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Master<SPI2, SCLK, SDO, SDI, CS>
{
    /// Create new instance of SPI controller for SPI2
    pub fn new(
        spi: SPI2,
        pins: Pins<SCLK, SDO, SDI, CS>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        Master::new_internal(spi, pins, config)
    }
}

#[cfg(not(esp32c3))]
impl<SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Master<SPI3, SCLK, SDO, SDI, CS>
{
    /// Create new instance of SPI controller for SPI3
    pub fn new(
        spi: SPI3,
        pins: Pins<SCLK, SDO, SDI, CS>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        Master::new_internal(spi, pins, config)
    }
}

struct Lock(spi_device_handle_t);

impl Lock {
    fn new(device: spi_device_handle_t) -> Result<Self, EspError> {
        esp!(unsafe { spi_device_acquire_bus(device, portMAX_DELAY) })?;

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

pub struct MasterBus {
    handle: spi_device_handle_t,
    trans_len: usize,
}

// These parameters assume full duplex.
fn polling_transmit(
    handle: spi_device_handle_t,
    read: *mut u8,
    write: *const u8,
    transaction_length: usize,
    rx_length: usize,
    _keep_cs_active: bool,
) -> Result<(), SpiError> {
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
        .map_err(SpiError::other)
}

impl MasterBus {
    fn polling_transmit(
        &mut self,
        read: *mut u8,
        write: *const u8,
        transaction_length: usize,
        rx_length: usize,
    ) -> Result<(), SpiError> {
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
    fn finish(&mut self) -> Result<(), SpiError> {
        polling_transmit(self.handle, ptr::null_mut(), ptr::null(), 0, 0, false)
    }
}

impl embedded_hal::spi::ErrorType for MasterBus {
    type Error = SpiError;
}

impl SpiBusFlush for MasterBus {
    fn flush(&mut self) -> Result<(), Self::Error> {
        // Since we use polling transactions, flushing isn't required.
        // In future, when DMA is available spi_device_get_trans_result
        // will be called here.
        Ok(())
    }
}

impl SpiBusRead for MasterBus {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in words.chunks_mut(self.trans_len) {
            self.polling_transmit(chunk.as_mut_ptr(), ptr::null(), chunk.len(), chunk.len())?;
        }
        Ok(())
    }
}

impl SpiBusWrite for MasterBus {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for chunk in words.chunks(self.trans_len) {
            self.polling_transmit(ptr::null_mut(), chunk.as_ptr(), chunk.len(), 0)?;
        }
        Ok(())
    }
}

impl SpiBus for MasterBus {
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
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

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in words.chunks_mut(self.trans_len) {
            let ptr = chunk.as_mut_ptr();
            let len = chunk.len();
            self.polling_transmit(ptr, ptr, len, len)?;
        }
        Ok(())
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Master<SPI, SCLK, SDO, SDI, CS>
{
    /// Internal implementation of new shared by all SPI controllers
    fn new_internal(
        spi: SPI,
        pins: Pins<SCLK, SDO, SDI, CS>,
        config: config::Config,
    ) -> Result<Self, EspError> {
        #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: pins.sclk.pin(),

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: pins.sdo.pin(),
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: pins.sdi.as_ref().map_or(-1, |p| p.pin()),
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
            max_transfer_sz: config.dma.max_transfer_size() as i32,
            ..Default::default()
        };

        #[cfg(not(any(esp_idf_version = "4.4", esp_idf_version_major = "5")))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: pins.sclk.pin(),

            mosi_io_num: pins.sdo.pin(),
            miso_io_num: pins.sdi.as_ref().map_or(-1, |p| p.pin()),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            max_transfer_sz: config.dma.max_transfer_size() as i32,
            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(SPI::device(), &bus_config, config.dma.into()) })?;

        let device_config = spi_device_interface_config_t {
            spics_io_num: pins.cs.as_ref().map_or(-1, |p| p.pin()),
            clock_speed_hz: config.baudrate.0 as i32,
            mode: (if config.data_mode.polarity == embedded_hal::spi::Polarity::IdleHigh {
                2
            } else {
                0
            }) | (if config.data_mode.phase
                == embedded_hal::spi::Phase::CaptureOnSecondTransition
            {
                1
            } else {
                0
            }),
            queue_size: 64,
            flags: if config.write_only {
                SPI_DEVICE_NO_DUMMY
            } else {
                0_u32
            },
            ..Default::default()
        };

        let mut device_handle: spi_device_handle_t = ptr::null_mut();

        esp!(unsafe {
            spi_bus_add_device(SPI::device(), &device_config, &mut device_handle as *mut _)
        })?;

        Ok(Self {
            spi,
            pins,
            device: device_handle,
            max_transfer_size: config.dma.max_transfer_size(),
        })
    }

    /// Release and return the raw interface to the underlying SPI peripheral
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> Result<(SPI, Pins<SCLK, SDO, SDI, CS>), EspError> {
        esp!(unsafe { spi_bus_remove_device(self.device) })?;
        esp!(unsafe { spi_bus_free(SPI::device()) })?;

        Ok((self.spi, self.pins))
    }

    fn lock_bus(&mut self) -> Result<Lock, SpiError> {
        Lock::new(self.device).map_err(SpiError::other)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal::spi::ErrorType for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin> SpiDevice
    for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Bus = MasterBus;

    fn transaction<R>(
        &mut self,
        f: impl FnOnce(&mut Self::Bus) -> Result<R, <Self::Bus as embedded_hal::spi::ErrorType>::Error>,
    ) -> Result<R, Self::Error> {
        let mut bus = MasterBus {
            handle: self.device,
            trans_len: self.max_transfer_size,
        };

        let lock = self.lock_bus()?;
        let trans_result = f(&mut bus);

        let finish_result = bus.finish();

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        let flush_result = bus.flush();

        core::mem::drop(lock);

        let result = trans_result?;
        finish_result?;
        flush_result?;
        Ok(result)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal_0_2::blocking::spi::Transfer<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        let _lock = self.lock_bus();
        let mut chunks = words.chunks_mut(self.max_transfer_size).peekable();
        while let Some(chunk) = chunks.next() {
            let ptr = chunk.as_mut_ptr();
            let len = chunk.len();
            polling_transmit(self.device, ptr, ptr, len, len, chunks.peek().is_some())?;
        }

        Ok(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal_0_2::blocking::spi::Write<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let _lock = self.lock_bus();
        let mut chunks = words.chunks(self.max_transfer_size).peekable();
        while let Some(chunk) = chunks.next() {
            polling_transmit(
                self.device,
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

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal_0_2::blocking::spi::WriteIter<u8> for Master<SPI, SCLK, SDO, SDI, CS>
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

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal_0_2::blocking::spi::Transactional<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;

    fn exec<'a>(
        &mut self,
        operations: &mut [embedded_hal_0_2::blocking::spi::Operation<'a, u8>],
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

macro_rules! impl_spi {
    ($spi:ident: $device:expr) => {
        pub struct $spi(::core::marker::PhantomData<*const ()>);

        impl $spi {
            /// # Safety
            ///
            /// Care should be taken not to instantiate this SPI instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $spi(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $spi {}

        impl Spi for $spi {
            #[inline(always)]
            fn device() -> spi_host_device_t {
                $device
            }
        }
    };
}

impl_spi!(SPI1: spi_host_device_t_SPI1_HOST);
impl_spi!(SPI2: spi_host_device_t_SPI2_HOST);

#[cfg(not(esp32c3))]
impl_spi!(SPI3: spi_host_device_t_SPI3_HOST);
