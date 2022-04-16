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
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                data_mode: embedded_hal::spi::MODE_0,
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

impl MasterBus {
    // These parameters assume full duplex.
    fn polling_transmit(
        &mut self,
        read: *mut u8,
        write: *const u8,
        transaction_length: usize,
        rx_length: usize,
    ) -> Result<(), SpiError> {
        let flags = 0;

        // This unfortunately means that this implementation is incorrect for esp-idf < 4.4.
        // The CS pin should be kept active through transactions.
        #[cfg(not(esp_idf_version = "4.3"))]
        let flags = SPI_TRANS_CS_KEEP_ACTIVE;

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

        esp!(unsafe { spi_device_polling_transmit(self.handle, &mut transaction as *mut _) })
            .map_err(SpiError::other)
    }

    /// Empty transaction to de-assert CS.
    fn finish(&mut self) -> Result<(), SpiError> {
        let flags = 0;

        let mut transaction = spi_transaction_t {
            flags,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: ptr::null(),
            },
            __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
                rx_buffer: ptr::null_mut(),
            },
            length: 0,
            rxlength: 0,
            ..Default::default()
        };

        esp!(unsafe { spi_device_polling_transmit(self.handle, &mut transaction as *mut _) })
            .map_err(SpiError::other)
    }
}

impl embedded_hal::spi::ErrorType for MasterBus {
    type Error = SpiError;
}

impl embedded_hal::spi::blocking::SpiBusFlush for MasterBus {
    fn flush(&mut self) -> Result<(), Self::Error> {
        // Since we use polling transactions, flushing isn't required.
        // In future, when DMA is available spi_device_get_trans_result
        // will be called here.
        Ok(())
    }
}

impl embedded_hal::spi::blocking::SpiBusRead for MasterBus {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for chunk in words.chunks_mut(self.trans_len) {
            self.polling_transmit(chunk.as_mut_ptr(), ptr::null(), chunk.len(), chunk.len())?;
        }
        Ok(())
    }
}

impl embedded_hal::spi::blocking::SpiBusWrite for MasterBus {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for chunk in words.chunks(self.trans_len) {
            self.polling_transmit(ptr::null_mut(), chunk.as_ptr(), chunk.len(), 0)?;
        }
        Ok(())
    }
}

impl embedded_hal::spi::blocking::SpiBus for MasterBus {
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
                use embedded_hal::spi::blocking::SpiBusRead;
                // Read remainder
                self.read(&mut read[write.len()..])?;
            }
            Ordering::Less => {
                use embedded_hal::spi::blocking::SpiBusWrite;
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
            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
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

            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        esp!(unsafe {
            spi_bus_initialize(SPI::device(), &bus_config, 0 /*TODO: DMA support*/)
        })?;

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

    fn lock_bus_for(&mut self, lock_bus: bool, size: usize) -> Result<Option<Lock>, SpiError> {
        if lock_bus && size > TRANS_LEN {
            Ok(Some(self.lock_bus()?))
        } else {
            Ok(None)
        }
    }

    fn transfer_internal(
        &mut self,
        read: &mut [u8],
        write: &[u8],
        lock_bus: bool,
    ) -> Result<(), SpiError> {
        let _lock = self.lock_bus_for(lock_bus, max(read.len(), write.len()))?;

        let len = max(read.len(), write.len());
        for offset in (0..len).step_by(TRANS_LEN) {
            let read_chunk_end = min(offset + TRANS_LEN, read.len());
            let write_chunk_end = min(offset + TRANS_LEN, write.len());

            if read_chunk_end != write_chunk_end {
                let mut buf = [0_u8; TRANS_LEN];

                let write_ptr = if write_chunk_end < offset + TRANS_LEN {
                    if write_chunk_end > offset {
                        buf[0..write_chunk_end - offset]
                            .copy_from_slice(&write[offset..write_chunk_end]);
                    }

                    buf.as_ptr()
                } else {
                    let chunk = &write[offset..write_chunk_end];

                    chunk.as_ptr()
                };

                let read_ptr = if read_chunk_end < offset + TRANS_LEN {
                    buf.as_mut_ptr()
                } else {
                    let chunk = &mut read[offset..read_chunk_end];

                    chunk.as_mut_ptr()
                };

                let transfer_len = max(read_chunk_end, write_chunk_end) - offset;

                self.transfer_internal_raw(read_ptr, transfer_len, write_ptr, transfer_len)?;

                if read_chunk_end > offset && read_chunk_end < offset + TRANS_LEN {
                    read[offset..read_chunk_end].copy_from_slice(&buf[0..read_chunk_end - offset]);
                }
            } else {
                let read_chunk = &mut read[offset..read_chunk_end];
                let write_chunk = &write[offset..write_chunk_end];

                self.transfer_internal_raw(
                    read_chunk.as_mut_ptr(),
                    read_chunk.len(),
                    write_chunk.as_ptr(),
                    write_chunk.len(),
                )?;
            }
        }

        Ok(())
    }

    fn transfer_inplace_internal(
        &mut self,
        data: &mut [u8],
        lock_bus: bool,
    ) -> Result<(), SpiError> {
        let _lock = self.lock_bus_for(lock_bus, data.len())?;

        let total_len = data.len();
        for offset in (0..data.len()).step_by(TRANS_LEN) {
            let chunk = &mut data[offset..min(offset + TRANS_LEN, total_len)];
            let len = chunk.len();
            let ptr = chunk.as_mut_ptr();

            self.transfer_internal_raw(ptr, len, ptr, len)?;
        }

        Ok(())
    }

    fn write_iter_internal<WI>(&mut self, words: WI) -> Result<(), SpiError>
    where
        WI: IntoIterator<Item = u8>,
    {
        let mut words = words.into_iter();

        let mut buf = [0_u8; TRANS_LEN];

        let mut lock = None;

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

            if offset == buf.len() && lock.is_none() {
                lock = Some(self.lock_bus()?);
            }

            let chunk = &mut buf[..offset];
            let ptr = chunk.as_mut_ptr();

            self.transfer_internal_raw(ptr, chunk.len(), ptr, chunk.len())?;
        }

        Ok(())
    }

    fn transfer_internal_raw(
        &mut self,
        read: *mut u8,
        read_len: usize,
        write: *const u8,
        write_len: usize,
    ) -> Result<(), SpiError> {
        let mut transaction = spi_transaction_t {
            flags: 0,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: write as *const _,
            },
            __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
                rx_buffer: read as *mut _,
            },
            length: (write_len * 8) as _,
            rxlength: (read_len * 8) as _,
            ..Default::default()
        };

        esp!(unsafe { spi_device_polling_transmit(self.device, &mut transaction as *mut _) })
            .map_err(SpiError::other)?;

        Ok(())
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal::spi::ErrorType for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal::spi::blocking::SpiDevice for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Bus = MasterBus;

    fn transaction<R>(
        &mut self,
        f: impl FnOnce(&mut Self::Bus) -> Result<R, <Self::Bus as embedded_hal::spi::ErrorType>::Error>,
    ) -> Result<R, Self::Error> {
        let mut bus = MasterBus {
            handle: self.device,
            trans_len: TRANS_LEN,
        };

        let lock = self.lock_bus()?;
        let trans_result = f(&mut bus);

        let finish_result = bus.finish();

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        use embedded_hal::spi::blocking::SpiBusFlush;
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
        self.transfer_inplace_internal(words, true)?;

        Ok(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    embedded_hal_0_2::blocking::spi::Write<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = SpiError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.transfer_internal(&mut [], words, true)
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
        self.write_iter_internal(words)
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
        let _lock = self.lock_bus()?;

        for operation in operations {
            match operation {
                embedded_hal_0_2::blocking::spi::Operation::Write(write) => {
                    self.transfer_internal(&mut [], write, false)?
                }
                embedded_hal_0_2::blocking::spi::Operation::Transfer(words) => {
                    self.transfer_inplace_internal(words, false)?
                }
            }
        }

        Ok(())
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
