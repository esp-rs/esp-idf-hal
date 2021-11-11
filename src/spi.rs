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

//use crate::prelude::*;

use crate::gpio::{self, InputPin, OutputPin};

use embedded_hal::blocking::spi::{Transfer, Write, WriteIter};
use embedded_hal::spi::{Phase, Polarity};
//use embedded_hal::spi::FullDuplex,

use esp_idf_sys::*;

pub trait Spi {
    fn device() -> spi_host_device_t;
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
    use crate::units::*;
    pub use embedded_hal::spi::{Mode, MODE_0, MODE_1, MODE_2, MODE_3};

    /// SPI Bit Order
    #[derive(PartialEq, Eq, Copy, Clone)]
    pub enum BitOrder {
        LSBFirst,
        MSBFirst,
    }

    /// SPI configuration
    #[derive(Copy, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub data_mode: embedded_hal::spi::Mode,
        pub bit_order: BitOrder,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn data_mode(mut self, data_mode: embedded_hal::spi::Mode) -> Self {
            self.data_mode = data_mode;
            self
        }

        pub fn bit_order(mut self, bit_order: BitOrder) -> Self {
            self.bit_order = bit_order;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                baudrate: Hertz(1_000_000),
                data_mode: MODE_0,
                bit_order: BitOrder::LSBFirst,
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
    bit_order: config::BitOrder,
}

unsafe impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin> Send for Master<SPI, SCLK, SDO, SDI, CS> {}

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
            sclk_io_num: SCLK::pin(),

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: SDO::pin(),
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: if pins.sdi.is_some() { SDI::pin() } else { -1 },
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
            sclk_io_num: SCLK::pin(),

            mosi_io_num: SDO::pin(),
            miso_io_num: if pins.sdi.is_some() { SDI::pin() } else { -1 },
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        esp!(unsafe {
            spi_bus_initialize(SPI::device(), &bus_config, 0 /*TODO: DMA support*/)
        })?;

        let device_config = spi_device_interface_config_t {
            spics_io_num: if pins.cs.is_some() { CS::pin() } else { -1 },
            clock_speed_hz: config.baudrate.0 as i32,
            mode: (if config.data_mode.polarity == Polarity::IdleHigh {
                2
            } else {
                0
            }) | (if config.data_mode.phase == Phase::CaptureOnSecondTransition {
                1
            } else {
                0
            }),
            queue_size: 64,
            ..Default::default()
        };

        let mut device_handle: spi_device_handle_t = core::ptr::null_mut();

        esp!(unsafe {
            spi_bus_add_device(SPI::device(), &device_config, &mut device_handle as *mut _)
        })?;

        Ok(Self {
            spi,
            pins,
            device: device_handle,
            bit_order: config.bit_order,
        })
    }

    /// Release and return the raw interface to the underlying SPI peripheral
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> Result<(SPI, Pins<SCLK, SDO, SDI, CS>), EspError> {
        esp!(unsafe { spi_bus_remove_device(self.device) })?;
        esp!(unsafe { spi_bus_free(SPI::device()) })?;

        Ok((self.spi, self.pins))
    }

    /// Generic transfer function
    ///
    /// This function locks the APB bus frequency and chunks the output
    /// for maximum write performance.
    fn transfer_internal<'a, T>(&mut self, words: &'a mut [T]) -> Result<&'a [T], EspError>
    where
        T: Word,
    {
        let mut tx_buffer = [0_u8; 64];
        let mut rx_buffer = [0_u8; 64];

        let mut transaction = spi_transaction_t {
            flags: 0, //SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: &tx_buffer as *const [u8] as *const _,
            },
            __bindgen_anon_2: spi_transaction_t__bindgen_ty_2 {
                rx_buffer: &mut rx_buffer as *mut [u8] as *mut _,
            },
            ..Default::default()
        };

        let mut words_index = 0;

        while words_index < words.len() {
            let mut index = 0;
            let mut words_write_index = words_index;
            while index + core::mem::size_of::<T>() <= tx_buffer.len()
                && words_write_index < words.len()
            {
                words[words_write_index].store(&mut tx_buffer[index..], self.bit_order);
                words_write_index += 1;
                index += core::mem::size_of::<T>();
            }

            if index == 0 {
                break;
            }

            transaction.length = index as u32 * 8;
            transaction.rxlength = index as u32 * 8;

            esp!(unsafe { spi_device_polling_transmit(self.device, &mut transaction as *mut _) })?;

            index = 0;
            while index < transaction.rxlength as usize && words_index < words.len() {
                words[words_index] = T::load(&rx_buffer[index..], self.bit_order);
                words_index += 1;
                index += core::mem::size_of::<T>();
            }
        }

        Ok(words)
    }

    /// Generic write function for iterators
    ///
    /// This function locks the APB bus frequency and chunks the output of the iterator
    /// for maximum write performance.
    fn write_internal<T>(&mut self, words: &'_ [T]) -> Result<(), EspError>
    where
        T: Word,
    {
        let mut tx_buffer = [0_u8; 64];

        let mut transaction = spi_transaction_t {
            flags: 0, //SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: &tx_buffer as *const [u8] as *const _,
            },
            ..Default::default()
        };

        let mut words_index = 0;

        while words_index < words.len() {
            let mut index = 0;
            while index + core::mem::size_of::<T>() <= tx_buffer.len() && words_index < words.len()
            {
                words[words_index].store(&mut tx_buffer[index..], self.bit_order);
                words_index += 1;
                index += core::mem::size_of::<T>();
            }

            if index == 0 {
                break;
            }

            transaction.length = index as u32 * 8;
            transaction.rxlength = 0;

            esp!(unsafe { spi_device_polling_transmit(self.device, &mut transaction as *mut _) })?;
        }

        Ok(())
    }

    /// Generic write function for iterators
    ///
    /// This function locks the APB bus frequency and chunks the output of the iterator
    /// for maximum write performance.
    fn write_iter_internal<T, WI>(&mut self, words: WI) -> Result<(), EspError>
    where
        T: Word,
        WI: IntoIterator<Item = T>,
    {
        let mut tx_buffer = [0_u8; 64];

        let mut transaction = spi_transaction_t {
            flags: 0, //SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            __bindgen_anon_1: spi_transaction_t__bindgen_ty_1 {
                tx_buffer: &tx_buffer as *const [u8] as *const _,
            },
            ..Default::default()
        };

        let mut iter = words.into_iter().peekable();
        while iter.peek().is_some() {
            let mut index = 0;
            while index + core::mem::size_of::<T>() <= tx_buffer.len() {
                if let Some(word) = iter.next() {
                    word.store(&mut tx_buffer[index..], self.bit_order);
                    index += core::mem::size_of::<T>();
                } else {
                    break;
                }
            }

            if index == 0 {
                break;
            }

            transaction.length = index as u32 * 8;
            transaction.rxlength = 0;

            esp!(unsafe { spi_device_polling_transmit(self.device, &mut transaction as *mut _) })?;
        }

        Ok(())
    }
}

pub trait Word: Copy + Clone + Sized {
    fn load(buffer: &[u8], bit_order: config::BitOrder) -> Self;
    fn store(self, buffer: &mut [u8], bit_order: config::BitOrder);
}

impl Word for u32 {
    #[inline(always)]
    fn load(buffer: &[u8], bit_order: config::BitOrder) -> Self {
        if bit_order == config::BitOrder::MSBFirst {
            ((buffer[0] as u32) << 24)
                | ((buffer[1] as u32) << 16)
                | ((buffer[2] as u32) << 8)
                | (buffer[3] as u32)
        } else {
            ((buffer[3] as u32) << 24)
                | ((buffer[2] as u32) << 16)
                | ((buffer[1] as u32) << 8)
                | (buffer[0] as u32)
        }
    }

    #[inline(always)]
    fn store(self, buffer: &mut [u8], bit_order: config::BitOrder) {
        if bit_order == config::BitOrder::MSBFirst {
            buffer[0] = ((self >> 24) & 0xff) as u8;
            buffer[1] = ((self >> 16) & 0xff) as u8;
            buffer[2] = ((self >> 8) & 0xff) as u8;
            buffer[3] = (self & 0xff) as u8;
        } else {
            buffer[3] = ((self >> 24) & 0xff) as u8;
            buffer[2] = ((self >> 16) & 0xff) as u8;
            buffer[1] = ((self >> 8) & 0xff) as u8;
            buffer[0] = (self & 0xff) as u8;
        }
    }
}

impl Word for u16 {
    #[inline(always)]
    fn load(buffer: &[u8], bit_order: config::BitOrder) -> Self {
        if bit_order == config::BitOrder::MSBFirst {
            ((buffer[0] as u16) << 8) | (buffer[1] as u16)
        } else {
            ((buffer[1] as u16) << 8) | (buffer[0] as u16)
        }
    }

    #[inline(always)]
    fn store(self, buffer: &mut [u8], bit_order: config::BitOrder) {
        if bit_order == config::BitOrder::MSBFirst {
            buffer[0] = ((self >> 8) & 0xff) as u8;
            buffer[1] = (self & 0xff) as u8;
        } else {
            buffer[1] = ((self >> 8) & 0xff) as u8;
            buffer[0] = (self & 0xff) as u8;
        }
    }
}

impl Word for u8 {
    #[inline(always)]
    fn load(buffer: &[u8], _bit_order: config::BitOrder) -> Self {
        buffer[0]
    }

    #[inline(always)]
    fn store(self, buffer: &mut [u8], _bit_order: config::BitOrder) {
        buffer[0] = self;
    }
}

/// Full-duplex implementation for writing/reading via SPI
///
/// *Note: these functions do not lock the frequency of the APB bus, so transactions may be
/// at lower frequency if APB bus is not locked in caller.*
// impl<
//         T: Word,
//         SPI: Spi,
//         SCLK: OutputPin,
//         SDO: OutputPin,
//         SDI: InputPin + OutputPin,
//         CS: OutputPin,
//     > FullDuplex<T> for Master<SPI, SCLK, SDO, SDI, CS>
// {
//     type Error = EspError;

//     fn read(&mut self) -> nb::Result<T, Self::Error> {
//         let spi = &self.instance;

//         if spi.cmd.read().usr().bit_is_set() {
//             return Err(nb::Error::WouldBlock);
//         }

//         let bits = (core::mem::size_of::<T>() * 8) as u32;

//         (spi.w[0].read().bits() & (0xffffffff >> (32 - bits)))
//             .try_into()
//             .map_err(|_| nb::Error::Other(Self::Error::ConversionFailed))
//     }

//     fn send(&mut self, value: T) -> nb::Result<(), Self::Error> {
//         let spi = &self.instance;

//         if spi.cmd.read().usr().bit_is_set() {
//             return Err(nb::Error::WouldBlock);
//         }

//         let bits = (core::mem::size_of::<T>() * 8 - 1) as u32;

//         spi.mosi_dlen
//             .write(|w| unsafe { w.usr_mosi_dbitlen().bits(bits) });
//         spi.miso_dlen
//             .write(|w| unsafe { w.usr_miso_dbitlen().bits(bits) });
//         spi.w[0].write(|w| unsafe { w.bits(value.into()) });

//         spi.cmd.modify(|_, w| w.usr().set_bit());

//         Ok(())
//     }
// }

// cannot use generics as it conflicts with the Default implementation
impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Transfer<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> core::result::Result<&'w [u8], Self::Error> {
        self.transfer_internal(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Transfer<u16> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn transfer<'w>(
        &mut self,
        words: &'w mut [u16],
    ) -> core::result::Result<&'w [u16], Self::Error> {
        self.transfer_internal(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    Transfer<u32> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn transfer<'w>(
        &mut self,
        words: &'w mut [u32],
    ) -> core::result::Result<&'w [u32], Self::Error> {
        self.transfer_internal(words)
    }
}

// cannot use generics as it conflicts with the Default implementation
impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin> Write<u8>
    for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write_iter_internal(words.iter().copied())
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin> Write<u16>
    for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
        self.write_internal(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin> Write<u32>
    for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write(&mut self, words: &[u32]) -> Result<(), Self::Error> {
        self.write_internal(words)
    }
}

// cannot use generics as it conflicts with the Default implementation
impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    WriteIter<u8> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u8>,
    {
        self.write_iter_internal(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    WriteIter<u16> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u16>,
    {
        self.write_iter_internal(words)
    }
}

impl<SPI: Spi, SCLK: OutputPin, SDO: OutputPin, SDI: InputPin + OutputPin, CS: OutputPin>
    WriteIter<u32> for Master<SPI, SCLK, SDO, SDI, CS>
{
    type Error = EspError;

    fn write_iter<WI>(&mut self, words: WI) -> Result<(), Self::Error>
    where
        WI: IntoIterator<Item = u32>,
    {
        self.write_iter_internal(words)
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
