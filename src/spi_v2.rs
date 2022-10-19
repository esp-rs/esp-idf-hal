
// dma is not implemented currently because a bus can only have
// one max_transfer_size but every channel on the bus could have
// different values -> need additional checks for it

// TODO: improve: need lot of cloning in RefCell handle

use embedded_hal_n::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite};
use esp_idf_sys::*;
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::gpio::{InputPin, OutputPin};
use crate::spi::{config, Dma};
use crate::delay::BLOCK;

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::hash::{Hash, Hasher};
use core::{ptr, panic};
use core::marker::PhantomData;
use core::cmp::{max, min, Ordering};

crate::embedded_hal_error!(
    SpiError,
    embedded_hal_n::spi::Error,
    embedded_hal_n::spi::ErrorKind
);

const ESP_MAX_SPI_DEVICES: usize = 3;
pub trait Spi: Send {
    fn device() -> spi_host_device_t;
}
/// A marker interface implemented by all SPI peripherals except SPI1 which
/// should use a fixed set of pins
pub trait SpiAnyPins: Spi {}

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

impl<'d> embedded_hal_n::spi::ErrorType for SpiBusMasterDriver<'d> {
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

// rename in something like RtosSpiDevice / EspSpiDevice ?

// stores a i32 than a pin definition because PeripheralRef is
// an exlusive ref- cannot save an shared ref to a pin?
pub struct SpiSlave {
    handle: RefCell<Option<spi_device_handle_t>>,
    config: spi_device_interface_config_t,
    cs_pin: i32,
}

// the implemantation of new ensures that an cs_pin is unique
// by taking a "impl Peripheral<P = impl OutputPin"
// so it should be enough to use cs_pin for a unique hash
impl Hash for SpiSlave{
    fn hash<H: Hasher>(&self, state: &mut H){
        self.cs_pin.hash(state);
    }
}

impl SpiSlave {
    fn new(
        cs: i32,
        config: config::Config,
    ) -> Result<SpiSlave, EspError> {
        let device_config = spi_device_interface_config_t {
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
        };
        Ok(
            SpiSlave{
                handle: RefCell::new(None),
                config: device_config,
                cs_pin: cs
            }
        )
    }
}
pub struct SpiMasterDriver<'d, SPI: Spi> {
    _spi: PeripheralRef<'d, SPI>,
    pub devices: HashMap<i32, SpiSlave>,
    devices_holding_handle: VecDeque<i32>,
}

impl<'d, SPI: SpiAnyPins> SpiMasterDriver<'d, SPI> {

    pub fn new(
        spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        cs_list: Vec<impl Peripheral<P = impl InputPin + OutputPin>>,
        config_list: Vec<config::Config>,
    ) -> Result<Self, EspError>
    {
        let spi = SpiMasterDriver::new_internal_bus(spi, sclk, sdo, sdi)?;

        let cs_list = std::iter::zip(cs_list, config_list);
        // always get a valid spi_device_handle_t if only 3 or less devices used
        let mut devices = HashMap::new();
        let mut devices_holding_handle = VecDeque::with_capacity(ESP_MAX_SPI_DEVICES);
        if cs_list.len() <= ESP_MAX_SPI_DEVICES {
            for (cs, config) in cs_list {
                let cs: i32 = cs.into_ref().pin();
                let device = SpiSlave::new(cs, config)?;
                Self::device_add_to_bus(&device)?;
                devices.insert(cs, device);
                devices_holding_handle.push_back(cs);
            }
        }
        else {
            for (cs, config) in cs_list {
                let cs: i32 = cs.into_ref().pin();
                let device = SpiSlave::new(cs, config)?;
                // first 3 devices can directly get a valid handle
                if devices_holding_handle.len() < ESP_MAX_SPI_DEVICES {
                    Self::device_add_to_bus(&device)?;
                    devices_holding_handle.push_back(cs);
                }
                devices.insert(cs, device);
            }
        }
        Ok(Self { _spi: spi, devices, devices_holding_handle })
    }

    pub fn transaction<R, E>(
        &mut self,
        device_id: i32,
        f: impl FnOnce(&mut SpiBusMasterDriver<'d>) -> Result<R, E>,
    ) -> Result<R, E> 
    where 
        E: From<EspError>,
    {
        // TODO clean up nested matches/ if else
        // check if there is a valid device id, otherwise error out
        let device = self.devices.get(&device_id);
        if let Some(device) = device{
            // we have a valid handle
            if let Some(handle) = device.handle.clone().into_inner() {
                // do nothing
            } else { // get valid handle
                // if free handle space than just get handle
                if self.devices_holding_handle.len() <= ESP_MAX_SPI_DEVICES {
                    Self::device_add_to_bus(&device)?;
                    self.devices_holding_handle.push_back(device.cs_pin);
                }
                // otherwise delete oldest handle from other device and push new handle

                // TODO: further check potential risk of removing old devices from bus
                // before old devices finished doing stuff on bus
                // maybe an aditional lock is needed
                else {
                    // we checked already that it is non_empty -> unwrap
                    let oldest = self.devices_holding_handle.pop_front().unwrap();
                    if let Some(oldest) = self.devices.get(&oldest){
                        Self::device_rm_from_bus(oldest)?;
                        Self::device_add_to_bus(&device)?;
                        self.devices_holding_handle.push_back(device.cs_pin);                 
                    } else {
                        // TODO better error 
                        println!("SPI MASTER ERROR: An Device ID was in device_with_handle list ...");
                        println!("SPI MASTER ERROR: .. but no such device was found in device list ");
                        // potentially unrechable?
                        unreachable!()
                    }
                }
            }
        } else { //no valid device_id
            // TODO handle case with msg to user but no panic !!
            println!("SPI MASTER ERROR: Unknown Device ID {} used in transaction", device_id);
            panic!()
        }
        let device = device.unwrap();
        let mut bus = SpiBusMasterDriver {
                // we made above sure that we have a valid handle -> unwrap
                handle: device.handle.clone().into_inner().unwrap(),
                trans_len: SOC_SPI_MAXIMUM_BUFFER_SIZE as usize ,
                _p: PhantomData,
        };
        
        let lock = Self::lock_bus(device.handle.clone().into_inner().unwrap())?;

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

    pub fn transfer(& mut self,device_id: i32, read: &mut [u8], write: &[u8]) -> Result<(), EspError> 
    {
        self.transaction(device_id, |bus| bus.transfer(read, write))
    }
    
    pub fn write(& mut self,device_id: i32,  write: &[u8]) -> Result<(), EspError> 
    {
        self.transaction(device_id, |bus| bus.write( write))
    }

    pub fn read(& mut self,device_id: i32, read: &mut [u8]) -> Result<(), EspError> 
    {
        self.transaction(device_id, |bus| bus.read(read))
    }

    pub fn transfer_in_place(& mut self,device_id: i32, buf: &mut [u8]) -> Result<(), EspError> 
    {
        self.transaction(device_id, |bus| bus.transfer_in_place(buf))
    }

    fn lock_bus(handle: spi_device_handle_t) -> Result<Lock, EspError> {
        Lock::new(handle)
    }
    fn new_internal_bus(
        spi: impl Peripheral<P = SPI> + 'd,
        sclk: impl Peripheral<P = impl OutputPin> + 'd,
        sdo: impl Peripheral<P = impl OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
    ) -> Result<(PeripheralRef<'d, SPI>), EspError> {
        crate::into_ref!(spi, sclk, sdo);
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
        Ok(spi)

    }


    // alternative name just add_to_bus // rm_to_bus
    fn device_add_to_bus(device: &SpiSlave) -> Result<(), EspError> {
        let mut device_handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe {
            spi_bus_add_device(SPI::device(), &device.config, &mut device_handle as *mut _)
        })?;

        device.handle.replace(Some(device_handle));
        Ok(()) 
    }

    fn device_rm_from_bus(device: & SpiSlave) -> Result<(), EspError>  {
        // make sure its only get called with valid handle
        if let Some(handle) = device.handle.clone().into_inner() {
            esp!(unsafe { spi_bus_remove_device(handle) })?;
            device.handle.replace(None);
        } else {
            println!("SPI MASTER ERROR: Cant remove non existing handle from Bus. Spi device id {} ", device.cs_pin)
        }

        Ok(())
    }
}

impl<'d, SPI: Spi> Drop for SpiMasterDriver<'d, SPI> {
    fn drop(&mut self) {
        for device in &self.devices_holding_handle{
            let device = self.devices.get(device).unwrap();
            if let Some(handle) = device.handle.clone().into_inner() {
                esp!(unsafe { spi_bus_remove_device(handle) }).unwrap();
                device.handle.replace(None);
            }
        }        
        esp!(unsafe { spi_bus_free(SPI::device()) }).unwrap();
    }
}

unsafe impl<'d, SPI: Spi> Send for SpiMasterDriver<'d, SPI> {}

fn to_spi_err(err: EspError) -> SpiError {
    SpiError::other(err)
}

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
