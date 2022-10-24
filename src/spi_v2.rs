#![feature(associated_type_bounds)]
// dma is not implemented currently because a bus can only have
// one max_transfer_size but every channel on the bus could have
// different values -> need additional checks for it

// TODO: improve: need lot of cloning in RefCell handle

use crate::delay::BLOCK;
use crate::gpio::{AnyIOPin, InputPin, OutputPin};
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::prelude::Peripherals;
use crate::spi::{config, Dma};
use embedded_hal_n::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite};
use esp_idf_sys::*;
use once_cell::sync::Lazy;

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

use core::cmp::{max, min, Ordering};
use core::marker::PhantomData;
use core::{panic, ptr};
use std::collections::{HashMap, VecDeque};
use std::hash::{Hash, Hasher};

crate::embedded_hal_error!(
    SpiError,
    embedded_hal_n::spi::Error,
    embedded_hal_n::spi::ErrorKind
);

const ESP_MAX_SPI_DEVICES: usize = 3;

struct Master{
    //bus: Mutex<Option<SpiMasterDriver< 'd, SPI>>>,
    bus: Option<SpiMaster2>,
    devices: HashMap<i32, RefCell<SpiSlave>>,
    devices_with_handle: VecDeque<i32>,
}

static MASTER: Lazy<Arc<Mutex<Master>>> = Lazy::new(|| {
    let bus = None;
    let devices = HashMap::new();
    let devices_with_handle = VecDeque::new();
    Arc::new(Mutex::new(Master { bus, devices, devices_with_handle }))
});
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
    slave_id: i32,
}

// the implemantation of new ensures that an cs_pin is unique
// by taking a "impl Peripheral<P = impl OutputPin"
// so it should be enough to use cs_pin for a unique hash
impl Hash for SpiSlave{
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.slave_id.hash(state);
    }
}
unsafe impl Send for SpiSlave {}
impl SpiSlave {

    pub fn new(
        cs: impl Peripheral<P = impl InputPin + OutputPin>,
        config: config::Config,
    ) -> Result<SpiSlave, EspError> {
        let mut me  = Self::old_new(cs, config);

        // free handle space ? -> add ourself
        let mut master = MASTER.lock();
        match master {
            Ok(master) => {
                if master.devices_with_handle.len() < ESP_MAX_SPI_DEVICES {
                    if let Some(bus) = &master.bus {
                        me.add_to_bus(bus.host_handle.unwrap());
                    } else {
                        println!("Error First initialize SpiMaster2 before using!!")
                    }  
                }
            }
            Err(_) => println!("Poison Error on MASTER usage in SpiSlave::new() call")
        }
        Ok(me)
    }

    pub fn old_new(
        cs: impl Peripheral<P = impl InputPin + OutputPin>,
        config: config::Config,
    ) -> SpiSlave{
        let cs: i32 = cs.into_ref().pin();
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
        Self {
            handle: RefCell::new(None),
            config: device_config,
            slave_id: cs,
        }
    }  
    
    fn add_to_bus(&self, host_handle: spi_host_device_t) -> Result<(), EspError> {
        let mut device_handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe {
            spi_bus_add_device(host_handle, &self.config, &mut device_handle as *mut _)
        })?;

        self.handle.replace(Some(device_handle));
        Ok(())
    }

    fn rm_from_bus(&self) -> Result<(), EspError>{
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

    pub fn get_id(&self) -> i32 {
        self.slave_id
    }

    // TODO! Error propagation
    fn give_handle(& self) {
        // get list of all how currently have handle
        let lock = MASTER.lock();
        match lock {
            Ok(mut master) => {
                // is room for free handle ?
                // normaly == should suffice( should be impossible to have it >= ...)
                if master.devices_with_handle.len() >= ESP_MAX_SPI_DEVICES { // make room
                    let oldest_id = master.devices_with_handle.pop_front().unwrap(); 
                    if let Some(oldest) = master.devices.get(&oldest_id) {
                        oldest.borrow().rm_from_bus();
                        if master.devices_with_handle.len() >= ESP_MAX_SPI_DEVICES {
                            println!("ERROR: after poping from device_with_handle it still has to much devices !!");
                            unreachable!();
                        }                        
                    }                    
                }
                if let Some(spi) = &master.bus {
                    self.add_to_bus(spi.host_handle.unwrap());
                    master.devices_with_handle.push_back(self.slave_id);
                }
            }
            Err(_) => println!("Poison Error on global MASTER <-  give_handle() call")
        }
    }

    pub fn transaction<'d, R, E>(
        &mut self,
        f: impl FnOnce(&mut SpiBusMasterDriver<'d>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
    {
        // check if our handle is valid and was not deleted by someone else
        let has_handle = self.handle.borrow();
        if let None = *has_handle {
            self.give_handle();
        }

        let handle = self.handle.clone().into_inner().unwrap();
        let mut bus = SpiBusMasterDriver {
            // we made above sure that we have a valid handle -> unwrap
            handle: handle,
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

    fn lock_bus(handle: spi_device_handle_t) -> Result<Lock, EspError> {
        Lock::new(handle)
    }
}

impl Drop for SpiSlave {
    fn drop(&mut self) {
        if let Ok(mut master) = MASTER.lock() {
            master.devices.remove(&self.slave_id);
        }
    }
}

pub struct SpiMaster2 {
    host_handle: Option<spi_host_device_t>,
}
unsafe impl Send for SpiMaster2 {}

impl SpiMaster2{
    pub fn new<SPI: SpiAnyPins>(
        _spi: impl Peripheral<P = SPI>,
        sclk: impl Peripheral<P = impl OutputPin> ,
        sdo: impl Peripheral<P = impl OutputPin> ,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin> >,
    ) -> Self {
        let host_handle = SPI::device();
        Self::new_internal_bus::<SPI>(sclk, sdo, sdi);
        Self{
            host_handle: Some(host_handle)
        }
        
    }
    fn new_internal_bus<SPI: SpiAnyPins>(
        sclk: impl Peripheral<P = impl OutputPin>,
        sdo: impl Peripheral<P = impl OutputPin> ,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin>>,
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
    pub fn init_master(self){
        if let Ok(mut master) = MASTER.lock() {
            master.bus = Some(self)
        }
    } 
}

impl Drop for SpiMaster2 {
    fn drop(&mut self) {
        if let Ok(mut master) = MASTER.lock() {
            for dev_id in &master.devices_with_handle {
                if let Some(device) = master.devices.get(dev_id) {
                    if let Some(handle) = device.borrow().handle.replace(None) {
                        esp!(unsafe { spi_bus_remove_device(handle) }).unwrap();
                    }
                }
            }
            master.devices_with_handle.clear();
        }
        if let Some( host_handle) = self.host_handle {
            esp!(unsafe { spi_bus_free(host_handle) }).unwrap();
        }
    }
}

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
