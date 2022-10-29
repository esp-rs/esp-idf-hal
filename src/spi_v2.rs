// dma is not implemented currently because a bus can only have
// one max_transfer_size but every channel on the bus could have
// different values -> need additional checks for it

// TODO: improve: need lot of cloning in RefCell handle

use crate::delay::BLOCK;
use crate::gpio::{self, InputPin, OutputPin, Pins,Output,InputOutput};
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::spi::{config, Dma};

use embedded_hal::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite, SpiDevice};
use esp_idf_sys::*;

use core::cmp::{max, min, Ordering};
use core::marker::PhantomData;
use core::ptr;

use std::cell::RefCell;
//use std::rc::{Rc, Weak};
use std::collections::{HashMap, VecDeque};
use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex, Weak};

crate::embedded_hal_error!(
    SpiError,
    embedded_hal::spi::Error,
    embedded_hal::spi::ErrorKind
);
#[cfg(not(esp32c3))]
const ESP_MAX_SPI_DEVICES: usize = 3;
#[cfg(esp32c3)]
const ESP_MAX_SPI_DEVICES: usize = 6;

pub trait Spi: Send {
    fn device() -> spi_host_device_t;
}
/// A marker interface implemented by all SPI peripherals except SPI1 which
/// should use a fixed set of pins
pub trait SpiAnyPins: Spi {}

pub struct SpiBusMasterDriver {
    handle: spi_device_handle_t,
    trans_len: usize,
    //_p: PhantomData<&'d ()>,
}

impl SpiBusMasterDriver {
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

impl embedded_hal::spi::ErrorType for SpiBusMasterDriver {
    type Error = SpiError;
}

impl SpiBusFlush for SpiBusMasterDriver {
    fn flush(&mut self) -> Result<(), Self::Error> {
        SpiBusMasterDriver::flush(self).map_err(to_spi_err)
    }
}

impl SpiBusRead for SpiBusMasterDriver {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::read(self, words).map_err(to_spi_err)
    }
}

impl SpiBusWrite for SpiBusMasterDriver {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        SpiBusMasterDriver::write(self, words).map_err(to_spi_err)
    }
}

impl SpiBus for SpiBusMasterDriver {
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

pub struct EspSpiDevice{
    handle: RefCell<Option<spi_device_handle_t>>,
    config: spi_device_interface_config_t,
    device_id: i32,
    bus: Arc<Mutex<SpiMasterDriver>>,
    //cs: PeripheralRef<'static,InputOutput>,
}

// the implemantation of new ensures that an cs_pin is unique
// by taking a "impl Peripheral<P = impl OutputPin"
// so it should be enough to use cs_pin for a unique hash
impl Hash for EspSpiDevice {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.device_id.hash(state);
    }
}
//unsafe Send for EspSpiDevice {}

impl EspSpiDevice {
    pub fn new(
        bus_handle: Arc<Mutex<SpiMasterDriver>>,
        cs: impl Peripheral<P = impl InputPin + OutputPin>,
        config: config::Config,
    ) -> Result<Arc<EspSpiDevice>, EspError> {
        let me = Self::create_conf(cs, config, bus_handle);

        let id = me.get_id();
        let me = Arc::new(me);

        let weak = RefCell::new(Arc::downgrade(&me));
        if let Ok(mut master) = me.bus.lock() {
            if master.devices_with_handle.len() < ESP_MAX_SPI_DEVICES {
                let host_handle = master.handle;
                me.add_to_bus(host_handle)?;
                master.devices_with_handle.push_back(id);
            }
            master.devices.insert(id, weak);
        }
        Ok(me)
    }

    pub fn create_conf(
        cs: impl Peripheral<P = impl InputPin + OutputPin>,
        config: config::Config,
        bus: Arc<Mutex<SpiMasterDriver>>,
    ) -> EspSpiDevice {
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
            device_id: cs,
            bus,
        }
    }

    fn add_to_bus(&self, bus_handle: spi_host_device_t) -> Result<(), EspError> {
        let mut device_handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe {
            spi_bus_add_device(bus_handle, &self.config, &mut device_handle as *mut _)
        })?;
        self.handle.replace(Some(device_handle));
        Ok(())
    }

    fn rm_from_bus(&self) -> Result<(), EspError> {
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
        self.device_id
    }

    // TODO! Error propagation
    fn give_handle(&self) -> Result<(), EspError> {
        // get list of all how currently have handle
        let lock = self.bus.lock();
        match lock {
            Ok(mut master) => {
                // is room for free handle ?
                // normaly == should suffice( should be impossible to have it >= ...)
                if master.devices_with_handle.len() >= ESP_MAX_SPI_DEVICES {
                    // make room
                    let oldest_id = master.devices_with_handle.pop_front().unwrap();
                    if let Some(oldest_handle) = master.devices.get(&oldest_id) {
                        if let Some(other_slave) = oldest_handle.borrow().upgrade() {
                            other_slave.rm_from_bus()?;
                        } else {
                            println!("Tryed to remove some other Device from bus that did't exist anymore");
                            unreachable!();
                        }
                        if master.devices_with_handle.len() >= ESP_MAX_SPI_DEVICES {
                            println!("ERROR: after poping from device_with_handle it still has to much devices !!");
                            unreachable!();
                        }
                    }
                }
                self.add_to_bus(master.handle)?;
                master.devices_with_handle.push_back(self.device_id);
            }
            Err(_) => println!("Poison Error on global MASTER <-  give_handle() call"),
        }
        Ok(())
    }

    pub fn transaction<R, E>(
        &self,
        f: impl FnOnce(&mut SpiBusMasterDriver) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
    {
        // check if our handle is valid and was not deleted by someone else
        let has_handle = self.handle.clone().into_inner();
        if has_handle.is_none() {
            // We got no valid handle :( but we take one now
            self.give_handle()?;
        } else {
            // We Reusing a valid handle :)

            // little time usage optimisation
            // we are used so we go back to the top of the device_wtih_handle que
            // makes it more likely to get our bus handle not removed
            //let master_lock = self.bus.lock();
            //if let Ok(mut master) = master_lock {
            //    if let Ok(idx) = master.devices_with_handle.binary_search(&self.device_id) {
            //        if let Some(val) = master.devices_with_handle.remove(idx) {
            //            master.devices_with_handle.push_back(val);
            //        }
            //    }
            //}
        }

        // DEBUG
        // show current slaves and handles
        //if let Ok(mini_lock) = self.bus.lock() {
        //    for handle in &mini_lock.devices_with_handle {
        //        let bla = &mini_lock.devices.get(handle);
        //        let blub = bla.unwrap().borrow().upgrade();
        //        if let Some(slave) = blub {
        //            let inner_handle = slave.handle.borrow();
        //            println!("DEVICE: {:?} and HANDLE Opt: {:?}", handle, inner_handle);
        //        }
        //    }
        //}

        let handle = self.handle.clone().into_inner().unwrap();
        let mut bus = SpiBusMasterDriver {
            handle,
            trans_len: SOC_SPI_MAXIMUM_BUFFER_SIZE as usize,
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

    pub fn transfer(&self, read: &mut [u8], write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer(read, write))
    }

    pub fn write(&self, write: &[u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.write(write))
    }

    pub fn read(&self, read: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.read(read))
    }

    pub fn transfer_in_place(&self, buf: &mut [u8]) -> Result<(), EspError> {
        self.transaction(|bus| bus.transfer_in_place(buf))
    }

    fn lock_bus(handle: spi_device_handle_t) -> Result<Lock, EspError> {
        Lock::new(handle)
    }
}

impl embedded_hal::spi::ErrorType for EspSpiDevice {
    type Error = SpiError;
}

impl SpiDevice for EspSpiDevice {
    type Bus = SpiBusMasterDriver;

    fn transaction<R>(
        &mut self,
        f: impl FnOnce(
            &mut Self::Bus,
        ) -> Result<R, <Self::Bus as embedded_hal::spi::ErrorType>::Error>,
    ) -> Result<R, Self::Error> {
        EspSpiDevice::transaction(self, f)
    }
}

impl Drop for EspSpiDevice {
    fn drop(&mut self) {
        if let Ok(mut master) = self.bus.lock() {
            master.devices.remove(&self.device_id);
        }
    }
}

pub struct SpiMasterDriver {
    devices: HashMap<i32, RefCell<Weak<EspSpiDevice>>>,
    devices_with_handle: VecDeque<i32>,
    handle: spi_host_device_t,
}

impl SpiMasterDriver {
    pub fn new_spi1<SPI: Spi>(
        _spi: impl Peripheral<P = SPI1>,
        sclk: impl Peripheral<P = gpio::Gpio6>,
        sdo: impl Peripheral<P = gpio::Gpio7>,
        sdi: Option<impl Peripheral<P = gpio::Gpio8>>,
    ) -> Result<Self, EspError> {
        Self::new_internal_bus::<SPI>(sclk, sdo, sdi)?;
        let devices = HashMap::new();
        let devices_with_handle = VecDeque::new();
        Ok(Self {
            devices,
            devices_with_handle,
            handle: SPI::device(),
        })
    }

    pub fn new<SPI: SpiAnyPins>(
        _spi: impl Peripheral<P = SPI>,
        sclk: impl Peripheral<P = impl OutputPin>,
        sdo: impl Peripheral<P = impl OutputPin>,
        sdi: Option<impl Peripheral<P = impl InputPin + OutputPin>>,
    ) -> Result<Self, EspError> {
        Self::new_internal_bus::<SPI>(sclk, sdo, sdi)?;
        let devices = HashMap::new();
        let devices_with_handle = VecDeque::new();
        Ok(Self {
            devices,
            devices_with_handle,
            handle: SPI::device(),
        })
    }
    fn new_internal_bus<SPI: Spi>(
        sclk: impl Peripheral<P = impl OutputPin>,
        sdo: impl Peripheral<P = impl OutputPin>,
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
}

impl Drop for SpiMasterDriver {
    fn drop(&mut self) {
        for dev_id in &self.devices_with_handle {
            if let Some(device_handle) = self.devices.get(dev_id) {
                if let Some(outer) = device_handle.borrow().upgrade() {
                    if let Some(dev_handle) = outer.handle.replace(None) {
                        esp!(unsafe { spi_bus_remove_device(dev_handle) }).unwrap();
                    }
                } else {
                    println!("Found residual EspSpiDevice in device_with_handle but no corrosponding dev in devices")
                }
            }
        }
        let handle = self.handle;
        esp!(unsafe { spi_bus_free(handle) }).unwrap();
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
