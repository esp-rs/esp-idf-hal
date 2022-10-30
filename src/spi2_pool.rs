use crate::gpio::{AnyOutputPin, InputPin, Output, OutputPin, PinDriver};
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::spi2::{config, SpiBusMasterDriver, SpiMasterDriver};

use std::borrow::Borrow;
use std::collections::HashMap;
use std::marker::PhantomData;

use core::ptr;

use esp_idf_sys::*;

// TODO -> custom error that wraps around esperror
// cs default high low setting -
// needed to makte SpiBusMasterDriver fields pub because different module
// needed to make SpiBusMasterDriver flush fn pub

// performance measurment on esp32c3 -> 
// debug build: 50 uS deley between cs low -> first clk out
// release build : 15 uS deley between cs low -> first cll out

// debug build: 4uS delay between last clk out -> cs pin high
// release build: 3uS delay between last clk out -> cs pin high

#[cfg(not(esp32c3))]
const ESP_MAX_SPI_DEVICES: usize = 3;
#[cfg(esp32c3)]
const ESP_MAX_SPI_DEVICES: usize = 6;

pub struct SpiConfigPool<'d, T> {
    shared_handles: HashMap<u32, spi_device_handle_t>,
    master: T,
    _p: PhantomData<&'d ()>,
}

impl<'d, T> SpiConfigPool<'d, T> {
    pub fn new(master: T, device_configs: HashMap<u32, config::Config>) -> Result<Self, EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        if device_configs.len() > ESP_MAX_SPI_DEVICES {
            //let bla = Err::<EspError,&str>("Provided more than the maximum device configs");
            println!("Provided more than the maximum allowed device configs");
        }
        let mut shared_handles: HashMap<u32, spi_device_handle_t> = HashMap::new();
        let mut shared_configs: HashMap<u32, spi_device_interface_config_t> = HashMap::new();
        for (id, config) in device_configs {
            let config = Self::create_conf(&config);
            shared_configs.insert(id, config);
        }

        let master_ref: &SpiMasterDriver = master.borrow();
        if let Ok(lock) = master_ref.handle.lock() {
            for (id, config) in shared_configs {
                let handle = Self::register_bus_config(*lock, config)?;
                shared_handles.insert(id, handle);
            }
        }
        Ok(Self {
            shared_handles,
            master,
            _p: PhantomData,
        })
    }

    fn register_bus_config(
        host: spi_host_device_t,
        conf: spi_device_interface_config_t,
    ) -> Result<spi_device_handle_t, EspError> {
        let mut device_handle: spi_device_handle_t = ptr::null_mut();
        esp!(unsafe { spi_bus_add_device(host, &conf, &mut device_handle as *mut _) })?;
        Ok(device_handle)
    }

    fn create_conf(config: &config::Config) -> spi_device_interface_config_t {
        spi_device_interface_config_t {
            //spics_io_num: -1,
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

    fn rm_from_bus(&self) -> Result<(), EspError> {
        for (id, handle) in &self.shared_handles {
            let inner: spi_device_handle_t = *handle;
            esp!(unsafe { spi_bus_remove_device(inner) })?;
        }
        Ok(())
    }
}

impl<'d, T> Drop for SpiConfigPool<'d, T> {
    fn drop(&mut self) {
        self.rm_from_bus().unwrap();
    }
}

pub struct SpiPoolDevice<'d, T> {
    pool: T,
    pub pin_driver: PinDriver<'d, AnyOutputPin, Output>,
    config_id: u32,
    _p: PhantomData<&'d ()>,
}

impl<'d, T> SpiPoolDevice<'d, T>
where
    T: Borrow<SpiConfigPool<'d, &'d SpiMasterDriver<'d>>> + 'd,
{
    pub fn new(
        pool: T,
        config_id: u32,
        cs_pin: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let cs: PeripheralRef<AnyOutputPin> = cs_pin.into_ref().map_into();
        let mut pin_driver = PinDriver::output(cs)?;
        pin_driver.set_high()?;
        Ok(Self {
            pool,
            pin_driver,
            config_id,
            _p: PhantomData,
        })
    }

    pub fn transaction<R, E>(
        &mut self,
        f: impl FnOnce(&mut SpiBusMasterDriver<'d>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
        //X: Borrow<&SpiMasterDriver<'d>> + 'd,
    {
        // get config_handle for our device
        let pool: &SpiConfigPool<&SpiMasterDriver> = self.pool.borrow();
        // create error here when device has wrong id
        let device_handle:&spi_device_handle_t = pool.shared_handles.get(&self.config_id).unwrap();

        // lock the bus through the mutex in SpiMasterDriver
        let master: &SpiMasterDriver = pool.master.borrow();
        // should we check for poison here?
        let driver_lock = master.handle.lock().unwrap();

        //self.pin_driver.set_low()?;

        //let handle = self.handle.clone().into_inner().unwrap();
        let mut bus = SpiBusMasterDriver {
            handle: *device_handle,
            trans_len: SOC_SPI_MAXIMUM_BUFFER_SIZE as usize,
            hardware_cs: false,
            _p: PhantomData,
        };
        self.pin_driver.set_low()?;
        let trans_result = f(&mut bus);

        let finish_result = bus.finish();

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        let flush_result = bus.flush();

        self.pin_driver.set_high()?;

        drop(driver_lock);
        println!("after lock drop");
        let result = trans_result?;
        println!("after trans result");
        flush_result?;
        println!("flush and finish");
        finish_result?;
        println!("end Transaction");
        Ok(result)
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.transfer(read, write))
    }

    pub fn write(&mut self, write: &[u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.write(write))
    }

    pub fn read(&mut self, read: &mut [u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.read(read))
    }

    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        self.transaction(|bus| bus.transfer_in_place(buf))
    }
}
