use crate::gpio::{AnyOutputPin, InputPin, Output, OutputPin, PinDriver};
use crate::peripheral::{Peripheral, PeripheralRef};
use crate::spi::{config, SpiBusMasterDriver, SpiMasterDriver};
use crate::task::CriticalSection;

use core::borrow::Borrow;
use core::marker::PhantomData;
use heapless::Vec;

use core::ptr;

use esp_idf_sys::*;

// TODO -> custom error that wraps around esperror
// cs default high low setting -
// needed to makte SpiBusMasterDriver fields pub because different module
// needed to make SpiBusMasterDriver flush fn pub

// TODO after interupt support of spi driver->
// try utilize transaction_cb_t pre_cb transaction_cb_t post_cb
// for software cs to boost performance

// performance measurment on esp32c3 ->
// debug build: 50 uS deley between cs low -> first clk out
// release build : 15 uS deley between cs low -> first clk out

// debug build: 4uS delay between last clk out -> cs pin high
// release build: 3uS delay between last clk out -> cs pin high

#[cfg(not(esp32c3))]
const ESP_MAX_SPI_DEVICES: usize = 3;
#[cfg(esp32c3)]
const ESP_MAX_SPI_DEVICES: usize = 6;

pub struct SpiConfigPool<'d, T> {
    shared_handles: [(u32, spi_device_handle_t); ESP_MAX_SPI_DEVICES],
    master: T,
    lock: CriticalSection,
    _p: PhantomData<&'d ()>,
}

impl<'d, T> SpiConfigPool<'d, T> {   
    pub fn new(master: T, device_configs: &[(u32, config::Config)]) -> Result<Self, EspError>
    where
        T: Borrow<SpiMasterDriver<'d>> + 'd,
    {
        if device_configs.len() > ESP_MAX_SPI_DEVICES {
            panic!("Provided more than the maximum allowed device configs");
        }
        let mut shared_handles: [(u32, spi_device_handle_t); ESP_MAX_SPI_DEVICES] = [(0,ptr::null_mut());ESP_MAX_SPI_DEVICES];
        let mut shared_configs:Vec<(u32, spi_device_interface_config_t),ESP_MAX_SPI_DEVICES> = Vec::new();
        for (id,config) in device_configs{

            let config = Self::create_conf(config);
            // vector should be always big enough because we cannot push more than ESP_MAX_SPI_DEVICES -> unwrap
            shared_configs.push((*id, config)).unwrap();
        }

        let master_ref: &SpiMasterDriver = master.borrow();
        {            
            for (idx, (id, config)) in shared_configs.into_iter().enumerate() {
                let handle = Self::register_bus_config(master_ref.handle, config)?;
                shared_handles[idx] = (id, handle);
            }

        }
        Ok(Self {
            shared_handles,
            master,
            lock: CriticalSection::new(),
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
        for (_id, handle) in &self.shared_handles {
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
    pin_driver: PinDriver<'d, AnyOutputPin, Output>,
    pool_config_id: u32,
    _p: PhantomData<&'d ()>,
}

impl<'d, T> SpiPoolDevice<'d, T>
where
    T: Borrow<SpiConfigPool<'d, &'d SpiMasterDriver<'d>>> + 'd,
{
    pub fn new(
        pool: T,
        pool_config_id: u32,
        cs_pin: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let cs: PeripheralRef<AnyOutputPin> = cs_pin.into_ref().map_into();
        let mut pin_driver = PinDriver::output(cs)?;
        // driver assumes here cs active low -> Todo impl SPI_DEVICE_POSITIVE_CS
        pin_driver.set_high()?;
        Ok(Self {
            pool,
            pin_driver,
            pool_config_id,
            _p: PhantomData,
        })
    }

    pub fn transaction<R, E>(
        &mut self,
        f: impl FnOnce(&mut SpiBusMasterDriver<'d>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<EspError>,
    {
        // get config_handle for our device
        let pool: &SpiConfigPool<&SpiMasterDriver> = self.pool.borrow();
        // create error here when device has wrong id
        // find handle in list of handles 
        let mut device_handle: Option<spi_device_handle_t> = None;
        //pool.shared_handles.get(&self.pool_config_id).unwrap();
        for (id,handle) in pool.shared_handles {
            if id == self.pool_config_id {
                device_handle = Some(handle);
                break;
            }
        }
        if device_handle.is_none() {
            panic!("Stored Config ID not found in SpiConfPool")
        }

        // ensure exlusive usage through the CriticalSection in ConfigPool
        let _guard = pool.lock.enter();
        let master: &SpiMasterDriver = pool.master.borrow();

        // if DMA used -> get trans length info from master
        let trans_len = master.max_transfer_size;

        let mut bus = SpiBusMasterDriver {
            handle: device_handle.unwrap(),
            trans_len,
            hardware_cs: false,
            _p: PhantomData,
        };
        // assuming the driver got init correctly ->
        // we just need to toggle and can allow both Modes
        // without an editional Mode check
        // where  Modes: ( cs active low or high )
        self.pin_driver.toggle()?;
        let trans_result = f(&mut bus);

        let finish_result = bus.finish();

        // Flush whatever is pending.
        // Note that this is done even when an error is returned from the transaction.
        let flush_result = bus.flush();

        self.pin_driver.toggle()?;

        drop(_guard);

        let result = trans_result?;
        flush_result?;
        finish_result?;

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

    pub fn cs_gpio_number(&self) -> i32 {
        self.pin_driver.pin()
    }
}
