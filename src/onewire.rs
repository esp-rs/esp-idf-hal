//! RMT-based Onewire Implementation
//!
//! The Onewire module driver can be used to communicate with onewire (1-Wire)
//! devices.
//!
//! This module is an abstraction around the esp-idf component [onewire_bus](https://components.espressif.com/components/espressif/onewire_bus)
//! implementation. It is recommended to read the usage of the C API in this [example](https://github.com/espressif/esp-idf/tree/v5.2.2/examples/peripherals/rmt/onewire)
//!
//!
//! This implementation currently supports the one-wire API from the new (v5) esp-idf API.
//!
//! The pin this peripheral is attached to must be
//! externally pulled-up with a 4.7kOhm resistor.
//!
//! todo:
//!  - crc checking on messages
//!  - helper methods on the driver for executing commands
//!
//! See the `examples/` folder of this repository for more.

use core::marker::PhantomData;
use core::ptr;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;
use crate::rmt::RmtChannel;

/// Onewire Address type
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct OWAddress(u64);

impl OWAddress {
    pub fn address(&self) -> u64 {
        self.0
    }

    pub fn family_code(&self) -> u8 {
        (self.0 & u64::from(0xffu8)) as u8
    }
}

/// Wrapper around a device iterator to search for available devices on the bus
pub struct DeviceSearch<'a, 'b> {
    search: onewire_device_iter_handle_t,
    _bus: &'a mut OWDriver<'b>,
}

impl<'a, 'b> DeviceSearch<'a, 'b> {
    fn new(bus: &'a mut OWDriver<'b>) -> Result<Self, EspError> {
        let mut my_iter: onewire_device_iter_handle_t = ptr::null_mut();

        esp!(unsafe { onewire_new_device_iter(bus.bus, &mut my_iter) })?;

        Ok(Self {
            search: my_iter,
            _bus: bus,
        })
    }

    /// Search for the next device on the bus and yield it.
    fn next_device(&mut self) -> Result<OWAddress, EspError> {
        let mut next_onewire_device = onewire_device_t::default();
        esp!(unsafe { onewire_device_iter_get_next(self.search, &mut next_onewire_device) })?;
        Ok(OWAddress(next_onewire_device.address))
    }
}

impl<'a, 'b> Iterator for DeviceSearch<'a, 'b> {
    type Item = Result<OWAddress, EspError>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.next_device() {
            Ok(addr) => Some(Ok(addr)),
            Err(err) if err.code() == ESP_ERR_NOT_FOUND => None,
            Err(err) => Some(Err(err)),
        }
    }
}

impl<'a, 'b> Drop for DeviceSearch<'a, 'b> {
    fn drop(&mut self) {
        esp!(unsafe { onewire_del_device_iter(self.search) }).unwrap();
    }
}

#[derive(Debug)]
pub struct OWDriver<'a> {
    bus: onewire_bus_handle_t,
    _channel: u8,
    _p: PhantomData<&'a mut ()>,
}

impl<'a> OWDriver<'a> {
    /// Create a new One Wire driver on the allocated pin.
    ///
    /// The pin will be used as an open drain output.
    pub fn new<C: RmtChannel>(
        pin: impl Peripheral<P = impl crate::gpio::InputPin + crate::gpio::OutputPin> + 'a,
        _channel: impl Peripheral<P = C> + 'a,
    ) -> Result<Self, EspError> {
        let mut bus: onewire_bus_handle_t = ptr::null_mut();

        let pin = pin.into_ref().pin();
        let bus_config = esp_idf_sys::onewire_bus_config_t { bus_gpio_num: pin };

        let rmt_config = esp_idf_sys::onewire_bus_rmt_config_t { max_rx_bytes: 10 };

        esp!(unsafe { onewire_new_bus_rmt(&bus_config, &rmt_config, &mut bus as _) })?;

        Ok(Self {
            bus,
            _channel: C::channel() as _,
            _p: PhantomData,
        })
    }

    pub fn read(&self, buff: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe { onewire_bus_read_bytes(self.bus, buff.as_mut_ptr() as *mut _, buff.len()) })?;

        Ok(())
    }

    pub fn write(&self, data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe { onewire_bus_write_bytes(self.bus, data.as_ptr(), data.len() as u8) })?;

        Ok(())
    }

    /// Send reset pulse to the bus, and check if there are devices attached to the bus
    ///
    /// If there are no devices on the bus, this will result in an error.
    pub fn reset(&self) -> Result<(), EspError> {
        esp!(unsafe { onewire_bus_reset(self.bus) })
    }

    /// Start a search for devices attached to the OneWire bus.
    pub fn search(&mut self) -> Result<DeviceSearch<'_, 'a>, EspError> {
        DeviceSearch::new(self)
    }
}

impl<'d> Drop for OWDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { onewire_bus_del(self.bus) }).unwrap();
    }
}

unsafe impl<'d> Send for OWDriver<'d> {}

/// Command codes
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum OWCommand {
    Search = 0xF0,      //Obtain IDs of all devices on the bus
    MatchRom = 0x55,    //Address specific device
    SkipRom = 0xCC,     //Skip addressing
    ReadRom = 0x33,     //Identification
    SearchAlarm = 0xEC, // Conditional search for all devices in an alarm state.
    ReadPowerSupply = 0xB4,
}
