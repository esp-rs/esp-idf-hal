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
//! See the `examples/` folder of this repository for more.

use core::borrow::Borrow;
use core::marker::PhantomData;
use core::ptr;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;

#[derive(Debug)]
pub struct OWDevice<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    _addr: u64,
    _bus: T,
    _p: PhantomData<&'b ()>,
}

impl<'b, T> OWDevice<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    pub fn new(addr: u64, bus: T) -> Self {
        Self {
            _addr: addr,
            _bus: bus,
            _p: PhantomData,
        }
    }

    /// get the handle of the bus the device is attached to
    pub fn bus(&self) -> &T {
        &self._bus
    }

    /// get the device address
    pub fn address(&self) -> u64 {
        self._addr
    }
}

/// wrapper around the device iterator to search for available devices on the bus
pub struct DeviceSearch<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    _search: onewire_device_iter_handle_t,
    _bus: T,
    _p: PhantomData<&'b ()>,
}

impl<'b, T> DeviceSearch<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    fn new(bus: T) -> Result<Self, EspError> {
        let mut my_iter: onewire_device_iter_handle_t = ptr::null_mut();

        esp!(unsafe { onewire_new_device_iter(bus.borrow()._bus, &mut my_iter) })?;

        Ok(Self {
            _search: my_iter,
            _bus: bus,
            _p: PhantomData,
        })
    }

    /// Search for the next device on the bus and yield it.
    fn next_device(&self) -> Result<u64, EspError> {
        let mut next_onewire_device = onewire_device_t::default();
        esp!(unsafe { onewire_device_iter_get_next(self._search, &mut next_onewire_device) })?;
        Ok(next_onewire_device.address)
        // Ok(Device::new(next_onewire_device, self._bus.borrow()))
    }
}

impl<'b, T> Iterator for DeviceSearch<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    type Item = u64;

    fn next(&mut self) -> Option<Self::Item> {
        if let Ok(dev) = self.next_device() {
            Some(dev)
        } else {
            None
        }
    }
}

impl<'b, T> Drop for DeviceSearch<'b, T>
where
    T: Borrow<OWDriver<'b>>,
{
    fn drop(&mut self) {
        esp!(unsafe { onewire_del_device_iter(self._search) }).unwrap();
    }
}
#[derive(Debug)]

pub struct OWDriver<'d> {
    _bus: onewire_bus_handle_t,
    _p: PhantomData<&'d ()>,
}

impl<'d> OWDriver<'d> {
    /// Create a new One Wire driver on the allocated pin.
    ///
    /// The pin will be used as an open drain output.
    pub fn new(
        pin: impl Peripheral<P = impl crate::gpio::InputPin + crate::gpio::OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let mut bus: onewire_bus_handle_t = ptr::null_mut();

        let pin = pin.into_ref().pin();
        let bus_config = esp_idf_sys::onewire_bus_config_t { bus_gpio_num: pin };

        let rmt_config = esp_idf_sys::onewire_bus_rmt_config_t { max_rx_bytes: 10 };

        esp!(unsafe { onewire_new_bus_rmt(&bus_config, &rmt_config, &mut bus as _) })?;

        Ok(Self {
            _bus: bus,
            _p: PhantomData,
        })
    }

    pub fn read(&self, buff: &mut [u8]) -> Result<usize, EspError> {
        esp!(unsafe {
            onewire_bus_read_bytes(self._bus, buff.as_mut_ptr() as *mut _, buff.len())
        })?;

        Ok(buff.len())
    }

    pub fn write(&self, data: &[u8]) -> Result<usize, EspError> {
        esp!(unsafe { onewire_bus_write_bytes(self._bus, data.as_ptr(), data.len() as u8) })?;

        Ok(data.len())
    }
    /// Send reset pulse to the bus, and check if there are devices attached to the bus
    ///
    /// If there are no devices on the bus, this will result in an error.
    pub fn reset(&self) -> Result<(), EspError> {
        esp!(unsafe { onewire_bus_reset(self._bus) })
    }

    pub fn search(&self) -> Result<DeviceSearch<impl Borrow<OWDriver<'_>>>, EspError> {
        DeviceSearch::new(self)
    }

    // pub fn search(&self) -> Result<impl Iterator<Item = u64> + '_, EspError> {
    //     DeviceSearch::new(self)
    // }
}

impl<'d> Drop for OWDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { onewire_bus_del(self._bus) }).unwrap();
    }
}

// command codes
#[repr(u8)]
pub enum OWCommand {
    Search = 0xf0,
    MatchRom = 0x55,
    SkipRom = 0xcc,
    SearchAlarm = 0xec,
    ReadPowerSupply = 0xb4,
}
