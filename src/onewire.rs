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

use core::marker::PhantomData;
use core::ptr;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;

#[derive(Debug)]
pub struct Device<'b> {
    _device: onewire_device_t,
    // _bus: &'b BusDriver<'b>, // not sure how I would hold a reference to this aside from the handle
    _p: PhantomData<&'b ()>, // holds the lifetime since the device is linked to the lifetime of the bus
}

impl<'a> Device<'a> {
    fn new(device_handle: onewire_device_t) -> Self {
        Self {
            _device: device_handle,
            _p: PhantomData,
        }
    }

    /// get the handle of the bus the device is attached to
    pub fn bus(&self) -> onewire_bus_handle_t {
        self._device.bus
    }

    /// get the device address
    pub fn address(&self) -> u64 {
        self._device.address
    }
}

/// wrapper around the device iterator to search for available devices on the bus
pub struct DeviceSearch<'b> {
    _search: onewire_device_iter_handle_t,
    _p: PhantomData<&'b ()>, // holds the lifetime since the search is linked to the lifetime of the bus
}

impl<'b> DeviceSearch<'b> {
    fn new(bus: &mut OneWireBusDriver) -> Result<Self, EspError> {
        let mut my_iter: onewire_device_iter_handle_t = ptr::null_mut();

        esp!(unsafe { onewire_new_device_iter(bus._bus, &mut my_iter) })?;

        Ok(Self {
            _search: my_iter,
            _p: PhantomData,
        })
    }

    /// Search for the next device on the bus and yield it.
    fn next_device(&mut self) -> Result<Device<'b>, EspError> {
        let mut next_onewire_device = onewire_device_t::default();
        esp!(unsafe { onewire_device_iter_get_next(self._search, &mut next_onewire_device) })?;

        Ok(Device::new(next_onewire_device))
    }
}

impl<'b> Iterator for DeviceSearch<'b> {
    type Item = Device<'b>;

    fn next(&mut self) -> Option<Self::Item> {
        if let Ok(dev) = self.next_device() {
            Some(dev)
        } else {
            None
        }
    }
}

impl<'b> Drop for DeviceSearch<'b> {
    fn drop(&mut self) {
        esp!(unsafe { onewire_del_device_iter(self._search) }).unwrap();
    }
}

pub struct OneWireBusDriver<'d> {
    _bus: onewire_bus_handle_t,
    _p: PhantomData<&'d ()>,
}

impl<'d> OneWireBusDriver<'d> {
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
    /// send reset pulse to the bus
    pub fn reset_bus(&mut self) -> Result<(), EspError> {
        esp!(unsafe { onewire_bus_reset(self._bus) })
    }

    pub fn search(&mut self) -> Result<DeviceSearch, EspError> {
        DeviceSearch::new(self)
    }
}

impl<'d> Drop for OneWireBusDriver<'d> {
    fn drop(&mut self) {
        esp!(unsafe { onewire_bus_del(self._bus) }).unwrap();
    }
}
