use core::marker::PhantomData;
use core::ptr;

use esp_idf_sys::*;

use crate::peripheral::Peripheral;

pub struct Device {
    _device: onewire::onewire_device_t,
}

/// wrapper around the device iterator to search for available devices on the bus
pub struct DeviceSearch {
    _search: onewire::onewire_device_iter_handle_t,
}

impl DeviceSearch {
    pub fn new(bus: &mut BusDriver) -> Result<Self, EspError> {
        let mut my_iter: onewire::onewire_device_iter_handle_t = ptr::null_mut();

        esp!(unsafe { onewire::onewire_new_device_iter(bus._bus, &mut my_iter) })?;

        Ok(Self { _search: my_iter })
    }

    pub fn next_device(&mut self) -> Result<Device, EspError> {
        let mut next_onewire_device = onewire::onewire_device_t::default();
        esp!(unsafe {
            onewire::onewire_device_iter_get_next(self._search, &mut next_onewire_device)
        })?;

        Ok(Device {
            _device: next_onewire_device,
        })asd
    }
}

/// beep beep
pub struct BusDriver<'d> {
    _bus: onewire::onewire_bus_handle_t,
    _p: PhantomData<&'d ()>,
}

impl<'d> BusDriver<'d> {
    pub fn new(
        pin: impl Peripheral<P = impl crate::gpio::InputPin + crate::gpio::OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let mut bus: onewire::onewire_bus_handle_t = ptr::null_mut();

        let pin = pin.into_ref().pin();
        let bus_config = esp_idf_sys::onewire::onewire_bus_config_t { bus_gpio_num: pin };

        let rmt_config = esp_idf_sys::onewire::onewire_bus_rmt_config_t { max_rx_bytes: 10 };

        esp!(unsafe { onewire::onewire_new_bus_rmt(&bus_config, &rmt_config, &mut bus as _) })?;

        Ok(Self {
            _bus: bus,
            _p: PhantomData,
        })
    }

    pub fn search(&mut self) -> Result<DeviceSearch, EspError> {
        DeviceSearch::new(self)
    }
}
