use core::marker::PhantomData;

use crate::peripheral::{sealed, Peripheral};

#[cfg(not(esp32s2))]
pub use split::*;

pub trait WifiModemPeripheral: Peripheral<P = Self> {}

#[cfg(not(esp32s2))]
pub trait BluetoothModemPeripheral: Peripheral<P = Self> {}

#[cfg(not(esp32s2))]
pub struct Modem(PhantomData<*const ()>, WifiModem, BluetoothModem);

#[cfg(esp32s2)]
pub struct Modem(PhantomData<*const ()>);

impl Modem {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this Mac instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        #[cfg(not(esp32s2))]
        let this = Modem(PhantomData, WifiModem::new(), BluetoothModem::new());

        #[cfg(esp32s2)]
        let this = Modem(PhantomData);

        this
    }

    #[cfg(all(not(esp32s2), esp_idf_esp32_wifi_sw_coexist_enable))]
    pub fn split(self) -> (WifiModem, BluetoothModem) {
        unsafe { (WifiModem::new(), BluetoothModem::new()) }
    }

    #[cfg(all(not(esp32s2), esp_idf_esp32_wifi_sw_coexist_enable))]
    pub fn split_ref(&mut self) -> (&mut WifiModem, &mut BluetoothModem) {
        (&mut self.1, &mut self.2)
    }
}

unsafe impl Send for Modem {}

impl sealed::Sealed for Modem {}

impl Peripheral for Modem {
    type P = Self;

    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self::new()
    }
}

impl WifiModemPeripheral for Modem {}

#[cfg(not(esp32s2))]
impl BluetoothModemPeripheral for Modem {}

#[cfg(not(esp32s2))]
mod split {
    crate::impl_peripheral!(WifiModem);

    impl super::WifiModemPeripheral for WifiModem {}

    crate::impl_peripheral!(BluetoothModem);

    impl super::BluetoothModemPeripheral for BluetoothModem {}
}
