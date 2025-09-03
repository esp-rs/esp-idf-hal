#[cfg(not(esp32s2))]
pub use split::*;

use crate::impl_peripheral;

#[cfg(not(any(esp32h2, esp32h4)))]
pub trait WifiModemPeripheral {}

#[cfg(any(esp32h2, esp32h4, esp32c6))]
pub trait ThreadModemPeripheral {}

#[cfg(not(esp32s2))]
pub trait BluetoothModemPeripheral {}

impl_peripheral!(Modem);

#[allow(clippy::needless_lifetimes)]
impl<'d> Modem<'d> {
    #[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
    pub fn split(self) -> (WifiModem<'d>, BluetoothModem<'d>) {
        unsafe { (WifiModem::steal(), BluetoothModem::steal()) }
    }

    #[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
    pub fn split_reborrow(&mut self) -> (WifiModem<'_>, BluetoothModem<'_>) {
        unsafe { (WifiModem::steal(), BluetoothModem::steal()) }
    }

    #[cfg(any(esp32h2, esp32h4))]
    pub fn split(self) -> (ThreadModem<'d>, BluetoothModem<'d>) {
        unsafe { (ThreadModem::steal(), BluetoothModem::steal()) }
    }

    #[cfg(any(esp32h2, esp32h4))]
    pub fn split_reborrow(&mut self) -> (ThreadModem<'_>, BluetoothModem<'_>) {
        unsafe { (ThreadModem::steal(), BluetoothModem::steal()) }
    }

    #[cfg(esp32c6)]
    pub fn split(self) -> (WifiModem<'d>, ThreadModem<'d>, BluetoothModem<'d>) {
        unsafe {
            (
                WifiModem::steal(),
                ThreadModem::steal(),
                BluetoothModem::steal(),
            )
        }
    }

    #[cfg(esp32c6)]
    pub fn split_reborrow(&mut self) -> (WifiModem<'_>, ThreadModem<'_>, BluetoothModem<'_>) {
        unsafe {
            (
                WifiModem::steal(),
                ThreadModem::steal(),
                BluetoothModem::steal(),
            )
        }
    }
}

#[cfg(not(esp32h2))]
impl WifiModemPeripheral for Modem<'_> {}

#[cfg(any(esp32h2, esp32c6))]
impl ThreadModemPeripheral for Modem<'_> {}

#[cfg(not(esp32s2))]
impl BluetoothModemPeripheral for Modem<'_> {}

#[cfg(not(esp32s2))]
mod split {
    #[cfg(not(esp32h2))]
    crate::impl_peripheral!(WifiModem);

    #[cfg(not(esp32h2))]
    impl super::WifiModemPeripheral for WifiModem<'_> {}

    #[cfg(any(esp32h2, esp32c6))]
    crate::impl_peripheral!(ThreadModem);

    #[cfg(any(esp32h2, esp32c6))]
    impl super::ThreadModemPeripheral for ThreadModem<'_> {}

    crate::impl_peripheral!(BluetoothModem);

    impl super::BluetoothModemPeripheral for BluetoothModem<'_> {}
}
