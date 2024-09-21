use core::marker::PhantomData;

use crate::peripheral::{sealed, Peripheral};

#[cfg(not(esp32s2))]
pub use split::*;

#[cfg(not(any(esp32h2, esp32h4)))]
pub trait WifiModemPeripheral: Peripheral<P = Self> {}

#[cfg(any(esp32h2, esp32h4, esp32c6))]
pub trait ThreadModemPeripheral: Peripheral<P = Self> {}

#[cfg(not(esp32s2))]
pub trait BluetoothModemPeripheral: Peripheral<P = Self> {}

#[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
pub struct Modem(PhantomData<*const ()>, WifiModem, BluetoothModem);

#[cfg(any(esp32h2, esp32h4))]
pub struct Modem(PhantomData<*const ()>, ThreadModem, BluetoothModem);

#[cfg(esp32c6)]
pub struct Modem(
    PhantomData<*const ()>,
    WifiModem,
    ThreadModem,
    BluetoothModem,
);

#[cfg(esp32s2)]
pub struct Modem(PhantomData<*const ()>);

impl Modem {
    /// # Safety
    ///
    /// Care should be taken not to instantiate this Mac instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        #[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
        let this = Modem(PhantomData, WifiModem::new(), BluetoothModem::new());

        #[cfg(any(esp32h2, esp32h4))]
        let this = Modem(PhantomData, ThreadModem::new(), BluetoothModem::new());

        #[cfg(esp32c6)]
        let this = Modem(
            PhantomData,
            WifiModem::new(),
            ThreadModem::new(),
            BluetoothModem::new(),
        );

        #[cfg(esp32s2)]
        let this = Modem(PhantomData);

        this
    }

    #[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
    pub fn split(self) -> (WifiModem, BluetoothModem) {
        unsafe { (WifiModem::new(), BluetoothModem::new()) }
    }

    #[cfg(not(any(esp32s2, esp32h2, esp32h4, esp32c6)))]
    pub fn split_ref(&mut self) -> (&mut WifiModem, &mut BluetoothModem) {
        (&mut self.1, &mut self.2)
    }

    #[cfg(any(esp32h2, esp32h4))]
    pub fn split(self) -> (ThreadModem, BluetoothModem) {
        unsafe { (ThreadModem::new(), BluetoothModem::new()) }
    }

    #[cfg(any(esp32h2, esp32h4))]
    pub fn split_ref(&mut self) -> (&mut ThreadModem, &mut BluetoothModem) {
        (&mut self.1, &mut self.2)
    }

    #[cfg(esp32c6)]
    pub fn split(self) -> (WifiModem, ThreadModem, BluetoothModem) {
        unsafe { (WifiModem::new(), ThreadModem::new(), BluetoothModem::new()) }
    }

    #[cfg(esp32c6)]
    pub fn split_ref(&mut self) -> (&mut WifiModem, &mut ThreadModem, &mut BluetoothModem) {
        (&mut self.1, &mut self.2, &mut self.3)
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

#[cfg(not(esp32h2))]
impl WifiModemPeripheral for Modem {}

#[cfg(any(esp32h2, esp32c6))]
impl ThreadModemPeripheral for Modem {}

#[cfg(not(esp32s2))]
impl BluetoothModemPeripheral for Modem {}

#[cfg(not(esp32s2))]
mod split {
    #[cfg(not(esp32h2))]
    crate::impl_peripheral!(WifiModem);

    #[cfg(not(esp32h2))]
    impl super::WifiModemPeripheral for WifiModem {}

    #[cfg(any(esp32h2, esp32c6))]
    crate::impl_peripheral!(ThreadModem);

    #[cfg(any(esp32h2, esp32c6))]
    impl super::ThreadModemPeripheral for ThreadModem {}

    crate::impl_peripheral!(BluetoothModem);

    impl super::BluetoothModemPeripheral for BluetoothModem {}
}
