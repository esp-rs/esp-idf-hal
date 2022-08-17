#[cfg(not(esp32s2))]
pub use split::*;

pub trait WifiModemCap {}

pub trait BluetoothModemCap {}

pub struct Modem(::core::marker::PhantomData<*const ()>);

impl Modem {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this Mac instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Modem(::core::marker::PhantomData)
    }

    #[cfg(all(not(esp32s2), esp_idf_esp32_wifi_sw_coexist_enable))]
    pub fn split(self) -> (WifiModem, BluetoothModem) {
        unsafe { (WifiModem::new(), BluetoothModem::new()) }
    }

    #[cfg(all(not(esp32s2), esp_idf_esp32_wifi_sw_coexist_enable))]
    pub fn combine(_wifi_modem: WifiModem, _bt_modem: BluetoothModem) -> Self {
        unsafe { Self::new() }
    }
}

unsafe impl Send for Modem {}

impl WifiModemCap for Modem {}

#[cfg(not(esp32s2))]
impl BluetoothModemCap for Modem {}

#[cfg(not(esp32s2))]
mod split {
    pub struct WifiModem(::core::marker::PhantomData<*const ()>);

    unsafe impl Send for WifiModem {}

    impl super::WifiModemCap for WifiModem {}

    pub struct BluetoothModem(::core::marker::PhantomData<*const ()>);

    unsafe impl Send for BluetoothModem {}

    impl super::BluetoothModemCap for BluetoothModem {}
}
