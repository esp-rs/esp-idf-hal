use std::sync::Mutex;

use anyhow::*;

use esp_idf_sys::*;

lazy_static! {
    static ref INITIALIZED: Mutex<(bool, bool)> = Mutex::new((false, false));
}

pub struct EspNetif;

impl EspNetif {
    pub fn new() -> Result<Self> {
        let mut initialized = INITIALIZED.lock().unwrap();

        if initialized.0 {
            bail!("Netif is already owned elsewhere");
        }

        if !initialized.1 {
            esp!(unsafe {esp_netif_init()})?;
        }

        *initialized = (true, true);

        Ok(EspNetif)
    }
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        let mut initialized = INITIALIZED.lock().unwrap();

        // ESP netif does not support deinitialization yet, so we only flag that it is no longer owned
        initialized.0 = false;
    }
}
