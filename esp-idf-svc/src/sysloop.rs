use std::sync::Mutex;

use anyhow::*;

use esp_idf_sys::*;

lazy_static! {
    static ref INITIALIZED: Mutex<bool> = Mutex::new(false);
}

pub struct EspSysLoop;

impl EspSysLoop {
    pub fn new() -> Result<Self> {
        let mut initialized = INITIALIZED.lock().unwrap();

        if *initialized {
            bail!("System event loop is already owned elsewhere");
        }

        esp!(unsafe {esp_event_loop_create_default()})?;
        *initialized = true;

        Ok(EspSysLoop)
    }
}

impl Drop for EspSysLoop {
    fn drop(&mut self) {
        let mut initialized = INITIALIZED.lock().unwrap();

        esp!(unsafe {esp_event_loop_delete_default()}).unwrap();

        *initialized = false;
    }
}
