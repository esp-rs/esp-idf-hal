use ::log::*;

use esp_idf_hal::mutex;

use esp_idf_sys::*;

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspSysLoopStack(PrivateData);

impl EspSysLoopStack {
    pub fn new() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        esp!(unsafe { esp_event_loop_create_default() })?;

        *taken = true;
        Ok(EspSysLoopStack(PrivateData))
    }
}

impl Drop for EspSysLoopStack {
    fn drop(&mut self) {
        {
            let mut taken = TAKEN.lock();

            esp!(unsafe { esp_event_loop_delete_default() }).unwrap();
            *taken = false;
        }

        info!("Dropped");
    }
}
