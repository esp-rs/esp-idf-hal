use log::*;

use mutex_trait::*;

use esp_idf_sys::*;

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspSysLoop(PrivateData);

impl EspSysLoop {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken|
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    esp!(esp_event_loop_create_default())?;

                    *taken = true;
                    Ok(EspSysLoop(PrivateData))
                }
            )
        }
    }
}

impl Drop for EspSysLoop {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                esp!(esp_event_loop_delete_default()).unwrap();
                *taken = false;
            });
        }

        info!("Dropped");
    }
}
