use log::*;

use esp_idf_sys::*;

use mutex_trait::*;

static mut TAKEN: EspMutex<(bool, bool)> = EspMutex::new((false, false));

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspNetif(PrivateData);

impl EspNetif {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if taken.0 {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    if !taken.1 {
                        esp!(esp_netif_init())?;
                    }

                    *taken = (true, true);
                    Ok(EspNetif(PrivateData))
                }
            })
        }
    }
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                // ESP netif does not support deinitialization yet, so we only flag that it is no longer owned
                *taken = (false, true);
            });
        }

        info!("Dropped");
    }
}
