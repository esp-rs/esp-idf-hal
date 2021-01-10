use std::sync::Mutex;

use anyhow::*;

use esp_idf_sys::*;

lazy_static! {
    static ref INITIALIZED: Mutex<bool> = Mutex::new(false);
}

pub struct EspDefaultNvs;

impl EspDefaultNvs {
    pub fn new() -> Result<Self> {
        let mut initialized = INITIALIZED.lock().unwrap();

        if *initialized {
            bail!("Default NVS storage is already owned elsewhere");
        }

        if !*initialized {
            unsafe {
                if let Some(err) = EspError::from(nvs_flash_init()) {
                    match err.code() as u32 {
                        ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                            esp!(nvs_flash_erase())?;
                            esp!(nvs_flash_init())?;
                        },
                        _ => ()
                    }
                }
            }

            *initialized = true;
        }

        Ok(EspDefaultNvs)
    }
}

impl Drop for EspDefaultNvs {
    fn drop(&mut self) {
        let mut initialized = INITIALIZED.lock().unwrap();

        esp!(unsafe {nvs_flash_deinit()}).unwrap();

        *initialized = false;
    }
}
