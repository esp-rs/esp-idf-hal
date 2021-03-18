use std::{collections::HashSet, ffi::CString, sync::Mutex};

use anyhow::*;

use esp_idf_sys::*;

lazy_static! {
    static ref DEFAULT_INITIALIZED: Mutex<bool> = Mutex::new(false);
    static ref INITIALIZED: Mutex<HashSet<CString>> = Mutex::new(HashSet::new());
}

#[derive(Debug)]
pub struct EspDefaultNvs;

impl EspDefaultNvs {
    pub fn new() -> Result<Self> {
        let mut initialized = DEFAULT_INITIALIZED.lock().unwrap();

        if *initialized {
            bail!("Default NVS partition is already owned elsewhere");
        }

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

        Ok(Self)
    }
}

impl Drop for EspDefaultNvs {
    fn drop(&mut self) {
        let mut initialized = DEFAULT_INITIALIZED.lock().unwrap();

        esp!(unsafe {nvs_flash_deinit()}).unwrap();

        *initialized = false;
    }
}

#[derive(Debug)]
pub struct EspNvs(pub(crate) CString);

impl EspNvs {
    pub fn new(partition: impl AsRef<str>) -> Result<Self> {
        let c_partition = CString::new(partition.as_ref()).unwrap();

        let mut initialized = INITIALIZED.lock().unwrap();

        if initialized.contains(c_partition.as_ref()) {
            bail!("NVS partition {} is already owned elsewhere", partition.as_ref());
        }

        unsafe {
            if let Some(err) = EspError::from(nvs_flash_init_partition(c_partition.as_ptr())) {
                match err.code() as u32 {
                    ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                        esp!(nvs_flash_erase_partition(c_partition.as_ptr()))?;
                        esp!(nvs_flash_init_partition(c_partition.as_ptr()))?;
                    },
                    _ => ()
                }
            }
        }

        initialized.insert(c_partition.clone());

        Ok(Self(c_partition))
    }
}

impl Drop for EspNvs {
    fn drop(&mut self) {
        let mut initialized = INITIALIZED.lock().unwrap();

        esp!(unsafe {nvs_flash_deinit_partition(self.0.as_ptr())}).unwrap();

        initialized.remove(self.0.as_ref());
    }
}
