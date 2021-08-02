extern crate alloc;

use log::*;

use mutex_trait::*;

use esp_idf_sys::*;

use crate::private::cstr::*;

static mut DEFAULT_TAKEN: EspMutex<bool> = EspMutex::new(false);
static mut NONDEFAULT_LOCKED: EspMutex<alloc::collections::BTreeSet<CString>> =
    EspMutex::new(alloc::collections::BTreeSet::new());

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspDefaultNvs(PrivateData);

impl EspDefaultNvs {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            DEFAULT_TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let default_nvs = Self::init()?;

                    *taken = true;
                    Ok(default_nvs)
                }
            })
        }
    }

    unsafe fn init() -> Result<Self, EspError> {
        if let Some(err) = EspError::from(nvs_flash_init()) {
            match err.code() as u32 {
                ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                    esp!(nvs_flash_erase())?;
                    esp!(nvs_flash_init())?;
                }
                _ => (),
            }
        }

        Ok(Self(PrivateData))
    }
}

impl Drop for EspDefaultNvs {
    fn drop(&mut self) {
        unsafe {
            DEFAULT_TAKEN.lock(|taken| {
                //esp!(nvs_flash_deinit()).unwrap(); TODO: To be checked why it fails
                *taken = false;
            });
        }

        info!("Dropped");
    }
}

#[derive(Debug)]
pub struct EspNvs(pub(crate) CString);

impl EspNvs {
    pub fn new(partition: impl AsRef<str>) -> Result<Self, EspError> {
        unsafe { NONDEFAULT_LOCKED.lock(|registrations| Self::init(partition, registrations)) }
    }

    fn init(
        partition: impl AsRef<str>,
        registrations: &mut alloc::collections::BTreeSet<CString>,
    ) -> Result<Self, EspError> {
        let c_partition = CString::new(partition.as_ref()).unwrap();

        if registrations.contains(c_partition.as_ref()) {
            return Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap());
        }

        unsafe {
            if let Some(err) = EspError::from(nvs_flash_init_partition(c_partition.as_ptr())) {
                match err.code() as u32 {
                    ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                        esp!(nvs_flash_erase_partition(c_partition.as_ptr()))?;
                        esp!(nvs_flash_init_partition(c_partition.as_ptr()))?;
                    }
                    _ => (),
                }
            }
        }

        registrations.insert(c_partition.clone());

        Ok(Self(c_partition))
    }
}

impl Drop for EspNvs {
    fn drop(&mut self) {
        unsafe {
            NONDEFAULT_LOCKED.lock(|registrations| {
                esp!(nvs_flash_deinit_partition(self.0.as_ptr())).unwrap();
                registrations.remove(self.0.as_ref());
            });
        }

        info!("Dropped");
    }
}
