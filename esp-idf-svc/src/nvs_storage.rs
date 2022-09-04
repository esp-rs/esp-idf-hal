use core::ptr;

extern crate alloc;
use alloc::sync::Arc;

use embedded_svc::storage::{RawStorage, StorageBase};

use esp_idf_sys::*;

use crate::nvs::*;

use crate::private::cstr::*;

enum EspNvsRef {
    Default(Arc<EspDefaultNvs>),
    Nvs(Arc<EspNvs>),
}

pub struct EspNvsStorage(EspNvsRef, nvs_handle_t);

impl EspNvsStorage {
    pub fn new_default(
        default_nvs: Arc<EspDefaultNvs>,
        namespace: impl AsRef<str>,
        read_write: bool,
    ) -> Result<Self, EspError> {
        let c_namespace = CString::new(namespace.as_ref()).unwrap();

        let mut handle: nvs_handle_t = 0;
        esp!(unsafe {
            nvs_open(
                c_namespace.as_ptr(),
                if read_write {
                    nvs_open_mode_t_NVS_READWRITE
                } else {
                    nvs_open_mode_t_NVS_READONLY
                },
                &mut handle as *mut _,
            )
        })?;

        Ok(Self(EspNvsRef::Default(default_nvs), handle))
    }

    pub fn new(
        nvs: Arc<EspNvs>,
        namespace: impl AsRef<str>,
        read_write: bool,
    ) -> Result<Self, EspError> {
        let c_namespace = CString::new(namespace.as_ref()).unwrap();

        let mut handle: nvs_handle_t = 0;
        esp!(unsafe {
            nvs_open_from_partition(
                nvs.0.as_ptr(),
                c_namespace.as_ptr(),
                if read_write {
                    nvs_open_mode_t_NVS_READWRITE
                } else {
                    nvs_open_mode_t_NVS_READONLY
                },
                &mut handle as *mut _,
            )
        })?;

        Ok(Self(EspNvsRef::Nvs(nvs), handle))
    }
}

impl Drop for EspNvsStorage {
    fn drop(&mut self) {
        unsafe {
            nvs_close(self.1);
        }
    }
}

impl StorageBase for EspNvsStorage {
    type Error = EspError;

    fn contains(&self, name: &str) -> Result<bool, Self::Error> {
        self.len(name).map(|v| v.is_some())
    }

    fn remove(&mut self, name: &str) -> Result<bool, Self::Error> {
        let c_key = CString::new(name).unwrap();

        // nvs_erase_key is not scoped by datatype
        let result = unsafe { nvs_erase_key(self.1, c_key.as_ptr()) };

        if result == ESP_ERR_NVS_NOT_FOUND as i32 {
            Ok(false)
        } else {
            esp!(result)?;
            esp!(unsafe { nvs_commit(self.1) })?;

            Ok(true)
        }
    }
}

impl RawStorage for EspNvsStorage {
    fn len(&self, name: &str) -> Result<Option<usize>, Self::Error> {
        let c_key = CString::new(name).unwrap();

        let mut value: u_int64_t = 0;

        // check for u64 value
        match unsafe { nvs_get_u64(self.1, c_key.as_ptr(), &mut value as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => {
                // check for blob value, by getting blob length
                let mut len: size_t = 0;
                match unsafe {
                    nvs_get_blob(self.1, c_key.as_ptr(), ptr::null_mut(), &mut len as *mut _)
                } {
                    ESP_ERR_NVS_NOT_FOUND => Ok(None),
                    err => {
                        // bail on error
                        esp!(err)?;

                        Ok(Some(len as _))
                    }
                }
            }
            err => {
                // bail on error
                esp!(err)?;

                // u64 value was found, decode it
                let len: u8 = (value & 0xff) as u8;

                Ok(Some(len as _))
            }
        }
    }

    fn get_raw<'a>(
        &self,
        name: &str,
        buf: &'a mut [u8],
    ) -> Result<Option<(&'a [u8], usize)>, Self::Error> {
        let c_key = CString::new(name).unwrap();

        let mut u64value: u_int64_t = 0;

        // check for u64 value
        match unsafe { nvs_get_u64(self.1, c_key.as_ptr(), &mut u64value as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => {
                // check for blob value, by getting blob length
                let mut len: size_t = 0;
                match unsafe {
                    nvs_get_blob(self.1, c_key.as_ptr(), ptr::null_mut(), &mut len as *mut _)
                } {
                    ESP_ERR_NVS_NOT_FOUND => Ok(None),
                    err => {
                        // bail on error
                        esp!(err)?;

                        len = buf.len() as _;

                        // fetch value if no error
                        esp!(unsafe {
                            nvs_get_blob(
                                self.1,
                                c_key.as_ptr(),
                                buf.as_mut_ptr() as *mut _,
                                &mut len as *mut _,
                            )
                        })?;

                        Ok(Some((&buf[..len as usize], len as _)))
                    }
                }
            }
            err => {
                // bail on error
                esp!(err)?;

                // u64 value was found, decode it
                let len: u8 = (u64value & 0xff) as u8;

                if buf.len() < len as _ {
                    // Buffer not large enough
                    return Err(EspError::from(ESP_ERR_NVS_INVALID_LENGTH).unwrap());
                }

                u64value >>= 8;

                let array: [u8; 7] = [
                    (u64value & 0xff) as u8,
                    ((u64value >> 8) & 0xff) as u8,
                    ((u64value >> 16) & 0xff) as u8,
                    ((u64value >> 24) & 0xff) as u8,
                    ((u64value >> 32) & 0xff) as u8,
                    ((u64value >> 40) & 0xff) as u8,
                    ((u64value >> 48) & 0xff) as u8,
                ];

                buf[..len as usize].copy_from_slice(&array[..len as usize]);

                Ok(Some((&buf[..len as usize], len as _)))
            }
        }
    }

    fn put_raw(&mut self, name: &str, buf: &[u8]) -> Result<bool, Self::Error> {
        let c_key = CString::new(name).unwrap();
        let mut u64value: u_int64_t = 0;

        // start by just clearing this key
        unsafe { nvs_erase_key(self.1, c_key.as_ptr()) };

        if buf.len() < 8 {
            for v in buf.iter().rev() {
                u64value <<= 8;
                u64value |= *v as u_int64_t;
            }

            u64value <<= 8;
            u64value |= buf.len() as u_int64_t;

            esp!(unsafe { nvs_set_u64(self.1, c_key.as_ptr(), u64value) })?;
        } else {
            esp!(unsafe {
                nvs_set_blob(
                    self.1,
                    c_key.as_ptr(),
                    buf.as_ptr() as *mut _,
                    buf.len() as u32,
                )
            })?;
        }

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(true)
    }
}

// TODO
// impl Storage for EspNvsStorage {
//     fn get<'a, T>(&'a self, name: &str) -> Result<Option<T>, Self::Error>
//     where
//         T: serde::Deserialize<'a> {
//         todo!()
//     }

//     fn set<T>(&mut self, name: &str, value: &T) -> Result<bool, Self::Error>
//     where
//         T: serde::Serialize {
//         todo!()
//     }
// }
