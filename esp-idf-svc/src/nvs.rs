use core::ptr;

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use embedded_svc::storage::{RawStorage, StorageBase};

use esp_idf_sys::*;

use crate::handle::RawHandle;
use crate::private::cstr::*;
use crate::private::mutex;

static DEFAULT_TAKEN: mutex::Mutex<bool> = mutex::Mutex::wrap(mutex::RawMutex::new(), false);
static NONDEFAULT_LOCKED: mutex::Mutex<alloc::collections::BTreeSet<CString>> =
    mutex::Mutex::wrap(mutex::RawMutex::new(), alloc::collections::BTreeSet::new());

pub type EspDefaultNvsPartition = EspNvsPartition<NvsDefault>;
pub type EspCustomNvsPartition = EspNvsPartition<NvsCustom>;

pub trait NvsPartitionId {
    fn is_default(&self) -> bool {
        self.name().to_bytes().is_empty()
    }

    fn name(&self) -> &CStr;
}

pub struct NvsDefault(());

impl NvsDefault {
    fn new() -> Result<Self, EspError> {
        let mut taken = DEFAULT_TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE)?;
        }

        let default_nvs = Self::init()?;

        *taken = true;
        Ok(default_nvs)
    }

    fn init() -> Result<Self, EspError> {
        if let Some(err) = EspError::from(unsafe { nvs_flash_init() }) {
            match err.code() {
                ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                    esp!(unsafe { nvs_flash_erase() })?;
                    esp!(unsafe { nvs_flash_init() })?;
                }
                _ => (),
            }
        }

        Ok(Self(()))
    }
}

impl Drop for NvsDefault {
    fn drop(&mut self) {
        //esp!(nvs_flash_deinit()).unwrap(); TODO: To be checked why it fails
        *DEFAULT_TAKEN.lock() = false;

        info!("Dropped");
    }
}

impl NvsPartitionId for NvsDefault {
    fn name(&self) -> &CStr {
        CStr::from_bytes_with_nul(b"\0").unwrap()
    }
}

pub struct NvsCustom(CString);

impl NvsCustom {
    fn new(partition: &str) -> Result<Self, EspError> {
        let mut registrations = NONDEFAULT_LOCKED.lock();

        Self::init(partition, &mut registrations)
    }

    fn init(
        partition: &str,
        registrations: &mut alloc::collections::BTreeSet<CString>,
    ) -> Result<Self, EspError> {
        let c_partition = CString::new(partition).unwrap();

        if registrations.contains(c_partition.as_ref()) {
            return Err(EspError::from(ESP_ERR_INVALID_STATE).unwrap());
        }

        unsafe {
            if let Some(err) = EspError::from(nvs_flash_init_partition(c_partition.as_ptr())) {
                match err.code() {
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

impl Drop for NvsCustom {
    fn drop(&mut self) {
        {
            let mut registrations = NONDEFAULT_LOCKED.lock();

            esp!(unsafe { nvs_flash_deinit_partition(self.0.as_ptr()) }).unwrap();
            registrations.remove(self.0.as_ref());
        }

        info!("Dropped");
    }
}

impl NvsPartitionId for NvsCustom {
    fn name(&self) -> &CStr {
        self.0.as_c_str()
    }
}

#[derive(Debug)]
pub struct EspNvsPartition<T: NvsPartitionId>(Arc<T>);

impl EspNvsPartition<NvsDefault> {
    pub fn take() -> Result<Self, EspError> {
        Ok(Self(Arc::new(NvsDefault::new()?)))
    }
}

impl EspNvsPartition<NvsCustom> {
    pub fn take(partition: &str) -> Result<Self, EspError> {
        Ok(Self(Arc::new(NvsCustom::new(partition)?)))
    }
}

impl<T> Clone for EspNvsPartition<T>
where
    T: NvsPartitionId,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl RawHandle for EspNvsPartition<NvsCustom> {
    type Handle = *const u8;

    fn handle(&self) -> Self::Handle {
        self.0.name().as_ptr() as *const _
    }
}

pub type EspDefaultNvs = EspNvs<NvsDefault>;
pub type EspCustomNvs = EspNvs<NvsCustom>;

pub struct EspNvs<T: NvsPartitionId>(EspNvsPartition<T>, nvs_handle_t);

impl<T: NvsPartitionId> EspNvs<T> {
    pub fn new(
        partition: EspNvsPartition<T>,
        namespace: &str,
        read_write: bool,
    ) -> Result<Self, EspError> {
        let c_namespace = CString::new(namespace).unwrap();

        let mut handle: nvs_handle_t = 0;

        if partition.0.is_default() {
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
        } else {
            esp!(unsafe {
                nvs_open_from_partition(
                    partition.0.name().as_ptr(),
                    c_namespace.as_ptr(),
                    if read_write {
                        nvs_open_mode_t_NVS_READWRITE
                    } else {
                        nvs_open_mode_t_NVS_READONLY
                    },
                    &mut handle as *mut _,
                )
            })?;
        }

        Ok(Self(partition, handle))
    }

    pub fn contains(&self, name: &str) -> Result<bool, EspError> {
        self.len(name).map(|v| v.is_some())
    }

    pub fn remove(&mut self, name: &str) -> Result<bool, EspError> {
        let c_key = CString::new(name).unwrap();

        // nvs_erase_key is not scoped by datatype
        let result = unsafe { nvs_erase_key(self.1, c_key.as_ptr()) };

        if result == ESP_ERR_NVS_NOT_FOUND {
            Ok(false)
        } else {
            esp!(result)?;
            esp!(unsafe { nvs_commit(self.1) })?;

            Ok(true)
        }
    }

    fn len(&self, name: &str) -> Result<Option<usize>, EspError> {
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

    pub fn get_raw<'a>(&self, name: &str, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, EspError> {
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

                        Ok(Some(&buf[..len as usize]))
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

                Ok(Some(&buf[..len as usize]))
            }
        }
    }

    fn set_raw(&mut self, name: &str, buf: &[u8]) -> Result<bool, EspError> {
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

impl<T: NvsPartitionId> Drop for EspNvs<T> {
    fn drop(&mut self) {
        unsafe {
            nvs_close(self.1);
        }
    }
}

unsafe impl<T: NvsPartitionId> Send for EspNvs<T> {}

impl RawHandle for EspNvs<NvsCustom> {
    type Handle = nvs_handle_t;

    fn handle(&self) -> Self::Handle {
        self.1
    }
}

impl<T: NvsPartitionId> StorageBase for EspNvs<T> {
    type Error = EspError;

    fn contains(&self, name: &str) -> Result<bool, Self::Error> {
        EspNvs::contains(self, name)
    }

    fn remove(&mut self, name: &str) -> Result<bool, Self::Error> {
        EspNvs::remove(self, name)
    }
}

impl<T: NvsPartitionId> RawStorage for EspNvs<T> {
    fn len(&self, name: &str) -> Result<Option<usize>, Self::Error> {
        EspNvs::len(self, name)
    }

    fn get_raw<'a>(&self, name: &str, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Self::Error> {
        EspNvs::get_raw(self, name, buf)
    }

    fn set_raw(&mut self, name: &str, buf: &[u8]) -> Result<bool, Self::Error> {
        EspNvs::set_raw(self, name, buf)
    }
}
