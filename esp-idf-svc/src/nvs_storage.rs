use core::{any::Any, ptr};

extern crate alloc;
use alloc::vec;
use alloc::sync::Arc;

use embedded_svc::storage::Storage;

use esp_idf_sys::*;

use crate::nvs::*;

use crate::private::cstr::*;

pub struct EspNvsStorage(Arc<dyn Any>, nvs_handle_t);

impl EspNvsStorage {
    pub fn new_default(default_nvs: Arc<EspDefaultNvs>, namespace: impl AsRef<str>, read_write: bool) -> Result<Self, EspError> {
        let c_namespace = CString::new(namespace.as_ref()).unwrap();

        let mut handle: nvs_handle_t = 0;
        esp!(unsafe {nvs_open(
            c_namespace.as_ptr(),
            if read_write {nvs_open_mode_t_NVS_READWRITE} else {nvs_open_mode_t_NVS_READONLY},
            &mut handle as *mut _)})?;

        Ok(Self(default_nvs, handle))
    }

    pub fn new(nvs: Arc<EspNvs>, namespace: impl AsRef<str>, read_write: bool) -> Result<Self, EspError> {
        let c_namespace = CString::new(namespace.as_ref()).unwrap();

        let mut handle: nvs_handle_t = 0;
        esp!(unsafe {nvs_open_from_partition(
            nvs.0.as_ptr(),
            c_namespace.as_ptr(),
            if read_write {nvs_open_mode_t_NVS_READWRITE} else {nvs_open_mode_t_NVS_READONLY},
            &mut handle as *mut _)})?;

        Ok(Self(nvs, handle))
    }
}

impl Drop for EspNvsStorage {
    fn drop(&mut self) {
        unsafe {nvs_close(self.1);}
    }
}

impl Storage for EspNvsStorage {
    type Error = EspError;

    fn contains(&self, key: impl AsRef<str>) -> Result<bool, Self::Error> {
        let c_key = CString::new(key.as_ref()).unwrap();

        let dummy: u_int64_t = 0;

        match unsafe {nvs_get_u64(self.1, c_key.as_ptr(), dummy as *mut _)} as u32 {
            ESP_ERR_NVS_NOT_FOUND => Ok(false),
            ESP_ERR_NVS_INVALID_LENGTH => Ok(true),
            result => {
                esp!(result)?;
                Ok(true)
            }
        }
    }

    fn remove(&mut self, key: impl AsRef<str>) -> Result<bool, Self::Error> {
        let c_key = CString::new(key.as_ref()).unwrap();

        let result = unsafe {nvs_erase_key(self.1, c_key.as_ptr())};

        if result == ESP_ERR_NVS_NOT_FOUND as i32 {
            Ok(false)
        } else {
            esp!(result)?;
            esp!(unsafe {nvs_commit(self.1)})?;

            Ok(true)
        }
    }

    fn get_raw(&self, key: impl AsRef<str>) -> Result<Option<vec::Vec<u8>>, Self::Error> {
        let c_key = CString::new(key.as_ref()).unwrap();

        let mut value: u_int64_t = 0;

        match unsafe {nvs_get_u64(self.1, c_key.as_ptr(), value as *mut _)} as u32 {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            ESP_ERR_NVS_INVALID_LENGTH => {
                let mut len: size_t = 0;

                esp!(unsafe {nvs_get_blob(self.1, c_key.as_ptr(), ptr::null_mut(), &mut len as *mut _)})?;

                let mut vec: vec::Vec<u8> = vec::Vec::with_capacity(len as usize);
                esp!(unsafe {nvs_get_blob(self.1, c_key.as_ptr(), vec.as_mut_ptr() as *mut _, &mut len as *mut _)})?;

                unsafe {vec.set_len(len as usize)};

                Ok(Some(vec))
            },
            result => {
                esp!(result)?;

                let len: u8 = (value & 0xff) as u8;
                value >>= 8;

                let array: [u8; 7] = [
                    (value & 0xff) as u8,
                    ((value >> 8) & 0xff) as u8,
                    ((value >> 16) & 0xff) as u8,
                    ((value >> 24) & 0xff) as u8,
                    ((value >> 32) & 0xff) as u8,
                    ((value >> 48) & 0xff) as u8,
                    ((value >> 56) & 0xff) as u8
                ];

                Ok(Some(array[..len as usize].to_vec()))
            }
        }
    }

    fn put_raw(&mut self, key: impl AsRef<str>, value: impl Into<vec::Vec<u8>>) -> Result<bool, Self::Error> {
        let c_key = CString::new(key.as_ref()).unwrap();
        let mut uvalue: u_int64_t = 0;

        let small: bool;
        let found: bool;

        match unsafe {nvs_get_u64(self.1, c_key.as_ptr(), uvalue as *mut _) as u32} {
            ESP_ERR_NVS_NOT_FOUND => {found = false; small = false;},
            ESP_ERR_NVS_INVALID_LENGTH => {found = true; small = false;},
            result => {esp!(result)?; found = true; small = true;}
        }

        let mut value = value.into();
        let new_small = value.len() < 8;

        if found && small != new_small {
            esp!(unsafe {nvs_erase_key(self.1, c_key.as_ptr())})?;
        }

        if new_small {
            for v in value.iter().rev() {
                uvalue <<= 8;
                uvalue = uvalue | *v as u_int64_t;
            }

            uvalue <<= 8;
            uvalue = uvalue | value.len() as u_int64_t;

            esp!(unsafe {nvs_set_u64(self.1, c_key.as_ptr(), uvalue)})?;
        } else {
            esp!(unsafe {nvs_set_blob(self.1, c_key.as_ptr(), value.as_mut_ptr() as *mut _, value.len() as u32)})?;
        }

        esp!(unsafe {nvs_commit(self.1)})?;

        Ok(found)
    }
}