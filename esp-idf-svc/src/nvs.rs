//! Non-Volatile Storage (NVS)
#[cfg(esp_idf_version_at_least_5_2_0)]
use core::marker::PhantomData;
use core::ptr;

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use embedded_svc::storage::{RawStorage, StorageBase};

use crate::sys::*;

use crate::handle::RawHandle;
use crate::private::cstr::*;
use crate::private::mutex;

static DEFAULT_TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);
static NONDEFAULT_LOCKED: mutex::Mutex<alloc::collections::BTreeSet<CString>> =
    mutex::Mutex::new(alloc::collections::BTreeSet::new());

pub type EspDefaultNvsPartition = EspNvsPartition<NvsDefault>;
pub type EspCustomNvsPartition = EspNvsPartition<NvsCustom>;
pub type EspEncryptedNvsPartition = EspNvsPartition<NvsEncrypted>;

pub trait NvsPartitionId {
    fn is_default(&self) -> bool {
        self.name().to_bytes().is_empty()
    }

    fn name(&self) -> &CStr;
}

pub struct NvsDefault(());

#[repr(u32)]
#[allow(non_upper_case_globals)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NvsDataType {
    U8 = nvs_type_t_NVS_TYPE_U8,
    I8 = nvs_type_t_NVS_TYPE_I8,
    U16 = nvs_type_t_NVS_TYPE_U16,
    I16 = nvs_type_t_NVS_TYPE_I16,
    U32 = nvs_type_t_NVS_TYPE_U32,
    I32 = nvs_type_t_NVS_TYPE_I32,
    U64 = nvs_type_t_NVS_TYPE_U64,
    I64 = nvs_type_t_NVS_TYPE_I64,
    Str = nvs_type_t_NVS_TYPE_STR,
    Blob = nvs_type_t_NVS_TYPE_BLOB,
}

#[allow(non_upper_case_globals)]
impl NvsDataType {
    /// Converts a `nvs_type_t` to an `NvsDataType`, returning `None` if the type is not recognized.
    #[must_use]
    pub fn from_nvs_type(nvs_type: nvs_type_t) -> Option<Self> {
        match nvs_type {
            nvs_type_t_NVS_TYPE_U8 => Some(Self::U8),
            nvs_type_t_NVS_TYPE_I8 => Some(Self::I8),
            nvs_type_t_NVS_TYPE_U16 => Some(Self::U16),
            nvs_type_t_NVS_TYPE_I16 => Some(Self::I16),
            nvs_type_t_NVS_TYPE_U32 => Some(Self::U32),
            nvs_type_t_NVS_TYPE_I32 => Some(Self::I32),
            nvs_type_t_NVS_TYPE_U64 => Some(Self::U64),
            nvs_type_t_NVS_TYPE_I64 => Some(Self::I64),
            nvs_type_t_NVS_TYPE_STR => Some(Self::Str),
            nvs_type_t_NVS_TYPE_BLOB => Some(Self::Blob),
            _ => None,
        }
    }
}

impl NvsDefault {
    fn new(reinit: bool) -> Result<Self, EspError> {
        let mut taken = DEFAULT_TAKEN.lock();

        if *taken {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        let default_nvs = Self::init(reinit)?;

        *taken = true;
        Ok(default_nvs)
    }

    fn init(reinit: bool) -> Result<Self, EspError> {
        if let Some(err) = EspError::from(unsafe { nvs_flash_init() }) {
            match err.code() {
                ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND if reinit => {
                    if err.code() == ESP_ERR_NVS_NEW_VERSION_FOUND {
                        warn!("NVS partition has a new version, erasing and re-initializing the partition");
                    } else {
                        warn!("NVS partition has no free pages, erasing and re-initializing the partition");
                    }

                    esp!(unsafe { nvs_flash_erase() })?;
                    esp!(unsafe { nvs_flash_init() })?;
                }
                _ => Err(err)?,
            }
        }

        Ok(Self(()))
    }
}

impl Drop for NvsDefault {
    fn drop(&mut self) {
        //esp!(nvs_flash_deinit()).unwrap(); TODO: To be checked why it fails
        *DEFAULT_TAKEN.lock() = false;

        info!("NvsDefault dropped");
    }
}

impl NvsPartitionId for NvsDefault {
    #[allow(clippy::manual_c_str_literals)]
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
        let c_partition = to_cstring_arg(partition)?;

        if registrations.contains(c_partition.as_ref()) {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        unsafe {
            if let Some(err) = EspError::from(nvs_flash_init_partition(c_partition.as_ptr())) {
                match err.code() {
                    ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => {
                        esp!(nvs_flash_erase_partition(c_partition.as_ptr()))?;
                        esp!(nvs_flash_init_partition(c_partition.as_ptr()))?;
                    }
                    _ => Err(err)?,
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

        info!("NvsCustom dropped");
    }
}

impl NvsPartitionId for NvsCustom {
    fn name(&self) -> &CStr {
        self.0.as_c_str()
    }
}
pub struct NvsEncrypted(CString);

impl NvsEncrypted {
    fn new(partition: &str, key_partition: Option<&str>) -> Result<Self, EspError> {
        let mut registrations = NONDEFAULT_LOCKED.lock();

        Self::init(partition, key_partition, &mut registrations)
    }

    fn init(
        partition: &str,
        key_partition: Option<&str>,
        registrations: &mut alloc::collections::BTreeSet<CString>,
    ) -> Result<Self, EspError> {
        let c_partition = to_cstring_arg(partition)?;

        if registrations.contains(c_partition.as_ref()) {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        let c_key_partition = if let Some(key_partition) = key_partition {
            Some(to_cstring_arg(key_partition)?)
        } else {
            None
        };

        let keys_partition_ptr = unsafe {
            esp_partition_find_first(
                esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS,
                match c_key_partition {
                    Some(ref v) => v.as_ptr(),
                    None => core::ptr::null(),
                },
            )
        };

        if keys_partition_ptr.is_null() {
            warn!("No NVS keys partition found");
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }

        let mut config = nvs_sec_cfg_t::default();
        match unsafe { nvs_flash_read_security_cfg(keys_partition_ptr, &mut config as *mut _) } {
            ESP_ERR_NVS_KEYS_NOT_INITIALIZED | ESP_ERR_NVS_CORRUPT_KEY_PART => {
                info!("Partition not initialized, generating keys");
                esp!(unsafe {
                    nvs_flash_generate_keys(keys_partition_ptr, &mut config as *mut _)
                })?;
            }
            other => esp!(other)?,
        }

        esp!(unsafe {
            nvs_flash_secure_init_partition(c_partition.as_ptr(), &mut config as *mut _)
        })?;

        registrations.insert(c_partition.clone());

        Ok(Self(c_partition))
    }
}

// These functions are copied from NvsCustom, maybe there's a way to write this in a shorter way?
impl Drop for NvsEncrypted {
    fn drop(&mut self) {
        {
            let mut registrations = NONDEFAULT_LOCKED.lock();

            esp!(unsafe { nvs_flash_deinit_partition(self.0.as_ptr()) }).unwrap();
            registrations.remove(self.0.as_ref());
        }

        info!("NvsEncrypted dropped");
    }
}

impl NvsPartitionId for NvsEncrypted {
    fn name(&self) -> &CStr {
        self.0.as_c_str()
    }
}

#[derive(Debug)]
pub struct EspNvsPartition<T: NvsPartitionId>(Arc<T>);

impl EspNvsPartition<NvsDefault> {
    /// Take the default NVS partition, initializing it if full or if a new version is detected
    pub fn take() -> Result<Self, EspError> {
        Self::take_with(true)
    }

    /// Take the default NVS partition
    ///
    /// # Arguments
    /// - `reinit`: Whether to reinitialize the partition if full or if a new version is detected
    pub fn take_with(reinit: bool) -> Result<Self, EspError> {
        Ok(Self(Arc::new(NvsDefault::new(reinit)?)))
    }
}

impl EspNvsPartition<NvsCustom> {
    pub fn take(partition: &str) -> Result<Self, EspError> {
        Ok(Self(Arc::new(NvsCustom::new(partition)?)))
    }
}

impl EspNvsPartition<NvsEncrypted> {
    pub fn take(partition: &str, keys_partition: Option<&str>) -> Result<Self, EspError> {
        Ok(Self(Arc::new(NvsEncrypted::new(
            partition,
            keys_partition,
        )?)))
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

impl RawHandle for EspNvsPartition<NvsEncrypted> {
    type Handle = *const u8;

    fn handle(&self) -> Self::Handle {
        self.0.name().as_ptr() as *const _
    }
}

pub type EspDefaultNvs = EspNvs<NvsDefault>;
pub type EspCustomNvs = EspNvs<NvsCustom>;
pub type EspEncryptedNvs = EspNvs<NvsEncrypted>;

#[allow(dead_code)]
pub struct EspNvs<T: NvsPartitionId>(EspNvsPartition<T>, nvs_handle_t);

impl<T: NvsPartitionId> EspNvs<T> {
    pub fn new(
        partition: EspNvsPartition<T>,
        namespace: &str,
        read_write: bool,
    ) -> Result<Self, EspError> {
        let c_namespace = to_cstring_arg(namespace)?;

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

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(all(esp_idf_version_major = "5", esp_idf_version_minor = "1"))
    ))]
    pub fn find_key(&self, name: &str) -> Result<Option<NvsDataType>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut entry_type: nvs_type_t = nvs_type_t_NVS_TYPE_ANY;

        let result = unsafe { nvs_find_key(self.1, c_key.as_ptr(), &mut entry_type as *mut _) };

        match result {
            ESP_OK => Ok(NvsDataType::from_nvs_type(entry_type)),
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                esp!(err)?;
                Ok(None)
            }
        }
    }

    pub fn remove(&self, name: &str) -> Result<bool, EspError> {
        let c_key = to_cstring_arg(name)?;

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

    /// Returns the length of the blob stored under the key `name`.
    ///
    /// If the key does not exist, `Ok(None)` is returned.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_NVS_INVALID_HANDLE` if the NVS handle is invalid.
    /// - `ESP_ERR_NVS_INVALID_NAME` if the key name is invalid.
    pub fn blob_len(&self, name: &str) -> Result<Option<usize>, EspError> {
        let c_key = to_cstring_arg(name)?;

        #[allow(unused_assignments)]
        let mut len = 0;

        match unsafe { nvs_get_blob(self.1, c_key.as_ptr(), ptr::null_mut(), &mut len as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(len))
            }
        }
    }

    pub fn get_blob<'a>(
        &self,
        name: &str,
        buf: &'a mut [u8],
    ) -> Result<Option<&'a [u8]>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut len = buf.len();

        match unsafe {
            nvs_get_blob(
                self.1,
                c_key.as_ptr(),
                buf.as_mut_ptr() as *mut _,
                &mut len as *mut _,
            )
        } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(&buf[..len]))
            }
        }
    }

    pub fn set_blob(&self, name: &str, buf: &[u8]) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        // start by just clearing this key
        unsafe { nvs_erase_key(self.1, c_key.as_ptr()) };

        esp!(unsafe { nvs_set_blob(self.1, c_key.as_ptr(), buf.as_ptr().cast(), buf.len()) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    /// Returns the length of the string stored under the key `name`.
    ///
    /// If the key does not exist, `Ok(None)` is returned.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_NVS_INVALID_HANDLE` if the NVS handle is invalid.
    /// - `ESP_ERR_NVS_INVALID_NAME` if the key name is invalid.
    ///
    /// # Note
    ///
    /// The stored string is a [`CString`], which is why the returned length
    /// includes the null terminator.
    ///
    /// Rust strings do not have a null terminator, so when constructing one,
    /// make sure to only use the bytes up to `len - 1`. Alternatively one
    /// can use [`CStr`] or [`CString`].
    pub fn str_len(&self, name: &str) -> Result<Option<usize>, EspError> {
        let c_key = to_cstring_arg(name)?;

        #[allow(unused_assignments)]
        let mut len = 0;

        match unsafe { nvs_get_str(self.1, c_key.as_ptr(), ptr::null_mut(), &mut len as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(len))
            }
        }
    }

    pub fn get_str<'a>(&self, name: &str, buf: &'a mut [u8]) -> Result<Option<&'a str>, EspError> {
        let c_key = to_cstring_arg(name)?;

        let mut len = buf.len();
        match unsafe {
            nvs_get_str(
                self.1,
                c_key.as_ptr(),
                buf.as_mut_ptr() as *mut _,
                &mut len as *mut _,
            )
        } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(unsafe {
                    core::str::from_utf8_unchecked(&(buf[..len - 1]))
                }))
            }
        }
    }

    pub fn set_str(&self, name: &str, val: &str) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;
        let c_val = to_cstring_arg(val)?;

        // start by just clearing this key
        unsafe { nvs_erase_key(self.1, c_key.as_ptr()) };

        esp!(unsafe { nvs_set_str(self.1, c_key.as_ptr(), c_val.as_ptr(),) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_u8(&self, name: &str) -> Result<Option<u8>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [u8; 1] = [0; 1];

        match unsafe { nvs_get_u8(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_u8(&self, name: &str, val: u8) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_u8(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_i8(&self, name: &str) -> Result<Option<i8>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [i8; 1] = [0; 1];

        match unsafe { nvs_get_i8(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_i8(&self, name: &str, val: i8) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_i8(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_u16(&self, name: &str) -> Result<Option<u16>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [u16; 1] = [0; 1];

        match unsafe { nvs_get_u16(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_u16(&self, name: &str, val: u16) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_u16(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_i16(&self, name: &str) -> Result<Option<i16>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [i16; 1] = [0; 1];

        match unsafe { nvs_get_i16(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_i16(&self, name: &str, val: i16) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_i16(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_u32(&self, name: &str) -> Result<Option<u32>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [u32; 1] = [0; 1];

        match unsafe { nvs_get_u32(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_u32(&self, name: &str, val: u32) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_u32(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_i32(&self, name: &str) -> Result<Option<i32>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [i32; 1] = [0; 1];

        match unsafe { nvs_get_i32(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_i32(&self, name: &str, val: i32) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_i32(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_u64(&self, name: &str) -> Result<Option<u64>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [u64; 1] = [0; 1];

        match unsafe { nvs_get_u64(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_u64(&self, name: &str, val: u64) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_u64(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    pub fn get_i64(&self, name: &str) -> Result<Option<i64>, EspError> {
        let c_key = to_cstring_arg(name)?;
        let mut result: [i64; 1] = [0; 1];

        match unsafe { nvs_get_i64(self.1, c_key.as_ptr(), &mut result[0] as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => Ok(None),
            err => {
                // bail on error
                esp!(err)?;

                Ok(Some(result[0]))
            }
        }
    }

    pub fn set_i64(&self, name: &str, val: i64) -> Result<(), EspError> {
        let c_key = to_cstring_arg(name)?;

        esp!(unsafe { nvs_set_i64(self.1, c_key.as_ptr(), val) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    /// Erases all key-value pairs in the NVS namespace.
    ///
    /// # Errors
    ///
    /// This function will return an error if the NVS erase operation fails, this can happen because of
    /// - a corrupted NVS partition
    /// - the NVS is opened in read-only mode
    /// - other internal errors from the underlying storage driver
    pub fn erase_all(&self) -> Result<(), EspError> {
        esp!(unsafe { nvs_erase_all(self.1) })?;

        esp!(unsafe { nvs_commit(self.1) })?;

        Ok(())
    }

    /// Returns struct to iterate over all keys stored in this NVS namespace with the specified data type.
    ///
    /// A data type of `None` will return all keys regardless of their type.
    ///
    /// # Mutating the NVS while iterating
    ///
    /// Both this function and others that mutate the NVS like [`EspNvs::remove`] only require an immutable
    /// reference, making it possible to mutate the NVS while iterating over it. For example, one could remove
    /// keys while iterating.
    ///
    /// It is **not** recommended to do this, because the iterator might skip keys. It will not result in
    /// a panic or undefinied behavior.
    ///
    /// # Errors
    ///
    /// This function will return an error if
    /// - there is no memory available for allocation of internal structures
    /// - for some reason the [`EspNvs::handle`] is invalid (should not happen)
    #[cfg(esp_idf_version_at_least_5_2_0)]
    pub fn keys(&self, data_type: Option<NvsDataType>) -> Result<EspNvsKeys<'_>, EspError> {
        let mut raw_iter: nvs_iterator_t = core::ptr::null_mut();

        match unsafe {
            nvs_entry_find_in_handle(
                self.1,
                data_type
                    .map(|ty| ty as u32)
                    .unwrap_or(nvs_type_t_NVS_TYPE_ANY),
                &mut raw_iter as *mut _,
            )
        } {
            ESP_ERR_NVS_NOT_FOUND => {
                return Ok(EspNvsKeys {
                    _nvs: PhantomData,
                    raw_iter: core::ptr::null_mut(),
                    is_exhausted: true,
                    key_name_buffer: [0; 16],
                });
            }
            other => esp!(other)?,
        }

        Ok(EspNvsKeys {
            _nvs: PhantomData,
            raw_iter,
            is_exhausted: false,
            key_name_buffer: [0; 16],
        })
    }
}

impl<T: NvsPartitionId> Drop for EspNvs<T> {
    fn drop(&mut self) {
        unsafe {
            nvs_close(self.1);
        }

        info!("EspNvs dropped");
    }
}

unsafe impl<T: NvsPartitionId> Send for EspNvs<T> {}

impl RawHandle for EspNvs<NvsCustom> {
    type Handle = nvs_handle_t;

    fn handle(&self) -> Self::Handle {
        self.1
    }
}
impl RawHandle for EspNvs<NvsEncrypted> {
    type Handle = nvs_handle_t;

    fn handle(&self) -> Self::Handle {
        self.1
    }
}
impl RawHandle for EspNvs<NvsDefault> {
    type Handle = nvs_handle_t;

    fn handle(&self) -> Self::Handle {
        self.1
    }
}

#[cfg(esp_idf_version_at_least_5_2_0)]
pub struct EspNvsKeys<'a> {
    // The EspNvs must not be dropped while the iterator is still in use,
    // this reference ensures that.
    _nvs: PhantomData<&'a ()>,
    raw_iter: nvs_iterator_t,
    is_exhausted: bool,
    key_name_buffer: [u8; 16],
}

#[cfg(esp_idf_version_at_least_5_2_0)]
impl<'a> EspNvsKeys<'a> {
    /// Returns the next key in the NVS namespace and its data type.
    ///
    /// After the last key is returned, this function will return `None` on subsequent calls.
    pub fn next_key(&mut self) -> Option<(&str, NvsDataType)> {
        if self.is_exhausted || self.raw_iter.is_null() {
            return None;
        }

        let mut info: nvs_entry_info_t = Default::default();
        match unsafe { nvs_entry_info(self.raw_iter, &mut info as *mut _) } {
            ESP_ERR_NVS_NOT_FOUND => {
                self.is_exhausted = true;
                None
            }
            ESP_OK => {
                // For the next iteration, the iterator must be advanced to the next entry,
                // otherwise it will return the same entry again.
                //
                // This function call will fail if the iterator is
                // - null, which is checked before this call
                // - exhausted (if it is, it will set self.raw_iter to null and iteration will stop)
                //
                // For convenience, the error is ignored here, because it should never happen anyway.
                // The usage example in C simply stops the iteration on error too and does not do any
                // error handling.
                let _ = esp!(unsafe { nvs_entry_next(&mut self.raw_iter as *mut _) });

                // Copy the current key name into the buffer to make a str
                // that lives for the lifetime of the &mut self borrow.
                self.key_name_buffer[..info.key.len()].copy_from_slice(&info.key[..]);

                Some((
                    from_cstr(&self.key_name_buffer[..info.key.len()]),
                    NvsDataType::from_nvs_type(info.type_).expect("Unknown NVS data type"),
                ))
            }
            // The nvs_entry_info only fails if any of the arguments are null.
            // The nvs_entry_info is never null, and self.raw_iter is checked for null before the invocation.
            //
            // Therefore this should never happen.
            err => unreachable!(
                "Unexpected error while iterating over NVS entries: {:?}",
                esp!(err)
            ),
        }
    }
}

#[cfg(esp_idf_version_at_least_5_2_0)]
impl<'a> Drop for EspNvsKeys<'a> {
    fn drop(&mut self) {
        unsafe { nvs_release_iterator(self.raw_iter) };
    }
}

/// A specialized key-value storage wrapper around `EspNvs` that provides a simplified interface
/// for storing and retrieving arbitrary data as byte (`u8`) slices.
///
/// `EspKeyValueStorage` provides an interface for:
/// - Storing any data that can be represented as `&[u8]`
/// - Automatic optimization: values ≤7 bytes stored as ESP-NVS `u64` values, larger values as ESP-NVS blobs
/// - Consistent `contains()` method that works correctly with this storage strategy
/// - Full compatibility with Rust serde implementations (postcard, json, etc.) in that these can naturally do serde
///   over byte slices
///
/// ## Usage
///
/// ```rust,no_run
/// use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspKeyValueStorage};
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let partition = EspDefaultNvsPartition::take()?;
/// let storage = EspKeyValueStorage::new(partition, "my_namespace", true)?;
///
/// // Store data as bytes
/// let data = b"hello world";
/// storage.set_raw("my_key", data)?;
///
/// // Check if key exists (this works correctly, unlike the original EspNvs bug)
/// assert!(storage.contains("my_key")?);
///
/// // Retrieve data
/// let mut buffer = [0u8; 64];
/// if let Some(retrieved) = storage.get_raw("my_key", &mut buffer)? {
///     assert_eq!(retrieved, data);
/// }
/// # Ok(())
/// # }
/// ```
pub struct EspKeyValueStorage<T: NvsPartitionId>(EspNvs<T>);

impl<T: NvsPartitionId> EspKeyValueStorage<T> {
    pub const fn new(nvs: EspNvs<T>) -> Self {
        Self(nvs)
    }

    pub fn contains(&self, name: &str) -> Result<bool, EspError> {
        self.len(name).map(|v| v.is_some())
    }

    pub fn remove(&self, name: &str) -> Result<bool, EspError> {
        self.0.remove(name)
    }

    fn len(&self, name: &str) -> Result<Option<usize>, EspError> {
        match self.0.get_u64(name)? {
            Some(value) => {
                // u64 value was found, decode it
                let len: u8 = (value & 0xff) as u8;
                Ok(Some(len as _))
            }
            None => self.0.blob_len(name),
        }
    }

    pub fn get_raw<'a>(&self, name: &str, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, EspError> {
        match self.0.get_u64(name)? {
            Some(mut value) => {
                // u64 value was found, decode it
                let len: u8 = (value & 0xff) as u8;

                if buf.len() < len as _ {
                    return Err(EspError::from_infallible::<ESP_ERR_NVS_INVALID_LENGTH>());
                }

                // Shift the u64 value to remove the length byte
                value >>= 8;

                let array: [u8; 7] = [
                    (value & 0xff) as u8,
                    ((value >> 8) & 0xff) as u8,
                    ((value >> 16) & 0xff) as u8,
                    ((value >> 24) & 0xff) as u8,
                    ((value >> 32) & 0xff) as u8,
                    ((value >> 40) & 0xff) as u8,
                    ((value >> 48) & 0xff) as u8,
                ];

                buf[..len as usize].copy_from_slice(&array[..len as usize]);

                Ok(Some(&buf[..len as usize]))
            }
            None => self.0.get_blob(name, buf),
        }
    }

    pub fn set_raw(&self, name: &str, buf: &[u8]) -> Result<bool, EspError> {
        // start by just clearing this key, ignoring the result since it may not exist
        // TODO: This is not optimal, because if the chip is shut-down right after
        // the call to `remove`, the key will be gone forever.

        _ = self.0.remove(name);

        if buf.len() < 8 {
            let mut u64value: u_int64_t = 0;

            for v in buf.iter().rev() {
                u64value <<= 8;
                u64value |= *v as u_int64_t;
            }

            u64value <<= 8;
            u64value |= buf.len() as u_int64_t;

            self.0.set_u64(name, u64value)?;
            Ok(true)
        } else {
            self.0.set_blob(name, buf)?;
            Ok(true)
        }
    }
}

impl<T: NvsPartitionId> StorageBase for EspKeyValueStorage<T> {
    type Error = EspError;

    fn contains(&self, name: &str) -> Result<bool, Self::Error> {
        EspKeyValueStorage::contains(self, name)
    }

    fn remove(&mut self, name: &str) -> Result<bool, Self::Error> {
        EspKeyValueStorage::remove(self, name)
    }
}

impl<T: NvsPartitionId> RawStorage for EspKeyValueStorage<T> {
    fn len(&self, name: &str) -> Result<Option<usize>, Self::Error> {
        EspKeyValueStorage::len(self, name)
    }

    fn get_raw<'a>(&self, name: &str, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Self::Error> {
        EspKeyValueStorage::get_raw(self, name, buf)
    }

    fn set_raw(&mut self, name: &str, buf: &[u8]) -> Result<bool, Self::Error> {
        EspKeyValueStorage::set_raw(self, name, buf)
    }
}
