//! ESP IDF partitions API
//!
//! This API provides access to the partitions in the ESP32 flash memory - with operations for reading, writing, and erasing.
//! The ESP-IDF Wear-Leveling algorithm is also supported.
//!
//! Note that ESP-IDF partitions are not created or dropped by this API - they always pre-existing and the API provides access to them.
//! To define your partitions, you need to use the ESP-IDF partition table CSV file, as described here:
//! https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html

use core::{borrow::BorrowMut, ffi::CStr};

use esp_idf_hal::sys::*;

use crate::handle::RawHandle;

#[cfg(feature = "embedded-storage")]
pub use embedded_storage::{EspEncrypted, EspFlashError};

/// The type of a partition
#[non_exhaustive]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum EspPartitionType {
    /// Application partition
    App(EspAppPartitionSubtype),
    /// Data partition
    Data(EspDataPartitionSubtype),
    /// Unknown partition type
    Unknown,
}

impl EspPartitionType {
    const fn raw(&self) -> (u32, u32) {
        match self {
            EspPartitionType::App(subtype) => {
                let subtype = match subtype {
                    EspAppPartitionSubtype::Factory => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_FACTORY
                    }
                    EspAppPartitionSubtype::Test => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_TEST
                    }
                    EspAppPartitionSubtype::Ota(subtype) => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MIN + *subtype as u32
                    }
                    EspAppPartitionSubtype::Unknown => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY
                    }
                };

                (esp_partition_type_t_ESP_PARTITION_TYPE_APP, subtype)
            }
            EspPartitionType::Data(subtype) => {
                let subtype = match subtype {
                    EspDataPartitionSubtype::Ota => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_OTA
                    }
                    EspDataPartitionSubtype::Phy => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_PHY
                    }
                    EspDataPartitionSubtype::Nvs => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_NVS
                    }
                    EspDataPartitionSubtype::Coredump => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_COREDUMP
                    }
                    EspDataPartitionSubtype::NvsKeys => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS
                    }
                    EspDataPartitionSubtype::Efuse => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM
                    }
                    EspDataPartitionSubtype::Undefined => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_UNDEFINED
                    }
                    EspDataPartitionSubtype::EspHttpd => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD
                    }
                    EspDataPartitionSubtype::Fat => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_FAT
                    }
                    EspDataPartitionSubtype::Spiffs => {
                        esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_SPIFFS
                    }
                    // Note: only available in the latest patch releases
                    // #[cfg(not(esp_idf_version_major = "4"))]
                    // EspDataPartitionSubtype::LittleFs => {
                    //     esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_LITTLEFS
                    // }
                    _ => esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
                };

                (esp_partition_type_t_ESP_PARTITION_TYPE_DATA, subtype)
            }
            EspPartitionType::Unknown => (
                esp_partition_type_t_ESP_PARTITION_TYPE_ANY,
                esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            ),
        }
    }
}
/// The subtype of an application partition
#[non_exhaustive]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum EspAppPartitionSubtype {
    /// Factory partition
    Factory,
    /// Test partition
    Test,
    /// OTA partition
    Ota(u8),
    /// Unknown app partition subtype
    Unknown,
}

/// The subtype of a data partition
#[non_exhaustive]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum EspDataPartitionSubtype {
    /// OTA data partition
    Ota,
    /// PHY data partition
    Phy,
    /// NVS data partition
    Nvs,
    /// Core dump data partition
    Coredump,
    /// NVS keys data partition (for encryption)
    NvsKeys,
    /// EFUSE data partition
    Efuse,
    /// Undefined data partition
    Undefined,
    /// ESPHTTPD data partition
    EspHttpd,
    /// FAT FS partition
    Fat,
    /// SPIFFS partition
    Spiffs,
    // /// LittleFS partition
    // LittleFs,
    /// Unknown data partition subtype
    Unknown,
}

/// The type of memory mapping
#[cfg(not(esp_idf_version_major = "4"))]
#[non_exhaustive]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum EspMemMapType {
    /// Data
    Data,
    /// Instruction (code)
    Instruction,
}

/// Represents a memory-mapping of a partition region
///
/// Drop this to unmap the memory region
#[cfg(not(esp_idf_version_major = "4"))]
pub struct EspMemMappedPartition<'a> {
    handle: esp_partition_mmap_handle_t,
    start: usize,
    _t: core::marker::PhantomData<&'a mut ()>,
}

#[cfg(not(esp_idf_version_major = "4"))]
impl EspMemMappedPartition<'_> {
    /// Returns the start address of the memory-mapped region
    pub const fn start(&self) -> usize {
        self.start
    }
}

#[cfg(not(esp_idf_version_major = "4"))]
impl Drop for EspMemMappedPartition<'_> {
    fn drop(&mut self) {
        unsafe {
            esp_partition_munmap(self.handle);
        }
    }
}

/// An iterator over the partitions in the ESP32 flash memory
pub struct EspPartitionIterator {
    raw_iter: esp_partition_iterator_t,
}

impl EspPartitionIterator {
    /// Create a new partition iterator
    ///
    /// # Arguments
    /// - `partition_type`: The type of partitions to iterate over
    ///
    /// # Safety
    /// Only one partition iterator should be created at a time
    pub unsafe fn new(partition_type: Option<EspPartitionType>) -> Result<Self, EspError> {
        let (partition_type, partition_subtype) = partition_type
            .map(|partition_type| partition_type.raw())
            .unwrap_or((
                esp_partition_type_t_ESP_PARTITION_TYPE_ANY,
                esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            ));

        let raw_iter = esp_partition_find(partition_type, partition_subtype, core::ptr::null());

        Ok(Self { raw_iter })
    }

    /// Return the next partition in the iterator
    pub fn next_partition(&mut self) -> Option<EspPartition> {
        if self.raw_iter.is_null() {
            return None;
        }

        let partition = unsafe { esp_partition_get(self.raw_iter) };

        let value = if partition.is_null() {
            None
        } else {
            Some(unsafe { EspPartition::wrap(partition) })
        };

        self.raw_iter = unsafe { esp_partition_next(self.raw_iter) };

        value
    }
}

impl Drop for EspPartitionIterator {
    fn drop(&mut self) {
        unsafe {
            esp_partition_iterator_release(self.raw_iter);
        }
    }
}

impl Iterator for EspPartitionIterator {
    type Item = EspPartition;

    fn next(&mut self) -> Option<Self::Item> {
        self.next_partition()
    }
}

/// Represents a partition in the ESP32 flash memory
#[repr(transparent)]
pub struct EspPartition(*const esp_partition_t);

impl EspPartition {
    /// Wrap a raw pointer into an `EspPartition` instance
    ///
    /// # Safety
    /// The raw pointer should be a valid one
    /// It should not be shared in multiple `EspPartition` instances
    pub unsafe fn wrap(partition: *const esp_partition_t) -> Self {
        Self(partition)
    }

    /// Create a new `EspPartition` instance for an existing partition identified by its label
    ///
    /// # Arguments
    /// - `label`: The label of the partition
    ///
    /// Return `None` if the partition with the label does not exist
    /// or `Some` with the partition if it exists.
    ///
    /// # Safety
    /// Only a single partition should be active at any point in time for that label.
    #[cfg(feature = "alloc")]
    pub unsafe fn new(label: &str) -> Result<Option<Self>, EspError> {
        let cstr = crate::private::cstr::to_cstring_arg(label)?;

        Self::cnew(&cstr)
    }

    /// Create a new `EspPartition` instance for an existing partition identified by its C-string label
    ///
    /// # Arguments
    /// - `clabel`: The label of the partition as a C string
    ///
    /// Return `None` if the partition with the label does not exist
    /// or `Some` with the partition if it exists.
    ///
    /// # Safety
    /// Only a single partition should be active at any point in time for that label.
    pub unsafe fn cnew(clabel: &CStr) -> Result<Option<Self>, EspError> {
        let partition = esp_partition_find_first(
            esp_partition_type_t_ESP_PARTITION_TYPE_ANY,
            esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            clabel.as_ptr(),
        );

        if partition.is_null() {
            Ok(None)
        } else {
            Ok(Some(Self(partition)))
        }
    }

    /// Find and return the first partition of a specific type
    ///
    /// # Arguments
    /// - `partition_type`: The type of the partition to find
    ///
    /// Return `None` if a partition of the specified type does not exist
    /// or `Some` with the first partition of the specified type if it exists.
    ///
    /// # Safety
    /// User should not end up with two `EspPartition` instances representing the same ESP IDF partition.
    pub unsafe fn find_first(partition_type: EspPartitionType) -> Result<Option<Self>, EspError> {
        let (partition_type, partition_subtype) = partition_type.raw();

        let partition =
            esp_partition_find_first(partition_type, partition_subtype, core::ptr::null());

        if partition.is_null() {
            Ok(None)
        } else {
            Ok(Some(Self(partition)))
        }
    }

    /// Return the label of the partition as a C string
    pub fn clabel(&self) -> &CStr {
        unsafe { CStr::from_ptr((*self.0).label.as_ptr()) }
    }

    /// Return the label of the partition
    pub fn label(&self) -> &str {
        self.clabel().to_str().unwrap()
    }

    /// Return the type of the partition
    #[allow(non_upper_case_globals)]
    pub fn partition_type(&self) -> EspPartitionType {
        match unsafe { (*self.0).type_ } {
            esp_partition_type_t_ESP_PARTITION_TYPE_APP => {
                EspPartitionType::App(match unsafe { (*self.0).subtype } {
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_FACTORY => {
                        EspAppPartitionSubtype::Factory
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_TEST => {
                        EspAppPartitionSubtype::Test
                    }
                    other => {
                        if (esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MIN
                            ..=esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MAX)
                            .contains(&other)
                        {
                            EspAppPartitionSubtype::Ota(
                                (other - esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MIN)
                                    as _,
                            )
                        } else {
                            EspAppPartitionSubtype::Unknown
                        }
                    }
                })
            }
            esp_partition_type_t_ESP_PARTITION_TYPE_DATA => {
                EspPartitionType::Data(match unsafe { (*self.0).subtype } {
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_OTA => {
                        EspDataPartitionSubtype::Ota
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_PHY => {
                        EspDataPartitionSubtype::Phy
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_NVS => {
                        EspDataPartitionSubtype::Nvs
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_COREDUMP => {
                        EspDataPartitionSubtype::Coredump
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS => {
                        EspDataPartitionSubtype::NvsKeys
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_EFUSE_EM => {
                        EspDataPartitionSubtype::Efuse
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_UNDEFINED => {
                        EspDataPartitionSubtype::Undefined
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD => {
                        EspDataPartitionSubtype::EspHttpd
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_FAT => {
                        EspDataPartitionSubtype::Fat
                    }
                    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_SPIFFS => {
                        EspDataPartitionSubtype::Spiffs
                    }
                    // #[cfg(not(esp_idf_version_major = "4"))]
                    // esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_LITTLEFS => {
                    //     EspDataPartitionSubtype::LittleFs
                    // }
                    _ => EspDataPartitionSubtype::Unknown,
                })
            }
            _ => EspPartitionType::Unknown,
        }
    }

    /// Return the address/offset of the partition in the flash storage
    pub fn address(&self) -> usize {
        unsafe { (*self.0).address as _ }
    }

    /// Return the size of the partition in bytes in the flash storage
    pub fn size(&self) -> usize {
        unsafe { (*self.0).size as _ }
    }

    /// Return the erase size block of the partition in bytes
    #[cfg(not(esp_idf_version_major = "4"))]
    pub fn erase_size(&self) -> usize {
        unsafe { (*self.0).erase_size as _ }
    }

    /// Return `true` if the partition is encrypted
    pub fn encrypted(&self) -> bool {
        unsafe { (*self.0).encrypted }
    }

    /// Return `true` if the partition is read-only
    #[cfg(any(
        all(not(esp_idf_version_major = "4"), not(esp_idf_version_major = "5")),
        all(
            esp_idf_version_major = "5",
            not(esp_idf_version_minor = "0"),
            not(esp_idf_version_minor = "1"),
        )
    ))]
    pub fn readonly(&self) -> bool {
        unsafe { (*self.0).readonly }
    }

    /// Read data from the partition, performing decryption of the
    /// data if the partition is encrypted.
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to read from, in bytes
    /// - `buf`: The buffer to read the data into
    ///
    /// Return an error if the read operation failed.
    /// The read operation would fail if the offset and buffer length are
    /// beyond the partition bounds.
    ///
    /// The read operation will also fail if the offset and the buffer length
    /// are not aligned with the partition read alignment.
    pub fn read(&mut self, offset: usize, buf: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_partition_read(self.0, offset as _, buf.as_ptr() as *mut _, buf.len() as _)
        })
    }

    /// Write data to the partition, performing encryption of the
    /// data if the partition is encrypted.
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to write to, in bytes
    /// - `data`: The data to write to the partition
    ///
    /// Return an error if the write operation failed.
    /// The write operation would fail if the offset and data length are
    /// beyond the partition bounds.
    ///
    /// The write operation will also fail if the offset and the data length
    /// are not aligned with the partition write alignment.
    pub fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_partition_write(
                self.0,
                offset as _,
                data.as_ptr() as *const _,
                data.len() as _,
            )
        })
    }

    /// Erase a region of the partition
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to start erasing from, in bytes
    /// - `size`: The size of the region to erase
    ///
    /// Return an error if the erase operation failed.
    /// The erase operation would fail if the offset and size are
    /// beyond the partition bounds.
    ///
    /// The erase operation will also fail if the offset and the size
    /// are not aligned with the partition erase block returned by `erase_size`.
    pub fn erase(&mut self, offset: usize, size: usize) -> Result<(), EspError> {
        esp!(unsafe { esp_partition_erase_range(self.0, offset as _, size as _) })
    }

    /// Read data from the partition without performing decryption
    ///
    /// Identical to `read` if the partition is not encrypted.
    pub fn read_raw(&mut self, offset: usize, buf: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_partition_read_raw(self.0, offset as _, buf.as_ptr() as *mut _, buf.len() as _)
        })
    }

    /// Write data to the partition without performing encryption
    ///
    /// Identical to `write` if the partition is not encrypted.
    pub fn write_raw(&mut self, offset: usize, data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_partition_write_raw(
                self.0,
                offset as _,
                data.as_ptr() as *const _,
                data.len() as _,
            )
        })
    }

    /// Map a region of the partition to memory
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to map from, in bytes
    /// - `size`: The size of the region to map, in bytes
    /// - `mmap_type`: The type of memory mapping
    ///
    /// Return an error if the memory mapping operation failed.
    ///
    /// # Safety
    /// TBD
    #[cfg(not(esp_idf_version_major = "4"))]
    pub unsafe fn mmap(
        &mut self,
        offset: usize,
        size: usize,
        mmap_type: EspMemMapType,
    ) -> Result<EspMemMappedPartition<'_>, EspError> {
        let mut handle: esp_partition_mmap_handle_t = Default::default();
        let mut out: *const core::ffi::c_void = core::ptr::null_mut();

        esp!(esp_partition_mmap(
            self.0,
            offset as _,
            size as _,
            mmap_type as _,
            &mut out,
            &mut handle
        ))?;

        Ok(EspMemMappedPartition {
            handle,
            start: out as _,
            _t: core::marker::PhantomData,
        })
    }
}

impl RawHandle for EspPartition {
    type Handle = *const esp_partition_t;

    fn handle(&self) -> Self::Handle {
        self.0
    }
}

unsafe impl Send for EspPartition {}

/// Represents a partition wrapped with the ESP-IDF Wear-Leveling algorithm
pub struct EspWlPartition<T> {
    _partition: T,
    handle: wl_handle_t,
}

impl<T> EspWlPartition<T>
where
    T: BorrowMut<EspPartition>,
{
    /// Wrap the provided raw partition with the ESP-IDF Wear-Leveling algorithm
    ///
    /// Return an error if the wrap operation failed, or the WL partition
    /// if the operation succeeded.
    ///
    /// Arguments:
    /// - `partition`: The partition to mount
    pub fn new(mut partition: T) -> Result<Self, EspError> {
        let mut handle: wl_handle_t = Default::default();

        esp!(unsafe { wl_mount(partition.borrow_mut().0, &mut handle) })?;

        Ok(Self {
            _partition: partition,
            handle,
        })
    }

    /// Return the size of the mounted WL partition
    pub fn size(&self) -> usize {
        unsafe { wl_size(self.handle) as _ }
    }

    /// Return the size of a sector in the mounted WL partition
    pub fn sector_size(&self) -> usize {
        unsafe { wl_sector_size(self.handle) as _ }
    }

    /// Read data from the mounted WL partition
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to read from, in bytes
    /// - `buf`: The buffer to read the data into
    ///
    /// Return an error if the read operation failed.
    /// The read operation would fail if the offset and buffer length are
    /// beyond the partition bounds.
    ///
    /// The read operation will also fail if the offset and the buffer length
    /// are not aligned with the partition read alignment.
    pub fn read(&mut self, offset: usize, buf: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe {
            wl_read(
                self.handle,
                offset as _,
                buf.as_ptr() as *mut _,
                buf.len() as _,
            )
        })
    }

    /// Write data to the mounted WL partition
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to write to, in bytes
    /// - `data`: The data to write to the partition
    ///
    /// Return an error if the write operation failed.
    /// The write operation would fail if the offset and data length are
    /// beyond the partition bounds.
    ///
    /// The write operation will also fail if the offset and the data length
    /// are not aligned with the partition write alignment.
    pub fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            wl_write(
                self.handle,
                offset as _,
                data.as_ptr() as *const _,
                data.len() as _,
            )
        })
    }

    /// Erase a region of the mounted WL partition
    ///
    /// # Arguments
    /// - `offset`: The offset in the partition to start erasing from, in bytes
    /// - `size`: The size of the region to erase, in bytes
    ///
    /// Return an error if the erase operation failed.
    /// The erase operation would fail if the offset and size are
    /// beyond the partition bounds.
    pub fn erase(&mut self, offset: usize, size: usize) -> Result<(), EspError> {
        esp!(unsafe { wl_erase_range(self.handle, offset as _, size as _) })
    }
}

impl<T> RawHandle for EspWlPartition<T> {
    type Handle = wl_handle_t;

    fn handle(&self) -> Self::Handle {
        self.handle
    }
}

impl<T> Drop for EspWlPartition<T> {
    fn drop(&mut self) {
        esp!(unsafe { wl_unmount(self.handle) }).unwrap();
    }
}

unsafe impl<T> Send for EspWlPartition<T> where T: Send {}

#[cfg(feature = "embedded-storage")]
mod embedded_storage {
    use core::borrow::BorrowMut;
    use core::fmt;

    use embedded_storage::nor_flash::{
        ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
    };
    use embedded_storage::ReadStorage;

    use esp_idf_hal::sys::{EspError, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_SIZE};

    use super::{EspPartition, EspWlPartition};

    impl ReadStorage for EspPartition {
        type Error = EspError;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            EspPartition::read(self, offset as _, buf)
        }

        fn capacity(&self) -> usize {
            self.size()
        }
    }

    impl ErrorType for EspPartition {
        type Error = EspFlashError;
    }

    impl ReadNorFlash for EspPartition {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            EspPartition::read(self, offset as _, buf)?;

            Ok(())
        }

        fn capacity(&self) -> usize {
            self.size()
        }
    }

    impl NorFlash for EspPartition {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 4096;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if to < from {
                Err(EspFlashError(EspError::from_infallible::<
                    ESP_ERR_INVALID_SIZE,
                >()))?;
            }

            EspPartition::erase(self, from as _, (to - from) as _)?;

            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            EspPartition::write(self, offset as _, bytes)?;

            Ok(())
        }
    }

    impl MultiwriteNorFlash for EspPartition {}

    #[derive(Copy, Clone, PartialEq, Eq, Debug)]
    pub struct EspFlashError(pub EspError);

    impl From<EspError> for EspFlashError {
        fn from(e: EspError) -> Self {
            Self(e)
        }
    }

    impl fmt::Display for EspFlashError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            self.0.fmt(f)
        }
    }

    #[cfg(feature = "std")]
    impl std::error::Error for EspFlashError {}

    impl NorFlashError for EspFlashError {
        fn kind(&self) -> NorFlashErrorKind {
            match self.0.code() as _ {
                ESP_ERR_INVALID_ARG => NorFlashErrorKind::NotAligned,
                ESP_ERR_INVALID_SIZE => NorFlashErrorKind::OutOfBounds,
                _ => NorFlashErrorKind::Other,
            }
        }
    }

    impl<T> ReadStorage for EspWlPartition<T>
    where
        T: BorrowMut<EspPartition>,
    {
        type Error = EspError;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            EspWlPartition::read(self, offset as _, buf)
        }

        fn capacity(&self) -> usize {
            self.size()
        }
    }

    impl<T> ErrorType for EspWlPartition<T> {
        type Error = EspFlashError;
    }

    impl<T> ReadNorFlash for EspWlPartition<T>
    where
        T: BorrowMut<EspPartition>,
    {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            EspWlPartition::read(self, offset as _, buf)?;

            Ok(())
        }

        fn capacity(&self) -> usize {
            self.size()
        }
    }

    impl<T> NorFlash for EspWlPartition<T>
    where
        T: BorrowMut<EspPartition>,
    {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 4096;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if to < from {
                Err(EspFlashError(EspError::from_infallible::<
                    ESP_ERR_INVALID_SIZE,
                >()))?;
            }

            EspWlPartition::erase(self, from as _, (to - from) as _)?;

            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            EspWlPartition::write(self, offset as _, bytes)?;

            Ok(())
        }
    }

    impl<T> MultiwriteNorFlash for EspWlPartition<T> where T: BorrowMut<EspPartition> {}

    /// A wrapper marker type for encrypted partitions
    ///
    /// The reason why it is necessary is because for encrypted partitions
    /// the write size is 16 bytes, while for non-encrypted partitions
    /// the write size is 1 byte.
    pub struct EspEncrypted<T>(T);

    impl<T> EspEncrypted<T> {
        /// Wrap the provided partition with the encrypted marker
        pub const fn new(partition: T) -> Self {
            Self(partition)
        }

        /// Release the partition from the encrypted marker
        pub fn release(self) -> T {
            self.0
        }
    }

    impl<T> ErrorType for EspEncrypted<T>
    where
        T: ErrorType,
    {
        type Error = T::Error;
    }

    impl<T> ReadStorage for EspEncrypted<T>
    where
        T: ReadStorage,
    {
        type Error = T::Error;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.0.read(offset, buf)
        }

        fn capacity(&self) -> usize {
            self.0.capacity()
        }
    }

    impl<T> ReadNorFlash for EspEncrypted<T>
    where
        T: ReadNorFlash,
    {
        const READ_SIZE: usize = T::READ_SIZE;

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.0.read(offset, buf)
        }

        fn capacity(&self) -> usize {
            self.0.capacity()
        }
    }

    impl<T> NorFlash for EspEncrypted<T>
    where
        T: NorFlash,
    {
        // Because the partition is encrypted
        // See https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/partition.html#_CPPv419esp_partition_writePK15esp_partition_t6size_tPKv6size_t
        const WRITE_SIZE: usize = 16;

        const ERASE_SIZE: usize = T::ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.0.erase(from, to)
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.write(offset, bytes)
        }
    }
}
