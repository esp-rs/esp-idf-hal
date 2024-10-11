//! Littlefs filesystem.
//!
//! To use, put this in your `Cargo.toml`:
//! ```
//! [[package.metadata.esp-idf-sys.extra_components]]
//! remote_component = { name = "joltwallet/littlefs", version = "1.14" }
//! ```
//!
//! To use with an SD card, put this in your `sdkconfig.defaults`:
//! ```
//! CONFIG_LITTLEFS_SDMMC_SUPPORT=y
//! ```

use alloc::ffi::CString;

use crate::{private::cstr::to_cstring_arg, sys::*};

extern crate alloc;

#[allow(dead_code)]
enum Partition<T> {
    SdCard(T),
    PartitionLabel(CString),
    RawPartition(*mut esp_partition_t),
}

#[derive(Clone)]
pub(crate) enum PartitionRawData {
    #[cfg(esp_idf_littlefs_sdmmc_support)]
    SdCard(*mut sdmmc_card_t),
    PartitionLabel(*const i8),
    RawPartition(*mut esp_partition_t),
}

/// Information about the filesystem.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub struct LittleFsInfo {
    pub total_bytes: usize,
    pub used_bytes: usize,
}

/// Represents a Littlefs filesystem.
pub struct Littlefs<T> {
    _partition: Partition<T>,
    partition_raw_data: PartitionRawData,
}

impl<T> Littlefs<T> {
    /// Create a new Littlefs filesystem instance for a given SD card driver.
    ///
    /// # Arguments
    /// - SD card driver.
    #[cfg(esp_idf_littlefs_sdmmc_support)]
    pub fn new_sdcard<H>(sd_card_driver: T) -> Result<Self, EspError>
    where
        T: core::borrow::BorrowMut<crate::hal::sd::SdCardDriver<H>>,
    {
        let card_raw_ptr = sd_card_driver.borrow().card() as *const _ as *mut _;

        Ok(Self {
            _partition: Partition::SdCard(sd_card_driver),
            partition_raw_data: PartitionRawData::SdCard(card_raw_ptr),
        })
    }

    /// Create a new Littlefs filesystem instance for a given partition label.
    ///
    /// # Safety
    /// - This method should be used with a valid partition label.
    /// - While the partition is in use by the filesystem, it should not be modified or used elsewhere.
    ///
    /// # Arguments
    /// - `partition_label`: Partition label.
    pub unsafe fn new_partition(partition_label: &str) -> Result<Self, EspError> {
        let partition_label = to_cstring_arg(partition_label)?;
        let partition_raw_data = PartitionRawData::PartitionLabel(partition_label.as_ptr());

        Ok(Self {
            _partition: Partition::PartitionLabel(partition_label),
            partition_raw_data,
        })
    }

    /// Create a new Littlefs filesystem instance for a given partition.
    ///
    /// # Safety
    /// - This method should be used with a valid partition.
    /// - While the partition is in use by the filesystem, it should not be modified or used elsewhere.
    ///
    /// # Arguments
    /// - `partition`: the raw ESP-IDF partition.
    pub unsafe fn new_raw_partition(partition: *mut esp_partition_t) -> Result<Self, EspError> {
        Ok(Self {
            _partition: Partition::RawPartition(partition),
            partition_raw_data: PartitionRawData::RawPartition(partition),
        })
    }

    pub(crate) fn partition_raw_data(&self) -> PartitionRawData {
        self.partition_raw_data.clone()
    }

    /// Format the Littlefs partition.
    pub fn format(&mut self) -> Result<(), EspError> {
        match self.partition_raw_data {
            #[cfg(esp_idf_littlefs_sdmmc_support)]
            PartitionRawData::SdCard(sd_card_ptr) => {
                esp!(unsafe { esp_littlefs_format_sdmmc(sd_card_ptr) })?;
            }
            PartitionRawData::PartitionLabel(label) => {
                esp!(unsafe { esp_littlefs_format(label) })?;
            }
            PartitionRawData::RawPartition(partition) => {
                esp!(unsafe { esp_littlefs_format_partition(partition) })?;
            }
        }

        Ok(())
    }
}
