use core::ffi::CStr;

use alloc::ffi::CString;

use crate::sys::*;

extern crate alloc;

/// Represents a Spiffs filesystem.
pub struct Spiffs {
    partition: CString,
}

impl Spiffs {
    /// Create a new - readonly - FAT filesystem instance for a given raw partition in the internal flash.
    /// This API is unsafe because currently `esp-idf-svc` does not have a safe way to
    /// represent a flash partition.
    ///
    /// # Arguments
    /// - Spiffs partition label.
    ///
    /// # Safety
    ///
    /// While the filesystem object is alive, the partition should not be modified elsewhere
    pub unsafe fn new(partition_label: &str) -> Result<Self, EspError> {
        Ok(Self {
            partition: crate::private::cstr::to_cstring_arg(partition_label)?,
        })
    }

    /// Get the partition label.
    pub fn partition_label(&self) -> &CStr {
        &self.partition
    }

    /// Check the filesystem for errors.
    pub fn check(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_spiffs_check(self.partition.as_ptr()) })
    }

    /// Format the partition.
    pub fn format(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_spiffs_format(self.partition.as_ptr()) })
    }

    /// Garbage collect the filesystem.
    #[cfg(not(esp_idf_version_major = "4"))]
    pub fn gc(&mut self, size_to_gc: usize) -> Result<(), EspError> {
        esp!(unsafe { esp_spiffs_gc(self.partition.as_ptr(), size_to_gc) })
    }
}
