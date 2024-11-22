use core::borrow::BorrowMut;

use alloc::boxed::Box;

use config::{FatFsType, FormatConfiguration};

use ::log::warn;

use crate::hal::sd::SdCardDriver;
use crate::sys::*;

extern crate alloc;

pub mod config {
    /// Type of FAT filesystem to create when formatting the partition.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub enum FatFsType {
        /// Automatically choose the best FAT type depending on volume and cluster size.
        Auto,
        /// FAT12 filesystem.
        Fat,
        /// FAT32 filesystem.
        Fat32,
        /// ExFAT filesystem.
        ExFat,
    }

    /// Configuration for formatting a FAT partition.
    pub struct FormatConfiguration {
        /// Type of FAT filesystem to create.
        pub fs_type: FatFsType,
        /// Whether to create a backup copy of the FAT table.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub fat_backup_copy: bool,
        /// Volume data alignment in number of sectors.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub volume_data_alignment: core::num::NonZeroU16,
        /// Number of root directory entries.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub root_dir_entries: core::num::NonZeroU16,
        /// Cluster size in bytes.
        pub cluster_size: u32,
    }

    impl FormatConfiguration {
        /// Create a new default configuration
        pub const fn new() -> Self {
            Self {
                fs_type: FatFsType::Auto,
                #[cfg(not(esp_idf_version_major = "4"))]
                fat_backup_copy: false,
                #[cfg(not(esp_idf_version_major = "4"))]
                volume_data_alignment: unsafe { core::num::NonZeroU16::new_unchecked(1) },
                #[cfg(not(esp_idf_version_major = "4"))]
                root_dir_entries: unsafe { core::num::NonZeroU16::new_unchecked(512) },
                cluster_size: 4096,
            }
        }
    }

    impl Default for FormatConfiguration {
        fn default() -> Self {
            Self::new()
        }
    }
}

enum Partition<T> {
    SdCard(T),
    RawPartition,
}

/// Represents a mounted FAT filesystem instance that can be used to interact with the filesystem.
/// The filesystem is automatically unmounted when the instance is dropped.
///
/// The interaction happens via the native, unsafe FATFS library API (i.e. `crate::sys::f_open`, `crate::sys::f_read` and so on).
/// An alternative way to mount the filesystem is to use the VFS API, which is more high-level and abstracts the underlying filesystem.
pub struct MountedFatfs<'a, T> {
    fs: &'a mut Fatfs<T>,
    fatfs: Box<FATFS>,
}

impl<T> MountedFatfs<'_, T> {
    /// Get the underlying FATFS instance.
    pub fn fatfs(&self) -> &FATFS {
        &self.fatfs
    }

    // TODO: Add safe methods to interact with the filesystem
}

impl<T> Drop for MountedFatfs<'_, T> {
    fn drop(&mut self) {
        let drive_path = self.fs.drive_path();

        let res = unsafe { f_mount(core::ptr::null_mut(), drive_path.as_ptr(), 0) };

        if res != FRESULT_FR_OK {
            panic!("Unmount failed: {res}");
        }
    }
}

/// Represents a FAT filesystem.
pub struct Fatfs<T> {
    drive: u8,
    _partition: Partition<T>,
}

impl<T> Fatfs<T> {
    /// Create a new FAT filesystem instance for a given SD card driver.
    ///
    /// # Arguments
    /// - Drive number to assign to the filesystem.
    /// - SD card driver instance.
    pub fn new_sdcard<H>(drive: u8, mut sd_card_driver: T) -> Result<Self, EspError>
    where
        T: BorrowMut<SdCardDriver<H>>,
    {
        unsafe {
            ff_diskio_register_sdmmc(
                drive,
                sd_card_driver.borrow_mut().card() as *const _ as *mut _,
            );
        }

        Ok(Self {
            drive,
            _partition: Partition::SdCard(sd_card_driver),
        })
    }

    /// Get the drive number of the filesystem.
    pub fn drive(&self) -> u8 {
        self.drive
    }

    /// Format the partition with the given configuration.
    ///
    /// # Arguments
    /// - Formatting configuration.
    /// - Buffer to use when formatting.
    pub fn format(
        &mut self,
        configuration: &FormatConfiguration,
        buf: &mut [u8],
    ) -> Result<(), EspError> {
        let drive_path = self.drive_path();

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            let opt = MKFS_PARM {
                fmt: match configuration.fs_type {
                    FatFsType::Auto => FM_ANY,
                    FatFsType::Fat => FM_FAT,
                    FatFsType::Fat32 => FM_FAT32,
                    FatFsType::ExFat => FM_EXFAT,
                } as _,
                au_size: configuration.cluster_size,
                n_fat: if configuration.fat_backup_copy { 2 } else { 1 },
                n_root: configuration.root_dir_entries.get() as _,
                align: configuration.volume_data_alignment.get() as _,
            };

            unsafe {
                f_mkfs(
                    drive_path.as_ptr(),
                    &opt,
                    buf.as_mut_ptr() as *mut _,
                    buf.len() as _,
                );
            }
        }

        #[cfg(esp_idf_version_major = "4")]
        {
            unsafe {
                f_mkfs(
                    drive_path.as_ptr(),
                    match configuration.fs_type {
                        FatFsType::Auto => FM_ANY,
                        FatFsType::Fat => FM_FAT,
                        FatFsType::Fat32 => FM_FAT32,
                        FatFsType::ExFat => FM_EXFAT,
                    } as _,
                    configuration.cluster_size,
                    buf.as_mut_ptr() as *mut _,
                    buf.len() as _,
                );
            }
        }

        Ok(())
    }

    /// Mount the filesystem and return a handle to it.
    pub fn mount(&mut self) -> Result<MountedFatfs<'_, T>, EspError> {
        let mut fatfs: Box<FATFS> = Box::default(); // TODO: Large stack size

        let drive_path = self.drive_path();

        let res = unsafe { f_mount(&mut *fatfs, drive_path.as_ptr(), 0) };

        if res != FRESULT_FR_OK {
            warn!("Mount failed: {res}");
            Err(EspError::from_infallible::<ESP_FAIL>())?
        }

        Ok(MountedFatfs { fs: self, fatfs })
    }

    pub(crate) fn drive_path_from(drive: u8) -> [core::ffi::c_char; 2] {
        [drive as _, 0]
    }

    pub(crate) fn drive_path(&self) -> [core::ffi::c_char; 2] {
        Self::drive_path_from(self.drive)
    }
}

impl Fatfs<()> {
    /// Create a new - readonly - FAT filesystem instance for a given raw partition in the internal flash.
    /// This API is unsafe because currently `esp-idf-hal` does not have a safe way to
    /// represent a flash partition - neither a raw one, nor a wear-leveling one.
    ///
    /// # Arguments
    /// - Drive number to assign to the filesystem.
    /// - Raw partition pointer.
    ///
    /// # Safety
    ///
    /// While the filesystem object is alive, the partition should not be modified elsewhere
    pub unsafe fn new_raw_part(
        drive: u8,
        partition: *const esp_partition_t,
    ) -> Result<Self, EspError> {
        unsafe {
            ff_diskio_register_raw_partition(drive, partition);
        }

        Ok(Self {
            drive,
            _partition: Partition::RawPartition,
        })
    }

    /// Create a new FAT filesystem instance for a given raw partition in the internal flash.
    /// This API is unsafe because currently `esp-idf-hal` does not have a safe way to
    /// represent a flash partition - neither a raw one, nor a wear-leveling one.
    ///
    /// # Arguments
    /// - Drive number to assign to the filesystem.
    /// - A handle to a wear-leveling partition.
    ///
    /// # Safety
    ///
    /// While the filesystem object is alive, the partition should not be modified elsewhere
    pub unsafe fn new_wl_part(drive: u8, partition: wl_handle_t) -> Result<Self, EspError> {
        unsafe {
            ff_diskio_register_wl_partition(drive, partition);
        }

        Ok(Self {
            drive,
            _partition: Partition::RawPartition,
        })
    }
}

impl<T> Drop for Fatfs<T> {
    fn drop(&mut self) {
        unsafe {
            ff_diskio_register(self.drive, core::ptr::null_mut());
        }
    }
}
