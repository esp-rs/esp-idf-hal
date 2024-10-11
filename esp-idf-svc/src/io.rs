pub use embedded_svc::utils::io as utils;
pub use esp_idf_hal::io::*;

#[cfg(esp_idf_comp_vfs_enabled)]
pub mod vfs {
    use crate::sys;

    #[cfg(all(feature = "experimental", feature = "alloc"))]
    extern crate alloc;

    /// Represents a mounted EventFD pseudo-filesystem.
    ///
    /// Operating on this filesystem is done only via the native, unsafe `sys::eventfd_*` function.
    pub struct MountedEventfs(());

    impl MountedEventfs {
        /// Mount the EventFD pseudo-filesystem.
        ///
        /// # Arguments
        /// - `max_fds`: The maximum number of file descriptors to allocate.
        #[allow(clippy::needless_update)]
        pub fn mount(max_fds: usize) -> Result<Self, sys::EspError> {
            sys::esp!(unsafe {
                sys::esp_vfs_eventfd_register(&sys::esp_vfs_eventfd_config_t {
                    max_fds: max_fds as _,
                    ..Default::default()
                })
            })?;

            Ok(Self(()))
        }
    }

    impl Drop for MountedEventfs {
        fn drop(&mut self) {
            sys::esp!(unsafe { sys::esp_vfs_eventfd_unregister() }).unwrap();
        }
    }

    /// Represents a mounted SPIFFS filesystem.
    #[cfg(all(feature = "experimental", feature = "alloc"))]
    pub struct MountedSpiffs<T> {
        _spiffs: T,
        path: alloc::ffi::CString,
    }

    #[cfg(all(feature = "experimental", feature = "alloc"))]
    impl<T> MountedSpiffs<T> {
        /// Mount a SPIFFS filesystem.
        ///
        /// # Arguments
        /// - `spiffs`: The SPIFFS filesystem instance to mount.
        /// - `path`: The path to mount the filesystem at.
        /// - `max_fds`: The maximum number of file descriptors to allocate.
        pub fn mount(mut spiffs: T, path: &str, max_fds: usize) -> Result<Self, sys::EspError>
        where
            T: core::borrow::BorrowMut<crate::fs::spiffs::Spiffs>,
        {
            let path = crate::private::cstr::to_cstring_arg(path)?;

            sys::esp!(unsafe {
                sys::esp_vfs_spiffs_register(&sys::esp_vfs_spiffs_conf_t {
                    base_path: path.as_ptr(),
                    max_files: max_fds as _,
                    partition_label: spiffs.borrow_mut().partition_label().as_ptr(),
                    format_if_mount_failed: false,
                })
            })?;

            Ok(Self {
                _spiffs: spiffs,
                path,
            })
        }
    }

    #[cfg(all(feature = "experimental", feature = "alloc"))]
    impl<T> Drop for MountedSpiffs<T> {
        fn drop(&mut self) {
            sys::esp!(unsafe { sys::esp_vfs_spiffs_unregister(self.path.as_ptr()) }).unwrap();
        }
    }

    /// Represents a mounted FAT filesystem.
    #[cfg(all(feature = "experimental", feature = "alloc"))]
    pub struct MountedFatfs<T> {
        _handle: *mut sys::FATFS,
        _fatfs: T,
        path: alloc::ffi::CString,
        drive: u8,
    }

    #[cfg(all(feature = "experimental", feature = "alloc"))]
    impl<T> MountedFatfs<T> {
        /// Mount a FAT filesystem.
        ///
        /// # Arguments
        /// - `fatfs`: The FAT filesystem instance to mount.
        /// - `path`: The path to mount the filesystem at.
        /// - `max_fds`: The maximum number of file descriptors to allocate.
        pub fn mount<H>(mut fatfs: T, path: &str, max_fds: usize) -> Result<Self, sys::EspError>
        where
            T: core::borrow::BorrowMut<crate::fs::fatfs::Fatfs<H>>,
        {
            let path = crate::private::cstr::to_cstring_arg(path)?;
            let drive_path = fatfs.borrow_mut().drive_path();

            let mut handle = core::ptr::null_mut();

            sys::esp!(unsafe {
                sys::esp_vfs_fat_register(
                    path.as_ptr(),
                    drive_path.as_ptr(),
                    max_fds as _,
                    &mut handle,
                )
            })?;

            unsafe {
                sys::f_mount(handle, drive_path.as_ptr(), 0); // TODO
            }

            let drive = fatfs.borrow_mut().drive();

            Ok(Self {
                _handle: handle,
                _fatfs: fatfs,
                path,
                drive,
            })
        }
    }

    #[cfg(all(feature = "experimental", feature = "alloc"))]
    impl<T> Drop for MountedFatfs<T> {
        fn drop(&mut self) {
            let drive_path = crate::fs::fatfs::Fatfs::<()>::drive_path_from(self.drive);

            unsafe {
                sys::f_mount(core::ptr::null_mut(), drive_path.as_ptr(), 0);
            }

            sys::esp!(unsafe { sys::esp_vfs_fat_unregister_path(self.path.as_ptr()) }).unwrap();
        }
    }

    /// Represents a mounted Littlefs filesystem.
    #[cfg(all(feature = "alloc", esp_idf_comp_joltwallet__littlefs_enabled))]
    pub struct MountedLittlefs<T> {
        _littlefs: T,
        partition_raw_data: crate::fs::littlefs::PartitionRawData,
    }

    #[cfg(all(feature = "alloc", esp_idf_comp_joltwallet__littlefs_enabled))]
    impl<T> MountedLittlefs<T> {
        /// Mount a Littlefs filesystem.
        ///
        /// # Arguments
        /// - `littlefs`: The Littlefs filesystem instance to mount.
        /// - `path`: The path to mount the filesystem at.
        pub fn mount<H>(mut littlefs: T, path: &str) -> Result<Self, sys::EspError>
        where
            T: core::borrow::BorrowMut<crate::fs::littlefs::Littlefs<H>>,
        {
            use crate::fs::littlefs::PartitionRawData;
            use crate::private::cstr::to_cstring_arg;

            let path = to_cstring_arg(path)?;

            let partition_raw_data = littlefs.borrow_mut().partition_raw_data();

            let conf = sys::esp_vfs_littlefs_conf_t {
                base_path: path.as_ptr(),
                partition_label: if let PartitionRawData::PartitionLabel(label) = partition_raw_data
                {
                    label
                } else {
                    core::ptr::null()
                },
                partition: if let PartitionRawData::RawPartition(partition) = partition_raw_data {
                    partition
                } else {
                    core::ptr::null_mut()
                },
                #[cfg(esp_idf_littlefs_sdmmc_support)]
                sdcard: if let PartitionRawData::SdCard(sdcard) = partition_raw_data {
                    sdcard
                } else {
                    core::ptr::null_mut()
                },
                ..Default::default()
            };

            sys::esp!(unsafe { sys::esp_vfs_littlefs_register(&conf) })?;

            Ok(Self {
                _littlefs: littlefs,
                partition_raw_data,
            })
        }

        pub fn info(&self) -> Result<crate::fs::littlefs::LittleFsInfo, sys::EspError> {
            use crate::fs::littlefs::PartitionRawData;

            let mut info = crate::fs::littlefs::LittleFsInfo {
                total_bytes: 0,
                used_bytes: 0,
            };

            match self.partition_raw_data {
                #[cfg(esp_idf_littlefs_sdmmc_support)]
                PartitionRawData::SdCard(sd_card_ptr) => {
                    sys::esp!(unsafe {
                        sys::esp_littlefs_sdmmc_info(
                            sd_card_ptr,
                            &mut info.total_bytes,
                            &mut info.used_bytes,
                        )
                    })?;
                }
                PartitionRawData::PartitionLabel(label) => {
                    sys::esp!(unsafe {
                        sys::esp_littlefs_info(label, &mut info.total_bytes, &mut info.used_bytes)
                    })?;
                }
                PartitionRawData::RawPartition(partition) => {
                    sys::esp!(unsafe {
                        sys::esp_littlefs_partition_info(
                            partition,
                            &mut info.total_bytes,
                            &mut info.used_bytes,
                        )
                    })?;
                }
            }

            Ok(info)
        }
    }

    #[cfg(all(feature = "alloc", esp_idf_comp_joltwallet__littlefs_enabled))]
    impl<T> Drop for MountedLittlefs<T> {
        fn drop(&mut self) {
            use crate::fs::littlefs::PartitionRawData;

            match self.partition_raw_data {
                PartitionRawData::PartitionLabel(label) => {
                    sys::esp!(unsafe { sys::esp_vfs_littlefs_unregister(label) }).unwrap();
                }
                PartitionRawData::RawPartition(partition) => {
                    sys::esp!(unsafe { sys::esp_vfs_littlefs_unregister_partition(partition) })
                        .unwrap();
                }
                #[cfg(esp_idf_littlefs_sdmmc_support)]
                PartitionRawData::SdCard(sdcard) => {
                    sys::esp!(unsafe { sys::esp_vfs_littlefs_unregister_sdmmc(sdcard) }).unwrap();
                }
            }
        }
    }
}
