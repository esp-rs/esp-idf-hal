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
}
