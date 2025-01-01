pub use embedded_svc::utils::io as utils;
pub use esp_idf_hal::io::*;

#[cfg(esp_idf_comp_vfs_enabled)]
pub mod vfs {
    use core::borrow::BorrowMut;
    use core::marker::PhantomData;

    use crate::hal::uart::UartDriver;
    #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
    use crate::hal::usb_serial::UsbSerialDriver;
    use crate::sys::{
        self, esp_vfs_dev_uart_use_driver, esp_vfs_dev_uart_use_nonblocking, EspError,
    };
    #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
    use crate::sys::{esp_vfs_usb_serial_jtag_use_driver, esp_vfs_usb_serial_jtag_use_nonblocking};

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

    /// A utility for setting up a buffered and blocking communication for the Rust `stdio` subsystem.
    ///
    /// By default, all communication via `std::io:stdin` / `std::io::stdout` on the ESP-IDF is non-blocking.
    /// One consequence of this, is that if the user wants to read from `std::io::stdin`, she has to constantly
    /// poll the driver, since the respective hardware FIFO buffers are relatively small-ish.
    /// Also the user would have to handle `WouldBlock` errors on every call, which is not very ergonomic.
    ///
    /// Instantiating the `BlockingStdIo` instructs the ESP-IDF VFS (Virtual File System) to use the
    /// interrupt-driven drivers instead, as well as their blocking read / write functions.
    pub struct BlockingStdIo<'d, T> {
        uart_port: Option<crate::sys::uart_port_t>,
        _driver: T,
        _t: PhantomData<&'d mut ()>,
    }

    impl<'d, T> BlockingStdIo<'d, T>
    where
        T: BorrowMut<UartDriver<'d>>,
    {
        /// Create a `BlockingStdIo` instance for a UART driver
        ///
        /// Arguments:
        /// - `driver`: The UART driver to use (i.e. a `UartDriver` instance that can be mutably borrowed)
        pub fn uart(driver: T) -> Result<Self, EspError> {
            unsafe { esp_vfs_dev_uart_use_driver(driver.borrow().port() as _) }

            Ok(Self {
                uart_port: Some(driver.borrow().port()),
                _driver: driver,
                _t: PhantomData,
            })
        }
    }

    #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
    impl<'d, T> BlockingStdIo<'d, T>
    where
        T: BorrowMut<UsbSerialDriver<'d>>,
    {
        /// Create a `BlockingStdIo` instance for a USB-SERIAL driver
        ///
        /// NOTE: By default, `println!` and `log!` output will be redirected to it in case
        /// no UART connection is established to a Host PC. The peripheral is initialized at
        /// startup and is using the ESP console slot 2 by default.
        ///
        /// NOTE: ESP console slot 2 cannot be used to read from the HOST, only writing is supported.
        /// If reading from the HOST is necessary, reconfigure the ESP console by setting
        /// the following into your projects sdkconfig.default file:
        /// ```
        /// CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
        /// ```
        ///
        /// Arguments:
        /// - `driver`: The USB-SERIAL driver to use (i.e. a `UsbSerialDriver` instance that can be mutably borrowed)
        pub fn usb_serial(driver: T) -> Result<Self, EspError> {
            unsafe { esp_vfs_usb_serial_jtag_use_driver() }

            Ok(Self {
                uart_port: None,
                _driver: driver,
                _t: PhantomData,
            })
        }
    }

    impl<T> Drop for BlockingStdIo<'_, T> {
        fn drop(&mut self) {
            if let Some(port) = self.uart_port {
                unsafe { esp_vfs_dev_uart_use_nonblocking(port as _) }
            } else {
                #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
                {
                    unsafe { esp_vfs_usb_serial_jtag_use_nonblocking() }
                }
            }
        }
    }
}
