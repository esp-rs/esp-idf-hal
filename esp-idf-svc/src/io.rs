pub use embedded_svc::utils::io as utils;
pub use esp_idf_hal::io::*;

#[cfg(esp_idf_comp_vfs_enabled)]
pub mod vfs {
    use crate::sys;

    #[allow(clippy::needless_update)]
    pub fn initialize_eventfd(max_fds: usize) -> Result<(), sys::EspError> {
        sys::esp!(unsafe {
            sys::esp_vfs_eventfd_register(&sys::esp_vfs_eventfd_config_t {
                max_fds: max_fds as _,
                ..Default::default()
            })
        })
    }
}
