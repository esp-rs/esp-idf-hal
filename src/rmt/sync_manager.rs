use core::ptr;

use esp_idf_sys::*;

use crate::rmt::{RmtChannel, TxChannel};

/// In some real-time control applications (e.g., to make two robotic arms move simultaneously),
/// you do not want any time drift between different channels. The RMT driver can help to manage
/// this by creating a so-called Sync Manager.
///
/// The procedure of RMT sync transmission is shown [here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html#multiple-channels-simultaneous-transmission).
pub struct SyncManager<'d, const N: usize> {
    handle: rmt_sync_manager_handle_t,
    // It is possible to convert a SyncManager back into its channels,
    // which would require taking ownership of them.
    //
    // Because it implements Drop, rust forbids moving fields out of the struct,
    // to workaround this, we the channels is wrapped in an Option that always
    // contains a value, except when dropped where it might be None.
    channels: Option<[TxChannel<'d>; N]>,
}

impl<'d, const N: usize> SyncManager<'d, N> {
    /// Create a synchronization manager for multiple TX channels,
    /// so that the managed channel can start transmitting at the same time.
    ///
    /// # Note
    ///
    /// All the channels managed by the sync manager should be enabled before creating
    /// the sync manager.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Create sync manager failed because of invalid argument
    /// - `ESP_ERR_NOT_SUPPORTED`: Create sync manager failed because it is not supported by hardware
    /// - `ESP_ERR_INVALID_STATE`: Create sync manager failed because not all channels are enabled
    /// - `ESP_ERR_NO_MEM`: Create sync manager failed because out of memory
    /// - `ESP_ERR_NOT_FOUND`: Create sync manager failed because all sync controllers are used up and no more free one
    /// - `ESP_FAIL`: Create sync manager failed because of other error
    pub fn new(channels: [TxChannel<'d>; N]) -> Result<Self, EspError> {
        let mut this = Self {
            handle: ptr::null_mut(),
            channels: Some(channels),
        };

        // The referenced handles are copied by the rmt_new_sync_manager, therefore
        // it is okay to reference a local variable here.
        let mut iter = this.channels_mut().iter().map(TxChannel::handle);
        let handles = [(); N].map(|_| iter.next().unwrap());

        let sys_config = rmt_sync_manager_config_t {
            tx_channel_array: handles.as_ptr(),
            array_size: handles.len(),
        };

        esp!(unsafe { rmt_new_sync_manager(&sys_config, &mut this.handle) })?;

        Ok(this)
    }

    /// Reset synchronization manager.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Reset the synchronization manager failed because of invalid argument
    /// - `ESP_FAIL`: Reset the synchronization manager failed because of other error
    pub fn reset(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_sync_reset(self.handle) })
    }

    /// Returns a mutable reference to the managed TX channels.
    pub fn channels_mut(&mut self) -> &mut [TxChannel<'d>; N] {
        // Safety: Channels are always Some except when dropped.
        unsafe { self.channels.as_mut().unwrap_unchecked() }
    }

    /// Returns a reference to the managed TX channels.
    pub fn channels(&self) -> &[TxChannel<'d>; N] {
        // Safety: Channels are always Some except when dropped.
        unsafe { self.channels.as_ref().unwrap_unchecked() }
    }

    /// Consumes the sync manager and returns the managed TX channels.
    pub fn into_channels(mut self) -> [TxChannel<'d>; N] {
        unsafe { self.channels.take().unwrap_unchecked() }
    }
}

impl<'d, const N: usize> Drop for SyncManager<'d, N> {
    fn drop(&mut self) {
        unsafe { rmt_del_sync_manager(self.handle) };
    }
}
