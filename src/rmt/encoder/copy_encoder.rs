use core::ptr;

use esp_idf_sys::*;

use super::Encoder;
use crate::rmt::Symbol;

/// A copy encoder copies the provided symbols from the user space into the driver layer.
///
/// It is usually used to encode non-`mut` data, i.e. data that does not change at runtime
/// after initialization, such as the leading code in the IR protocol.
#[derive(Debug)]
pub struct CopyEncoder {
    handle: rmt_encoder_handle_t,
}

impl CopyEncoder {
    /// Constructs a new copy encoder with default configuration.
    pub fn new() -> Result<Self, EspError> {
        Self::with_config(&rmt_copy_encoder_config_t {})
    }

    /// Constructs a new copy encoder with the provided configuration.
    pub fn with_config(config: &rmt_copy_encoder_config_t) -> Result<Self, EspError> {
        let mut handle: rmt_encoder_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_copy_encoder(config, &mut handle) })?;
        Ok(Self { handle })
    }
}

impl Drop for CopyEncoder {
    fn drop(&mut self) {
        // This is calling encoder->del(encoder);
        unsafe { rmt_del_encoder(self.handle) };
    }
}

impl Encoder for CopyEncoder {
    type Item = Symbol;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}
