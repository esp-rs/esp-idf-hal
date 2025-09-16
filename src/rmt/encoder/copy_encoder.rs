use core::ptr;

use esp_idf_sys::*;

use super::Encoder;

#[derive(Debug)]
pub struct CopyEncoder {
    handle: rmt_encoder_handle_t,
}

impl CopyEncoder {
    pub fn new() -> Result<Self, EspError> {
        Self::with_config(&rmt_copy_encoder_config_t {})
    }

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
    type Item = rmt_symbol_word_t;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}
