use core::ptr;

use esp_idf_sys::*;

use crate::rmt::encoder::Encoder;
use crate::rmt::{PinState, Pulse, PulseTicks, Symbol};

#[derive(Clone, Copy)]
pub struct BytesEncoderConfig {
    pub bit0: Symbol,
    pub bit1: Symbol,
    pub msb_first: bool,
}

impl Default for BytesEncoderConfig {
    fn default() -> Self {
        let low_pulse = Pulse::new(PinState::Low, PulseTicks::new(1).unwrap());
        let high_pulse = Pulse::new(PinState::High, PulseTicks::new(1).unwrap());

        Self {
            bit0: Symbol::new(low_pulse, low_pulse),
            bit1: Symbol::new(high_pulse, high_pulse),
            msb_first: false,
        }
    }
}

#[derive(Debug)]
pub struct BytesEncoder {
    handle: rmt_encoder_handle_t,
}

impl BytesEncoder {
    pub fn new() -> Result<Self, EspError> {
        Self::with_config(&Default::default())
    }

    pub fn with_config(config: &BytesEncoderConfig) -> Result<Self, EspError> {
        let sys_config = rmt_bytes_encoder_config_t {
            bit0: config.bit0.0,
            bit1: config.bit1.0,
            flags: rmt_bytes_encoder_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_bytes_encoder_config_t__bindgen_ty_1::new_bitfield_1(
                    config.msb_first as u32,
                ),
                ..Default::default()
            },
        };

        let mut handle: rmt_encoder_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_bytes_encoder(&sys_config, &mut handle) })?;
        Ok(Self { handle })
    }
}

impl Drop for BytesEncoder {
    fn drop(&mut self) {
        // This is calling encoder->del(encoder);
        unsafe { rmt_del_encoder(self.handle) };
    }
}

impl Encoder for BytesEncoder {
    type Item = u8;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}
