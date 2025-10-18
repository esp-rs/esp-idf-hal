use core::ptr;

use esp_idf_sys::*;

use crate::rmt::encoder::RawEncoder;
use crate::rmt::{PinState, Pulse, PulseTicks, Symbol};

/// The configuration for the [`BytesEncoder`].
#[derive(Debug, Clone)]
pub struct BytesEncoderConfig {
    /// Specifies the symbol used to represent a `0` bit.
    pub bit0: Symbol,
    /// Specifies the symbol used to represent a `1` bit.
    pub bit1: Symbol,
    /// If `true`, bits are encoded most-significant bit first,
    /// otherwise it will be least-significant bit (LSB) first.
    pub msb_first: bool,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for BytesEncoderConfig {
    fn default() -> Self {
        let low_pulse = Pulse::new(PinState::Low, PulseTicks::new(1).unwrap());
        let high_pulse = Pulse::new(PinState::High, PulseTicks::new(1).unwrap());

        Self {
            bit0: Symbol::new(low_pulse, low_pulse),
            bit1: Symbol::new(high_pulse, high_pulse),
            msb_first: false,
            __internal: (),
        }
    }
}

/// An encoder that dynamically encodes a user space byte stream into RMT symbols.
///
/// It is usually used to encode dynamic data, e.g., the address and command fields in the IR protocol.
#[derive(Debug)]
pub struct BytesEncoder {
    handle: rmt_encoder_handle_t,
}

impl BytesEncoder {
    /// Constructs a new bytes encoder with default configuration.
    pub fn new() -> Result<Self, EspError> {
        Self::with_config(&Default::default())
    }

    /// Constructs a new bytes encoder with the provided configuration.
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

impl RawEncoder for BytesEncoder {
    type Item = u8;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}
