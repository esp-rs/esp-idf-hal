mod copy_encoder;
pub use copy_encoder::*;
mod bytes_encoder;
pub use bytes_encoder::*;
mod simple_encoder;
pub use simple_encoder::*;

use esp_idf_sys::*;

/// This trait represents an RMT encoder that is used to encode data for transmission.
pub trait Encoder {
    /// The type of input data that the encoder can encode.
    type Item;

    /// Returns a mutable reference to the underlying `rmt_encoder_t`.
    ///
    /// The functions of the `rmt_encoder_t` will be called by the RMT driver.
    fn handle(&mut self) -> &mut rmt_encoder_t;

    fn reset(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_encoder_reset(self.handle()) })
    }
}
