mod copy_encoder;
pub use copy_encoder::*;
mod bytes_encoder;
pub use bytes_encoder::*;

mod encoder_allocator;
use encoder_allocator::RmtEncoderAllocator;

#[cfg(all(feature = "alloc", esp_idf_version_at_least_5_3_0))]
mod simple_encoder;
#[cfg(all(feature = "alloc", esp_idf_version_at_least_5_3_0))]
pub use simple_encoder::*;

use core::alloc::Layout;
use core::mem;
use core::ptr::NonNull;

use esp_idf_sys::*;

/// This trait represents an RMT encoder that is used to encode data for transmission.
pub trait RawEncoder {
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

impl<E: RawEncoder> RawEncoder for &mut E {
    type Item = E::Item;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        (*self).handle()
    }

    fn reset(&mut self) -> Result<(), EspError> {
        (*self).reset()
    }
}

pub enum EncoderState {
    EncodingReset,
    EncodingComplete,
    EncodingMemoryFull,
    // EncodingWithEof, // TODO: likely a new state
}

impl From<EncoderState> for rmt_encode_state_t {
    fn from(value: EncoderState) -> Self {
        match value {
            EncoderState::EncodingReset => rmt_encode_state_t_RMT_ENCODING_RESET,
            EncoderState::EncodingComplete => rmt_encode_state_t_RMT_ENCODING_COMPLETE,
            EncoderState::EncodingMemoryFull => rmt_encode_state_t_RMT_ENCODING_MEM_FULL,
            // EncoderState::EncodingWithEof => rmt_encode_state_t_RMT_ENCODING_WITH_EOF,
        }
    }
}

impl From<rmt_encode_state_t> for EncoderState {
    fn from(value: rmt_encode_state_t) -> Self {
        #[allow(non_upper_case_globals)]
        match value {
            rmt_encode_state_t_RMT_ENCODING_RESET => Self::EncodingReset,
            rmt_encode_state_t_RMT_ENCODING_COMPLETE => Self::EncodingComplete,
            rmt_encode_state_t_RMT_ENCODING_MEM_FULL => Self::EncodingMemoryFull,
            // rmt_encode_state_t_RMT_ENCODING_WITH_EOF => Self::EncodingWithEof,
            _ => panic!("Unknown rmt_encode_state_t value: {}", value),
        }
    }
}

/// A trait for implementing custom RMT encoders in rust.
///
/// An RMT encoder is part of the RMT TX transaction, whose responsibility
/// is to generate and write the correct RMT symbols into hardware memory
/// or DMA buffer at a specific time.
///
/// There are some special restrictions for an encoding function:
/// - During a single transaction, the encoding function may be called
///   multiple times. This is necessary because the target RMT memory
///   block cannot hold all the artifacts at once. To overcome this
///   limitation, the driver utilizes a ping-pong approach, where the
///   encoding session is divided into multiple parts. This means that
///   the encoder needs to keep track of its state to continue encoding
///   from where it left off in the previous part.
/// - The encoding function is running in the ISR context.
///   To speed up the encoding session, it is highly recommended to put
///   the encoding function into IRAM. This can also avoid the cache miss during encoding.
pub trait Encoder {
    type Item;

    /// Encode the user data into RMT symbols and write into RMT memory.
    ///
    /// This function might be called multiple times within a single transaction.
    /// The encode function should return the state of the current encoding session.
    ///
    /// The supported states are listed in [`EncoderState`]. If the result contains
    /// [`EncoderState::EncodingComplete`], it means the current encoder has finished
    /// work.
    ///
    /// If the result contains [`EncoderState::EncodingMemoryFull`], the program needs
    /// to yield from the current session, as there is no space to save more encoding
    /// artifacts.
    ///
    /// # Note
    ///
    /// It is recommended to put this function implementation in the IRAM, to achieve a high
    /// performance and less interrupt latency.
    ///
    /// # ISR Safety
    ///
    /// The encoding function will also be called from an ISR context, thus the function must not
    /// call any blocking API.
    unsafe fn encode(
        // TODO: is this function unsafe? It shouldn't be?
        &mut self,
        tx_channel: rmt_channel_handle_t,
        primary_data: &[Self::Item],
    ) -> (usize, EncoderState);

    /// Reset encoding state.
    ///
    /// # Errors
    ///
    /// With `ESP_FAIL` when it fails to reset the encoder.
    fn reset(&mut self) -> Result<(), EspError>;
}

impl<E: RawEncoder> Encoder for E {
    type Item = E::Item;

    unsafe fn encode(
        &mut self,
        tx_channel: rmt_channel_handle_t,
        primary_data: &[Self::Item],
    ) -> (usize, EncoderState) {
        let handle = self.handle();
        let Some(encode) = handle.encode else {
            // TODO: panic?
            return (0, EncoderState::EncodingReset);
        };

        let mut ret_state: rmt_encode_state_t = rmt_encode_state_t_RMT_ENCODING_RESET;
        let data_size = mem::size_of_val::<[Self::Item]>(primary_data);

        let written = encode(
            handle,
            tx_channel,
            primary_data.as_ptr() as *const core::ffi::c_void,
            data_size,
            &raw mut ret_state,
        );

        (written, ret_state.into())
    }

    fn reset(&mut self) -> Result<(), EspError> {
        let handle = self.handle();
        if let Some(reset) = handle.reset {
            esp!(unsafe { reset(handle) })?;
        }

        Ok(())
    }
}

#[must_use]
pub fn into_raw<E: Encoder>(encoder: E) -> EncoderWrapper<E> {
    EncoderWrapper::new(encoder).expect("Failed to allocate memory for RMT encoder")
}

struct InternalEncoderWrapper<E> {
    base: rmt_encoder_t, // the base "class", declares the standard encoder interface
    encoder: Option<E>,
}

#[derive(Debug)]
#[repr(C)]
pub struct EncoderWrapper<E>(NonNull<InternalEncoderWrapper<E>>);

impl<E: Encoder> EncoderWrapper<E> {
    pub fn new(encoder: E) -> Result<Self, EspError> {
        let ptr = RmtEncoderAllocator.allocate_value(InternalEncoderWrapper::new(encoder))?;

        Ok(Self(ptr))
    }
}

impl<E> Drop for EncoderWrapper<E> {
    fn drop(&mut self) {
        unsafe {
            RmtEncoderAllocator.deallocate(
                self.0.cast::<u8>(),
                Layout::new::<InternalEncoderWrapper<E>>(),
            );
        }
    }
}

impl<E: Encoder> InternalEncoderWrapper<E> {
    unsafe fn containerof(ptr: *mut rmt_encoder_t) -> *mut Self {
        // The given pointer points to the base field of this struct,
        // in the C-Code they use the __containerof macro to get the pointer to the whole struct
        //
        // The below does the same.
        //
        // SAFETY: This struct is #[repr(C)]
        ptr.cast::<u8>()
            .sub(mem::offset_of!(Self, base))
            .cast::<Self>()
    }

    pub fn new(encoder: E) -> Self {
        Self {
            base: rmt_encoder_t {
                encode: Some(Self::encode),
                reset: Some(Self::reset),
                del: Some(Self::del),
            },
            encoder: Some(encoder),
        }
    }

    fn encoder(&mut self) -> &mut E {
        unsafe { self.encoder.as_mut().unwrap_unchecked() }
    }

    pub unsafe extern "C" fn encode(
        encoder: *mut rmt_encoder_t,
        tx_channel: rmt_channel_handle_t,
        primary_data: *const ::core::ffi::c_void,
        data_size: usize,
        ret_state: *mut rmt_encode_state_t,
    ) -> usize {
        let this = Self::containerof(encoder).as_mut().unwrap();

        let primary_data = core::slice::from_raw_parts(
            primary_data.cast::<E::Item>(),
            data_size / mem::size_of::<E::Item>(),
        );

        let (written, state) = this.encoder().encode(tx_channel, primary_data);

        *ret_state = state.into();

        written
    }

    pub unsafe extern "C" fn reset(encoder: *mut rmt_encoder_t) -> esp_err_t {
        let this = Self::containerof(encoder).as_mut().unwrap();

        match this.encoder().reset() {
            Ok(()) => ESP_OK,
            Err(_) => ESP_FAIL,
        }
    }

    pub unsafe extern "C" fn del(encoder: *mut rmt_encoder_t) -> esp_err_t {
        let this = Self::containerof(encoder).as_mut().unwrap();

        // drop the encoder
        this.encoder.take();

        ESP_OK
    }
}

impl<E: Encoder> RawEncoder for EncoderWrapper<E> {
    type Item = E::Item;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut self.0.as_mut().base }
    }
}
