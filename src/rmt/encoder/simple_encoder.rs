use core::ffi::c_void;
use core::pin::Pin;
use core::{mem, ptr, slice};

use alloc::boxed::Box;

use esp_idf_sys::*;

use crate::rmt::encoder::Encoder;
use crate::rmt::Symbol;

/// The encoder was unable to encode all the input data into the provided buffer,
/// it might have encoded some data (or it didn't), but there remains more to encode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotEnoughSpace;

/// Configuration for the [`SimpleEncoder`].
#[derive(Debug, Clone)]
pub struct SimpleEncoderConfig {
    /// Minimum amount of free space, in RMT symbols, the encoder needs in order
    /// to guarantee it always returns non-zero. Defaults to 64 if zero / not given.
    pub min_chunk_size: usize,
    // This field is intentionally hidden to prevent non-exhaustive pattern matching.
    // You should only construct this struct using the `..Default::default()` pattern.
    // If you use this field directly, your code might break in future versions.
    #[doc(hidden)]
    #[allow(dead_code)]
    pub __internal: (),
}

impl Default for SimpleEncoderConfig {
    fn default() -> Self {
        Self {
            min_chunk_size: 64,
            __internal: (),
        }
    }
}

/// A helper to write symbols into a buffer, tracking how many symbols were written.
#[derive(Debug)]
pub struct SymbolBuffer<'a> {
    symbols: &'a mut [Symbol],
    written: usize,
}

impl<'a> SymbolBuffer<'a> {
    /// Constructs a `SymbolBuffer` from its raw parts.
    ///
    /// # Safety
    ///
    /// You must ensure that the provided pointer is valid for both reads and writes, not null,
    /// and properly aligned to construct a slice from it.
    ///
    /// The `written` parameter must not exceed the length of the slice.
    /// The `free` parameter must be such that `written + free` does not exceed the length of the
    /// slice.
    #[must_use]
    pub unsafe fn from_raw_parts(
        symbols: *mut rmt_symbol_word_t,
        written: usize,
        free: usize,
    ) -> Self {
        let symbols = slice::from_raw_parts_mut(symbols as *mut Symbol, written + free);
        Self { symbols, written }
    }

    /// Returns how many symbols have been written so far.
    ///
    /// This can also be interpreted as the position of the next
    /// element to write into the buffer.
    ///
    /// Initially this will be `0`. If you write `n` symbols in the
    /// callback and then return, the next time the callback is called,
    /// the position will always be `n`.
    /// If you then encode `m` more symbols, the next callback will always
    /// be called with position `n + m`, and so on.
    ///
    /// In other words, the position is preserved across callback invocations.
    ///
    /// The only exception is when the encoder is reset,
    /// (e.g. to beging a new transaction) in which case the position
    /// will always restart at `0`.
    #[must_use]
    pub const fn position(&self) -> usize {
        self.written
    }

    /// Returns how many more symbols can be written to the buffer.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.symbols.len() - self.written
    }

    /// Tries to write all symbols to the buffer.
    ///
    /// # Errors
    ///
    /// If there is not enough space, it will not write anything and return [`NotEnoughSpace`]
    /// to indicate that the buffer is too small.
    pub fn write_all(&mut self, symbols: &[Symbol]) -> Result<(), NotEnoughSpace> {
        if self.remaining() < symbols.len() {
            return Err(NotEnoughSpace);
        }

        for &symbol in symbols {
            self.symbols[self.written] = symbol;
            self.written += 1;
        }

        Ok(())
    }

    /// Returns the underlying slice of symbols that have been written so far.
    #[must_use]
    pub fn as_mut_slice(&mut self) -> &mut [Symbol] {
        &mut self.symbols[..self.written]
    }
}

/// This function implements the C API that the simple encoder expects for the callback
/// and delegates the call to the rust type `S` which implements `EncoderCallback`.
unsafe extern "C" fn delegator<S: EncoderCallback>(
    data: *const c_void,
    data_size: usize,
    symbols_written: usize,
    symbols_free: usize,
    symbols: *mut rmt_symbol_word_t,
    done: *mut bool,
    arg: *mut c_void,
) -> usize {
    let callback = &mut *(arg as *mut S);
    // The size of the data is in bytes -> must be converted to number of T items.
    let data_slice = slice::from_raw_parts(
        data as *const S::Item,
        data_size / mem::size_of::<S::Item>(),
    );
    let mut buffer = SymbolBuffer::from_raw_parts(symbols, symbols_written, symbols_free);

    // The function returns the following status:
    // 1. `0` if the given buffer for writing symbols is too small,
    //    like it needs 8 symbols to encode one data entry, but a buffer
    //    with only 4 symbols is given.
    // 2. a value `n > 0` indicating that `n` symbols were written in this callback round.
    //
    // If it finishes encoding all data, it sets `*done = true`.
    // This can happen with status 1 or 2 (you don't have to write symbols to indicate that you are done).
    //
    // The number of written symbols is tracked by the `SymbolBuffer`.
    //
    // Important: It counts the output symbols, not the input data items!
    //            If one processes a byte as an input into 8 symbols, then
    //            the function should return `8`.
    if let Ok(()) = callback.encode(data_slice, &mut buffer) {
        *done = true;
    }

    // Calculate how many symbols were written in this call:
    buffer.position() - symbols_written
}

/// Trait that is implemented by types that can be used as callbacks for the [`SimpleEncoder`].
pub trait EncoderCallback {
    /// The type of input data that the encoder can encode.
    type Item;

    /// This function encodes the provided input data into RMT symbols and writes them into the provided
    /// `SymbolBuffer`.
    ///
    /// To do this, a buffer is allocated in which the resulting symbols can be written. It might happen that there
    /// is not enough space in the buffer to encode all input data. In this case, the function has two options:
    /// 1. It can immediately return [`NotEnoughSpace`] to indicate that the buffer is too small.
    ///    The function will later be called again with a larger buffer. You should eventually process the data,
    ///    and not return [`NotEnoughSpace`] forever.
    ///
    /// 2. It can start encoding the input data, and write symbols into the buffer until it runs out of space.
    ///    It should return with [`NotEnoughSpace`]. The function will later be called again with more
    ///    space, making it possible to continue encoding the remaining input data.
    ///
    /// The function takes a slice of input data of your chosen type [`EncoderCallback::Item`], this is the same data
    /// that is passed to the RMT driver when sending data. The slice will not change between unfinished calls.
    ///
    /// For example, if you start processing an `input_data` of 10 elements, and you return with
    /// [`NotEnoughSpace`] after encoding 4 elements, the next time the function is called,
    /// the `input_data` will still be the same 10 elements.
    ///
    /// It is your responsibility to keep track of how many input elements you have already processed.
    /// If the number of output symbols are a multiple of the number of input elements, you can use [`SymbolBuffer::position`]
    /// to track how many input elements have been processed so far:
    /// 1 input element = 8 output symbols -> position / 8 = number of input elements processed.
    ///
    /// Once you have processed all input elements, you should return with [`Ok`].
    fn encode(
        &mut self,
        input_data: &[Self::Item],
        buffer: &mut SymbolBuffer<'_>,
    ) -> Result<(), NotEnoughSpace>; // TODO: will the callback be in ISR?
}

#[derive(Debug)]
pub struct SimpleEncoder<T> {
    _encoder: Pin<Box<T>>,
    handle: rmt_encoder_handle_t,
}

impl<T: EncoderCallback> SimpleEncoder<T> {
    /// Constructs a new simple encoder with the given callback and configuration.
    pub fn with_config(encoder: T, config: &SimpleEncoderConfig) -> Result<Self, EspError> {
        let mut encoder = Box::pin(encoder);

        // SAFETY: The reference will only be used to mutate the encoder and never to move it.
        let reference = unsafe { encoder.as_mut().get_unchecked_mut() };
        let sys_config = rmt_simple_encoder_config_t {
            callback: Some(delegator::<T>),
            arg: reference as *mut T as *mut c_void,
            min_chunk_size: config.min_chunk_size,
        };

        let mut handle: rmt_encoder_handle_t = ptr::null_mut();
        // SAFETY: the config is copied by the c code
        esp!(unsafe { rmt_new_simple_encoder(&sys_config, &mut handle) })?;
        Ok(Self {
            handle,
            _encoder: encoder,
        })
    }
}

impl<T: EncoderCallback> Encoder for SimpleEncoder<T> {
    type Item = T::Item;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}

impl<T> Drop for SimpleEncoder<T> {
    fn drop(&mut self) {
        // This is calling encoder->del(encoder);
        unsafe { rmt_del_encoder(self.handle) };
    }
}
