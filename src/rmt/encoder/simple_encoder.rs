use core::ffi::c_void;
use core::marker::PhantomData;
use core::{mem, ptr, slice};

use esp_idf_sys::*;

use crate::rmt::encoder::Encoder;
use crate::rmt::Symbol;

pub struct SimpleEncoderConfig {
    pub min_chunk_size: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotEnoughSpace;

pub type EncoderCallback<T, A> =
    fn(data: &[T], writer: &mut SymbolWriter<'_>, arg: &mut A) -> Result<(), NotEnoughSpace>;

pub struct DelegatorState<'a, T, A> {
    pub callback: EncoderCallback<T, A>,
    pub arg: &'a mut A,
}

pub struct SymbolWriter<'a> {
    symbols: &'a mut [Symbol],
    written: usize,
}

impl<'a> SymbolWriter<'a> {
    #[must_use]
    pub unsafe fn from_raw_parts(
        symbols: *mut rmt_symbol_word_t,
        written: usize,
        free: usize,
    ) -> Self {
        let symbols = slice::from_raw_parts_mut(symbols as *mut Symbol, written + free);
        Self { symbols, written }
    }

    pub fn remaining(&self) -> usize {
        self.symbols.len() - self.written
    }

    /// Tries to write all symbols to the buffer.
    ///
    /// If there is not enough space, it will not write anything and return `NotEnoughSpace`.
    pub fn write_all(&mut self, symbols: &[Symbol]) -> Result<(), NotEnoughSpace> {
        if self.remaining() < symbols.len() {
            return Err(NotEnoughSpace);
        }

        for &symbol in symbols {
            self.symbols[self.written] = symbol;
            self.written += 1;
        }
        self.written += 1;
        Ok(())
    }
}

unsafe extern "C" fn delegator<T, A>(
    data: *const c_void,
    data_size: usize,
    symbols_written: usize,
    symbols_free: usize,
    symbols: *mut rmt_symbol_word_t,
    done: *mut bool,
    arg: *mut c_void,
) -> usize {
    let state = &mut *(arg as *mut DelegatorState<T, A>);
    let data_slice = slice::from_raw_parts(data as *const T, data_size / mem::size_of::<T>());
    let mut writer = SymbolWriter::from_raw_parts(symbols, symbols_written, symbols_free);

    // TODO: are panics allowed in isr context?
    debug_assert!(!arg.is_null());

    match (state.callback)(data_slice, &mut writer, &mut *state.arg) {
        Ok(()) => {
            *done = true;
            // Calculate how many symbols were written in this call:
            writer.written - symbols_written
        }
        Err(NotEnoughSpace) => writer.written - symbols_written,
    }
}

impl Default for SimpleEncoderConfig {
    fn default() -> Self {
        Self { min_chunk_size: 64 }
    }
}

#[derive(Debug)]
pub struct SimpleEncoder<'a, T, A> {
    handle: rmt_encoder_handle_t,
    _p: PhantomData<(T, &'a mut A)>,
}

impl<'a, T, A> SimpleEncoder<'a, T, A> {
    pub fn with_config(
        state: &mut DelegatorState<'a, T, A>,
        config: &SimpleEncoderConfig,
    ) -> Result<Self, EspError> {
        let sys_config = rmt_simple_encoder_config_t {
            callback: Some(delegator::<T, A> as _),
            arg: (state as *mut _) as *mut c_void,
            min_chunk_size: config.min_chunk_size,
        };

        let mut handle: rmt_encoder_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_simple_encoder(&sys_config, &mut handle) })?;
        Ok(Self {
            handle,
            _p: PhantomData,
        })
    }
}

impl<'a, T, A> Drop for SimpleEncoder<'a, T, A> {
    fn drop(&mut self) {
        // This is calling encoder->del(encoder);
        unsafe { rmt_del_encoder(self.handle) };
    }
}

impl<'a, T, A> Encoder for SimpleEncoder<'a, T, A> {
    type Item = T;

    fn handle(&mut self) -> &mut rmt_encoder_t {
        unsafe { &mut *self.handle }
    }
}
