use core::{ffi, fmt, num::NonZeroI32, slice, str};

use crate::{esp_err_t, esp_err_to_name, ESP_OK};

/// A wrapped [`esp_err_t`] to check if an error occurred.
///
/// An [`esp_err_t`] is returned from most esp-idf APIs as a status code. If it is equal
/// to [`ESP_OK`] it means **no** error occurred.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct EspError(NonZeroI32);

const _: () = if ESP_OK != 0 {
    panic!("ESP_OK *has* to be 0")
};

impl EspError {
    /// Wrap an [`esp_err_t`], return [`Some`] if `error` is **not** [`ESP_OK`].
    pub const fn from(error: esp_err_t) -> Option<Self> {
        match NonZeroI32::new(error) {
            None => None,
            Some(err) => Some(Self(err)),
        }
    }

    /// Wrap a [`NonZeroI32`]. Since [`ESP_OK`] is 0, this can never fail;
    pub const fn from_non_zero(error: NonZeroI32) -> Self {
        Self(error)
    }

    /// Wrap an [`esp_err_t`], throw a compile time error if `error` is [`ESP_OK`].
    pub const fn from_infallible<const E: esp_err_t>() -> Self {
        // workaround until feature(inline_const) is stabilized: https://github.com/rust-lang/rust/pull/104087
        struct Dummy<const D: esp_err_t>;
        impl<const D: esp_err_t> Dummy<D> {
            pub const ERR: EspError = match EspError::from(D) {
                Some(err) => err,
                None => panic!("ESP_OK can't be an error"),
            };
        }
        Dummy::<E>::ERR
    }

    /// Convert `error` into a [`Result`] with `Ok(value)` if no error occurred.
    ///
    /// If `error` is [`ESP_OK`] return [`Ok`] of `value` otherwise return [`Err`] of
    /// wrapped `error`.
    pub fn check_and_return<T>(error: esp_err_t, value: T) -> Result<T, Self> {
        match NonZeroI32::new(error) {
            None => Ok(value),
            Some(err) => Err(Self(err)),
        }
    }

    /// Convert `error` into a [`Result`] with `Ok(())` if not error occurred..
    ///
    /// If `error` equals to [`ESP_OK`] return [`Ok`], otherwise return [`Err`] with the
    /// wrapped [`esp_err_t`].
    pub fn convert(error: esp_err_t) -> Result<(), Self> {
        EspError::check_and_return(error, ())
    }

    /// Panic with a specific error message of the contained [`esp_err_t`].
    #[track_caller]
    pub fn panic(&self) {
        panic!("ESP-IDF ERROR: {self}");
    }

    /// Get the wrapped [`esp_err_t`].
    pub fn code(&self) -> esp_err_t {
        self.0.get()
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EspError {}

impl fmt::Display for EspError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unsafe fn strlen(c_s: *const ffi::c_char) -> usize {
            let mut len = 0;
            while *c_s.offset(len) != 0 {
                len += 1;
            }

            len as usize
        }

        unsafe {
            let c_s = esp_err_to_name(self.code());
            str::from_utf8_unchecked(slice::from_raw_parts(c_s as *const u8, strlen(c_s))).fmt(f)
        }
    }
}

/// Convert an [`esp_err_t`] into a [`Result<(), EspError>`](Result).
///
/// See [`EspError::convert`].
#[macro_export]
macro_rules! esp {
    ($err:expr) => {{
        $crate::EspError::convert($err as $crate::esp_err_t)
    }};
}

/// Convert an [`esp_err_t`] into a [`Result<T, EspError>`](Result).
///
/// See [`EspError::check_and_return`].
#[macro_export]
macro_rules! esp_result {
    ($err:expr, $value:expr) => {{
        $crate::EspError::check_and_return($err as $crate::esp_err_t, $value)
    }};
}

/// Panic with an error-specific message if `err` is not [`ESP_OK`].
///
/// See [`EspError::from`] and [`EspError::panic`].
#[macro_export]
macro_rules! esp_nofail {
    ($err:expr) => {{
        if let ::core::option::Option::Some(error) =
            $crate::EspError::from($err as $crate::esp_err_t)
        {
            error.panic();
        }
    }};
}
