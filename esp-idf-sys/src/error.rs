use core::{ffi, fmt, slice, str};

use crate::{esp_err_t, esp_err_to_name, ESP_OK};

/// A wrapped [`esp_err_t`] to check if an error occurred.
///
/// An [`esp_err_t`] is returned from most esp-idf APIs as a status code. If it is equal
/// to [`ESP_OK`] it means **no** error occurred.
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct EspError(esp_err_t);

impl EspError {
    /// Wrap an [`esp_err_t`], return [`Some`] if `error` is **not** [`ESP_OK`].
    pub const fn from(error: esp_err_t) -> Option<Self> {
        if error == ESP_OK {
            None
        } else {
            Some(EspError(error))
        }
    }

    /// Convert `error` into a [`Result`] with `Ok(value)` if no error occurred.
    ///
    /// If `error` is [`ESP_OK`] return [`Ok`] of `value` otherwise return [`Err`] of
    /// wrapped `error`.
    pub fn check_and_return<T>(error: esp_err_t, value: T) -> Result<T, Self> {
        if error == ESP_OK {
            Ok(value)
        } else {
            Err(EspError(error))
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
        self.0
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
