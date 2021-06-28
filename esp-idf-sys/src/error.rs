#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::{fmt, slice, str};

use crate::*;

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct EspError(esp_err_t);

impl EspError {
    pub fn from(error: esp_err_t) -> Option<Self> {
        if error == 0 {
            None
        } else {
            Some(EspError(error))
        }
    }

    pub fn check_and_return<T>(error: esp_err_t, value: T) -> Result<T, Self> {
        if error == 0 {
            Ok(value)
        } else {
            Err(EspError(error))
        }
    }

    pub fn convert(error: esp_err_t) -> Result<(), Self> {
        EspError::check_and_return(error, ())
    }

    pub fn panic(&self) {
        panic!("ESP-IDF ERROR: {}", self);
    }

    pub fn code(&self) -> esp_err_t {
        self.0
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EspError {}

impl fmt::Display for EspError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        unsafe fn strlen(c_s: *const c_types::c_char) -> usize {
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

#[macro_export]
macro_rules! esp {
    ($err:expr) => {{
        esp_idf_sys::EspError::convert($err as esp_idf_sys::esp_err_t)
    }}
}

#[macro_export]
macro_rules! esp_result {
    ($err:expr, $value:expr) => {{
        esp_idf_sys::EspError::check_and_return($err as esp_idf_sys::esp_err_t, $value)
    }}
}

#[macro_export]
macro_rules! esp_nofail {
    ($err:expr) => {{
        if let Some(error) = esp_idf_sys::EspError::from($err as esp_idf_sys::esp_err_t) {
            error.panic();
        }
    }}
}
