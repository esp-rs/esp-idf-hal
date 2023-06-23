#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
pub use alloc::ffi::CString;

pub use core::ffi::{c_char, CStr};

#[cfg(feature = "alloc")]
pub fn set_str(buf: &mut [u8], s: &str) -> Result<(), esp_idf_sys::EspError> {
    assert!(s.len() < buf.len());
    let cs = to_cstring_arg(s)?;
    let ss: &[u8] = cs.as_bytes_with_nul();
    buf[..ss.len()].copy_from_slice(ss);

    Ok(())
}

pub unsafe fn from_cstr_ptr<'a>(ptr: *const c_char) -> &'a str {
    CStr::from_ptr(ptr).to_str().unwrap()
}

pub fn from_cstr(buf: &[u8]) -> &str {
    // We have to find the first '\0' ourselves, because the passed buffer might
    // be wider than the ASCIIZ string it contains
    let len = buf.iter().position(|e| *e == 0).unwrap() + 1;

    unsafe { CStr::from_bytes_with_nul_unchecked(&buf[0..len]) }
        .to_str()
        .unwrap()
}

#[cfg(feature = "alloc")]
pub struct RawCstrs(alloc::vec::Vec<CString>);

#[cfg(feature = "alloc")]
impl RawCstrs {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self(alloc::vec::Vec::new())
    }

    #[allow(dead_code)]
    pub fn as_ptr(&mut self, s: impl AsRef<str>) -> *const c_char {
        let cs = CString::new(s.as_ref()).unwrap();

        let cstr_ptr = cs.as_ptr();

        self.0.push(cs);

        cstr_ptr
    }

    #[allow(dead_code)]
    pub fn as_nptr<S>(&mut self, s: Option<S>) -> *const c_char
    where
        S: AsRef<str>,
    {
        s.map(|s| self.as_ptr(s)).unwrap_or(core::ptr::null())
    }
}

#[cfg(feature = "alloc")]
impl Default for RawCstrs {
    fn default() -> Self {
        RawCstrs::new()
    }
}

#[cfg(feature = "alloc")]
pub fn nul_to_invalid_arg(err: alloc::ffi::NulError) -> esp_idf_sys::EspError {
    esp_idf_sys::EspError::from_non_zero(
        core::num::NonZeroI32::new(esp_idf_sys::ESP_ERR_INVALID_ARG).unwrap(),
    )
}

#[cfg(feature = "alloc")]
pub fn to_cstring_arg(value: &str) -> Result<CString, esp_idf_sys::EspError> {
    CString::new(value).map_err(nul_to_invalid_arg)
}
