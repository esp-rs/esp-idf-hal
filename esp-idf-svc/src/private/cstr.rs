#[cfg(feature = "std")]
pub use std::ffi::{CStr, CString};

#[cfg(not(feature = "std"))]
pub use cstr_core::CStr;

#[cfg(all(not(feature = "std"), feature = "alloc"))]
pub use cstr_core::CString;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
pub fn set_str(buf: &mut [u8], s: &str) {
    let cs = CString::new(s).unwrap();
    let ss: &[u8] = cs.as_bytes_with_nul();
    buf[..ss.len()].copy_from_slice(&ss);
}

#[cfg(feature = "alloc")]
pub fn from_cstr(buf: &[u8]) -> alloc::string::String {
    let c_str: &CStr = CStr::from_bytes_with_nul(buf).unwrap();

    alloc::string::String::from(c_str.to_str().unwrap())
}
