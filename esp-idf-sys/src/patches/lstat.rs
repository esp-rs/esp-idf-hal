use crate::*;
use core::ffi;

static mut __LSTAT_INTERNAL_REFERENCE: *mut ffi::c_void = lstat as *mut _;

pub fn link_patches() -> *mut ffi::c_void {
    unsafe { __LSTAT_INTERNAL_REFERENCE }
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn lstat(path: *const ffi::c_char, buf: *mut stat) -> ffi::c_int {
    stat(path, buf)
}
