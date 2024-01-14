use crate::stat;
use core::ffi;

static mut __ATEXIT_INTERNAL_REFERENCE: *mut ffi::c_void = atexit as *mut _;

pub fn link_patches() -> *mut ffi::c_void {
    unsafe { __ATEXIT_INTERNAL_REFERENCE }
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn atexit(f: Option<unsafe extern "C" fn()>) -> ffi::c_int {
    crate::esp_register_shutdown_handler(f)
}
