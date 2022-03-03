use crate::*;

static mut __LSTAT_INTERNAL_REFERENCE: *mut c_types::c_void = lstat as *mut _;

pub fn link_patches() -> *mut c_types::c_void {
    unsafe { __LSTAT_INTERNAL_REFERENCE }
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn lstat(path: *const c_types::c_char, buf: *mut stat) -> c_types::c_int {
    stat(path, buf)
}
