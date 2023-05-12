use crate::stat;
use core::ffi;

static mut __LSTAT_INTERNAL_REFERENCE: *mut ffi::c_void = lstat as *mut _;

pub fn link_patches() -> *mut ffi::c_void {
    unsafe { __LSTAT_INTERNAL_REFERENCE }
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn lstat(path: *const ffi::c_char, buf: *mut stat) -> ffi::c_int {
    extern "C" {
        // Declare it manually, because if certain ESP IDF components are not included (VFS?)
        // as is the case for CMake based builds, the signature of the `stat` fn is actually
        // not defined in the `esp-idf-sys` bindings
        #[link_name = "stat"]
        fn esp_idf_sys_stat(path: *const ffi::c_char, buf: *mut stat) -> ffi::c_int;
    }

    esp_idf_sys_stat(path, buf)
}
