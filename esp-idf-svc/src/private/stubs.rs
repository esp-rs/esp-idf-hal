use core::ffi;

// TODO: Figure out which library references this
#[no_mangle]
pub extern "C" fn timegm(_: ffi::c_void) -> ffi::c_int {
    // Not supported but don't crash just in case
    0
}

// Called by the rand crate
#[no_mangle]
pub extern "C" fn pthread_atfork(
    _: *const ffi::c_void,
    _: *const ffi::c_void,
    _: *const ffi::c_void,
) -> ffi::c_int {
    0
}
