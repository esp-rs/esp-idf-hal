use esp_idf_sys::c_types;

// TODO: Figure out which library references this
#[no_mangle]
pub extern "C" fn timegm(_: c_types::c_void) -> c_types::c_int {
    // Not supported but don't crash just in case
    0
}

// Called by the rand crate
#[no_mangle]
pub extern "C" fn pthread_atfork(
    _: *const c_types::c_void,
    _: *const c_types::c_void,
    _: *const c_types::c_void,
) -> c_types::c_int {
    0
}
