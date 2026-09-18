// TODO: This is __wrong__ and needs to be replaced with a proper implementation e.g. in the spirit of this:
// https://en.wikipedia.org/wiki/Readers%E2%80%93writer_lock#Using_a_condition_variable_and_a_mutex

use crate::*;
use core::ffi;

static mut __PTHREAD_RWLOCK_INTERNAL_REFERENCE: *mut ffi::c_void = pthread_rwlock_init as *mut _;

pub fn link_patches() -> *mut ffi::c_void {
    unsafe { __PTHREAD_RWLOCK_INTERNAL_REFERENCE }
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_init(
    rwlock: *mut ffi::c_void,
    attr: *const ffi::c_void,
) -> ffi::c_int {
    pthread_mutex_init(rwlock as *mut _, attr as *const _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_rdlock(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_lock(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_tryrdlock(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_trylock(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_wrlock(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_lock(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_trywrlock(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_trylock(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_unlock(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_unlock(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlock_destroy(rwlock: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutex_destroy(rwlock as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlockattr_init(attr: *mut ffi::c_void) -> ffi::c_int {
    pthread_mutexattr_init(attr as *mut _)
}

#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn pthread_rwlockattr_destroy(attr: *mut ffi::c_void) -> ffi::c_int {
    pthread_rwlockattr_init(attr as *mut _)
}
