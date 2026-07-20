//! Safe interaction with the NimBLE os_mbuf buffer system

use core::ffi::{c_int, c_void};
use core::marker::PhantomData;

use crate::sys::*;

use super::BleError;

/// View of an os_mbuf, the data buffers used by NimBLE
pub struct Mbuf<'a> {
    om: *mut os_mbuf,
    _p: PhantomData<&'a mut os_mbuf>,
}

impl Mbuf<'_> {
    pub(crate) fn from_raw(om: *mut os_mbuf) -> Self {
        Self {
            om,
            _p: PhantomData,
        }
    }

    /// Copy this Mbuf into `buf`, returning the number of bytes copied or error if buf is too small
    pub fn read(&self, buf: &mut [u8]) -> Result<usize, BleError> {
        let mut copied: u16 = 0;

        BleError::from_raw(unsafe {
            ble_hs_mbuf_to_flat(
                self.om,
                buf.as_mut_ptr() as *mut c_void,
                buf.len() as u16,
                &mut copied,
            )
        })?;

        Ok(copied as usize)
    }

    /// Append `buf` to the mbuf.
    pub fn append(&mut self, buf: &[u8]) -> Result<(), BleError> {
        BleError::from_raw(unsafe {
            r_os_mbuf_append(self.om, buf.as_ptr() as *const c_void, buf.len() as u16)
        })
    }
}

/// Allocate an `os_mbuf` and copy `buf` into it. Errors with `BLE_HS_ENOMEM` if
/// allocation fails.
pub(crate) fn mbuf_from_slice(buf: &[u8]) -> Result<*mut os_mbuf, BleError> {
    let om = unsafe { ble_hs_mbuf_from_flat(buf.as_ptr() as *const c_void, buf.len() as u16) };

    if om.is_null() {
        Err(BleError::new(BLE_HS_ENOMEM as c_int))
    } else {
        Ok(om)
    }
}
