//! NimBLE GATT: shared types and helpers.

use core::ffi::c_int;

use enumset::{EnumSet, EnumSetType};

use crate::sys::*;

use super::BleError;

pub mod gatts;

/// A GATT attribute handle (e.g. a characteristic's value handle).
pub type Handle = u16;

/// Set the preferred ATT MTU. Safe to call before or after the host starts.
pub fn set_preferred_mtu(mtu: u16) -> Result<(), BleError> {
    BleError::from_raw(unsafe { ble_att_set_preferred_mtu(mtu) })
}

/// The negotiated ATT MTU for a connection
pub fn att_mtu(conn_handle: gatts::ConnectionId) -> Result<u16, BleError> {
    match unsafe { ble_att_mtu(conn_handle) } {
        0 => Err(BleError::new(BLE_HS_ENOTCONN as c_int)),
        mtu => Ok(mtu),
    }
}

/// A GATT characteristic operation flag (`BLE_GATT_CHR_F_*`).
#[derive(Debug, EnumSetType)]
pub enum BleGattCharFlag {
    Read,
    Write,
    Notify,
    Indicate,
}

impl From<BleGattCharFlag> for ble_gatt_chr_flags {
    fn from(flag: BleGattCharFlag) -> Self {
        match flag {
            BleGattCharFlag::Read => BLE_GATT_CHR_F_READ,
            BleGattCharFlag::Write => BLE_GATT_CHR_F_WRITE,
            BleGattCharFlag::Notify => BLE_GATT_CHR_F_NOTIFY,
            BleGattCharFlag::Indicate => BLE_GATT_CHR_F_INDICATE,
        }
    }
}

pub(crate) fn flags_to_repr(flags: EnumSet<BleGattCharFlag>) -> ble_gatt_chr_flags {
    flags
        .iter()
        .fold(0, |acc, flag| acc | ble_gatt_chr_flags::from(flag))
}
