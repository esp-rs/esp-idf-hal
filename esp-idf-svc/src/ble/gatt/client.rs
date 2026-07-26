//! NimBLE GATT client: connection, discovery, read/write, and received notifications, as client
//! operations on the [`BleDriver`].
//!
//! Every `ble_gattc_*` operation is initiate-now, complete-later: you start it, and NimBLE invokes
//! a callback when it finishes. We route *all* of those per-operation callbacks through one shared
//! [`gattc_subscribe`](BleDriver::gattc_subscribe) hook, correlated by `conn_handle` (GATT serializes
//! one transaction per connection). Received notifications/indications ([`GattcEvent::Notify`])
//! arrive on the connection's GAP callback and are demuxed here.

use core::ffi::{c_int, c_void};
use core::ptr;

use crate::sys::*;

use super::super::mbuf::Mbuf;
use super::super::{BleAddr, BleDriver, BleError, BleUuid, ConnHandle};
use super::AttrHandle;

/// A GATT-client event, delivered on the host task to the single
/// [`gattc_subscribe`](BleDriver::gattc_subscribe) hook.
///
/// The discovery variants fire once per discovered item and then once more with a `None` payload
/// to signal completion. `status` is the raw ATT/`BLE_HS_*` status (`0` on success).
pub enum GattcEvent<'a> {
    /// A service discovered by [`discover_services`](BleDriver::discover_services).
    Service {
        conn_handle: ConnHandle,
        status: u16,
        service: Option<GattcService>,
    },
    /// A characteristic discovered by
    /// [`discover_characteristics`](BleDriver::discover_characteristics).
    Characteristic {
        conn_handle: ConnHandle,
        status: u16,
        chr: Option<GattcChr>,
    },
    /// Completion of a [`read`](BleDriver::read). On success `data` holds the value; check `status`
    /// before reading it.
    ReadComplete {
        conn_handle: ConnHandle,
        status: u16,
        attr_handle: AttrHandle,
        data: Mbuf<'a>,
    },
    /// Completion of a [`write`](BleDriver::write).
    WriteComplete {
        conn_handle: ConnHandle,
        status: u16,
        attr_handle: AttrHandle,
    },
    /// A notification or indication pushed by the peer (after subscribing by writing its CCCD).
    Notify {
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        indication: bool,
        data: Mbuf<'a>,
    },
}

impl<'a> GattcEvent<'a> {
    /// Build [`Notify`](Self::Notify) from a raw GAP event. Called from the GAP trampoline's
    /// demux for `BLE_GAP_EVENT_NOTIFY_RX`.
    pub(crate) fn from_notify_rx(event: &'a ble_gap_event) -> Self {
        let notify_rx = unsafe { &event.__bindgen_anon_1.notify_rx };

        Self::Notify {
            conn_handle: notify_rx.conn_handle,
            attr_handle: notify_rx.attr_handle,
            indication: notify_rx.indication() != 0,
            data: Mbuf::from_raw(notify_rx.om),
        }
    }
}

/// A remote service discovered on a peer (safe view of `ble_gatt_svc`).
pub struct GattcService {
    pub start_handle: AttrHandle,
    pub end_handle: AttrHandle,
    pub uuid: BleUuid,
}

impl From<&ble_gatt_svc> for GattcService {
    fn from(svc: &ble_gatt_svc) -> Self {
        Self {
            start_handle: svc.start_handle,
            end_handle: svc.end_handle,
            // `ble_uuid_any_t`'s first union member is the `ble_uuid_t` header, at offset 0.
            uuid: unsafe { BleUuid::from_raw((&svc.uuid as *const ble_uuid_any_t).cast()) },
        }
    }
}

/// A remote characteristic discovered on a peer (safe view of `ble_gatt_chr`).
pub struct GattcChr {
    pub def_handle: AttrHandle,
    pub val_handle: AttrHandle,
    /// Characteristic properties bitmask (`BLE_GATT_CHR_PROP_*`).
    pub properties: u8,
    pub uuid: BleUuid,
}

impl From<&ble_gatt_chr> for GattcChr {
    fn from(chr: &ble_gatt_chr) -> Self {
        Self {
            def_handle: chr.def_handle,
            val_handle: chr.val_handle,
            properties: chr.properties,
            uuid: unsafe { BleUuid::from_raw((&chr.uuid as *const ble_uuid_any_t).cast()) },
        }
    }
}

/// GATT-client operations on the [`BleDriver`]. Available for any role (`S`) — a device can be
/// both a server and a client. `&self`, so callable re-entrantly from within the client callback.
impl<'d, S> BleDriver<'d, S> {
    /// Subscribe to GATT-client events ([`GattcEvent`]): per-operation completions plus received
    /// notifications/indications.
    pub fn gattc_subscribe<F>(&self, callback: F)
    where
        F: for<'a> FnMut(GattcEvent<'a>) + Send + 'static,
    {
        unsafe { self.gattc_subscribe_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The non-`'static` counterpart of [`gattc_subscribe`](Self::gattc_subscribe). See
    /// [`BleDriver::host_subscribe_nonstatic`](crate::ble::BleDriver::host_subscribe_nonstatic) for the borrowing
    /// rules and the `core::mem::forget` hazard.
    pub unsafe fn gattc_subscribe_nonstatic<F>(&self, callback: F)
    where
        F: for<'a> FnMut(GattcEvent<'a>) + Send + 'd,
    {
        unsafe { super::super::SINGLETON.gattc.subscribe_nonstatic(callback) };
    }

    /// Stop delivering GATT-client events to the subscribed hook.
    pub fn gattc_unsubscribe(&self) {
        super::super::SINGLETON.gattc.unsubscribe();
    }

    /// Initiate a connection to `peer`. The connect/disconnect outcome arrives on the GAP hook
    /// ([`gap_subscribe`](BleDriver::gap_subscribe)); the connection's received notifications arrive
    /// on the GATTC hook.
    pub fn connect(&self, own_addr_type: u8, peer: &BleAddr) -> Result<(), BleError> {
        // bindgen does not emit `BLE_HS_FOREVER` (its C macro is `INT32_MAX`); inline it.
        const BLE_HS_FOREVER: c_int = i32::MAX;

        BleError::from_raw(unsafe {
            ble_gap_connect(
                own_addr_type,
                peer.raw() as *const _,
                BLE_HS_FOREVER,
                ptr::null(),
                Some(super::super::BleSingleton::gap_event_cb),
                ptr::null_mut(),
            )
        })
    }

    /// Terminate the connection `conn_handle`.
    pub fn disconnect(&self, conn_handle: ConnHandle) -> Result<(), BleError> {
        // 0x13 = BLE_ERR_REM_USER_CONN_TERM ("remote user terminated connection").
        BleError::from_raw(unsafe { ble_gap_terminate(conn_handle, 0x13) })
    }

    /// Discover all of the peer's primary services. Results arrive as [`GattcEvent::Service`].
    pub fn discover_services(&self, conn_handle: ConnHandle) -> Result<(), BleError> {
        BleError::from_raw(unsafe {
            ble_gattc_disc_all_svcs(
                conn_handle,
                Some(super::super::BleSingleton::gattc_disc_svc_cb),
                ptr::null_mut(),
            )
        })
    }

    /// Discover the peer's characteristics in the attribute-handle range `[start_handle,
    /// end_handle]` (e.g. a service's range). Results arrive as [`GattcEvent::Characteristic`].
    pub fn discover_characteristics(
        &self,
        conn_handle: ConnHandle,
        start_handle: AttrHandle,
        end_handle: AttrHandle,
    ) -> Result<(), BleError> {
        BleError::from_raw(unsafe {
            ble_gattc_disc_all_chrs(
                conn_handle,
                start_handle,
                end_handle,
                Some(super::super::BleSingleton::gattc_disc_chr_cb),
                ptr::null_mut(),
            )
        })
    }

    /// Read the value of `attr_handle` on the peer (a Read Request). The result arrives as
    /// [`GattcEvent::ReadComplete`].
    pub fn read(&self, conn_handle: ConnHandle, attr_handle: AttrHandle) -> Result<(), BleError> {
        BleError::from_raw(unsafe {
            ble_gattc_read(
                conn_handle,
                attr_handle,
                Some(super::super::BleSingleton::gattc_read_cb),
                ptr::null_mut(),
            )
        })
    }

    /// Write `data` to `attr_handle` on the peer as a **Write Request** (acknowledged): the peer
    /// sends a Write Response, and its completion arrives as [`GattcEvent::WriteComplete`].
    pub fn write(
        &self,
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        data: &[u8],
    ) -> Result<(), BleError> {
        // NimBLE copies `data` into its own mbuf, so it need not outlive the call.
        BleError::from_raw(unsafe {
            ble_gattc_write_flat(
                conn_handle,
                attr_handle,
                data.as_ptr() as *const c_void,
                data.len() as u16,
                Some(super::super::BleSingleton::gattc_write_cb),
                ptr::null_mut(),
            )
        })
    }

    /// Write `data` to `attr_handle` on the peer as a **Write Command** (unacknowledged): the peer
    /// sends no response, so this is fire-and-forget — there is **no** completion event. The
    /// returned `Result` only reflects whether the command was accepted for transmission.
    pub fn write_cmd(
        &self,
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        data: &[u8],
    ) -> Result<(), BleError> {
        BleError::from_raw(unsafe {
            ble_gattc_write_no_rsp_flat(
                conn_handle,
                attr_handle,
                data.as_ptr() as *const c_void,
                data.len() as u16,
            )
        })
    }
}
