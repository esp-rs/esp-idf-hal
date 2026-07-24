//! NimBLE GAP: device name, connection events, and extended advertising.

use core::ffi::c_int;
use core::ptr;

use alloc::ffi::CString;

use crate::sys::*;

use super::gatt::gatts::ConnectionId;
use super::gatt::Handle;
use super::{BleAddr, BleError};

/// Set the device name exposed via the GAP service.
pub fn svc_set_device_name(name: &str) -> Result<(), BleError> {
    let name = CString::new(name).map_err(|_| BleError::new(BLE_HS_EINVAL as c_int))?;

    // NimBLE copies the name into its own buffer, so the drop is safe
    BleError::from_raw(unsafe { ble_svc_gap_device_name_set(name.as_ptr()) })
}

// Advertising. Drive these from an `on_sync` closure once the host has synced,
// and restart from an `on_gap_event` disconnect handler.
//
// NimBLE exposes two mutually-exclusive advertising APIs. The legacy API below is
// the default; the extended API (`ext_adv_*`) is only compiled when the controller
// is built with `CONFIG_BT_NIMBLE_EXT_ADV=y`.

/// Parameters for a legacy advertising procedure (safe version of `ble_gap_adv_params`).
#[cfg(not(esp_idf_bt_nimble_ext_adv))]
#[derive(Clone, Copy, Default)]
pub struct BleAdvParams {
    pub conn_mode: u8,
    pub disc_mode: u8,
    pub itvl_min: u16,
    pub itvl_max: u16,
    pub channel_map: u8,
    pub filter_policy: u8,
    pub high_duty_cycle: bool,
}

#[cfg(not(esp_idf_bt_nimble_ext_adv))]
impl From<&BleAdvParams> for ble_gap_adv_params {
    fn from(params: &BleAdvParams) -> Self {
        let mut raw: ble_gap_adv_params = unsafe { core::mem::zeroed() };

        raw.conn_mode = params.conn_mode;
        raw.disc_mode = params.disc_mode;
        raw.itvl_min = params.itvl_min;
        raw.itvl_max = params.itvl_max;
        raw.channel_map = params.channel_map;
        raw.filter_policy = params.filter_policy;
        raw.set_high_duty_cycle(params.high_duty_cycle as _);

        raw
    }
}

/// Parameters for an extended advertising instance (safe version of `ble_gap_ext_adv_params`).
///
/// Only available when the controller is built with `CONFIG_BT_NIMBLE_EXT_ADV=y`;
/// the default build exposes the legacy [`BleAdvParams`].
#[cfg(esp_idf_bt_nimble_ext_adv)]
#[derive(Clone, Copy, Default)]
pub struct BleExtAdvParams {
    pub connectable: bool,
    pub scannable: bool,
    pub legacy_pdu: bool,
    pub directed: bool,
    pub anonymous: bool,
    pub high_duty_directed: bool,
    pub include_tx_power: bool,
    pub itvl_min: u32,
    pub itvl_max: u32,
    pub channel_map: u8,
    pub own_addr_type: u8,
    pub primary_phy: u8,
    pub secondary_phy: u8,
    pub tx_power: i8,
    pub sid: u8,
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
impl From<&BleExtAdvParams> for ble_gap_ext_adv_params {
    fn from(params: &BleExtAdvParams) -> Self {
        let mut raw: ble_gap_ext_adv_params = unsafe { core::mem::zeroed() };

        raw.set_connectable(params.connectable as _);
        raw.set_scannable(params.scannable as _);
        raw.set_legacy_pdu(params.legacy_pdu as _);
        raw.set_directed(params.directed as _);
        raw.set_anonymous(params.anonymous as _);
        raw.set_high_duty_directed(params.high_duty_directed as _);
        raw.set_include_tx_power(params.include_tx_power as _);

        raw.itvl_min = params.itvl_min;
        raw.itvl_max = params.itvl_max;
        raw.channel_map = params.channel_map;
        raw.own_addr_type = params.own_addr_type;
        raw.primary_phy = params.primary_phy;
        raw.secondary_phy = params.secondary_phy;
        raw.tx_power = params.tx_power;
        raw.sid = params.sid;

        raw
    }
}

/// Structured advertising payload (safe version of `ble_hs_adv_fields`).
/// You can also directly use ext_adv_set_data/adv_set_data to set the raw data yourself.
#[derive(Clone, Copy, Default)]
pub struct BleAdvFields<'a> {
    pub flags: u8,
    pub name: Option<&'a str>,
    pub tx_power_level: Option<i8>,
    pub appearance: Option<u16>,
    pub service_data_uuid16: Option<&'a [u8]>,
    pub manufacturer_data: Option<&'a [u8]>,
}

impl From<&BleAdvFields<'_>> for ble_hs_adv_fields {
    fn from(fields: &BleAdvFields) -> Self {
        let mut raw: ble_hs_adv_fields = unsafe { core::mem::zeroed() };

        raw.flags = fields.flags;

        if let Some(name) = fields.name {
            raw.name = name.as_ptr();
            raw.name_len = name.len() as _;
            raw.set_name_is_complete(1);
        }

        if let Some(tx_power_level) = fields.tx_power_level {
            raw.tx_pwr_lvl = tx_power_level;
            raw.set_tx_pwr_lvl_is_present(1);
        }

        if let Some(appearance) = fields.appearance {
            raw.appearance = appearance;
            raw.set_appearance_is_present(1);
        }

        if let Some(service_data) = fields.service_data_uuid16 {
            raw.svc_data_uuid16 = service_data.as_ptr();
            raw.svc_data_uuid16_len = service_data.len() as _;
        }

        if let Some(manufacturer_data) = fields.manufacturer_data {
            raw.mfg_data = manufacturer_data.as_ptr();
            raw.mfg_data_len = manufacturer_data.len() as _;
        }

        raw
    }
}

pub enum BleGapEvent {
    Connect {
        conn_handle: ConnectionId,
        status: Result<(), BleError>,
    },
    Disconnect {
        conn_handle: ConnectionId,
        reason: BleError,
    },
    Subscribe {
        conn_handle: ConnectionId,
        attr_handle: Handle,
        cur_indicate: bool,
        cur_notify: bool,
    },
    Mtu {
        conn_handle: ConnectionId,
        value: u16,
    },
    NotifyTx {
        conn_handle: ConnectionId,
        status: i32,
    },
    Other,
}

impl From<&ble_gap_event> for BleGapEvent {
    fn from(event: &ble_gap_event) -> Self {
        let anon = &event.__bindgen_anon_1;

        match event.type_ as u32 {
            BLE_GAP_EVENT_CONNECT => {
                let connect = unsafe { &anon.connect };
                Self::Connect {
                    conn_handle: connect.conn_handle,
                    status: BleError::from_raw(connect.status),
                }
            }
            BLE_GAP_EVENT_DISCONNECT => {
                let disconnect = unsafe { &anon.disconnect };
                Self::Disconnect {
                    conn_handle: disconnect.conn.conn_handle,
                    reason: BleError::new(disconnect.reason),
                }
            }
            BLE_GAP_EVENT_SUBSCRIBE => {
                let subscribe = unsafe { &anon.subscribe };
                Self::Subscribe {
                    conn_handle: subscribe.conn_handle,
                    attr_handle: subscribe.attr_handle,
                    cur_indicate: subscribe.cur_indicate() != 0,
                    cur_notify: subscribe.cur_notify() != 0,
                }
            }
            BLE_GAP_EVENT_MTU => {
                let mtu = unsafe { &anon.mtu };
                Self::Mtu {
                    conn_handle: mtu.conn_handle,
                    value: mtu.value,
                }
            }
            BLE_GAP_EVENT_NOTIFY_TX => {
                let notify_tx = unsafe { &anon.notify_tx };
                Self::NotifyTx {
                    conn_handle: notify_tx.conn_handle,
                    status: notify_tx.status,
                }
            }
            _ => Self::Other,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct BleConnDesc(ble_gap_conn_desc);

impl BleConnDesc {
    pub const fn peer_addr(&self) -> BleAddr {
        BleAddr::new(self.0.peer_id_addr.type_, self.0.peer_id_addr.val)
    }
}

/// Look up a connection descriptor by handle.
pub fn conn_find(conn_handle: ConnectionId) -> Result<BleConnDesc, BleError> {
    let mut desc: ble_gap_conn_desc = unsafe { core::mem::zeroed() };
    BleError::from_raw(unsafe { ble_gap_conn_find(conn_handle, &mut desc) })?;

    Ok(BleConnDesc(desc))
}

#[cfg(not(esp_idf_bt_nimble_ext_adv))]
pub fn adv_set_data(data: &[u8]) -> Result<(), BleError> {
    // NimBLE copies the payload into its own buffer, so `data` need not outlive the call.
    BleError::from_raw(unsafe { ble_gap_adv_set_data(data.as_ptr(), data.len() as c_int) })
}

#[cfg(not(esp_idf_bt_nimble_ext_adv))]
pub fn adv_set_fields(fields: &BleAdvFields) -> Result<(), BleError> {
    let raw: ble_hs_adv_fields = fields.into();

    BleError::from_raw(unsafe { ble_gap_adv_set_fields(&raw) })
}

#[cfg(not(esp_idf_bt_nimble_ext_adv))]
pub fn adv_start(own_addr_type: u8, params: &BleAdvParams) -> Result<(), BleError> {
    let raw: ble_gap_adv_params = params.into();

    // bindgen does not emit `BLE_HS_FOREVER`, as its C macro expands to `INT32_MAX` rather than to
    // an integer literal. Advertise with no timeout.
    const BLE_HS_FOREVER: c_int = i32::MAX;

    let rc = unsafe {
        ble_gap_adv_start(
            own_addr_type,
            ptr::null(),
            BLE_HS_FOREVER as _,
            &raw,
            Some(super::gap_event_cb),
            ptr::null_mut(),
        )
    };
    if rc == BLE_HS_EALREADY as c_int {
        return Ok(());
    }

    BleError::from_raw(rc)
}

#[cfg(not(esp_idf_bt_nimble_ext_adv))]
pub fn adv_stop() -> Result<(), BleError> {
    BleError::from_raw(unsafe { ble_gap_adv_stop() })
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_configure(instance: u8, params: &BleExtAdvParams) -> Result<i8, BleError> {
    let raw: ble_gap_ext_adv_params = params.into();
    let mut selected_tx_power: i8 = 0;

    BleError::from_raw(unsafe {
        ble_gap_ext_adv_configure(
            instance,
            &raw,
            &mut selected_tx_power,
            Some(super::gap_event_cb),
            ptr::null_mut(),
        )
    })?;

    Ok(selected_tx_power)
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_set_addr(instance: u8, addr: &BleAddr) -> Result<(), BleError> {
    BleError::from_raw(unsafe { ble_gap_ext_adv_set_addr(instance, addr.raw()) })
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_set_data(instance: u8, data: &[u8]) -> Result<(), BleError> {
    let om = super::mbuf::mbuf_from_slice(data)?;

    // `ble_gap_ext_adv_set_data` takes ownership of `om` and frees it on all paths (no leak, no double-free).
    BleError::from_raw(unsafe { ble_gap_ext_adv_set_data(instance, om) })
}

/// Encode `fields` into an advertising payload and install it on `instance`.
#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_set_fields(instance: u8, fields: &BleAdvFields) -> Result<(), BleError> {
    let raw: ble_hs_adv_fields = fields.into();

    let mut buf = [0u8; BLE_HS_ADV_MAX_SZ as usize];
    let mut len: u8 = 0;
    BleError::from_raw(unsafe {
        ble_hs_adv_set_fields(&raw, buf.as_mut_ptr(), &mut len, buf.len() as u8)
    })?;

    ext_adv_set_data(instance, &buf[..len as usize])
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_start(instance: u8) -> Result<(), BleError> {
    let rc = unsafe { ble_gap_ext_adv_start(instance, 0, 0) };
    if rc == BLE_HS_EALREADY as c_int {
        return Ok(());
    }

    BleError::from_raw(rc)
}

#[cfg(esp_idf_bt_nimble_ext_adv)]
pub fn ext_adv_stop(instance: u8) -> Result<(), BleError> {
    BleError::from_raw(unsafe { ble_gap_ext_adv_stop(instance) })
}
