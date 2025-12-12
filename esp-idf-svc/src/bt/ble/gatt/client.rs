use core::borrow::Borrow;
use core::fmt::{self, Debug};
use core::marker::PhantomData;

use ::log::trace;
use enumset::EnumSet;
use num_enum::TryFromPrimitive;

use crate::bt::ble::gap::BleAddrType;
use crate::bt::ble::gatt::{GattId, Property};
use crate::bt::{BdAddr, BleEnabled, BtDriver, BtSingleton, BtUuid};
use crate::sys::*;

use super::{GattConnParams, GattConnReason, GattInterface, GattStatus, Handle};

pub type AppId = u16;
pub type ConnectionId = u16;
pub type TransferId = u32;

pub const INVALID_HANDLE: u16 = 0;

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum ServiceSource {
    /// Service information from a remote device. Relates to BTA_GATTC_SERVICE_INFO_FROM_REMOTE_DEVICE.
    RemoteDevice = esp_service_source_t_ESP_GATT_SERVICE_FROM_REMOTE_DEVICE,
    /// Service information from NVS flash. Relates to BTA_GATTC_SERVICE_INFO_FROM_NVS_FLASH.
    Nvs = esp_service_source_t_ESP_GATT_SERVICE_FROM_NVS_FLASH,
    /// Service source is unknown. Relates to BTA_GATTC_SERVICE_INFO_FROM_UNKNOWN
    Uknown = esp_service_source_t_ESP_GATT_SERVICE_FROM_UNKNOWN,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum LinkRole {
    Master = 0,
    Slave = 1,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum DbAttrType {
    /// Primary service attribute, with start and end handle
    PrimaryService {
        start_handle: Handle,
        end_handle: Handle,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_PRIMARY_SERVICE,
    /// Secondary service attribute
    SecondaryService {
        start_handle: Handle,
        end_handle: Handle,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_SECONDARY_SERVICE,
    /// Characteristic attribute
    Characteristic {
        start_handle: Handle,
        end_handle: Handle,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_CHARACTERISTIC,
    /// Descriptor attribute - with the characteristic handle
    Descriptor { handle: Handle } = esp_gatt_db_attr_type_t_ESP_GATT_DB_DESCRIPTOR,
    /// Included service attribute
    IncludedService {
        start_handle: Handle,
        end_handle: Handle,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_INCLUDED_SERVICE,
    /// All attribute types
    AllAttributes {
        start_handle: Handle,
        end_handle: Handle,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_ALL,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum GattAuthReq {
    /// No authentication required. Corresponds to BTA_GATT_AUTH_REQ_NONE
    None = esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_NONE,
    /// Unauthenticated encryption. Corresponds to BTA_GATT_AUTH_REQ_NO_MITM
    NoMitm = esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_NO_MITM,
    /// Authenticated encryption (MITM protection). Corresponds to BTA_GATT_AUTH_REQ_MITM
    Mitm = esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_MITM,
    /// Signed data, no MITM protection. Corresponds to BTA_GATT_AUTH_REQ_SIGNED_NO_MITM
    SignedNoMitm = esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_SIGNED_NO_MITM,
    /// Signed data with MITM protection. Corresponds to BTA_GATT_AUTH_REQ_SIGNED_MITM
    SignedMitm = esp_gatt_auth_req_t_ESP_GATT_AUTH_REQ_SIGNED_MITM,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum GattWriteType {
    /// Write operation where no response is needed
    NoResponse = esp_gatt_write_type_t_ESP_GATT_WRITE_TYPE_NO_RSP,
    /// Write operation that requires a remote response
    RequireResponse = esp_gatt_write_type_t_ESP_GATT_WRITE_TYPE_RSP,
}

#[derive(Debug, Default, Copy, Clone)]
pub struct BleConnParams {
    /// Initial scan interval, in units of 0.625ms, the range is 0x0004(2.5ms) to 0xFFFF(10.24s)
    pub scan_interval: u16,
    /// Initial scan window, in units of 0.625ms, the range is 0x0004(2.5ms) to 0xFFFF(10.24s)
    pub scan_window: u16,
    /// Minimum connection interval, in units of 1.25ms, the range is 0x0006(7.5ms) to 0x0C80(4s)
    pub interval_min: u16,
    /// Maximum connection interval, in units of 1.25ms, the range is 0x0006(7.5ms) to 0x0C80(4s)
    pub interval_max: u16,
    /// Connection latency, the range is 0x0000(0) to 0x01F3(499)
    pub latency: u16,
    /// Connection supervision timeout, in units of 10ms, the range is from 0x000A(100ms) to 0x0C80(32s)
    pub supervision_timeout: u16,
    /// Minimum connection event length, in units of 0.625ms, setting to 0 for no preferred parameters
    pub min_ce_len: u16,
    /// Maximum connection event length, in units of 0.625ms, setting to 0 for no preferred parameters
    pub max_ce_len: u16,
}

impl From<BleConnParams> for esp_ble_conn_params_t {
    fn from(params: BleConnParams) -> Self {
        Self {
            scan_interval: params.scan_interval,
            scan_window: params.scan_window,
            interval_min: params.interval_min,
            interval_max: params.interval_max,
            latency: params.latency,
            supervision_timeout: params.supervision_timeout,
            min_ce_len: params.min_ce_len,
            max_ce_len: params.max_ce_len,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct GattCreateConnParams {
    /// The Bluetooth address of the remote device
    pub addr: BdAddr,
    /// Address type of the remote device
    pub addr_type: BleAddrType,
    /// Direct connection or background auto connection(by now, background auto connection is not supported)
    pub is_direct: bool,
    /// Set to true for BLE 5.0 or higher to enable auxiliary connections; set to false for BLE 4.2 or lower.
    pub is_aux: bool,
    /// Specifies the address type used in the connection request. Set to 0xFF if the address type is unknown.
    pub own_addr_type: BleAddrType,
    /// Connection parameters for the LE 1M PHY
    pub phy_1m_conn_params: Option<BleConnParams>,
    /// Connection parameters for the LE 2M PHY
    pub phy_2m_conn_params: Option<BleConnParams>,
    /// Connection parameters for the LE Coded PHY
    pub phy_coded_conn_params: Option<BleConnParams>,
}

impl GattCreateConnParams {
    pub const fn new(addr: BdAddr, addr_type: BleAddrType) -> Self {
        Self {
            addr,
            addr_type,
            is_direct: true,
            is_aux: false,
            own_addr_type: BleAddrType::Public,
            phy_1m_conn_params: None,
            phy_2m_conn_params: None,
            phy_coded_conn_params: None,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(transparent)]
pub struct ServiceElement(esp_gattc_service_elem_t);

impl ServiceElement {
    pub const fn new() -> Self {
        Self(esp_gattc_service_elem_t {
            uuid: BtUuid::uuid16(0).raw(),
            is_primary: false,
            start_handle: 0,
            end_handle: 0,
        })
    }

    pub fn uuid(&self) -> BtUuid {
        self.0.uuid.into()
    }

    pub fn is_primary(&self) -> bool {
        self.0.is_primary
    }

    pub fn start_handle(&self) -> Handle {
        self.0.start_handle
    }
    pub fn end_handle(&self) -> Handle {
        self.0.end_handle
    }
}

impl Default for ServiceElement {
    fn default() -> Self {
        Self::new()
    }
}

impl Debug for ServiceElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ServiceElement")
            .field("uuid", &self.uuid())
            .field("is_primary", &self.0.is_primary)
            .field("start_handle", &self.0.start_handle)
            .field("end_handle", &self.0.end_handle)
            .finish()
    }
}

#[derive(Copy, Clone)]
#[repr(transparent)]
pub struct IncludeServiceElement(esp_gattc_incl_svc_elem_t);

impl IncludeServiceElement {
    pub const fn new() -> Self {
        Self(esp_gattc_incl_svc_elem_t {
            uuid: BtUuid::uuid16(0).raw(),
            handle: 0,
            incl_srvc_s_handle: 0,
            incl_srvc_e_handle: 0,
        })
    }

    pub fn uuid(&self) -> BtUuid {
        self.0.uuid.into()
    }

    pub fn handle(&self) -> Handle {
        self.0.handle
    }
    pub fn start_handle(&self) -> Handle {
        self.0.incl_srvc_s_handle
    }
    pub fn end_handle(&self) -> Handle {
        self.0.incl_srvc_e_handle
    }
}

impl Default for IncludeServiceElement {
    fn default() -> Self {
        Self::new()
    }
}

impl Debug for IncludeServiceElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IncludeService")
            .field("uuid", &self.uuid())
            .field("handle", &self.handle())
            .field("start_handle", &self.start_handle())
            .field("end_handle", &self.end_handle())
            .finish()
    }
}

#[derive(Copy, Clone)]
#[repr(transparent)]
pub struct CharacteristicElement(esp_gattc_char_elem_t);

impl CharacteristicElement {
    pub const fn new() -> Self {
        Self(esp_gattc_char_elem_t {
            uuid: BtUuid::uuid16(0).raw(),
            char_handle: 0,
            properties: 0,
        })
    }

    pub fn uuid(&self) -> BtUuid {
        self.0.uuid.into()
    }

    pub fn handle(&self) -> Handle {
        self.0.char_handle
    }

    pub fn properties(&self) -> EnumSet<Property> {
        EnumSet::from_repr(self.0.properties)
    }
}

impl Default for CharacteristicElement {
    fn default() -> Self {
        Self::new()
    }
}
impl Debug for CharacteristicElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CharacteristicElement")
            .field("uuid", &self.uuid())
            .field("handle", &self.handle())
            .field("properties", &self.properties())
            .finish()
    }
}

#[derive(Copy, Clone)]
#[repr(transparent)]
pub struct DescriptorElement(esp_gattc_descr_elem_t);

impl DescriptorElement {
    pub const fn new() -> Self {
        Self(esp_gattc_descr_elem_t {
            uuid: BtUuid::uuid16(0).raw(),
            handle: 0,
        })
    }

    pub fn uuid(&self) -> BtUuid {
        self.0.uuid.into()
    }

    pub fn handle(&self) -> Handle {
        self.0.handle
    }
}

impl Default for DescriptorElement {
    fn default() -> Self {
        Self::new()
    }
}
impl Debug for DescriptorElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DescriptorElement")
            .field("uuid", &self.uuid())
            .field("handle", &self.handle())
            .finish()
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum DbElementAttrType {
    /// Primary service attribute, with start and end handle
    PrimaryService {
        start_handle: Handle,
        end_handle: Handle,
        attribute_handle: Handle, // same as start_handle
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_PRIMARY_SERVICE,
    /// Secondary service attribute
    SecondaryService {
        start_handle: Handle,
        end_handle: Handle,
        attribute_handle: Handle, // same as start_handle
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_SECONDARY_SERVICE,
    /// Characteristic attribute
    Characteristic {
        handle: Handle,
        properties: EnumSet<Property>,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_CHARACTERISTIC,
    /// Descriptor attribute - with the characteristic handle
    Descriptor { handle: Handle } = esp_gatt_db_attr_type_t_ESP_GATT_DB_DESCRIPTOR,
    /// Included service attribute
    IncludedService { attribute_handle: Handle } =
        esp_gatt_db_attr_type_t_ESP_GATT_DB_INCLUDED_SERVICE,
    AllAttributes {
        start_handle: Handle,
        end_handle: Handle,
        attribute_handle: Handle,
        properties: EnumSet<Property>,
    } = esp_gatt_db_attr_type_t_ESP_GATT_DB_ALL,
}

#[derive(Copy, Clone)]
#[repr(transparent)]
pub struct DbElement(esp_gattc_db_elem_t);

impl DbElement {
    pub const fn new() -> Self {
        Self(esp_gattc_db_elem_t {
            uuid: BtUuid::uuid16(0).raw(),
            type_: 0,
            attribute_handle: 0,
            start_handle: 0,
            end_handle: 0,
            properties: 0,
        })
    }

    pub fn uuid(&self) -> BtUuid {
        self.0.uuid.into()
    }

    pub fn attribute_type(&self) -> DbElementAttrType {
        let elem = self.0;

        #[allow(non_upper_case_globals)]
        match elem.type_ {
            esp_gatt_db_attr_type_t_ESP_GATT_DB_PRIMARY_SERVICE => {
                DbElementAttrType::PrimaryService {
                    start_handle: elem.start_handle,
                    end_handle: elem.end_handle,
                    attribute_handle: elem.attribute_handle,
                }
            }

            esp_gatt_db_attr_type_t_ESP_GATT_DB_SECONDARY_SERVICE => {
                DbElementAttrType::SecondaryService {
                    start_handle: elem.start_handle,
                    end_handle: elem.end_handle,
                    attribute_handle: elem.attribute_handle,
                }
            }

            esp_gatt_db_attr_type_t_ESP_GATT_DB_CHARACTERISTIC => {
                DbElementAttrType::Characteristic {
                    handle: elem.attribute_handle,
                    properties: EnumSet::from_repr(elem.properties),
                }
            }

            esp_gatt_db_attr_type_t_ESP_GATT_DB_DESCRIPTOR => DbElementAttrType::Descriptor {
                handle: elem.attribute_handle,
            },

            esp_gatt_db_attr_type_t_ESP_GATT_DB_INCLUDED_SERVICE => {
                DbElementAttrType::IncludedService {
                    attribute_handle: elem.attribute_handle,
                }
            }

            _ => DbElementAttrType::AllAttributes {
                start_handle: elem.start_handle,
                end_handle: elem.end_handle,
                attribute_handle: elem.attribute_handle,
                properties: EnumSet::from_repr(elem.properties),
            },
        }
    }
}

impl Default for DbElement {
    fn default() -> Self {
        Self::new()
    }
}

impl Debug for DbElement {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DbElement")
            .field("uuid", &self.uuid())
            .field("attribute_type", &self.attribute_type())
            .finish()
    }
}

pub struct EventRawData<'a>(pub &'a esp_ble_gattc_cb_param_t);

impl Debug for EventRawData<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("EventRawData").finish()
    }
}

#[derive(Debug)]
pub enum GattcEvent<'a> {
    ClientRegistered {
        /// Operation status
        status: GattStatus,
        /// Application id which input in register API
        app_id: AppId,
    },
    ClientUnregistered,
    Open {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Remote bluetooth device address
        addr: BdAddr,
        /// MTU size
        mtu: u16,
    },
    ReadCharacteristic {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Characteristic handle
        handle: Handle,
        /// Characteristic value
        value: Option<&'a [u8]>,
    },
    ReadDescriptor {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Descriptor handle
        handle: Handle,
        /// Descriptor value
        value: Option<&'a [u8]>,
    },
    ReadMultipleChar {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Characteristic handle
        handle: Handle,
        /// Concatenated values of all characteristics
        value: Option<&'a [u8]>,
    },
    ReadMultipleVarChar {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Characteristic handle
        handle: Handle,
        /// Concatenated values of all characteristics
        value: Option<&'a [u8]>,
    },
    WriteCharacteristic {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Characteristic handle
        handle: Handle,
    },
    WriteDescriptor {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Desrciptor handle
        handle: Handle,
    },
    PrepareWrite {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Characteristic or desrciptor handle
        handle: Handle,
        /// The position offset to write
        offset: u16,
    },
    ExecWrite {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
    },
    Close {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// Remote bluetooth device address
        addr: BdAddr,
        /// Indicate the reason of close
        reason: GattConnReason,
    },
    SearchComplete {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// The source of the service information
        searched_service_source: ServiceSource,
    },
    SearchResult {
        /// Connection ID
        conn_id: ConnectionId,
        /// Service start handle
        start_handle: Handle,
        /// Service end handle
        end_handle: Handle,
        /// Service ID, including service UUID and other information
        srvc_id: GattId,
        /// True indicates a primary service, false otherwise
        is_primary: bool,
    },
    Notify {
        /// Connection ID
        conn_id: ConnectionId,
        /// Remote Bluetooth device address
        addr: BdAddr,
        /// The characteristic or descriptor handle
        handle: Handle,
        /// Notify attribute value
        value: &'a [u8],
        /// True means notification; false means indication
        is_notify: bool,
    },
    RegisterNotify {
        /// Operation status
        status: GattStatus,
        /// The characteristic or descriptor handle
        handle: Handle,
    },
    UnregisterNotify {
        /// Operation status
        status: GattStatus,
        /// The characteristic or descriptor handle
        handle: Handle,
    },
    ServiceChanged {
        /// Remote Bluetooth device address
        addr: BdAddr,
    },
    Mtu {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// MTU size
        mtu: u16,
    },
    Congest {
        /// Connection id
        conn_id: ConnectionId,
        /// Congested or not
        congested: bool,
    },
    Connected {
        /// Connection ID
        conn_id: ConnectionId,
        /// Link role
        link_role: LinkRole,
        /// Remote device address
        addr: BdAddr,
        /// Remote device address type
        addr_type: BleAddrType,
        /// Current connection parameters
        conn_params: GattConnParams,
        /// HCI connection handle
        conn_handle: Handle,
    },
    Disconnected {
        /// Connection ID
        conn_id: ConnectionId,
        /// Remote device address
        addr: BdAddr,
        /// Disconnection reason
        reason: GattConnReason,
    },
    QueueFull {
        /// Operation status
        status: GattStatus,
        /// Connection ID
        conn_id: ConnectionId,
        /// True indicates the GATTC command queue is full; false otherwise
        is_full: bool,
    },
    SetAssociation {
        /// Operation status
        status: GattStatus,
    },
    AddressList {
        /// Operation status
        status: GattStatus,
        /// Address list which has been retrieved from the local GATTC cache
        address_list: &'a [BdAddr],
    },
    DiscoveryCompleted {
        /// Operation status
        status: GattStatus,
        /// Connection ID
        conn_id: ConnectionId,
    },
    Other {
        raw_event: esp_gattc_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_gattc_cb_event_t, &'a esp_ble_gattc_cb_param_t)> for GattcEvent<'a> {
    fn from(value: (esp_gattc_cb_event_t, &'a esp_ble_gattc_cb_param_t)) -> Self {
        let (event, param) = value;

        match event {
            esp_gattc_cb_event_t_ESP_GATTC_REG_EVT => unsafe {
                Self::ClientRegistered {
                    status: param.reg.status.try_into().unwrap(),
                    app_id: param.reg.app_id,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_UNREG_EVT => Self::ClientUnregistered,
            esp_gattc_cb_event_t_ESP_GATTC_OPEN_EVT => unsafe {
                Self::Open {
                    status: param.open.status.try_into().unwrap(),
                    conn_id: param.open.conn_id,
                    addr: param.open.remote_bda.into(),
                    mtu: param.open.mtu,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_READ_CHAR_EVT => unsafe {
                Self::ReadCharacteristic {
                    status: param.read.status.try_into().unwrap(),
                    conn_id: param.read.conn_id,
                    handle: param.read.handle,
                    value: if param.read.value_len > 0 {
                        Some(core::slice::from_raw_parts(
                            param.read.value,
                            param.read.value_len as _,
                        ))
                    } else {
                        None
                    },
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_WRITE_CHAR_EVT => unsafe {
                Self::WriteCharacteristic {
                    status: param.write.status.try_into().unwrap(),
                    conn_id: param.write.conn_id,
                    handle: param.write.handle,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_CLOSE_EVT => unsafe {
                Self::Close {
                    status: param.close.status.try_into().unwrap(),
                    conn_id: param.close.conn_id,
                    addr: param.close.remote_bda.into(),
                    reason: param.close.reason.try_into().unwrap(),
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_SEARCH_CMPL_EVT => unsafe {
                Self::SearchComplete {
                    status: param.search_cmpl.status.try_into().unwrap(),
                    conn_id: param.search_cmpl.conn_id,
                    searched_service_source: param
                        .search_cmpl
                        .searched_service_source
                        .try_into()
                        .unwrap(),
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_SEARCH_RES_EVT => unsafe {
                Self::SearchResult {
                    conn_id: param.search_res.conn_id,
                    start_handle: param.search_res.start_handle,
                    end_handle: param.search_res.end_handle,
                    srvc_id: param.search_res.srvc_id.into(),
                    is_primary: param.search_res.is_primary,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_READ_DESCR_EVT => unsafe {
                Self::ReadDescriptor {
                    status: param.read.status.try_into().unwrap(),
                    conn_id: param.read.conn_id,
                    handle: param.read.handle,
                    value: if param.read.value_len > 0 {
                        Some(core::slice::from_raw_parts(
                            param.read.value,
                            param.read.value_len as _,
                        ))
                    } else {
                        None
                    },
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_WRITE_DESCR_EVT => unsafe {
                Self::WriteDescriptor {
                    status: param.write.status.try_into().unwrap(),
                    conn_id: param.write.conn_id,
                    handle: param.write.handle,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_NOTIFY_EVT => unsafe {
                Self::Notify {
                    conn_id: param.notify.conn_id,
                    addr: param.notify.remote_bda.into(),
                    handle: param.notify.handle,
                    value: core::slice::from_raw_parts(
                        param.notify.value,
                        param.notify.value_len as _,
                    ),
                    is_notify: param.notify.is_notify,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_PREP_WRITE_EVT => unsafe {
                Self::PrepareWrite {
                    status: param.write.status.try_into().unwrap(),
                    conn_id: param.write.conn_id,
                    handle: param.write.handle,
                    offset: param.write.offset,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_EXEC_EVT => unsafe {
                Self::ExecWrite {
                    status: param.exec_cmpl.status.try_into().unwrap(),
                    conn_id: param.exec_cmpl.conn_id,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_SRVC_CHG_EVT => unsafe {
                Self::ServiceChanged {
                    addr: param.srvc_chg.remote_bda.into(),
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_CFG_MTU_EVT => unsafe {
                Self::Mtu {
                    status: param.cfg_mtu.status.try_into().unwrap(),
                    conn_id: param.cfg_mtu.conn_id,
                    mtu: param.cfg_mtu.mtu,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_CONGEST_EVT => unsafe {
                Self::Congest {
                    conn_id: param.congest.conn_id,
                    congested: param.congest.congested,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_REG_FOR_NOTIFY_EVT => unsafe {
                Self::RegisterNotify {
                    status: param.reg_for_notify.status.try_into().unwrap(),
                    handle: param.reg_for_notify.handle,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_UNREG_FOR_NOTIFY_EVT => unsafe {
                Self::UnregisterNotify {
                    status: param.unreg_for_notify.status.try_into().unwrap(),
                    handle: param.unreg_for_notify.handle,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_CONNECT_EVT => unsafe {
                Self::Connected {
                    conn_id: param.connect.conn_id,
                    link_role: param.connect.link_role.try_into().unwrap(),
                    addr: param.connect.remote_bda.into(),
                    addr_type: param.connect.ble_addr_type.try_into().unwrap(),
                    conn_handle: param.connect.conn_handle,
                    conn_params: GattConnParams {
                        interval_ms: param.connect.conn_params.interval as u32 * 125 / 100,
                        latency_ms: param.connect.conn_params.latency as u32 * 125 / 100,
                        timeout_ms: param.connect.conn_params.timeout as u32 * 10,
                    },
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_DISCONNECT_EVT => unsafe {
                Self::Disconnected {
                    conn_id: param.disconnect.conn_id,
                    addr: param.disconnect.remote_bda.into(),
                    reason: param.disconnect.reason.try_into().unwrap(),
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_READ_MULTIPLE_EVT => unsafe {
                Self::ReadMultipleChar {
                    status: param.read.status.try_into().unwrap(),
                    conn_id: param.read.conn_id,
                    handle: param.read.handle,
                    value: if param.read.value_len > 0 {
                        Some(core::slice::from_raw_parts(
                            param.read.value,
                            param.read.value_len as _,
                        ))
                    } else {
                        None
                    },
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_QUEUE_FULL_EVT => unsafe {
                Self::QueueFull {
                    status: param.queue_full.status.try_into().unwrap(),
                    conn_id: param.queue_full.conn_id,
                    is_full: param.queue_full.is_full,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_SET_ASSOC_EVT => unsafe {
                Self::SetAssociation {
                    status: param.set_assoc_cmp.status.try_into().unwrap(),
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_GET_ADDR_LIST_EVT => unsafe {
                let addr: *mut BdAddr = param.get_addr_list.addr_list as _;
                let addr_list =
                    core::slice::from_raw_parts(addr, param.get_addr_list.num_addr as _);

                Self::AddressList {
                    status: param.get_addr_list.status.try_into().unwrap(),
                    address_list: addr_list,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_DIS_SRVC_CMPL_EVT => unsafe {
                Self::DiscoveryCompleted {
                    status: param.dis_srvc_cmpl.status.try_into().unwrap(),
                    conn_id: param.dis_srvc_cmpl.conn_id,
                }
            },
            esp_gattc_cb_event_t_ESP_GATTC_READ_MULTI_VAR_EVT => unsafe {
                Self::ReadMultipleVarChar {
                    status: param.read.status.try_into().unwrap(),
                    conn_id: param.read.conn_id,
                    handle: param.read.handle,
                    value: if param.read.value_len > 0 {
                        Some(core::slice::from_raw_parts(
                            param.read.value,
                            param.read.value_len as _,
                        ))
                    } else {
                        None
                    },
                }
            },
            _ => Self::Other {
                raw_event: event,
                raw_data: EventRawData(param),
            },
        }
    }
}

pub struct EspGattc<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspGattc<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    pub fn new(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_ble_gattc_register_callback(Some(Self::event_handler)) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut((GattInterface, GattcEvent)) + Send + 'static,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    /// # Safety
    ///
    /// This method - in contrast to method `subscribe` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    ///   scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent to a hidden ESP IDF thread.
    /// This means that if the service is forgotten, Rust is free to e.g. unwind the stack
    /// and the closure now owned by this other thread will end up with references to variables that no longer exist.
    ///
    /// The destructor of the service takes care - prior to the service being dropped and e.g.
    /// the stack being unwind - to remove the closure from the hidden thread and destroy it.
    /// Unfortunately, when the service is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    pub unsafe fn subscribe_nonstatic<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut((GattInterface, GattcEvent)) + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    /// Register a GATT Client application.
    /// * `app_id` The ID for different application (max id: `0x7fff`)
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ClientRegistered`]
    /// 2. The maximum number of applications is limited to 4
    pub fn register_app(&self, app_id: AppId) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_app_register(app_id) })
    }

    /// Unregister a GATT Client application.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ClientUnregistered`]
    pub fn unregister_app(&self, gattc_if: GattInterface) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_app_unregister(gattc_if) })
    }

    /// Create an ACL connection.
    ///
    /// Note: *Do not* enable `BT_BLE_42_FEATURES_SUPPORTED` and `BT_BLE_50_FEATURES_SUPPORTED` in the menuconfig simultaneously.
    ///
    /// # Note
    /// 1. The function always triggers [`GattcEvent::Connected`] and [`GattcEvent::Open`]
    /// 2. When the device acts as GATT server, besides the above two events, this function triggers [`GattsEvent::PeerConnected`](super::server::GattsEvent::PeerConnected) as well.\
    /// 3. This function will establish an ACL connection as a Central and a virtual connection as a GATT Client
    /// 4. If the ACL connection already exists, it will create a virtual connection only
    pub fn enh_open(
        &self,
        gattc_if: GattInterface,
        conn_params: &GattCreateConnParams,
    ) -> Result<(), EspError> {
        let mut phy_mask = 0;

        let phy_1m_conn_params: Option<esp_ble_conn_params_t> =
            if let Some(phy_1m_conn_params) = conn_params.phy_1m_conn_params {
                phy_mask |= ESP_BLE_PHY_1M_PREF_MASK;
                Some(phy_1m_conn_params.into())
            } else {
                None
            };

        let phy_2m_conn_params: Option<esp_ble_conn_params_t> =
            if let Some(phy_2m_conn_params) = conn_params.phy_2m_conn_params {
                phy_mask |= ESP_BLE_PHY_2M_PREF_MASK;
                Some(phy_2m_conn_params.into())
            } else {
                None
            };

        let phy_coded_conn_params: Option<esp_ble_conn_params_t> =
            if let Some(phy_coded_conn_params) = conn_params.phy_coded_conn_params {
                phy_mask |= ESP_BLE_PHY_CODED_PREF_MASK;
                Some(phy_coded_conn_params.into())
            } else {
                None
            };

        let conn_params = esp_ble_gatt_creat_conn_params_t {
            remote_bda: conn_params.addr.into(),
            remote_addr_type: conn_params.addr_type as _,
            is_direct: conn_params.is_direct,
            is_aux: conn_params.is_aux,
            own_addr_type: conn_params.own_addr_type as _,
            phy_mask: phy_mask as _,
            phy_1m_conn_params: phy_1m_conn_params.as_ref().map_or(core::ptr::null(), |p| p),
            phy_2m_conn_params: phy_2m_conn_params.as_ref().map_or(core::ptr::null(), |p| p),
            phy_coded_conn_params: phy_coded_conn_params
                .as_ref()
                .map_or(core::ptr::null(), |p| p),
        };

        esp!(unsafe { esp_ble_gattc_enh_open(gattc_if, &conn_params as *const _ as *mut _) })
    }

    pub fn open(
        &self,
        gattc_if: GattInterface,
        addr: BdAddr,
        addr_type: BleAddrType,
        is_direct: bool,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_open(
                gattc_if,
                &addr.raw() as *const _ as *mut _,
                addr_type as _,
                is_direct,
            )
        })
    }

    pub fn aux_open(
        &self,
        gattc_if: GattInterface,
        addr: BdAddr,
        addr_type: BleAddrType,
        is_direct: bool,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_aux_open(
                gattc_if,
                &addr.raw() as *const _ as *mut _,
                addr_type as _,
                is_direct,
            )
        })
    }

    /// Close the virtual GATT Client connection.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::Close`]
    /// 2. There may be multiple virtual GATT server connections when multiple `app_id` got registered
    /// 3. This API closes one virtual GATT server connection only, if there exist other virtual GATT server connections. It does not close the physical connection.
    /// 4. The API [`EspBleGap::disconnect()`](crate::bt::ble::gap::EspBleGap::disconnect) can be used to disconnect the physical connection directly.
    /// 5. If there is only one virtual GATT connection left, this API will terminate the ACL connection in addition and triggers [`GattcEvent::Disconnect`]. Then there is no need to call `disconnect()` anymore.
    pub fn close(&self, gattc_if: GattInterface, conn_id: ConnectionId) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_close(gattc_if, conn_id) })
    }

    /// Configure the MTU size in the GATT channel.
    ///
    /// #Note
    /// 1. This function triggers [`GattcEvent::Mtu`]
    /// 2. You should call [`gatt::set_local_mtu()`](crate::bt::ble::gatt::set_local_mtu()) to set the desired MTU size locally before this API. If not set, the GATT channel uses the default MTU size (23 bytes)
    pub fn mtu_req(&self, gattc_if: GattInterface, conn_id: ConnectionId) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_send_mtu_req(gattc_if, conn_id) })
    }

    /// Search services from the local GATTC cache.
    /// * `filter_uuid` A UUID of the intended service. If None is passed, this API will return all services.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::SearchResult`] each time a service is retrieved
    /// 2. This function triggers [`GattcEvent::SearchComplete`] when the search is completed
    /// 3. The 128-bit base UUID will be converted to a 16-bit UUID automatically in the search results. Other types of UUID remain unchanged
    pub fn search_service(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        filter_uuid: Option<BtUuid>,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_search_service(
                gattc_if,
                conn_id,
                filter_uuid.map_or(core::ptr::null_mut(), |f| &f.raw() as *const _ as *mut _),
            )
        })
    }

    /// Get the service with the given service UUID in the local GATTC cache.
    /// * `svc_uuid` The service UUID. If `None` is passed, the API will retrieve all services.
    /// * `offset` The position offset to retrieve
    /// * `results` That will be updated with the services found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. [`cache_refresh()`] can be used to discover services again
    pub fn get_service(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        svc_uuid: Option<BtUuid>,
        offset: u16,
        results: &mut [ServiceElement],
    ) -> Result<usize, EspError> {
        let mut count: u16 = if svc_uuid.is_some() {
            1
        } else {
            results.len() as _
        };

        esp!(unsafe {
            esp_ble_gattc_get_service(
                gattc_if,
                conn_id,
                svc_uuid.map_or(core::ptr::null_mut(), |s| &s.raw() as *const _ as *mut _),
                results as *const _ as *mut _,
                &mut count,
                offset,
            )
        })?;

        Ok(if offset > count {
            0
        } else {
            (count - offset).min(results.len() as u16) as _
        })
    }

    /// Get all characteristics with the given handle range in the local GATTC cache.
    /// * `start_handle` The attribute start handle
    /// * `end_handle` The attribute end handle
    /// * `offset` The position offset to retrieve
    /// * `results` That will be updated with the characteristics found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle`
    pub fn get_all_characteristics(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        offset: u16,
        results: &mut [CharacteristicElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_all_char(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                results as *const _ as *mut _,
                &mut count,
                offset,
            )
        })?;

        Ok(if offset > count {
            0
        } else {
            (count - offset).min(results.len() as u16) as _
        })
    }

    /// Get all descriptors with the given characteristic in the local GATTC cache.
    /// * `char_handle` The given characteristic handle
    /// * `offset` The position offset to retrieve
    /// * `results` That will be updated with the descriptors found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `char_handle` must be greater than 0
    pub fn get_all_descriptors(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        char_handle: Handle,
        offset: u16,
        results: &mut [DescriptorElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_all_descr(
                gattc_if,
                conn_id,
                char_handle,
                results as *const _ as *mut _,
                &mut count,
                offset,
            )
        })?;

        Ok(if offset > count {
            0
        } else {
            (count - offset).min(results.len() as u16) as _
        })
    }

    /// Get the characteristic with the given characteristic UUID in the local GATTC cache.
    /// * `start_handle` The attribute start handle
    /// * `end_handle` The attribute end handle
    /// * `char_uuid` The characteristic UUID
    /// * `results` That will be updated with the characteristics found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle`
    pub fn get_characteristic_by_uuid(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        char_uuid: BtUuid,
        results: &mut [CharacteristicElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_char_by_uuid(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                char_uuid.raw(),
                results as *const _ as *mut _,
                &mut count,
            )
        })?;

        Ok(count.min(results.len() as u16) as _)
    }

    /// Get the descriptor with the given characteristic UUID in the local GATTC cache.
    /// * `start_handle` The attribute start handle
    /// * `end_handle` The attribute end handle
    /// * `char_uuid` The characteristic UUID
    /// * `descr_uuid` The descriptor UUID
    /// * `results` That will be updated with the descriptors found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle`
    #[allow(clippy::too_many_arguments)]
    pub fn get_descriptor_by_uuid(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        char_uuid: BtUuid,
        descr_uuid: BtUuid,
        results: &mut [DescriptorElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_descr_by_uuid(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                char_uuid.raw(),
                descr_uuid.raw(),
                results as *const _ as *mut _,
                &mut count,
            )
        })?;

        Ok(count.min(results.len() as u16) as _)
    }

    /// Get the descriptor with the given characteristic handle in the local GATTC cache.
    /// * `char_handle` The characteristic handle
    /// * `descr_uuid` The descriptor UUID
    /// * `results` That will be updated with the descriptors found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `char_handle` must be greater than 0
    pub fn get_descriptor_by_char_handle(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        char_handle: Handle,
        descr_uuid: BtUuid,
        results: &mut [DescriptorElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_descr_by_char_handle(
                gattc_if,
                conn_id,
                char_handle,
                descr_uuid.raw(),
                results as *const _ as *mut _,
                &mut count,
            )
        })?;

        Ok(count.min(results.len() as u16) as _)
    }

    /// Get the included services with the given service handle in the local GATTC cache.
    /// * `incl_uuid` The included service UUID
    /// * `results` That will be updated with the include services found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle`
    pub fn get_include_service(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        incl_uuid: BtUuid,
        results: &mut [IncludeServiceElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_include_service(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                &incl_uuid.raw() as *const _ as *mut _,
                results as *const _ as *mut _,
                &mut count,
            )
        })?;

        Ok(count.min(results.len() as u16) as _)
    }

    /// Get the attribute count with the given service or characteristic in the local GATTC cache.
    /// * `type` The attribute type
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle` if the `type` is not [`DbAttrType::Descriptor`]
    pub fn get_attr_count(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        attr_type: DbAttrType,
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = 0;

        let (start_handle, end_handle, char_handle) = match attr_type {
            DbAttrType::PrimaryService {
                start_handle,
                end_handle,
            } => (start_handle, end_handle, INVALID_HANDLE),
            DbAttrType::SecondaryService {
                start_handle,
                end_handle,
            } => (start_handle, end_handle, INVALID_HANDLE),
            DbAttrType::Characteristic {
                start_handle,
                end_handle,
            } => (start_handle, end_handle, INVALID_HANDLE),
            DbAttrType::Descriptor { handle } => (INVALID_HANDLE, INVALID_HANDLE, handle),
            DbAttrType::IncludedService {
                start_handle,
                end_handle,
            } => (start_handle, end_handle, INVALID_HANDLE),
            DbAttrType::AllAttributes {
                start_handle,
                end_handle,
            } => (start_handle, end_handle, INVALID_HANDLE),
        };

        check_gatt_status(unsafe {
            esp_ble_gattc_get_attr_count(
                gattc_if,
                conn_id,
                *(&attr_type as *const _ as *const _),
                start_handle,
                end_handle,
                char_handle,
                &mut count,
            )
        })?;

        Ok(count as _)
    }

    /// Get the GATT database elements.
    /// * `results` That will be updated with the db elements found in the local GATTC cache
    ///
    /// # Returns
    /// The number of elements in `results`, which could be 0; this is not the actual number of elements
    ///
    /// # Note
    /// 1. This API does not trigger any event
    /// 2. `start_handle` must be greater than 0, and smaller than `end_handle`
    pub fn get_db(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        results: &mut [DbElement],
    ) -> Result<usize, GattStatus> {
        let mut count: u16 = results.len() as _;

        check_gatt_status(unsafe {
            esp_ble_gattc_get_db(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                results as *const _ as *mut _,
                &mut count,
            )
        })?;

        Ok(count.min(results.len() as u16) as _)
    }

    /// Read the characteristics value of the given characteristic handle.
    /// * `handle` Characteristic handle to read
    /// * `auth_req` Authenticate request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ReadCharacteristic`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `handle` must be greater than 0
    pub fn read_characteristic(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_read_char(gattc_if, conn_id, handle, auth_req as _) })
    }

    /// Read the characteristics value of the given characteristic UUID.
    /// * `start_handle` The attribute start handle
    /// * `end_handle` The attribute end handle
    /// * `uuid` The pointer to UUID of attribute to read
    /// * `auth_req` Authenticate request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ReadCharacteristic`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `start_handle` must be greater than 0, and smaller than `end_handle`
    pub fn read_by_type(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        start_handle: Handle,
        end_handle: Handle,
        uuid: BtUuid,
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_read_by_type(
                gattc_if,
                conn_id,
                start_handle,
                end_handle,
                &uuid.raw() as *const _ as *mut _,
                auth_req as _,
            )
        })
    }

    /// Read multiple characteristic or descriptor values.
    /// * `read_multi` Handles to read, max 10 handles
    /// * `auth_req` Authenticate request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ReadMultipleChar`]
    /// 2. This function should be called only after the connection has been established
    pub fn read_multiple(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        read_multi: &[Handle],
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        if read_multi.is_empty() || read_multi.len() > ESP_GATT_MAX_READ_MULTI_HANDLES as _ {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;
        }

        let mut handles: [u16; ESP_GATT_MAX_READ_MULTI_HANDLES as _] =
            [0; ESP_GATT_MAX_READ_MULTI_HANDLES as _];

        handles[..read_multi.len()].copy_from_slice(read_multi);

        let read_multi = esp_gattc_multi_t {
            num_attr: read_multi.len() as _,
            handles,
        };

        esp!(unsafe {
            esp_ble_gattc_read_multiple(
                gattc_if,
                conn_id,
                &read_multi as *const esp_gattc_multi_t as *mut _,
                auth_req as _,
            )
        })
    }

    /// Read multiple variable length characteristic values.
    /// * `read_multi` Handles to read, max 10 handles
    /// * `auth_req` Authenticate request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ReadMultipleVarChar`]
    /// 2. This function should be called only after the connection has been established
    pub fn read_multiple_variable(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        read_multi: &[Handle],
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        if read_multi.is_empty() || read_multi.len() > ESP_GATT_MAX_READ_MULTI_HANDLES as _ {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;
        }

        let mut handles: [u16; ESP_GATT_MAX_READ_MULTI_HANDLES as _] =
            [0; ESP_GATT_MAX_READ_MULTI_HANDLES as _];

        handles[..read_multi.len()].copy_from_slice(read_multi);

        let read_multi = esp_gattc_multi_t {
            num_attr: read_multi.len() as _,
            handles,
        };

        esp!(unsafe {
            esp_ble_gattc_read_multiple_variable(
                gattc_if,
                conn_id,
                &read_multi as *const esp_gattc_multi_t as *mut _,
                auth_req as _,
            )
        })
    }

    /// Read a characteristics descriptor.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ReadDescriptor`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `handle` must be greater than 0
    pub fn read_descriptor(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_read_char_descr(gattc_if, conn_id, handle, auth_req as _) })
    }

    /// Write the characteristic value of a given characteristic handle.
    /// * `handle` The characteristic handle to write
    /// * `value` The value to write
    /// * `write_type` The type of Attribute write operation
    /// * `auth_req` Authentication request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::WriteCharacteristic`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `handle` must be greater than 0
    pub fn write_characteristic(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        value: &[u8],
        write_type: GattWriteType,
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_write_char(
                gattc_if,
                conn_id,
                handle,
                value.len() as _,
                if value.is_empty() {
                    core::ptr::null_mut()
                } else {
                    value.as_ptr() as *const _ as *mut _
                },
                write_type as _,
                auth_req as _,
            )
        })
    }

    /// Write Characteristic descriptor value of a given descriptor handle.
    /// * `handle` The descriptor handle to write
    /// * `value` The value to write
    /// * `write_type` The type of Attribute write operation
    /// * `auth_req` Authentication request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::WriteDescriptor`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `handle` must be greater than 0
    pub fn write_descriptor(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        value: &[u8],
        write_type: GattWriteType,
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_write_char_descr(
                gattc_if,
                conn_id,
                handle,
                value.len() as _,
                if value.is_empty() {
                    core::ptr::null_mut()
                } else {
                    value.as_ptr() as *const _ as *mut _
                },
                write_type as _,
                auth_req as _,
            )
        })
    }

    /// Prepare to write a characteristic value which is longer than the MTU size to a specified characteristic handle.
    /// * `handle` The characteristic handle to write
    /// * `offset` The position offset to write
    /// * `value` The value to write
    /// * `auth_req` Authentication request type
    ///
    /// # Note
    /// 1. This function should be called only after the connection has been established
    /// 2. After using this API, use [`execute_write()`] to write
    /// 3. This function triggers [`GattcEvent::PrepareWrite`]
    /// 4. If `value.len()` is less than or equal to MTU size, it is recommended to [`write_characteristic()`] to write directly
    /// 5. `handle` must be greater than 0
    pub fn prepare_write_characteristic(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        offset: u16,
        value: &[u8],
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_prepare_write(
                gattc_if,
                conn_id,
                handle,
                offset,
                value.len() as _,
                if value.is_empty() {
                    core::ptr::null_mut()
                } else {
                    value.as_ptr() as *const _ as *mut _
                },
                auth_req as _,
            )
        })
    }

    /// Prepare to write a characteristic descriptor value at a given handle.
    /// * `handle` The characteristic descriptor handle to write
    /// * `offset` The position offset to write
    /// * `value` The value to write
    /// * `auth_req` Authentication request type
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::WriteCharacteristic`]
    /// 2. This function should be called only after the connection has been established
    /// 3. `handle` must be greater than 0
    pub fn prepare_write_descriptor(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        handle: Handle,
        offset: u16,
        value: &[u8],
        auth_req: GattAuthReq,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_prepare_write_char_descr(
                gattc_if,
                conn_id,
                handle,
                offset,
                value.len() as _,
                if value.is_empty() {
                    core::ptr::null_mut()
                } else {
                    value.as_ptr() as *const _ as *mut _
                },
                auth_req as _,
            )
        })
    }

    /// Execute a prepared writing sequence.
    /// * `is_execute` True if it is to execute the writing sequence; false if it is to cancel the writing sequence.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::ExecWrite`]
    pub fn execute_write(
        &self,
        gattc_if: GattInterface,
        conn_id: ConnectionId,
        is_execute: bool,
    ) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_execute_write(gattc_if, conn_id, is_execute,) })
    }

    /// Register to receive notification/indication of a characteristic.
    /// * `server_bda` Target GATT server device address
    /// * `handle` Target GATT characteristic handle
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::RegisterNotify`]
    /// 2. You should call [`write_descriptor()`] after this API to write Client Characteristic Configuration (CCC)
    ///    descriptor to the value of 1 (Enable Notification) or 2 (Enable Indication)
    /// 3. `handle` must be greater than 0
    pub fn register_for_notify(
        &self,
        gattc_if: GattInterface,
        server_addr: BdAddr,
        handle: Handle,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_register_for_notify(
                gattc_if,
                &server_addr.raw() as *const _ as *mut _,
                handle,
            )
        })
    }

    /// Unregister the notification of a service.
    /// * `server_bda` Target GATT server device address
    /// * `handle` Target GATT characteristic handle
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::UnregisterNotify`]
    /// 2. You should call [`write_descriptor()`] after this API to write Client Characteristic Configuration (CCC)
    ///    descriptor value to 0
    /// 3. `handle` must be greater than 0
    pub fn unregister_for_notify(
        &self,
        gattc_if: GattInterface,
        server_addr: BdAddr,
        handle: Handle,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_unregister_for_notify(
                gattc_if,
                &server_addr.raw() as *const _ as *mut _,
                handle,
            )
        })
    }

    /// Refresh the cache of the remote device.
    /// * `remote_bda` Remote device address
    ///
    /// # Note
    /// 1. If the device is connected, this API will restart the discovery of service information of the remote device
    /// 2. This function triggers [`GattcEvent::DiscoveryCompleted`] only after the ACL connection is established. Otherwise,
    ///    no events will be triggered
    pub fn cache_refresh(&self, remote_bda: BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_cache_refresh(&remote_bda.raw() as *const _ as *mut _,) })
    }

    /// Add or remove the association between the address in the local GATTC cache with the source address
    /// of the remote device.
    /// * `src_addr` The source address intended to be associated to the `assoc_addr` which has been stored in the local GATTC cache
    /// * `assoc_addr` The associated device address intended to share the attribute table with the source address
    /// * `is_assoc` True if adding the association; false if removing the association
    ///
    /// # Note
    /// 1. This API is primarily used when the client has a stored server-side database (`assoc_addr`) and needs to connect to
    ///    another device (`src_addr`) with the same attribute database. By invoking this API, the stored database is utilized
    ///    as the peer server database, eliminating the need for attribute database search and discovery. This reduces
    ///    processing time and accelerates the connection process
    /// 2. The attribute table of a device with `assoc_addr` must be stored in the local GATTC cache first.
    ///    Then, the attribute table of the device with `src_addr` must be the same as the one with `assoc_addr`
    /// 3. This function triggers [`GattcEvent::SetAssociation`]
    pub fn cache_assoc(
        &self,
        gattc_if: GattInterface,
        src_addr: BdAddr,
        assoc_addr: BdAddr,
        is_assoc: bool,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gattc_cache_assoc(
                gattc_if,
                &src_addr.raw() as *const _ as *mut _,
                &assoc_addr.raw() as *const _ as *mut _,
                is_assoc,
            )
        })
    }

    /// Get the address list stored in the local GATTC cache.
    ///
    /// # Note
    /// 1. This function triggers [`GattcEvent::AddressList`]
    pub fn get_address_list(&self, gattc_if: GattInterface) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_cache_get_addr_list(gattc_if,) })
    }

    /// Clean the service cache of the target device in the local GATTC cache.
    /// * `remote_bda` Remote device address
    pub fn cache_clean(&self, remote_addr: BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gattc_cache_clean(&remote_addr.raw() as *const _ as *mut _,) })
    }

    unsafe extern "C" fn event_handler(
        event: esp_gattc_cb_event_t,
        gattc_if: esp_gatt_if_t,
        param: *mut esp_ble_gattc_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = GattcEvent::from((event, param));

        trace!("Got event {{ {event:#?} }}");

        SINGLETON.call((gattc_if, event));
    }
}

impl<'d, M, T> Drop for EspGattc<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        esp!(unsafe { esp_ble_gattc_register_callback(None) }).unwrap();

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T> Send for EspGattc<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T> Sync for EspGattc<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

static SINGLETON: BtSingleton<(GattInterface, GattcEvent), ()> = BtSingleton::new(());

fn check_gatt_status(status: esp_gatt_status_t) -> Result<(), GattStatus> {
    let status = status.try_into().unwrap();
    if status == GattStatus::Ok {
        Ok(())
    } else {
        Err(status)
    }
}
