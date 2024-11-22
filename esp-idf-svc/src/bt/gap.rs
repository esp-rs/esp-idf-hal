#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use core::cmp::min;
use core::convert::TryInto;
#[cfg(any(
    esp_idf_version_major = "4",
    all(
        esp_idf_version_major = "5",
        any(esp_idf_version_minor = "0", esp_idf_version_minor = "1")
    ),
))]
use core::ffi;
use core::fmt::{self, Debug};
use core::{borrow::Borrow, marker::PhantomData};

use enumset::{EnumSet, EnumSetType};

use crate::sys::*;

use ::log::debug;

use num_enum::TryFromPrimitive;

use super::{BdAddr, BtClassicEnabled, BtDriver, BtSingleton, BtStatus, BtUuid};

#[cfg(esp_idf_bt_ssp_enabled)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum IOCapabilities {
    DisplayOnly = ESP_BT_IO_CAP_OUT as _,
    DisplayInput = ESP_BT_IO_CAP_IO as _,
    InputOnly = ESP_BT_IO_CAP_IN as _,
    None = ESP_BT_IO_CAP_NONE as _,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum DiscoveryMode {
    NonDiscoverable = esp_bt_discovery_mode_t_ESP_BT_NON_DISCOVERABLE,
    Limited = esp_bt_discovery_mode_t_ESP_BT_LIMITED_DISCOVERABLE,
    Discoverable = esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum InqMode {
    General = esp_bt_inq_mode_t_ESP_BT_INQ_MODE_GENERAL_INQUIRY,
    Limited = esp_bt_inq_mode_t_ESP_BT_INQ_MODE_LIMITED_INQUIRY,
}

#[derive(Debug)]
#[repr(transparent)]
pub struct Eir<'a>(&'a [u8]);

impl<'a> Eir<'a> {
    pub fn flags<'d, M, T>(&self, _gap: &EspGap<'d, M, T>) -> Option<EnumSet<EirFlags>>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::Flags)
            .map(|slice| EnumSet::from_repr(slice[0]))
    }

    pub fn uuid16<'d, M, T>(&self) -> Option<BtUuid>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::Uuid16)
            .map(|slice| BtUuid::uuid16(u16::from_ne_bytes([slice[0], slice[1]])))
    }

    pub fn uuid32<'d, M, T>(&self) -> Option<BtUuid>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::Uuid32).map(|slice| {
            BtUuid::uuid32(u32::from_ne_bytes([slice[0], slice[1], slice[2], slice[3]]))
        })
    }

    pub fn uuid128<'d, M, T>(&self) -> Option<BtUuid>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::Uuid128).map(|slice| {
            BtUuid::uuid128(u128::from_ne_bytes([
                slice[0], slice[1], slice[2], slice[3], slice[4], slice[5], slice[6], slice[7],
                slice[8], slice[9], slice[10], slice[11], slice[12], slice[13], slice[14],
                slice[15],
            ]))
        })
    }

    pub fn short_local_name<'d, M, T>(&self) -> Option<&str>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::ShortLocalName)
            .map(|slice| unsafe { core::str::from_utf8_unchecked(slice) })
    }

    pub fn local_name<'d, M, T>(&self) -> Option<&str>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::LocalName)
            .map(|slice| unsafe { core::str::from_utf8_unchecked(slice) })
    }

    pub fn url<'d, M, T>(&self) -> Option<&str>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::Url)
            .map(|slice| unsafe { core::str::from_utf8_unchecked(slice) })
    }

    pub fn manufacturer_data<'d, M, T>(&self) -> Option<&[u8]>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        self.resolve(EirType::ManufacturerData)
    }

    // pub fn uui16_incomplete(&self) -> Option<&[u8]> {
    //     todo!()
    // }

    // pub fn uui32_incomplete(&self) -> Option<&[u8]> {
    //     todo!()
    // }

    // pub fn uuid128(&self) -> Option<&[u8]> {
    //     todo!()
    // }

    // pub fn uui128_incomplete(&self) -> Option<&[u8]> {
    //     todo!()
    // }

    fn resolve(&self, eir_type: EirType) -> Option<&[u8]> {
        let mut len = self.0.len() as _;
        let addr = unsafe {
            esp_bt_gap_resolve_eir_data(self.0.as_ptr() as *mut _, eir_type as _, &mut len)
        };

        if addr.is_null() {
            None
        } else {
            Some(unsafe { core::slice::from_raw_parts(addr, len as _) })
        }
    }
}

impl<'a> From<&'a [u8]> for Eir<'a> {
    fn from(value: &'a [u8]) -> Self {
        Self(value)
    }
}

impl<'a> From<Eir<'a>> for &'a [u8] {
    fn from(value: Eir<'a>) -> Self {
        value.0
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum EirType {
    Flags = ESP_BT_EIR_TYPE_FLAGS as _,
    Uuid16Incomplete = ESP_BT_EIR_TYPE_INCMPL_16BITS_UUID as _,
    Uuid16 = ESP_BT_EIR_TYPE_CMPL_16BITS_UUID as _,
    Uuid32Incomplete = ESP_BT_EIR_TYPE_INCMPL_32BITS_UUID as _,
    Uuid32 = ESP_BT_EIR_TYPE_CMPL_32BITS_UUID as _,
    Uuid128Incomplete = ESP_BT_EIR_TYPE_INCMPL_128BITS_UUID as _,
    Uuid128 = ESP_BT_EIR_TYPE_CMPL_128BITS_UUID as _,
    ShortLocalName = ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME as _,
    LocalName = ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME as _,
    TxPowerLevel = ESP_BT_EIR_TYPE_TX_POWER_LEVEL as _,
    Url = ESP_BT_EIR_TYPE_URL as _,
    ManufacturerData = ESP_BT_EIR_TYPE_MANU_SPECIFIC as _,
}

#[derive(Debug, EnumSetType, TryFromPrimitive)]
#[enumset(repr = "u8")]
#[repr(u8)]
pub enum EirFlags {
    LimitDisc = 0,        //ESP_BT_EIR_FLAG_LIMIT_DISC as _,
    GenDisc = 1,          //ESP_BT_EIR_FLAG_GEN_DISC as _,
    BredrNotSpt = 2,      //ESP_BT_EIR_FLAG_BREDR_NOT_SPT as _,
    DmtControllerSpt = 3, //ESP_BT_EIR_FLAG_DMT_CONTROLLER_SPT as _,
    DmtHostSpt = 4,       //ESP_BT_EIR_FLAG_DMT_HOST_SPT as _,
}

#[derive(Debug, Clone)]
pub struct EirData<'a> {
    pub fec_required: bool,          // FEC is required or not, true by default
    pub include_txpower: bool,       // EIR data include TX power, false by default
    pub include_uuid: bool,          // EIR data include UUID, false by default
    pub flags: EnumSet<EirFlags>, // EIR flags, see ESP_BT_EIR_FLAG for details, EIR will not include flag if it is 0, 0 by default
    pub manufacturer_data: &'a [u8], // Manufacturer data point
    pub url: &'a str,             // URL point
}

impl<'a> From<&EirData<'a>> for esp_bt_eir_data_t {
    #[allow(clippy::needless_update)]
    fn from(data: &EirData<'a>) -> Self {
        Self {
            fec_required: data.fec_required,
            include_txpower: data.include_txpower,
            include_uuid: data.include_uuid,
            flag: data.flags.as_repr(),
            p_manufacturer_data: data.manufacturer_data.as_ptr() as *mut _,
            manufacturer_len: data.manufacturer_data.len() as _,
            p_url: data.url.as_ptr() as *mut _,
            url_len: data.url.len() as _,
            // Necessary as a strange flag - `include_name` - appeared in branch `release/v5.0` and ONLY there
            ..Default::default()
        }
    }
}

impl<'a> From<&esp_bt_eir_data_t> for EirData<'a> {
    fn from(data: &esp_bt_eir_data_t) -> Self {
        Self {
            fec_required: data.fec_required,
            include_txpower: data.include_txpower,
            include_uuid: data.include_uuid,
            flags: EnumSet::from_repr(data.flag),
            manufacturer_data: unsafe {
                core::slice::from_raw_parts(data.p_manufacturer_data, data.manufacturer_len as _)
            },
            url: unsafe {
                core::str::from_utf8_unchecked(core::slice::from_raw_parts(
                    data.p_url,
                    data.url_len as _,
                ))
            },
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, TryFromPrimitive)]
#[repr(u32)]
pub enum CodMode {
    SetMajorMinor = esp_bt_cod_mode_t_ESP_BT_SET_COD_MAJOR_MINOR,
    SetServicdeClass = esp_bt_cod_mode_t_ESP_BT_SET_COD_SERVICE_CLASS,
    ClearServiceClass = esp_bt_cod_mode_t_ESP_BT_CLR_COD_SERVICE_CLASS,
    SetAll = esp_bt_cod_mode_t_ESP_BT_SET_COD_ALL,
    Init = esp_bt_cod_mode_t_ESP_BT_INIT_COD,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum CodMajorDeviceType {
    Miscellaneous = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_MISC,
    Computer = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_COMPUTER,
    Phone = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_PHONE,
    Lan = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_LAN_NAP,
    AudioVideo = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_AV,
    Peripheral = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_PERIPHERAL,
    Imaging = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_IMAGING,
    Wearable = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_WEARABLE,
    Toy = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_TOY,
    Health = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_HEALTH,
    Other = esp_bt_cod_major_dev_t_ESP_BT_COD_MAJOR_DEV_UNCATEGORIZED,
}

#[derive(Debug, EnumSetType, TryFromPrimitive)]
#[enumset(repr = "u16")]
#[repr(u16)]
pub enum CodServiceClass {
    LimitedDiscoverable = 0, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_LMTD_DISCOVER: esp_bt_cod_srvc_t = 1;
    Unknown1 = 1,
    Unknown2 = 2,
    Positioning = 3, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_POSITIONING: esp_bt_cod_srvc_t = 8;
    Networking = 4, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_NETWORKING: esp_bt_cod_srvc_t = 16;
    Rendering = 5,  //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_RENDERING: esp_bt_cod_srvc_t = 32;
    Capturing = 6,  //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_CAPTURING: esp_bt_cod_srvc_t = 64;
    ObjectTransfer = 7, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_OBJ_TRANSFER: esp_bt_cod_srvc_t = 128;
    Audio = 8, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_AUDIO: esp_bt_cod_srvc_t = 256;
    Telephony = 9, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_TELEPHONY: esp_bt_cod_srvc_t = 512;
    Information = 10, //pub const esp_bt_cod_srvc_t_ESP_BT_COD_SRVC_INFORMATION: esp_bt_cod_srvc_t = 1024;
    Unknown3 = 11,
    Unknown4 = 12,
    Unknown5 = 13,
    Unknown6 = 14,
    Unknown7 = 15,
}

#[derive(Debug, Copy, Clone)]
#[repr(transparent)]
pub struct Cod(esp_bt_cod_t);

impl Cod {
    pub fn new(major: CodMajorDeviceType, minor: u32, services: EnumSet<CodServiceClass>) -> Self {
        let mut cod: esp_bt_cod_t = Default::default();

        cod.set_major(major as _);
        cod.set_minor(minor);
        cod.set_service(services.as_repr() as _);

        Self(cod)
    }

    pub fn raw(&self) -> esp_bt_cod_t {
        self.0
    }

    pub fn major(&self) -> CodMajorDeviceType {
        self.0.major().try_into().unwrap()
    }

    pub fn minor(&self) -> u32 {
        self.0.minor()
    }

    pub fn services(&self) -> EnumSet<CodServiceClass> {
        EnumSet::from_repr((self.0.service() & 0xffff) as u16)
    }
}

impl From<Cod> for esp_bt_cod_t {
    fn from(cod: Cod) -> Self {
        cod.0
    }
}

impl From<esp_bt_cod_t> for Cod {
    fn from(cod: esp_bt_cod_t) -> Self {
        Self(cod)
    }
}

impl PartialEq for Cod {
    fn eq(&self, other: &Self) -> bool {
        self.0.major() == other.0.major()
            && self.0.minor() == other.0.minor()
            && self.0.service() == other.0.service()
    }
}

impl Eq for Cod {}

#[derive(Debug)]
pub enum DeviceProp<'a> {
    BdName(&'a str),
    Cod(Cod),
    Rssi(i8),
    Eir(Eir<'a>),
}

#[derive(Clone, Debug)]
#[repr(transparent)]
pub struct PropData<'a> {
    data: esp_bt_gap_dev_prop_t,
    _p: PhantomData<&'a ()>,
}

#[allow(non_upper_case_globals)]
impl<'a> PropData<'a> {
    pub fn prop(&self) -> DeviceProp {
        unsafe {
            match self.data.type_ {
                esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_BDNAME => {
                    DeviceProp::BdName(core::str::from_utf8_unchecked(core::slice::from_raw_parts(
                        self.data.val as *mut u8 as *const _,
                        self.data.len as _,
                    )))
                }
                esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_COD => {
                    let esp_cod: esp_bt_cod_t = core::mem::transmute(self.data.val as u32);
                    DeviceProp::Cod(esp_cod.into())
                }
                esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_RSSI => {
                    let rssi = self.data.val as u32 as i8;
                    DeviceProp::Rssi(rssi)
                }
                esp_bt_gap_dev_prop_type_t_ESP_BT_GAP_DEV_PROP_EIR => DeviceProp::Eir(
                    core::slice::from_raw_parts(
                        self.data.val as *mut u8 as *const _,
                        self.data.len as _,
                    )
                    .into(),
                ),
                _ => unreachable!(),
            }
        }
    }
}

pub struct EventRawData<'a>(pub &'a esp_bt_gap_cb_param_t);

impl<'a> Debug for EventRawData<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("EventRawData").finish()
    }
}

#[derive(Debug)]
pub enum GapEvent<'a> {
    DeviceDiscoveryStarted,
    DeviceDiscoveryStopped,
    DeviceDiscovered {
        bd_addr: BdAddr,
        props: &'a [PropData<'a>],
    },
    RemoteServiceDiscovered {
        bd_addr: BdAddr,
        status: BtStatus,
        services: &'a [BtUuid],
    },
    RemoteServiceDetails {
        bd_addr: BdAddr,
        status: BtStatus,
    },
    AuthenticationCompleted {
        bd_addr: BdAddr,
        status: BtStatus,
        device_name: &'a str,
    },
    PairingPinRequest {
        bd_addr: BdAddr,
        min_16_digit: bool,
    },
    PairingUserConfirmationRequest {
        bd_addr: BdAddr,
        number: u32,
    },
    SspPasskey {
        bd_addr: BdAddr,
        passkey: u32,
    },
    SspPasskeyRequest {
        bd_addr: BdAddr,
    },
    Rssi {
        bd_addr: BdAddr,
        status: BtStatus,
        rssi: i8,
    },
    EirDataConfigured {
        status: BtStatus,
        eir_types: &'a [EirType],
    },
    AfhChannelsConfigured(BtStatus),
    RemoteName {
        bd_addr: BdAddr,
        status: BtStatus,
        name: &'a str,
    },
    ModeChange {
        bd_addr: BdAddr,
        mode: u8,
    },
    BondedServiceRemoved {
        bd_addr: BdAddr,
        status: BtStatus,
    },
    AclConnected {
        bd_addr: BdAddr,
        status: BtStatus,
        handle: u16,
    },
    AclDisconnected {
        bd_addr: BdAddr,
        status: BtStatus,
        handle: u16,
    },
    QosComplete {
        bd_addr: BdAddr,
        status: BtStatus,
        t_poll: u32,
    },
    Other {
        raw_event: esp_bt_gap_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_bt_gap_cb_event_t, &'a esp_bt_gap_cb_param_t)> for GapEvent<'a> {
    fn from(value: (esp_bt_gap_cb_event_t, &'a esp_bt_gap_cb_param_t)) -> Self {
        let (event, param) = value;

        unsafe {
            match event {
                esp_bt_gap_cb_event_t_ESP_BT_GAP_DISC_RES_EVT => Self::DeviceDiscovered {
                    bd_addr: param.disc_res.bda.into(),
                    props: core::slice::from_raw_parts(
                        param.disc_res.prop as *mut PropData as *const _,
                        param.disc_res.num_prop as _,
                    ),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_DISC_STATE_CHANGED_EVT => {
                    if param.disc_st_chg.state
                        == esp_bt_gap_discovery_state_t_ESP_BT_GAP_DISCOVERY_STOPPED
                    {
                        Self::DeviceDiscoveryStopped
                    } else {
                        Self::DeviceDiscoveryStarted
                    }
                }
                esp_bt_gap_cb_event_t_ESP_BT_GAP_RMT_SRVCS_EVT => Self::RemoteServiceDiscovered {
                    bd_addr: param.rmt_srvcs.bda.into(),
                    status: param.rmt_srvcs.stat.try_into().unwrap(),
                    services: core::slice::from_raw_parts(
                        param.rmt_srvcs.uuid_list as *mut BtUuid as *const _,
                        param.rmt_srvcs.num_uuids as _,
                    ),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_RMT_SRVC_REC_EVT => Self::RemoteServiceDetails {
                    bd_addr: param.rmt_srvcs.bda.into(),
                    status: param.rmt_srvcs.stat.try_into().unwrap(),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_AUTH_CMPL_EVT => Self::AuthenticationCompleted {
                    bd_addr: param.auth_cmpl.bda.into(),
                    status: param.auth_cmpl.stat.try_into().unwrap(),
                    device_name: core::str::from_utf8_unchecked(
                        &param.auth_cmpl.device_name
                            [..strlen(&param.auth_cmpl.device_name as *const _ as *const _) as _],
                    ),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_PIN_REQ_EVT => Self::PairingPinRequest {
                    bd_addr: param.pin_req.bda.into(),
                    min_16_digit: param.pin_req.min_16_digit,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_CFM_REQ_EVT => {
                    Self::PairingUserConfirmationRequest {
                        bd_addr: param.cfm_req.bda.into(),
                        number: param.cfm_req.num_val,
                    }
                }
                esp_bt_gap_cb_event_t_ESP_BT_GAP_KEY_NOTIF_EVT => Self::SspPasskey {
                    bd_addr: param.key_notif.bda.into(),
                    passkey: param.key_notif.passkey,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_KEY_REQ_EVT => Self::SspPasskeyRequest {
                    bd_addr: param.key_req.bda.into(),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_READ_RSSI_DELTA_EVT => Self::Rssi {
                    bd_addr: param.read_rssi_delta.bda.into(),
                    status: param.read_rssi_delta.stat.try_into().unwrap(),
                    rssi: param.read_rssi_delta.rssi_delta,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_CONFIG_EIR_DATA_EVT => Self::EirDataConfigured {
                    status: param.config_eir_data.stat.try_into().unwrap(),
                    eir_types: core::mem::transmute::<&[u8], &[EirType]>(
                        &param.config_eir_data.eir_type[..param.config_eir_data.eir_type_num as _],
                    ),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_SET_AFH_CHANNELS_EVT => {
                    Self::AfhChannelsConfigured(param.set_afh_channels.stat.try_into().unwrap())
                }
                esp_bt_gap_cb_event_t_ESP_BT_GAP_READ_REMOTE_NAME_EVT => Self::RemoteName {
                    bd_addr: param.read_rmt_name.bda.into(),
                    status: param.read_rmt_name.stat.try_into().unwrap(),
                    name: core::str::from_utf8_unchecked(
                        &param.read_rmt_name.rmt_name
                            [..strlen(param.read_rmt_name.rmt_name.as_ptr() as *const _) as _],
                    ),
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_MODE_CHG_EVT => Self::ModeChange {
                    bd_addr: param.mode_chg.bda.into(),
                    mode: param.mode_chg.mode,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT => {
                    Self::BondedServiceRemoved {
                        bd_addr: param.remove_bond_dev_cmpl.bda.into(),
                        status: param.remove_bond_dev_cmpl.status.try_into().unwrap(),
                    }
                }
                esp_bt_gap_cb_event_t_ESP_BT_GAP_QOS_CMPL_EVT => Self::QosComplete {
                    bd_addr: param.qos_cmpl.bda.into(),
                    status: param.qos_cmpl.stat.try_into().unwrap(),
                    t_poll: param.qos_cmpl.t_poll,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT => Self::AclConnected {
                    bd_addr: param.acl_conn_cmpl_stat.bda.into(),
                    status: param.acl_conn_cmpl_stat.stat.try_into().unwrap(),
                    handle: param.acl_conn_cmpl_stat.handle,
                },
                esp_bt_gap_cb_event_t_ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT => {
                    Self::AclDisconnected {
                        bd_addr: param.acl_disconn_cmpl_stat.bda.into(),
                        status: param.acl_disconn_cmpl_stat.reason.try_into().unwrap(),
                        handle: param.acl_disconn_cmpl_stat.handle,
                    }
                }
                _ => Self::Other {
                    raw_event: event,
                    raw_data: EventRawData(param),
                },
            }
        }
    }
}

pub struct EspGap<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspGap<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    pub fn new(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_bt_gap_register_callback(Some(Self::event_handler)) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut(GapEvent) + Send + 'static,
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
        F: FnMut(GapEvent) + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    pub fn set_scan_mode(
        &self,
        connectable: bool,
        discovery_mode: DiscoveryMode,
    ) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_set_scan_mode(connectable as _, discovery_mode as _) })
    }

    pub fn start_discovery(
        &self,
        inq_mode: InqMode,
        inq_duration: u8,
        num_rsps: usize,
    ) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_start_discovery(inq_mode as _, inq_duration, num_rsps as _) })
    }

    pub fn stop_discovery(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_cancel_discovery() })
    }

    pub fn request_remote_services(&self, bd_addr: &BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_get_remote_services(bd_addr as *const _ as *mut _) })
    }

    pub fn set_eir_data_conf(&self, eir_data: &EirData) -> Result<(), EspError> {
        let data = eir_data.into();
        esp!(unsafe { esp_bt_gap_config_eir_data(&data as *const esp_bt_eir_data_t as *mut _) })
    }

    pub fn get_cod(&self) -> Result<Cod, EspError> {
        let mut cod = Default::default();
        esp!(unsafe { esp_bt_gap_get_cod(&mut cod) })?;

        Ok(cod.into())
    }

    pub fn set_cod(&self, cod: Cod, mode: CodMode) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_set_cod(cod.into(), mode as _) })
    }

    pub fn request_rssi_delta(&self, bd_addr: &BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_read_rssi_delta(bd_addr as *const _ as *mut _) })
    }

    pub fn get_bond_services<'a>(
        &self,
        buf: &'a mut [BdAddr],
    ) -> Result<(&'a [BdAddr], usize), EspError> {
        let mut dev_num = buf.len() as _;

        esp!(unsafe { esp_bt_gap_get_bond_device_list(&mut dev_num, buf.as_ptr() as *mut _) })?;

        Ok((&buf[..min(dev_num as _, buf.len())], dev_num as _))
    }

    pub fn remove_bond_service(&self, bd_addr: &BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_remove_bond_device(bd_addr as *const _ as *mut _) })
    }

    pub fn set_pin(&self, pin: &str) -> Result<(), EspError> {
        esp!(unsafe {
            esp_bt_gap_set_pin(
                esp_bt_pin_type_t_ESP_BT_PIN_TYPE_FIXED,
                pin.len() as _,
                pin.as_ptr() as *mut _,
            )
        })
    }

    pub fn request_variable_pin(&self) -> Result<(), EspError> {
        esp!(unsafe {
            esp_bt_gap_set_pin(
                esp_bt_pin_type_t_ESP_BT_PIN_TYPE_VARIABLE,
                0,
                core::ptr::null_mut(),
            )
        })
    }

    pub fn reply_variable_pin(&self, bd_addr: &BdAddr, pin: Option<&[u8]>) -> Result<(), EspError> {
        esp!(unsafe {
            esp_bt_gap_pin_reply(
                bd_addr as *const _ as *mut _,
                pin.is_some(),
                pin.map_or(0, |pin| pin.len() as _),
                pin.map_or(core::ptr::null_mut(), |pin| pin.as_ptr() as *mut _),
            )
        })
    }

    pub fn reply_passkey(&self, bd_addr: &BdAddr, passkey: Option<u32>) -> Result<(), EspError> {
        esp!(unsafe {
            esp_bt_gap_ssp_passkey_reply(
                bd_addr as *const _ as *mut _,
                passkey.is_some(),
                passkey.unwrap_or(0),
            )
        })
    }

    #[cfg(esp_idf_bt_ssp_enabled)]
    pub fn reply_ssp_confirm(&self, bd_addr: &BdAddr, confirm: bool) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_ssp_confirm_reply(bd_addr as *const _ as *mut _, confirm) })
    }

    pub fn set_afh_channels_conf(&self, channels: &[u8; 10]) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_set_afh_channels(channels as *const _ as *mut _) })
    }

    pub fn request_remote_name(&self, bd_addr: &BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_read_remote_name(bd_addr as *const _ as *mut _) })
    }

    pub fn set_qos_conf(&self, bd_addr: &BdAddr, poll: u32) -> Result<(), EspError> {
        esp!(unsafe { esp_bt_gap_set_qos(bd_addr as *const _ as *mut _, poll) })
    }

    #[cfg(esp_idf_bt_ssp_enabled)]
    pub fn set_ssp_io_cap(&self, io_cap: IOCapabilities) -> Result<(), EspError> {
        let io_cap: esp_bt_io_cap_t = io_cap as _;
        esp!(unsafe {
            esp_bt_gap_set_security_param(
                esp_bt_sp_param_t_ESP_BT_SP_IOCAP_MODE,
                &io_cap as *const _ as *mut ffi::c_void,
                1,
            )
        })
    }

    unsafe extern "C" fn event_handler(
        event: esp_bt_gap_cb_event_t,
        param: *mut esp_bt_gap_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = GapEvent::from((event, param));

        debug!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }
}

impl<'d, M, T> Drop for EspGap<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        // Not possible because this function rejects NULL arguments
        // esp!(unsafe { esp_bt_gap_register_callback(None) }).unwrap();

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T> Send for EspGap<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T> Sync for EspGap<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
{
}

static SINGLETON: BtSingleton<GapEvent, ()> = BtSingleton::new(());
