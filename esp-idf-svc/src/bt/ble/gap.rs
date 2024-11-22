use core::borrow::Borrow;
use core::fmt::{self, Debug};
use core::marker::PhantomData;
use core::{ffi::CStr, ops::BitOr};

use crate::bt::BtSingleton;
use crate::sys::*;

use ::log::debug;

use crate::{
    bt::{BdAddr, BleEnabled, BtDriver, BtStatus, BtUuid},
    private::cstr::to_cstring_arg,
};

#[derive(Default, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum IOCapabilities {
    #[default]
    DisplayOnly = 0,
    DisplayYesNo = 1,
    KeyboardOnly = 2,
    NoInputNoOutput = 3,
    Keyboard = 4,
}

#[derive(Default, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum AuthenticationRequest {
    #[default]
    NoBonding = 0b0000_0000,
    Bonding = 0b0000_0001,
    Mitm = 0b0000_0010,
    MitmBonding = 0b0000_0011,
    SecureOnly = 0b0000_0100,
    SecureBonding = 0b0000_0101,
    SecureMitm = 0b0000_0110,
    SecureMitmBonding = 0b0000_0111,
}

#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum KeyMask {
    EncryptionKey = 0b0000_0001,
    IdentityResolvingKey = 0b0000_0010,
    ConnectionSignatureResolvingKey = 0b0000_0100,
    LinkKey = 0b0000_1000,
    Inner0011 = 0b0000_0011,
    Inner0101 = 0b0000_0101,
    Inner1001 = 0b0000_1001,
    Inner1010 = 0b0000_1010,
    Inner1100 = 0b0000_1100,
    Inner1101 = 0b0000_1101,
    Inner1011 = 0b0000_1011,
    Inner1111 = 0b0000_1111,
}

impl BitOr for KeyMask {
    type Output = KeyMask;

    fn bitor(self, rhs: Self) -> Self::Output {
        (self as u8 | rhs as u8).into()
    }
}

impl From<u8> for KeyMask {
    fn from(from: u8) -> Self {
        match from {
            0b0000_0001 => KeyMask::EncryptionKey,
            0b0000_0010 => KeyMask::IdentityResolvingKey,
            0b0000_0100 => KeyMask::ConnectionSignatureResolvingKey,
            0b0000_1000 => KeyMask::LinkKey,
            0b0000_0011 => KeyMask::Inner0011,
            0b0000_0101 => KeyMask::Inner0101,
            0b0000_1001 => KeyMask::Inner1001,
            0b0000_1010 => KeyMask::Inner1010,
            0b0000_1100 => KeyMask::Inner1100,
            0b0000_1101 => KeyMask::Inner1101,
            0b0000_1011 => KeyMask::Inner1011,
            0b0000_1111 => KeyMask::Inner1111,
            _ => unimplemented!("This does not correspond to a valid KeyMask"),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum BleEncryption {
    Encryption = 0x01,
    EncryptionNoMitm = 0x02,
    EncryptionMitm = 0x03,
}

#[derive(Default, Clone)]
pub struct SecurityConfiguration {
    pub auth_req_mode: AuthenticationRequest,
    pub io_capabilities: IOCapabilities,
    pub initiator_key: Option<KeyMask>,
    pub responder_key: Option<KeyMask>,
    pub max_key_size: Option<u8>,
    pub min_key_size: Option<u8>,
    pub static_passkey: Option<u32>,
    pub only_accept_specified_auth: bool,
    pub enable_oob: bool,
    // app_key_size: u8,
}

#[allow(clippy::upper_case_acronyms)]
#[repr(u16)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AppearanceCategory {
    Unknown = 0x00,
    Phone,
    Computer,
    Watch,
    Clock,
    Display,
    RemoteControl,
    EyeGlass,
    Tag,
    Keyring,
    MediaPlayer,
    BarcodeScanner,
    Thermometer,
    HeartRateSensor,
    BloodPressure,
    HumanInterfaceDevice,
    GlucoseMeter,
    RunningWalkingSensor,
    Cycling,
    ControlDevice,
    NetworkDevice,
    Sensor,
    LightFixtures,
    Fan,
    HVAC,
    AirConditionning,
    Humidifier,
    Heating,
    AccessControl,
    MotorizedDevice,
    PowerDevice,
    LightSource,
    WindowCovering,
    AudioSink,
    AudioSource,
    MotorizedVehicle,
    DomesticAppliance,
    WearableAudioDevice,
    Aircraft,
    AVEquipment,
    DisplayEquipment,
    HearingAid,
    Gaming,
    Signage,
    PulseOximeter = 0x31,
    WeightScale,
    PersonalMobilityDevice,
    ContinuousGlucoseMonitor,
    InsulinPump,
    MedicationDelivery,
    OutdoorSportsActivity = 0x51,
}

impl From<AppearanceCategory> for i32 {
    fn from(cat: AppearanceCategory) -> Self {
        ((cat as u16) << 6) as _
    }
}

#[derive(Clone, Debug)]
pub struct AdvConfiguration<'a> {
    pub set_scan_rsp: bool,
    pub include_name: bool,
    pub include_txpower: bool,
    pub min_interval: i32,
    pub max_interval: i32,
    pub appearance: AppearanceCategory,
    pub flag: u8,
    pub service_uuid: Option<BtUuid>,
    pub service_data: Option<&'a [u8]>,
    pub manufacturer_data: Option<&'a [u8]>,
}

impl Default for AdvConfiguration<'_> {
    fn default() -> Self {
        Self {
            set_scan_rsp: false,
            include_name: false,
            include_txpower: false,
            min_interval: 0,
            max_interval: 0,
            appearance: AppearanceCategory::Unknown,
            service_uuid: None,
            service_data: None,
            manufacturer_data: None,
            flag: ESP_BLE_ADV_FLAG_NON_LIMIT_DISC as _,
        }
    }
}

impl<'a> From<&'a AdvConfiguration<'a>> for esp_ble_adv_data_t {
    fn from(data: &'a AdvConfiguration<'a>) -> Self {
        Self {
            set_scan_rsp: data.set_scan_rsp,
            include_name: data.include_name,
            include_txpower: data.include_txpower,
            min_interval: data.min_interval,
            max_interval: data.max_interval,
            manufacturer_len: data.manufacturer_data.as_ref().map_or(0, |m| m.len()) as _,
            p_manufacturer_data: data
                .manufacturer_data
                .map_or(core::ptr::null_mut(), |s| s.as_ptr() as _),
            service_data_len: data.service_data.as_ref().map_or(0, |s| s.len()) as _,
            p_service_data: data
                .service_data
                .map_or(core::ptr::null_mut(), |s| s.as_ptr() as _),
            service_uuid_len: data
                .service_uuid
                .as_ref()
                .map_or(0, |uuid| uuid.as_bytes().len()) as _,
            p_service_uuid: data
                .service_uuid
                .as_ref()
                .map_or(core::ptr::null_mut(), |uuid| uuid.as_bytes().as_ptr() as _),
            appearance: data.appearance.into(),
            flag: data.flag,
        }
    }
}

pub struct EventRawData<'a>(pub &'a esp_ble_gap_cb_param_t);

impl Debug for EventRawData<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("EventRawData").finish()
    }
}

#[derive(Debug)]
pub enum BleGapEvent<'a> {
    AdvertisingConfigured(BtStatus),
    ScanResponseConfigured(BtStatus),
    ScanParameterConfigured(BtStatus),
    // TODO
    ScanResult(esp_ble_gap_cb_param_t_ble_scan_result_evt_param),
    RawAdvertisingConfigured(BtStatus),
    RawScanResponseConfigured(BtStatus),
    AdvertisingStarted(BtStatus),
    ScanStarted(BtStatus),
    AuthenticationComplete {
        // TODO: More fields here
        bd_addr: BdAddr,
        status: BtStatus,
    },
    // TODO: Parameters
    Key,
    // TODO: Parameters
    SecurityRequest,
    PasskeyNotification {
        addr: BdAddr,
        passkey: u32,
    },
    // TODO: Parameters
    PasskeyRequest,
    OOBRequest {
        oob_c: &'a [u8],
        oob_r: &'a [u8],
    },
    LocalIR,
    LocalER,
    // TODO: Parameters
    NumericComparisonRequest,
    AdvertisingStopped(BtStatus),
    ScanStopped(BtStatus),
    StaticRandomAddressConfigured(BtStatus),
    ConnectionParamsConfigured {
        addr: BdAddr,
        status: BtStatus,
        min_int_ms: u32,
        max_int_ms: u32,
        latency_ms: u32,
        conn_int: u16,
        timeout_ms: u32,
    },
    PacketLengthConfigured {
        status: BtStatus,
        rx_len: u16,
        tx_len: u16,
    },
    LocalPrivacyConfigured(BtStatus),
    DeviceBondRemoved {
        bd_addr: BdAddr,
        status: BtStatus,
    },
    DeviceBondCleared(BtStatus),
    // TODO
    DeviceBond(esp_ble_gap_cb_param_t_ble_get_bond_dev_cmpl_evt_param),
    ReadRssiConfigured {
        bd_addr: BdAddr,
        rssdi: i8,
        status: BtStatus,
    },
    WhitelistUpdated {
        status: BtStatus,
        wl_operation: u32,
    },
    // TODO
    DuplicateListUpdated(
        esp_ble_gap_cb_param_t_ble_update_duplicate_exceptional_list_cmpl_evt_param,
    ),
    ChannelsConfigured(BtStatus),
    // BLE 5.0
    // TODO
    ReadFeaturesConfigured(esp_ble_gap_cb_param_t_ble_read_phy_cmpl_evt_param),
    PreferredDefaultPhyConfigured(BtStatus),
    PreferredPhyConfigured(BtStatus),
    ExtendedAdvertisingRandomAddressConfigured(BtStatus),
    ExtendedAdvertisingParametersConfigured(BtStatus),
    ExtendedAdvertisingConfigured(BtStatus),
    ExtendedAdvertisingScanResponseConfigured(BtStatus),
    ExtendedAdvertisingStarted(BtStatus),
    ExtendedAdvertisingStopped(BtStatus),
    ExtendedAdvertisingRemoved(BtStatus),
    ExtendedAdvertisingCleared(BtStatus),
    PeriodicAdvertisingParametersConfigured(BtStatus),
    PeriodicAdvertisingDataSetComplete(BtStatus),
    PeriodicAdvertisingStarted(BtStatus),
    PeriodicAdvertisingStopped(BtStatus),
    PeriodicAdvertisingSyncCreated(BtStatus),
    PeriodicAdvertisingSyncCanceled(BtStatus),
    PeriodicAdvertisingSyncTerminated(BtStatus),
    PeriodicAdvertisingDeviceListAdded(BtStatus),
    PeriodicAdvertisingDeviceListRemoved(BtStatus),
    PeriodicAdvertisingDeviceListCleared(BtStatus),
    ExtendedAdvertisingScanParametersConfigured(BtStatus),
    ExtendedAdvertisingScanStarted(BtStatus),
    ExtendedAdvertisingScanStopped(BtStatus),
    ExtendedAdvertisingExtendedConnectionParamsConfigured(BtStatus),
    /*
    #if (BLE_50_FEATURE_SUPPORT == TRUE)
        PHY_UPDATE_COMPLETE_EVT,
        EXT_ADV_REPORT_EVT,
        SCAN_TIMEOUT_EVT,
        ADV_TERMINATED_EVT,
        SCAN_REQ_RECEIVED_EVT,
        CHANNEL_SELECT_ALGORITHM_EVT,
        PERIODIC_ADV_REPORT_EVT,
        PERIODIC_ADV_SYNC_LOST_EVT,
        PERIODIC_ADV_SYNC_ESTAB_EVT,
    #endif // #if (BLE_50_FEATURE_SUPPORT == TRUE)
        EVT_MAX,
    */
    Other {
        raw_event: esp_gap_ble_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_gap_ble_cb_event_t, &'a esp_ble_gap_cb_param_t)> for BleGapEvent<'a> {
    fn from(value: (esp_gap_ble_cb_event_t, &'a esp_ble_gap_cb_param_t)) -> Self {
        let (event, param) = value;

        unsafe {
            match event {
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT => {
                    Self::AdvertisingConfigured(param.adv_data_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT => {
                    Self::ScanResponseConfigured(
                        param.scan_rsp_data_cmpl.status.try_into().unwrap(),
                    )
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT => {
                    Self::ScanParameterConfigured(param.scan_param_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RESULT_EVT => {
                    Self::ScanResult(param.scan_rst)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT => {
                    Self::RawAdvertisingConfigured(
                        param.adv_data_raw_cmpl.status.try_into().unwrap(),
                    )
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT => {
                    Self::RawScanResponseConfigured(
                        param.scan_rsp_data_raw_cmpl.status.try_into().unwrap(),
                    )
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_START_COMPLETE_EVT => {
                    Self::AdvertisingStarted(param.adv_start_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_START_COMPLETE_EVT => {
                    Self::ScanStarted(param.scan_start_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_AUTH_CMPL_EVT => Self::AuthenticationComplete {
                    bd_addr: param.ble_security.auth_cmpl.bd_addr.into(),
                    status: if param.ble_security.auth_cmpl.success {
                        BtStatus::Success
                    } else {
                        BtStatus::Fail
                    },
                },
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_KEY_EVT => Self::Key,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SEC_REQ_EVT => Self::SecurityRequest,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_PASSKEY_NOTIF_EVT => Self::PasskeyNotification {
                    addr: param.ble_security.key_notif.bd_addr.into(),
                    passkey: param.ble_security.key_notif.passkey,
                },
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_PASSKEY_REQ_EVT => Self::PasskeyRequest,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_OOB_REQ_EVT => Self::OOBRequest {
                    oob_c: &param.ble_security.oob_data.oob_c,
                    oob_r: &param.ble_security.oob_data.oob_r,
                },
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_LOCAL_IR_EVT => Self::LocalIR,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_LOCAL_ER_EVT => Self::LocalER,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_NC_REQ_EVT => Self::NumericComparisonRequest,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT => {
                    Self::AdvertisingStopped(param.adv_stop_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT => {
                    Self::ScanStopped(param.scan_stop_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT => {
                    Self::StaticRandomAddressConfigured(
                        param.set_rand_addr_cmpl.status.try_into().unwrap(),
                    )
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT => {
                    Self::ConnectionParamsConfigured {
                        addr: param.update_conn_params.bda.into(),
                        status: param.update_conn_params.status.try_into().unwrap(),
                        min_int_ms: param.update_conn_params.min_int as u32 * 125 / 100,
                        max_int_ms: param.update_conn_params.max_int as u32 * 125 / 100,
                        latency_ms: param.update_conn_params.latency as u32 * 125 / 100,
                        conn_int: param.update_conn_params.conn_int,
                        timeout_ms: param.update_conn_params.timeout as u32 * 10,
                    }
                }
                #[cfg(esp_idf_version_major = "4")]
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT => {
                    Self::PacketLengthConfigured {
                        status: param.pkt_data_lenth_cmpl.status.try_into().unwrap(),
                        rx_len: param.pkt_data_lenth_cmpl.params.rx_len,
                        tx_len: param.pkt_data_lenth_cmpl.params.tx_len,
                    }
                }
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT => {
                    Self::PacketLengthConfigured {
                        status: param.pkt_data_length_cmpl.status.try_into().unwrap(),
                        rx_len: param.pkt_data_length_cmpl.params.rx_len,
                        tx_len: param.pkt_data_length_cmpl.params.tx_len,
                    }
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT => {
                    Self::LocalPrivacyConfigured(
                        param.local_privacy_cmpl.status.try_into().unwrap(),
                    )
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT => {
                    Self::DeviceBondRemoved {
                        bd_addr: param.remove_bond_dev_cmpl.bd_addr.into(),
                        status: param.remove_bond_dev_cmpl.status.try_into().unwrap(),
                    }
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT => {
                    Self::DeviceBondCleared(param.clear_bond_dev_cmpl.status.try_into().unwrap())
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT => {
                    Self::DeviceBond(param.get_bond_dev_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT => {
                    Self::ReadRssiConfigured {
                        bd_addr: param.read_rssi_cmpl.remote_addr.into(),
                        rssdi: param.read_rssi_cmpl.rssi,
                        status: param.read_rssi_cmpl.status.try_into().unwrap(),
                    }
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT => {
                    Self::WhitelistUpdated {
                        #[cfg(esp_idf_version_major = "4")]
                        wl_operation: param.update_whitelist_cmpl.wl_opration,
                        #[cfg(not(esp_idf_version_major = "4"))]
                        wl_operation: param.update_whitelist_cmpl.wl_operation,
                        status: param.update_whitelist_cmpl.status.try_into().unwrap(),
                    }
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT => {
                    Self::DuplicateListUpdated(param.update_duplicate_exceptional_list_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_CHANNELS_EVT => {
                    Self::ChannelsConfigured(param.ble_set_channels.stat.try_into().unwrap())
                }
                _ => Self::Other {
                    raw_event: event,
                    raw_data: EventRawData(param),
                },
            }
        }
    }
}

pub struct EspBleGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspBleGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    pub fn new(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_ble_gap_register_callback(Some(Self::event_handler)) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut(BleGapEvent) + Send + 'static,
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
        F: FnMut(BleGapEvent) + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    pub fn set_device_name(&self, device_name: &str) -> Result<(), EspError> {
        let device_name = to_cstring_arg(device_name)?;

        esp!(unsafe { esp_ble_gap_set_device_name(device_name.as_ptr()) })
    }

    pub fn set_device_name_cstr(&self, device_name: &CStr) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gap_set_device_name(device_name.as_ptr()) })
    }

    pub fn set_security_conf(&self, conf: &SecurityConfiguration) -> Result<(), EspError> {
        fn set<T>(param: esp_ble_sm_param_t, value: &T) -> Result<(), EspError> {
            esp!(unsafe {
                esp_ble_gap_set_security_param(
                    param,
                    value as *const _ as *const core::ffi::c_void as *mut _,
                    core::mem::size_of::<T>() as _,
                )
            })
        }

        set(
            esp_ble_sm_param_t_ESP_BLE_SM_IOCAP_MODE,
            &conf.io_capabilities,
        )?;

        if let Some(initiator_key) = &conf.initiator_key {
            set(esp_ble_sm_param_t_ESP_BLE_SM_SET_INIT_KEY, initiator_key)?;
        }

        if let Some(responder_key) = &conf.responder_key {
            set(esp_ble_sm_param_t_ESP_BLE_SM_SET_RSP_KEY, &responder_key)?;
        }

        if let Some(max_key_size) = &conf.max_key_size {
            set(esp_ble_sm_param_t_ESP_BLE_SM_MAX_KEY_SIZE, &max_key_size)?;
        }

        if let Some(min_key_size) = &conf.min_key_size {
            set(esp_ble_sm_param_t_ESP_BLE_SM_MIN_KEY_SIZE, &min_key_size)?;
        }

        if let Some(passkey) = &conf.static_passkey {
            set(
                esp_ble_sm_param_t_ESP_BLE_SM_SET_STATIC_PASSKEY,
                &passkey.to_ne_bytes(),
            )?;
        }

        set(
            esp_ble_sm_param_t_ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
            &conf.only_accept_specified_auth,
        )?;
        set(esp_ble_sm_param_t_ESP_BLE_SM_OOB_SUPPORT, &conf.enable_oob)?;

        Ok(())
    }

    pub fn set_adv_conf(&self, conf: &AdvConfiguration) -> Result<(), EspError> {
        let adv_data = conf.into();

        esp!(unsafe {
            esp_ble_gap_config_adv_data(&adv_data as *const esp_ble_adv_data_t as *mut _)
        })
    }

    pub fn set_raw_adv_conf(&self, conf: &[u8]) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gap_config_adv_data_raw(conf.as_ptr() as *mut _, conf.len() as _,) })
    }

    pub fn set_raw_scan_rsp_conf(&self, conf: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gap_config_scan_rsp_data_raw(conf.as_ptr() as *mut _, conf.len() as _)
        })
    }

    pub fn set_encryption(&self, addr: BdAddr, encryption: BleEncryption) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_set_encryption(&addr.0 as *const _ as *mut _, encryption as u32) })
    }

    pub fn start_advertising(&self) -> Result<(), EspError> {
        let mut adv_param: esp_ble_adv_params_t = esp_ble_adv_params_t {
            // TODO
            adv_int_min: 0x20,
            adv_int_max: 0x40,
            adv_type: 0x00,      // ADV_TYPE_IND,
            own_addr_type: 0x00, // BLE_ADDR_TYPE_PUBLIC,
            peer_addr: [0; 6],
            peer_addr_type: 0x00,    // BLE_ADDR_TYPE_PUBLIC,
            channel_map: 0x07,       // ADV_CHNL_ALL,
            adv_filter_policy: 0x00, // ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };

        esp!(unsafe { esp_ble_gap_start_advertising(&mut adv_param) })
    }

    pub fn stop_advertising(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gap_stop_advertising() })
    }

    pub fn set_conn_params_conf(
        &self,
        addr: BdAddr,
        min_int_ms: u32,
        max_int_ms: u32,
        latency_ms: u32,
        timeout_ms: u32,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gap_update_conn_params(&esp_ble_conn_update_params_t {
                min_int: (min_int_ms * 100 / 125) as _,
                max_int: (max_int_ms * 100 / 125) as _,
                latency: (latency_ms / 10) as _,
                timeout: (timeout_ms / 10) as _,
                bda: addr.0,
            } as *const _ as *mut _)
        })
    }

    unsafe extern "C" fn event_handler(
        event: esp_gap_ble_cb_event_t,
        param: *mut esp_ble_gap_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = BleGapEvent::from((event, param));

        debug!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }
}

impl<'d, M, T> Drop for EspBleGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        esp!(unsafe { esp_ble_gap_register_callback(None) }).unwrap();

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T> Send for EspBleGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T> Sync for EspBleGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

static SINGLETON: BtSingleton<BleGapEvent, ()> = BtSingleton::new(());
