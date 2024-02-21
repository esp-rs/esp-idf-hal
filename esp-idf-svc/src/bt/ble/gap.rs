use core::ops::BitOr;
use core::{
    borrow::Borrow,
    fmt::{self, Debug},
    marker::PhantomData,
};

use crate::sys::*;

use log::{debug, info};

use crate::{
    bt::{BdAddr, BleEnabled, BtCallback, BtDriver, BtStatus, BtUuid},
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
pub struct Configuration<'a> {
    pub set_scan_rsp: bool,
    pub include_name: bool,
    pub include_txpower: bool,
    pub min_interval: i32,
    pub max_interval: i32,
    pub manufacturer: Option<&'a str>,
    pub service: Option<&'a str>,
    pub service_uuid: Option<BtUuid>,
    pub appearance: AppearanceCategory,
    pub flag: u8,
}

impl<'a> Default for Configuration<'a> {
    fn default() -> Self {
        Self {
            set_scan_rsp: false,
            include_name: false,
            include_txpower: false,
            min_interval: 0,
            max_interval: 0,
            manufacturer: None,
            service: None,
            service_uuid: None,
            appearance: AppearanceCategory::Unknown,
            flag: ESP_BLE_ADV_FLAG_NON_LIMIT_DISC as _,
        }
    }
}

impl<'a> From<&'a Configuration<'a>> for esp_ble_adv_data_t {
    fn from(data: &'a Configuration<'a>) -> Self {
        Self {
            set_scan_rsp: data.set_scan_rsp,
            include_name: data.include_name,
            include_txpower: data.include_txpower,
            min_interval: data.min_interval,
            max_interval: data.max_interval,
            manufacturer_len: data.manufacturer.as_ref().map_or(0, |m| m.len()) as _,
            p_manufacturer_data: data
                .manufacturer
                .map_or(core::ptr::null_mut(), |s| s.as_ptr() as _),
            service_data_len: data.service.as_ref().map_or(0, |s| s.len()) as _,
            p_service_data: data
                .service
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

#[derive(Debug)]
pub enum GapEvent<'a> {
    AdvertisingDatasetComplete(BtStatus),
    ScanResponseDatasetComplete(BtStatus),
    ScanParameterDatasetComplete(BtStatus),
    ScanResult(esp_ble_gap_cb_param_t_ble_scan_result_evt_param),
    RawAdvertisingDatasetComplete(BtStatus),
    RawScanResponseDatasetComplete(BtStatus),
    AdvertisingStartComplete(BtStatus),
    ScanStartComplete(BtStatus),
    AuthenticationComplete {
        bd_addr: BdAddr,
        status: BtStatus,
        device_name: &'a str,
    },
    Key(esp_ble_sec_t),
    SecurityRequest(esp_ble_sec_t),
    PasskeyNotification(esp_ble_sec_t),
    PasskeyRequest(esp_ble_sec_t),
    OOBRequest,
    LocalIR,
    LocalER,
    NumericComparisonRequest(esp_ble_sec_t),
    AdvertisingStopComplete(BtStatus),
    ScanStopComplete(BtStatus),
    SetStaticRandomAddressComplete(BtStatus),
    UpdateConnectionParamsComplete(esp_ble_gap_cb_param_t_ble_update_conn_params_evt_param),
    SetPacketLengthComplete(esp_ble_gap_cb_param_t_ble_pkt_data_length_cmpl_evt_param),
    SetLocalPrivacy(BtStatus),
    RemoveDeviceBondComplete {
        bd_addr: BdAddr,
        status: BtStatus,
    },
    ClearDeviceBondComplete(BtStatus),
    GetDeviceBondComplete(esp_ble_gap_cb_param_t_ble_get_bond_dev_cmpl_evt_param),
    ReadRssiComplete {
        bd_addr: BdAddr,
        rssdi: i8,
        status: BtStatus,
    },
    UpdateWhitelistComplete(esp_ble_gap_cb_param_t_ble_update_whitelist_cmpl_evt_param),
    UpdateDuplicateListComplete(
        esp_ble_gap_cb_param_t_ble_update_duplicate_exceptional_list_cmpl_evt_param,
    ),
    SetChannelsComplete(BtStatus),
    // BLE 5.0
    ReadFeaturesComplete(esp_ble_gap_cb_param_t_ble_read_phy_cmpl_evt_param),
    SetPreferredDefaultPhyComplete(BtStatus),
    SetPreferredPhyComplete(BtStatus),
    ExtendedAdvertisingSetRandomAddressComplete(BtStatus),
    ExtendedAdvertisingSetParametersComplete(BtStatus),
    ExtendedAdvertisingDataSetComplete(BtStatus),
    ExtendedAdvertisingScanResponseComplete(BtStatus),
    ExtendedAdvertisingStartComplete(BtStatus),
    ExtendedAdvertisingStopComplete(BtStatus),
    ExtendedAdvertisingSetRemoveComplete(BtStatus),
    ExtendedAdvertisingSetClearComplete(BtStatus),
    PeriodicAdvertisingSetParametersComplete(BtStatus),
    PeriodicAdvertisingDataSetComplete(BtStatus),
    PeriodicAdvertisingStartComplete(BtStatus),
    PeriodicAdvertisingStopComplete(BtStatus),
    PeriodicAdvertisingCreateSyncComplete(BtStatus),
    PeriodicAdvertisingCancelSyncComplete(BtStatus),
    PeriodicAdvertisingTerminateSyncComplete(BtStatus),
    PeriodicAdvertisingAddDeviceListComplete(BtStatus),
    PeriodicAdvertisingRemoveDeviceListComplete(BtStatus),
    PeriodicAdvertisingClearDeviceListComplete(BtStatus),
    ExtendedAdvertisingScanParametersComplete(BtStatus),
    ExtendedAdvertisingScanStartComplete(BtStatus),
    ExtendedAdvertisingScanStopComplete(BtStatus),
    ExtendedAdvertisingPreferExtendedConnectionParamsComplete(BtStatus),
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
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_gap_ble_cb_event_t, &'a esp_ble_gap_cb_param_t)> for GapEvent<'a> {
    fn from(value: (esp_gap_ble_cb_event_t, &'a esp_ble_gap_cb_param_t)) -> Self {
        let (evt, param) = value;

        unsafe {
            match evt {
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT => {
                    Self::AdvertisingDatasetComplete(param.adv_data_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT => {
                    Self::ScanResponseDatasetComplete(param.scan_rsp_data_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT => {
                    Self::ScanParameterDatasetComplete(param.scan_param_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RESULT_EVT => {
                    Self::ScanResult(param.scan_rst)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT => {
                    Self::RawAdvertisingDatasetComplete(param.adv_data_raw_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT => {
                    Self::RawScanResponseDatasetComplete(param.scan_rsp_data_raw_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_START_COMPLETE_EVT => {
                    Self::AdvertisingStartComplete(param.adv_start_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_START_COMPLETE_EVT => {
                    Self::ScanStartComplete(param.scan_start_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_AUTH_CMPL_EVT => {
                    Self::AuthenticationComplete(param.ble_security)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_KEY_EVT => Self::Key(param.ble_security),
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SEC_REQ_EVT => {
                    Self::SecurityRequest(param.ble_security)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_PASSKEY_NOTIF_EVT => {
                    Self::PasskeyNotification(param.ble_security)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_PASSKEY_REQ_EVT => {
                    Self::PasskeyRequest(param.ble_security)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_OOB_REQ_EVT => Self::OOBRequest,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_LOCAL_IR_EVT => Self::LocalIR,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_LOCAL_ER_EVT => Self::LocalER,
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_NC_REQ_EVT => {
                    Self::NumericComparisonRequest(param.ble_security)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT => {
                    Self::AdvertisingStopComplete(param.adv_stop_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT => {
                    Self::ScanStopComplete(param.scan_stop_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT => {
                    Self::SetStaticRandomAddressComplete(param.set_rand_addr_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT => {
                    Self::UpdateConnectionParamsComplete(param.update_conn_params)
                }
                #[cfg(esp_idf_version_major = "4")]
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT => {
                    Self::SetPacketLengthComplete(param.pkt_data_lenth_cmpl)
                }
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT => {
                    Self::SetPacketLengthComplete(param.pkt_data_length_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT => {
                    Self::SetLocalPrivacy(param.local_privacy_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT => {
                    Self::RemoveDeviceBondComplete(param.remove_bond_dev_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT => {
                    Self::ClearDeviceBondComplete(param.clear_bond_dev_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT => {
                    Self::GetDeviceBondComplete(param.get_bond_dev_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT => {
                    Self::ReadRssiComplete(param.read_rssi_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT => {
                    Self::UpdateWhitelistComplete(param.update_whitelist_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_UPDATE_DUPLICATE_EXCEPTIONAL_LIST_COMPLETE_EVT => {
                    Self::UpdateDuplicateListComplete(param.update_duplicate_exceptional_list_cmpl)
                }
                esp_gap_ble_cb_event_t_ESP_GAP_BLE_SET_CHANNELS_EVT => {
                    Self::SetChannelsComplete(param.ble_set_channels)
                }
                _ => {
                    log::warn!("Unhandled event {:?}", evt);
                    panic!("Unhandled event {:?}", evt)
                }
            }
        }
    }
}

pub struct EspGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    pub fn new<F>(driver: T) -> Result<Self, EspError> {
        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn initialize<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: Fn(GapEvent) + Send + 'static,
    {
        self.internal_initialize(events_cb)
    }

    /// # Safety
    ///
    /// This method - in contrast to method `initialize` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow 
    /// - in the closure - variables that live on the stack - or more generally - in the same 
    /// scope where the service is created.
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
    pub unsafe fn initialize_nonstatic<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: Fn(GapEvent) + Send + 'd,
    {
        self.internal_initialize(events_cb)
    }

    fn internal_initialize<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: Fn(GapEvent) + Send + 'd,
    {
        CALLBACK.set(events_cb)?;

        esp!(unsafe { esp_ble_gap_register_callback(Some(Self::event_handler)) })
    }

    pub fn set_device_name(&mut self, device_name: &str) -> Result<(), EspError> {
        let device_name = to_cstring_arg(device_name)?;

        esp!(unsafe { esp_ble_gap_set_device_name(device_name.as_ptr()) })
    }

    pub fn set_security_conf(&mut self, conf: &SecurityConfiguration) -> Result<(), EspError> {
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

    pub fn configure_gatt_encryption(
        mut remote_bda: [u8; ESP_BD_ADDR_LEN as _],
        encryption_config: BleEncryption,
    ) {
        esp!(unsafe { esp_ble_set_encryption(remote_bda.as_mut_ptr(), encryption_config as u32) })
            .expect("Unable to set security level");
    }

    pub fn set_conf(&mut self, conf: &Configuration) -> Result<(), EspError> {
        let adv_data = conf.into();

        esp!(unsafe {
            esp_ble_gap_config_adv_data(&adv_data as *const esp_ble_adv_data_t as *mut _)
        })
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        info!("start_advertise enter");

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

    pub fn stop(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gap_stop_advertising() })
    }

    unsafe extern "C" fn event_handler(
        event: esp_gap_ble_cb_event_t,
        param: *mut esp_ble_gap_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = GapEvent::from((event, param));

        info!("Got GAP event {{ {:#?} }}", event);

        CALLBACK.call(event);
    }
}

impl<'d, M, T> Drop for EspGap<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    fn drop(&mut self) {
        esp!(unsafe { esp_ble_gap_register_callback(None) }).unwrap();

        CALLBACK.clear().unwrap();
    }
}

static CALLBACK: BtCallback<GapEvent, ()> = BtCallback::new(());
