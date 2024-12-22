use enumset::{EnumSet, EnumSetType};
use num_enum::TryFromPrimitive;

use crate::bt::BtUuid;
use crate::sys::*;

pub mod server;

pub type GattInterface = u8;
pub type Handle = u16;

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum GattStatus {
    Ok = esp_gatt_status_t_ESP_GATT_OK,
    InvalidHandle = esp_gatt_status_t_ESP_GATT_INVALID_HANDLE,
    ReadNotPermitted = esp_gatt_status_t_ESP_GATT_READ_NOT_PERMIT,
    WriteNotPermitted = esp_gatt_status_t_ESP_GATT_WRITE_NOT_PERMIT,
    InvalidPdu = esp_gatt_status_t_ESP_GATT_INVALID_PDU,
    InsufficientAuthentication = esp_gatt_status_t_ESP_GATT_INSUF_AUTHENTICATION,
    ReqNotSupported = esp_gatt_status_t_ESP_GATT_REQ_NOT_SUPPORTED,
    InvalidOffset = esp_gatt_status_t_ESP_GATT_INVALID_OFFSET,
    InsufficientAuthorization = esp_gatt_status_t_ESP_GATT_INSUF_AUTHORIZATION,
    PrepareQueueFull = esp_gatt_status_t_ESP_GATT_PREPARE_Q_FULL,
    NotFound = esp_gatt_status_t_ESP_GATT_NOT_FOUND,
    NotLong = esp_gatt_status_t_ESP_GATT_NOT_LONG,
    InsufficientKeySize = esp_gatt_status_t_ESP_GATT_INSUF_KEY_SIZE,
    InvalidAttrLen = esp_gatt_status_t_ESP_GATT_INVALID_ATTR_LEN,
    ErrUnlikely = esp_gatt_status_t_ESP_GATT_ERR_UNLIKELY,
    InsufficientEncryption = esp_gatt_status_t_ESP_GATT_INSUF_ENCRYPTION,
    UsupportedGroupType = esp_gatt_status_t_ESP_GATT_UNSUPPORT_GRP_TYPE,
    InsufficientResource = esp_gatt_status_t_ESP_GATT_INSUF_RESOURCE,
    NoResources = esp_gatt_status_t_ESP_GATT_NO_RESOURCES,
    InternalError = esp_gatt_status_t_ESP_GATT_INTERNAL_ERROR,
    WrongState = esp_gatt_status_t_ESP_GATT_WRONG_STATE,
    DbFull = esp_gatt_status_t_ESP_GATT_DB_FULL,
    Busy = esp_gatt_status_t_ESP_GATT_BUSY,
    Error = esp_gatt_status_t_ESP_GATT_ERROR,
    CmdStarted = esp_gatt_status_t_ESP_GATT_CMD_STARTED,
    IllegalParam = esp_gatt_status_t_ESP_GATT_ILLEGAL_PARAMETER,
    Pending = esp_gatt_status_t_ESP_GATT_PENDING,
    AuthenticationFailed = esp_gatt_status_t_ESP_GATT_AUTH_FAIL,
    More = esp_gatt_status_t_ESP_GATT_MORE,
    InvalidCfg = esp_gatt_status_t_ESP_GATT_INVALID_CFG,
    ServiceStarted = esp_gatt_status_t_ESP_GATT_SERVICE_STARTED,
    #[cfg(all(esp_idf_version_major = "4", not(esp_idf_version_full = "4.4.8")))]
    EncryptedNoMitm = esp_gatt_status_t_ESP_GATT_ENCRYPED_NO_MITM,
    #[cfg(any(not(esp_idf_version_major = "4"), esp_idf_version_full = "4.4.8"))]
    EncryptedNoMitm = esp_gatt_status_t_ESP_GATT_ENCRYPTED_NO_MITM,
    NotEncrypted = esp_gatt_status_t_ESP_GATT_NOT_ENCRYPTED,
    Congested = esp_gatt_status_t_ESP_GATT_CONGESTED,
    DuplicateReg = esp_gatt_status_t_ESP_GATT_DUP_REG,
    AlreadyOpen = esp_gatt_status_t_ESP_GATT_ALREADY_OPEN,
    Cancel = esp_gatt_status_t_ESP_GATT_CANCEL,
    StackRsp = esp_gatt_status_t_ESP_GATT_STACK_RSP,
    AppRsp = esp_gatt_status_t_ESP_GATT_APP_RSP,
    UnknownErr = esp_gatt_status_t_ESP_GATT_UNKNOWN_ERROR,
    CccCfgErr = esp_gatt_status_t_ESP_GATT_CCC_CFG_ERR,
    PrcInProgress = esp_gatt_status_t_ESP_GATT_PRC_IN_PROGRESS,
    OutOfRange = esp_gatt_status_t_ESP_GATT_OUT_OF_RANGE,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum GattConnReason {
    /// Unknown
    Unknown = esp_gatt_conn_reason_t_ESP_GATT_CONN_UNKNOWN,
    /// General L2cap failure
    L2cFailure = esp_gatt_conn_reason_t_ESP_GATT_CONN_L2C_FAILURE,
    /// Connection timeout
    Timeout = esp_gatt_conn_reason_t_ESP_GATT_CONN_TIMEOUT,
    /// Connection terminated by peer user
    TerminatedByPeer = esp_gatt_conn_reason_t_ESP_GATT_CONN_TERMINATE_PEER_USER,
    /// Connection terminated by local host
    TerminatedByLocalHost = esp_gatt_conn_reason_t_ESP_GATT_CONN_TERMINATE_LOCAL_HOST,
    /// Connection failed to establish
    FailedToEstablish = esp_gatt_conn_reason_t_ESP_GATT_CONN_FAIL_ESTABLISH,
    /// Connection failed for LMP response timeout
    LmpTimeout = esp_gatt_conn_reason_t_ESP_GATT_CONN_LMP_TIMEOUT,
    /// L2CAP connection cancelled
    L2capConnCancelled = esp_gatt_conn_reason_t_ESP_GATT_CONN_CONN_CANCEL,
    /// No connection to cancel
    NoConnection = esp_gatt_conn_reason_t_ESP_GATT_CONN_NONE,
}

#[derive(Clone, Debug)]
pub struct GattConnParams {
    /// Connection interval
    pub interval_ms: u32,
    /// Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3
    pub latency_ms: u32,
    /// Supervision timeout for the LE Link. Range: 0x000A to 0x0C80.
    /// Mandatory Range: 0x000A to 0x0C80 Time = N * 10 msec
    /// Time Range: 100 msec to 32 seconds
    pub timeout_ms: u32,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GattId {
    /// UUID
    pub uuid: BtUuid,
    /// Instance id
    pub inst_id: u8,
}

impl From<esp_gatt_id_t> for GattId {
    fn from(id: esp_gatt_id_t) -> Self {
        Self {
            uuid: BtUuid::from(id.uuid),
            inst_id: id.inst_id,
        }
    }
}

impl From<GattId> for esp_gatt_id_t {
    fn from(id: GattId) -> Self {
        Self {
            uuid: id.uuid.into(),
            inst_id: id.inst_id,
        }
    }
}

// TODO: Maybe rather model as GattServiceId(esp_gatt_srvc_id) + repr(transparent)?
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GattServiceId {
    // Gatt id, includes uuid and instance
    pub id: GattId,
    /// This service is primary or not
    pub is_primary: bool,
}

impl From<esp_gatt_srvc_id_t> for GattServiceId {
    fn from(id: esp_gatt_srvc_id_t) -> Self {
        Self {
            id: GattId::from(id.id),
            is_primary: id.is_primary,
        }
    }
}

impl From<GattServiceId> for esp_gatt_srvc_id_t {
    fn from(id: GattServiceId) -> Self {
        Self {
            id: id.id.into(),
            is_primary: id.is_primary,
        }
    }
}

#[repr(u16)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ServiceUuid {
    GenericAccess = 0x1800,
    GenericAttribute,
    ImmediateAlert,
    LinkLoss,
    TxPower,
    CurrentTime,
    ReferenceTimeUpdate,
    NextDSTChange = 0x1807,
    Glucose,
    HealthThermometer,
    DeviceInformation,
    HeartRate = 0x180D,
    PhoneAlertStatus,
    Battery,
    BloodPressure,
    AlertNotification,
    HumanInterfaceDevice,
    ScanParameters,
    RunningSpeedAndCadence,
    AutomationIO,
    CyclingSpeedAndCadence,
    CyclingPower = 0x1818,
    LocationAndNavigation,
    EnvironmentalSensing,
    BodyComposition,
    UserData,
    WeightScale,
    BondManagement,
    ContinuousGlucoseMonitoring,
    InternetProtocolSupport,
    IndoorPositioning,
    PulseOximeter,
    HTTPProxy,
    TransportDiscovery,
    ObjectTransfer,
    FitnessMachine,
    MeshProvisioning,
    MeshProxy,
    ReconnectionConfiguration,
    InsulinDelivery = 0x183A,
    BinarySensor,
    EmergencyConfiguration,
    PhysicalActivityMonitor = 0x183E,
    AudioInputControl = 0x1843,
    VolumeControl,
    VolumeOffsetControl,
    CoordinatedSetIdentification,
    DeviceTime,
    MediaControl,
    GenericMediaControl,
    ConstantToneExtension,
    TelephoneBearer,
    GenericTelephoneBearer,
    MicrophoneControl,
    AudioStreamControl,
    BroadcastAudioScan,
    PublishedAudioCapabilities,
    BasicAudioAnnouncement,
    BroadcastAudioAnnouncement,
    CommonAudio,
    HearingAccess,
    TMAS,
    PublicBroadcastAnnouncement,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum AutoResponse {
    ByApp = ESP_GATT_RSP_BY_APP as _,
    ByGatt = ESP_GATT_AUTO_RSP as _,
}

#[derive(Debug, EnumSetType, TryFromPrimitive)]
#[enumset(repr = "u16")]
#[repr(u16)]
pub enum Permission {
    Read = 0,              // ESP_GATT_PERM_READ
    ReadEncrypted = 1,     // ESP_GATT_PERM_READ_ENCRYPTED
    ReadEncryptedMitm = 2, // ESP_GATT_PERM_READ_ENC_MITM
    Unknown = 3,
    Write = 4,               // ESP_GATT_PERM_WRITE
    WriteEncrypted = 5,      // ESP_GATT_PERM_WRITE_ENCRYPTED
    WriteEncryptedMitm = 6,  // ESP_GATT_PERM_WRITE_ENC_MITM
    WriteSigned = 7,         // ESP_GATT_PERM_WRITE_SIGNED
    WriteSiognedMitm = 8,    // ESP_GATT_PERM_WRITE_SIGNED_MITM
    ReadAuthorization = 9,   // ESP_GATT_PERM_READ_AUTHORIZATION
    WriteAuthorization = 10, // ESP_GATT_PERM_WRITE_AUTHORIZATION
}

#[derive(Debug, EnumSetType, TryFromPrimitive)]
#[enumset(repr = "u8")]
#[repr(u8)]
pub enum Property {
    Broadcast = 0,       // ESP_GATT_CHAR_PROP_BIT_BROADCAST
    Read = 1,            // ESP_GATT_CHAR_PROP_BIT_READ
    WriteNoResponse = 2, // ESP_GATT_CHAR_PROP_BIT_WRITE_NR
    Write = 3,           // ESP_GATT_CHAR_PROP_BIT_WRITE
    Notify = 4,          // ESP_GATT_CHAR_PROP_BIT_NOTIFY
    Indicate = 5,        // ESP_GATT_CHAR_PROP_BIT_INDICATE
    Auth = 6,            // ESP_GATT_CHAR_PROP_BIT_AUTH
    ExtendedProps = 7,   // ESP_GATT_CHAR_PROP_BIT_EXT_PROP
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct GattCharacteristic {
    pub uuid: BtUuid,
    pub permissions: EnumSet<Permission>,
    pub properties: EnumSet<Property>,
    pub max_len: usize,
    pub auto_rsp: AutoResponse,
}

impl GattCharacteristic {
    pub const fn new(
        uuid: BtUuid,
        permissions: EnumSet<Permission>,
        properties: EnumSet<Property>,
        max_len: usize,
        auto_rsp: AutoResponse,
    ) -> Self {
        Self {
            uuid,
            permissions,
            properties,
            max_len,
            auto_rsp,
        }
    }
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct GattDescriptor {
    pub uuid: BtUuid,
    pub permissions: EnumSet<Permission>,
}

impl GattDescriptor {
    pub const fn new(uuid: BtUuid, permissions: EnumSet<Permission>) -> Self {
        Self { uuid, permissions }
    }
}

#[derive(Clone)]
#[repr(transparent)]
pub struct GattResponse(esp_gatt_rsp_t);

impl GattResponse {
    #[inline(always)]
    pub const fn new() -> Self {
        Self(esp_gatt_rsp_t {
            attr_value: esp_gatt_value_t {
                len: 0,
                value: [0; ESP_GATT_MAX_ATTR_LEN as _],
                handle: 0,
                offset: 0,
                auth_req: 0,
            },
        })
    }

    pub fn attr_handle(&mut self, handle: Handle) -> &mut Self {
        self.0.attr_value.handle = handle;
        self
    }

    pub fn auth_req(&mut self, auth_req: u8) -> &mut Self {
        self.0.attr_value.auth_req = auth_req;
        self
    }

    pub fn offset(&mut self, offset: u16) -> &mut Self {
        self.0.attr_value.offset = offset;
        self
    }

    pub fn value(&mut self, value: &[u8]) -> Result<&mut Self, EspError> {
        if value.len() > 600 {
            return Err(EspError::from_infallible::<{ ESP_ERR_INVALID_ARG }>());
        }

        self.0.attr_value.len = value.len() as u16;
        unsafe { self.0.attr_value.value[..value.len()].copy_from_slice(value) };

        Ok(self)
    }
}

impl Default for GattResponse {
    fn default() -> Self {
        Self::new()
    }
}
