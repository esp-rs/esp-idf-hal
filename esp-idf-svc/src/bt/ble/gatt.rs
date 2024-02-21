use crate::sys::{
    esp_attr_control_t, esp_attr_value_t, esp_gatt_char_prop_t, esp_gatt_perm_t, ESP_GATT_AUTO_RSP,
    ESP_GATT_RSP_BY_APP,
};

use crate::bt::BtUuid;

pub mod server;

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

pub struct AttributeValue<const S: usize> {
    len: usize,
    value: [u8; S],
}

impl<const S: usize> From<&AttributeValue<S>> for esp_attr_value_t {
    fn from(val: &AttributeValue<S>) -> Self {
        Self {
            attr_max_len: S as _,
            attr_len: val.len as _,
            attr_value: val.value.as_ptr() as _,
        }
    }
}

impl<const S: usize> Default for AttributeValue<S> {
    fn default() -> Self {
        Self {
            len: S,
            value: [0; S],
        }
    }
}

impl<const S: usize> AttributeValue<S> {
    pub fn new_with_value(value: &[u8]) -> Self {
        let actual_len = core::cmp::min(value.len(), S);
        let mut val = Self {
            len: S,
            value: [0; S],
        };
        val.value[0..actual_len].copy_from_slice(&value[0..actual_len]);
        val
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum AutoResponse {
    ByApp,
    ByGatt,
}

impl From<AutoResponse> for esp_attr_control_t {
    fn from(auto: AutoResponse) -> Self {
        Self {
            auto_rsp: match auto {
                AutoResponse::ByApp => ESP_GATT_RSP_BY_APP,
                AutoResponse::ByGatt => ESP_GATT_AUTO_RSP,
            } as _,
        }
    }
}

pub struct GattCharacteristic<const S: usize> {
    pub(crate) uuid: BtUuid,
    pub(crate) permissions: esp_gatt_perm_t,
    pub(crate) property: esp_gatt_char_prop_t,
    pub(crate) value: AttributeValue<S>,
    pub(crate) auto_rsp: AutoResponse,
}

impl<const S: usize> GattCharacteristic<S> {
    pub fn new(
        uuid: BtUuid,
        permissions: esp_gatt_perm_t,
        property: esp_gatt_char_prop_t,
        value: AttributeValue<S>,
        auto_rsp: AutoResponse,
    ) -> Self {
        Self {
            uuid,
            permissions,
            property,
            value,
            auto_rsp,
        }
    }
}

pub struct GattDescriptor {
    pub(crate) uuid: BtUuid,
    pub(crate) permissions: esp_gatt_perm_t,
}

impl GattDescriptor {
    pub fn new(uuid: BtUuid, permissions: esp_gatt_perm_t) -> Self {
        Self { uuid, permissions }
    }
}
