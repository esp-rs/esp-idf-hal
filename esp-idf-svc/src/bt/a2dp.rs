#![allow(non_upper_case_globals)]

use core::borrow::Borrow;
use core::convert::TryInto;
use core::fmt::{self, Debug};
use core::marker::PhantomData;
#[cfg(esp_idf_bt_a2dp_use_external_codec)]
use core::mem::MaybeUninit;
#[cfg(esp_idf_bt_a2dp_use_external_codec)]
use core::ptr::NonNull;

use ::log::{info, trace};

use num_enum::TryFromPrimitive;

use crate::bt::{BtClassicEnabled, BtDriver};
use crate::sys::*;

use super::{BdAddr, BtSingleton};

pub trait A2dpMode: Send {
    fn sink() -> bool;
    fn source() -> bool;
}

pub trait SinkEnabled: A2dpMode {}
pub trait SourceEnabled: A2dpMode {}

pub struct Sink;
impl SinkEnabled for Sink {}

impl A2dpMode for Sink {
    fn sink() -> bool {
        true
    }

    fn source() -> bool {
        false
    }
}

pub struct Source;
impl SourceEnabled for Source {}

impl A2dpMode for Source {
    fn sink() -> bool {
        false
    }

    fn source() -> bool {
        true
    }
}

pub struct Duplex;
impl SinkEnabled for Duplex {}
impl SourceEnabled for Duplex {}

impl A2dpMode for Duplex {
    fn sink() -> bool {
        true
    }

    fn source() -> bool {
        true
    }
}

#[derive(Clone)]
pub enum Codec {
    Sbc([u8; 4]),
    Mpeg1_2([u8; 4]),
    Mpeg2_4([u8; 6]),
    Atrac([u8; 7]),
    Unknown,
}

impl Codec {
    /// Negotiated audio sample rate (not bitrate — the method name is kept for
    /// API stability).
    pub fn bitrate(&self) -> Option<u32> {
        match self {
            Self::Sbc(data) => {
                let oct0 = data[0];
                Some(if (oct0 & (0x01 << 6)) != 0 {
                    32000
                } else if (oct0 & (0x01 << 5)) != 0 {
                    44100
                } else if (oct0 & (0x01 << 4)) != 0 {
                    48000
                } else {
                    16000
                })
            }
            Self::Mpeg2_4(data) => {
                // AAC sample-frequency bitmap (AVDTP A2DP 4.5.2):
                // byte 1 covers 8 kHz..44.1 kHz, byte 2 high nibble covers 48..96 kHz.
                // After negotiation exactly one bit is set.
                if data[2] & 0x80 != 0 {
                    Some(48000)
                } else if data[1] & 0x01 != 0 {
                    Some(44100)
                } else if data[1] & 0x02 != 0 {
                    Some(32000)
                } else if data[1] & 0x04 != 0 {
                    Some(24000)
                } else if data[1] & 0x08 != 0 {
                    Some(22050)
                } else if data[1] & 0x10 != 0 {
                    Some(16000)
                } else if data[1] & 0x20 != 0 {
                    Some(12000)
                } else if data[1] & 0x40 != 0 {
                    Some(11025)
                } else if data[1] & 0x80 != 0 {
                    Some(8000)
                } else if data[2] & 0x40 != 0 {
                    Some(64000)
                } else if data[2] & 0x20 != 0 {
                    Some(88200)
                } else if data[2] & 0x10 != 0 {
                    Some(96000)
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    pub fn stereo(&self) -> Option<bool> {
        match self {
            Self::Sbc(data) => Some((data[0] & (0x01 << 3)) == 0),
            // AAC byte 2 bit 2 = 2-channel mode (stereo); bit 3 = 1-channel (mono).
            Self::Mpeg2_4(data) => Some((data[2] & 0x04) != 0),
            _ => None,
        }
    }

    /// Short codec name for display.
    pub const fn name(&self) -> &'static str {
        match self {
            Self::Sbc(_) => "SBC",
            Self::Mpeg1_2(_) => "MPEG-1/2 Audio",
            Self::Mpeg2_4(_) => "AAC",
            Self::Atrac(_) => "ATRAC",
            Self::Unknown => "Unknown",
        }
    }

    /// SBC capability that advertises every common parameter combination
    /// (all sample frequencies, all channel modes, all block lengths, all
    /// subbands, all allocation methods, bitpool 2..=250). Matches the
    /// in-stack default `bta_av_co_sbc_sink_caps`.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    pub fn sbc_default() -> Self {
        Self::Sbc([0xFF, 0xFF, 2, 250])
    }

    /// AAC capability advertising MPEG-2/4 AAC LC, 44.1 + 48 kHz, stereo,
    /// VBR up to 256 kbps. Matches what mainstream phones source.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    pub fn aac_default() -> Self {
        // byte 0: MPEG-2 AAC LC (bit 7) + MPEG-4 AAC LC (bit 6)
        // byte 1: 44.1 kHz (bit 0)
        // byte 2: 48 kHz (bit 7) + stereo (bit 2)
        // bytes 3..6: VBR=1, max bitrate = 256 kbps (0x3E800)
        Self::Mpeg2_4([0xC0, 0x01, 0x84, 0x83, 0xE8, 0x00])
    }

    /// Build a raw `esp_a2d_mcc_t` from this codec for passing to
    /// `esp_a2d_sink_register_stream_endpoint`. Returns `ESP_ERR_INVALID_ARG`
    /// for `Unknown` or codecs not yet implemented (M12, ATRAC).
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    pub fn to_raw_mcc(&self) -> Result<esp_a2d_mcc_t, EspError> {
        let mut mcc = MaybeUninit::<esp_a2d_mcc_t>::zeroed();
        // Safety: zeroed is a valid bit-pattern for esp_a2d_mcc_t (a u8 tag
        // plus a union of byte arrays). We then write the active variant.
        unsafe {
            let m = &mut *mcc.as_mut_ptr();
            match self {
                Self::Sbc(bytes) => {
                    m.type_ = ESP_A2D_MCT_SBC as _;
                    m.cie.sbc_info = core::mem::transmute(*bytes);
                }
                Self::Mpeg2_4(bytes) => {
                    m.type_ = ESP_A2D_MCT_M24 as _;
                    m.cie.m24_info = core::mem::transmute(*bytes);
                }
                _ => return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>()),
            }
            Ok(mcc.assume_init())
        }
    }
}

impl Debug for Codec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Sbc(data) => f.debug_tuple("Sbc").field(data).finish()?,
            Self::Mpeg1_2(data) => f.debug_tuple("Mpeg1_2").field(data).finish()?,
            Self::Mpeg2_4(data) => f.debug_tuple("Mpeg2_4").field(data).finish()?,
            Self::Atrac(data) => f.debug_tuple("Atrac").field(data).finish()?,
            Self::Unknown => write!(f, "Unknown")?,
        }

        write!(
            f,
            " / bitrate: {:?}, stereo: {:?}",
            self.bitrate(),
            self.stereo()
        )
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum ConnectionStatus {
    Disconnected = esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_DISCONNECTED,
    Connecting = esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_CONNECTING,
    Connected = esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_CONNECTED,
    Disconnecting = esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_DISCONNECTING,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum AudioStatus {
    // The C enum dropped REMOTE_SUSPEND in favour of plain SUSPEND in
    // esp-idf >= 5.2; the Rust variant name is kept stable for API users.
    #[cfg(not(esp_idf_version_at_least_5_2_0))]
    SuspendedByRemote = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND,
    #[cfg(esp_idf_version_at_least_5_2_0)]
    SuspendedByRemote = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_SUSPEND,
    #[cfg(not(esp_idf_version_at_least_5_2_0))]
    Stopped = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_STOPPED,
    Started = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_STARTED,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum MediaControlCommand {
    None = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_NONE,
    CheckSourceReady = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY,
    Start = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_START,
    // STOP was dropped in esp-idf >= 5.2.
    #[cfg(not(esp_idf_version_at_least_5_2_0))]
    Stop = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_STOP,
    Suspend = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_SUSPEND,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum MediaControlStatus {
    Success = esp_a2d_media_ctrl_ack_t_ESP_A2D_MEDIA_CTRL_ACK_SUCCESS,
    Failure = esp_a2d_media_ctrl_ack_t_ESP_A2D_MEDIA_CTRL_ACK_FAILURE,
    Busy = esp_a2d_media_ctrl_ack_t_ESP_A2D_MEDIA_CTRL_ACK_BUSY,
}

#[cfg(esp_idf_bt_a2dp_use_external_codec)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum SepRegState {
    Success = esp_a2d_sep_reg_state_t_ESP_A2D_SEP_REG_SUCCESS,
    Fail = esp_a2d_sep_reg_state_t_ESP_A2D_SEP_REG_FAIL,
    Unsupported = esp_a2d_sep_reg_state_t_ESP_A2D_SEP_REG_UNSUPPORTED,
    InvalidState = esp_a2d_sep_reg_state_t_ESP_A2D_SEP_REG_INVALID_STATE,
}

/// Owned wrapper around a Bluedroid-allocated audio buffer delivered to
/// the sink in external-codec mode. The buffer carries the raw, undecoded
/// AVDTP media payload — the application is responsible for decoding it
/// (typically via `esp_audio_codec`).
///
/// The underlying buffer is released back to Bluedroid when this value is
/// dropped, so callers can move it into a queue/channel for off-task
/// decoding without worrying about manual frees.
#[cfg(esp_idf_bt_a2dp_use_external_codec)]
pub struct A2dpAudioBuf(NonNull<esp_a2d_audio_buff_t>);

#[cfg(esp_idf_bt_a2dp_use_external_codec)]
impl A2dpAudioBuf {
    /// Number of encoded frames contained in this buffer.
    pub fn frames(&self) -> u16 {
        unsafe { (*self.0.as_ptr()).number_frame }
    }

    /// AVDTP RTP timestamp of the first frame.
    pub fn timestamp(&self) -> u32 {
        unsafe { (*self.0.as_ptr()).timestamp }
    }

    /// View of the encoded audio payload.
    pub fn data(&self) -> &[u8] {
        unsafe {
            let raw = self.0.as_ptr();
            core::slice::from_raw_parts((*raw).data, (*raw).data_len as usize)
        }
    }
}

#[cfg(esp_idf_bt_a2dp_use_external_codec)]
impl Debug for A2dpAudioBuf {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("A2dpAudioBuf")
            .field("frames", &self.frames())
            .field("timestamp", &self.timestamp())
            .field("len", &self.data().len())
            .finish()
    }
}

#[cfg(esp_idf_bt_a2dp_use_external_codec)]
impl Drop for A2dpAudioBuf {
    fn drop(&mut self) {
        unsafe { esp_a2d_audio_buff_free(self.0.as_ptr()) };
    }
}

// Safety: the buffer pointer is only ever touched through the wrapper, and
// Bluedroid hands ownership over by value to the audio_data callback.
#[cfg(esp_idf_bt_a2dp_use_external_codec)]
unsafe impl Send for A2dpAudioBuf {}

pub struct EventRawData<'a>(pub &'a esp_a2d_cb_param_t);

impl<'a> Debug for EventRawData<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("RawData").finish()
    }
}

#[derive(Debug)]
pub enum A2dpEvent<'a> {
    ConnectionState {
        bd_addr: BdAddr,
        status: ConnectionStatus,
        disconnect_abnormal: bool,
    },
    AudioState {
        bd_addr: BdAddr,
        status: AudioStatus,
    },
    AudioCodecConfigured {
        bd_addr: BdAddr,
        codec: Codec,
    },
    MediaControlAcknowledged {
        command: MediaControlCommand,
        status: MediaControlStatus,
    },
    Initialized,
    Deinitialized,
    #[cfg(not(esp_idf_version_major = "4"))]
    SinkServiceCapabilitiesConfigured(u16),
    #[cfg(not(esp_idf_version_major = "4"))]
    SinkDelaySetState {
        set: bool,
        delay: u16,
    },
    #[cfg(not(esp_idf_version_major = "4"))]
    SinkDelay(u16),
    #[cfg(not(esp_idf_version_major = "4"))]
    SourceDelay(u16),
    SinkData(&'a [u8]),
    SourceData(&'a mut [u8]),
    /// Undecoded sink audio frames delivered in external-codec mode.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    SinkAudioData(A2dpAudioBuf),
    /// Result of a prior `register_sink_endpoint` call.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    SinkEndpointRegistered {
        seid: u8,
        state: SepRegState,
    },
    Other {
        raw_event: esp_a2d_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_a2d_cb_event_t, &'a esp_a2d_cb_param_t)> for A2dpEvent<'a> {
    fn from(value: (esp_a2d_cb_event_t, &'a esp_a2d_cb_param_t)) -> Self {
        let (event, param) = value;

        unsafe {
            match event {
                esp_a2d_cb_event_t_ESP_A2D_CONNECTION_STATE_EVT => Self::ConnectionState {
                    bd_addr: param.conn_stat.remote_bda.into(),
                    status: param.conn_stat.state.try_into().unwrap(),
                    disconnect_abnormal: param.conn_stat.disc_rsn
                        != esp_a2d_disc_rsn_t_ESP_A2D_DISC_RSN_NORMAL,
                },
                esp_a2d_cb_event_t_ESP_A2D_AUDIO_STATE_EVT => Self::AudioState {
                    bd_addr: param.audio_stat.remote_bda.into(),
                    status: param.audio_stat.state.try_into().unwrap(),
                },
                esp_a2d_cb_event_t_ESP_A2D_AUDIO_CFG_EVT => Self::AudioCodecConfigured {
                    bd_addr: param.audio_cfg.remote_bda.into(),
                    codec: match param.audio_cfg.mcc.type_ as u32 {
                        // The cie.*_info union members are #[repr(C, packed)] bindgen
                        // structs whose layout is byte-identical to the corresponding
                        // [u8; N] wire form, so we transmute to expose the raw bytes.
                        ESP_A2D_MCT_SBC => {
                            Codec::Sbc(core::mem::transmute(param.audio_cfg.mcc.cie.sbc_info))
                        }
                        ESP_A2D_MCT_M12 => {
                            Codec::Mpeg1_2(core::mem::transmute(param.audio_cfg.mcc.cie.m12_info))
                        }
                        ESP_A2D_MCT_M24 => {
                            Codec::Mpeg2_4(core::mem::transmute(param.audio_cfg.mcc.cie.m24_info))
                        }
                        ESP_A2D_MCT_ATRAC => {
                            Codec::Atrac(core::mem::transmute(param.audio_cfg.mcc.cie.atrac_info))
                        }
                        _ => Codec::Unknown,
                    },
                },
                esp_a2d_cb_event_t_ESP_A2D_MEDIA_CTRL_ACK_EVT => Self::MediaControlAcknowledged {
                    command: param.media_ctrl_stat.cmd.try_into().unwrap(),
                    status: param.media_ctrl_stat.status.try_into().unwrap(),
                },
                esp_a2d_cb_event_t_ESP_A2D_PROF_STATE_EVT => {
                    if param.a2d_prof_stat.init_state == esp_a2d_init_state_t_ESP_A2D_INIT_SUCCESS {
                        Self::Initialized
                    } else {
                        Self::Deinitialized
                    }
                }
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_a2d_cb_event_t_ESP_A2D_SNK_PSC_CFG_EVT => {
                    Self::SinkServiceCapabilitiesConfigured(param.a2d_psc_cfg_stat.psc_mask)
                } // TODO
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_a2d_cb_event_t_ESP_A2D_SNK_SET_DELAY_VALUE_EVT => Self::SinkDelaySetState {
                    set: param.a2d_set_delay_value_stat.set_state
                        == esp_a2d_set_delay_value_state_t_ESP_A2D_SET_SUCCESS,
                    delay: param.a2d_set_delay_value_stat.delay_value,
                },
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_a2d_cb_event_t_ESP_A2D_SNK_GET_DELAY_VALUE_EVT => {
                    Self::SinkDelay(param.a2d_get_delay_value_stat.delay_value)
                }
                #[cfg(not(esp_idf_version_major = "4"))]
                esp_a2d_cb_event_t_ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT => {
                    Self::SourceDelay(param.a2d_report_delay_value_stat.delay_value)
                }
                #[cfg(esp_idf_bt_a2dp_use_external_codec)]
                esp_a2d_cb_event_t_ESP_A2D_SEP_REG_STATE_EVT => Self::SinkEndpointRegistered {
                    seid: param.a2d_sep_reg_stat.seid,
                    state: param.a2d_sep_reg_stat.reg_state.try_into().unwrap(),
                },
                _ => Self::Other {
                    raw_event: event,
                    raw_data: EventRawData(param),
                },
            }
        }
    }
}

pub struct EspA2dp<'d, M, T, S>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
    S: A2dpMode,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
    _s: PhantomData<S>,
}

impl<'d, M, T> EspA2dp<'d, M, T, Sink>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    pub fn new_sink(driver: T) -> Result<Self, EspError> {
        Self::new(driver)
    }

    /// Create a sink that operates in external-codec mode: Bluedroid does
    /// no in-stack decoding, and instead delivers raw AVDTP media payloads
    /// to the user via [`A2dpEvent::SinkAudioData`]. The caller must then
    /// register one or more stream endpoints with
    /// [`Self::register_sink_endpoint`] before a peer can connect.
    ///
    /// Requires `CONFIG_BT_A2DP_USE_EXTERNAL_CODEC=y` in `sdkconfig`.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    pub fn new_external_codec(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_a2d_register_callback(Some(Self::event_handler)) })?;
        esp!(unsafe {
            esp_a2d_sink_register_audio_data_callback(Some(Self::sink_audio_data_handler))
        })?;
        esp!(unsafe { esp_a2d_sink_init() })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
            _s: PhantomData,
        })
    }

    /// Register a Stream Endpoint advertising the given codec capability.
    /// `seid` must be < `CONFIG_BT_A2DP_SEP_NUM_MAX` and lower SEIDs are
    /// negotiated with higher priority. The async result is delivered
    /// later as [`A2dpEvent::SinkEndpointRegistered`].
    ///
    /// SBC must be registered for A2DP-spec compliance, so a typical
    /// AAC-preferring sink registers AAC at seid 0 and SBC at seid 1.
    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    pub fn register_sink_endpoint(&self, seid: u8, codec: &Codec) -> Result<(), EspError> {
        let mcc = codec.to_raw_mcc()?;
        esp!(unsafe { esp_a2d_sink_register_stream_endpoint(seid, &mcc) })
    }
}

impl<'d, M, T> EspA2dp<'d, M, T, Source>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    pub fn new_source(driver: T) -> Result<Self, EspError> {
        Self::new(driver)
    }
}

impl<'d, M, T> EspA2dp<'d, M, T, Duplex>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    pub fn new_duplex(driver: T) -> Result<Self, EspError> {
        Self::new(driver)
    }
}

impl<'d, M, T, S> EspA2dp<'d, M, T, S>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
    S: A2dpMode,
{
    pub fn new(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_a2d_register_callback(Some(Self::event_handler)) })?;

        if S::sink() {
            esp!(unsafe { esp_a2d_sink_register_data_callback(Some(Self::sink_data_handler)) })?;
            esp!(unsafe { esp_a2d_sink_init() })?;
        }

        if S::source() {
            esp!(unsafe {
                esp_a2d_source_register_data_callback(Some(Self::source_data_handler))
            })?;
            esp!(unsafe { esp_a2d_source_init() })?;
        }

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
            _s: PhantomData,
        })
    }

    pub fn connect_sink(&self, bd_addr: &BdAddr) -> Result<(), EspError>
    where
        S: SinkEnabled,
    {
        esp!(unsafe { esp_a2d_sink_connect(bd_addr as *const _ as *mut _) })
    }

    pub fn disconnect_sink(&self, bd_addr: &BdAddr) -> Result<(), EspError>
    where
        S: SinkEnabled,
    {
        esp!(unsafe { esp_a2d_sink_disconnect(bd_addr as *const _ as *mut _) })
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    pub fn request_delay(&self) -> Result<(), EspError>
    where
        S: SinkEnabled,
    {
        esp!(unsafe { esp_a2d_sink_get_delay_value() })
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    pub fn set_delay(&self, delay: core::time::Duration) -> Result<(), EspError>
    where
        S: SinkEnabled,
    {
        esp!(unsafe { esp_a2d_sink_set_delay_value((delay.as_micros() / 100) as _) })
    }

    pub fn connect_source(&self, bd_addr: &BdAddr) -> Result<(), EspError>
    where
        S: SourceEnabled,
    {
        esp!(unsafe { esp_a2d_source_connect(bd_addr as *const _ as *mut _) })
    }

    pub fn disconnect_source(&self, bd_addr: &BdAddr) -> Result<(), EspError>
    where
        S: SourceEnabled,
    {
        esp!(unsafe { esp_a2d_source_disconnect(bd_addr as *const _ as *mut _) })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut(A2dpEvent) -> usize + Send + 'static,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    /// # Safety
    ///
    /// This method - in contrast to method `subscribe` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    /// scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: <https://github.com/rust-lang/rust/issues/24456>
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
        F: FnMut(A2dpEvent) -> usize + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    unsafe extern "C" fn event_handler(event: esp_a2d_cb_event_t, param: *mut esp_a2d_cb_param_t) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = A2dpEvent::from((event, param));

        info!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }

    unsafe extern "C" fn sink_data_handler(buf: *const u8, len: u32) {
        let event = A2dpEvent::SinkData(core::slice::from_raw_parts(buf, len as _));
        trace!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }

    unsafe extern "C" fn source_data_handler(buf: *mut u8, len: i32) -> i32 {
        if buf.is_null() || len <= 0 {
            return 0; // nothing to read; matches the "bytes provided" contract
        }
        let event = A2dpEvent::SourceData(core::slice::from_raw_parts_mut(buf, len as _));
        trace!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event) as _
    }

    #[cfg(esp_idf_bt_a2dp_use_external_codec)]
    unsafe extern "C" fn sink_audio_data_handler(
        _conn_hdl: esp_a2d_conn_hdl_t,
        buf: *mut esp_a2d_audio_buff_t,
    ) {
        // Bluedroid never passes a null buffer here; if it ever did, the
        // safest thing is to drop the event silently.
        let Some(nn) = NonNull::new(buf) else {
            return;
        };
        let event = A2dpEvent::SinkAudioData(A2dpAudioBuf(nn));
        trace!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }
}

impl<'d, M, T, S> Drop for EspA2dp<'d, M, T, S>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
    S: A2dpMode,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        if S::sink() {
            esp!(unsafe { esp_a2d_sink_deinit() }).unwrap();
        }

        if S::source() {
            esp!(unsafe { esp_a2d_source_deinit() }).unwrap();
        }

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T, S> Send for EspA2dp<'d, M, T, S>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
    S: A2dpMode,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T, S> Sync for EspA2dp<'d, M, T, S>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
    S: A2dpMode,
{
}

static SINGLETON: BtSingleton<A2dpEvent, usize> = BtSingleton::new(0);
