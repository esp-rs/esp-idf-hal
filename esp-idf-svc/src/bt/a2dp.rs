#![allow(non_upper_case_globals)]

use core::borrow::Borrow;
use core::convert::TryInto;
use core::fmt::{self, Debug};
use core::marker::PhantomData;

use ::log::{debug, info};

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
    pub fn bitrate(&self) -> Option<u32> {
        if let Self::Sbc(data) = self {
            let oct0 = data[0];
            let sample_rate = if (oct0 & (0x01 << 6)) != 0 {
                32000
            } else if (oct0 & (0x01 << 5)) != 0 {
                44100
            } else if (oct0 & (0x01 << 4)) != 0 {
                48000
            } else {
                16000
            };

            Some(sample_rate)
        } else {
            None
        }
    }

    pub fn stereo(&self) -> Option<bool> {
        if let Self::Sbc(data) = self {
            Some((data[0] & (0x01 << 3)) == 0)
        } else {
            None
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
    SuspendedByRemote = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND,
    #[cfg(any(
        esp_idf_version_major = "4",
        all(
            esp_idf_version_major = "5",
            any(esp_idf_version_minor = "0", esp_idf_version_minor = "1")
        ),
    ))]
    Stopped = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_STOPPED,
    Started = esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_STARTED,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum MediaControlCommand {
    None = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_NONE,
    CheckSourceReady = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY,
    Start = esp_a2d_media_ctrl_t_ESP_A2D_MEDIA_CTRL_START,
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
                        ESP_A2D_MCT_SBC => Codec::Sbc(param.audio_cfg.mcc.cie.sbc),
                        ESP_A2D_MCT_M12 => Codec::Mpeg1_2(param.audio_cfg.mcc.cie.m12),
                        ESP_A2D_MCT_M24 => Codec::Mpeg2_4(param.audio_cfg.mcc.cie.m24),
                        ESP_A2D_MCT_ATRAC => Codec::Atrac(param.audio_cfg.mcc.cie.atrac),
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
        debug!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event);
    }

    unsafe extern "C" fn source_data_handler(buf: *mut u8, len: i32) -> i32 {
        let event = A2dpEvent::SourceData(core::slice::from_raw_parts_mut(buf, len as _));
        debug!("Got event {{ {:#?} }}", event);

        SINGLETON.call(event) as _
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
