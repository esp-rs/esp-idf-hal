//! MQTT protocol client
use core::ffi::c_void;
use core::fmt::Debug;
use core::{slice, time};

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;

use embedded_svc::mqtt::client::{asynch, Client, Connection, Enqueue, ErrorType, Publish};

use crate::private::unblocker::Unblocker;
use crate::sys::*;

use crate::handle::RawHandle;

use crate::private::cstr::*;
use crate::private::zerocopy::{Channel, QuitOnDrop, Receiver};
use crate::tls::*;

pub use embedded_svc::mqtt::client::{
    Details, Event, EventPayload, InitialChunkData, MessageId, QoS, SubsequentChunkData,
};

#[allow(unused_imports)]
pub use super::*;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MqttProtocolVersion {
    V3_1,
    V3_1_1,
}

impl From<MqttProtocolVersion> for esp_mqtt_protocol_ver_t {
    fn from(pv: MqttProtocolVersion) -> Self {
        match pv {
            MqttProtocolVersion::V3_1 => esp_mqtt_protocol_ver_t_MQTT_PROTOCOL_V_3_1,
            MqttProtocolVersion::V3_1_1 => esp_mqtt_protocol_ver_t_MQTT_PROTOCOL_V_3_1_1,
        }
    }
}

#[derive(Debug)]
pub struct LwtConfiguration<'a> {
    pub topic: &'a str,
    pub payload: &'a [u8],
    pub qos: QoS,
    pub retain: bool,
}

#[derive(Debug)]
pub struct MqttClientConfiguration<'a> {
    pub protocol_version: Option<MqttProtocolVersion>,

    pub client_id: Option<&'a str>,

    pub connection_refresh_interval: time::Duration,
    pub keep_alive_interval: Option<time::Duration>,
    pub reconnect_timeout: Option<time::Duration>,
    pub network_timeout: time::Duration,

    pub lwt: Option<LwtConfiguration<'a>>,

    pub disable_clean_session: bool,

    pub task_prio: u8,
    pub task_stack: usize,
    pub buffer_size: usize,
    pub out_buffer_size: usize,

    pub username: Option<&'a str>,
    pub password: Option<&'a str>,

    pub use_global_ca_store: bool,
    pub skip_cert_common_name_check: bool,
    pub crt_bundle_attach: Option<unsafe extern "C" fn(conf: *mut c_void) -> esp_err_t>,

    pub server_certificate: Option<X509<'static>>,

    pub client_certificate: Option<X509<'static>>,
    pub private_key: Option<X509<'static>>,
    pub private_key_password: Option<&'a str>,

    #[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
    pub psk: Option<Psk<'a>>,
    // pub alpn_protos: &'a [&'a str],
    // pub use_secure_element: bool,
    // void *ds_data;                          /*!< carrier of handle for digital signature parameters */
}

impl<'a> Default for MqttClientConfiguration<'a> {
    fn default() -> Self {
        Self {
            protocol_version: None,

            client_id: None,

            connection_refresh_interval: time::Duration::from_secs(0),
            keep_alive_interval: Some(time::Duration::from_secs(0)),
            reconnect_timeout: Some(time::Duration::from_secs(0)),
            network_timeout: time::Duration::from_secs(0),

            lwt: None,

            disable_clean_session: false,

            task_prio: 0,
            task_stack: 0,
            buffer_size: 0,
            out_buffer_size: 0,

            username: None,
            password: None,

            use_global_ca_store: false,
            skip_cert_common_name_check: false,

            crt_bundle_attach: Default::default(),

            server_certificate: None,

            client_certificate: None,
            private_key: None,
            private_key_password: None,

            #[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
            psk: None,
        }
    }
}

#[cfg(esp_idf_version_major = "4")]
impl<'a> TryFrom<&'a MqttClientConfiguration<'a>>
    for (esp_mqtt_client_config_t, RawCstrs, Option<TlsPsk>)
{
    type Error = EspError;

    fn try_from(conf: &'a MqttClientConfiguration<'a>) -> Result<Self, Self::Error> {
        let mut cstrs = RawCstrs::new();

        let mut c_conf = esp_mqtt_client_config_t {
            protocol_ver: if let Some(protocol_version) = conf.protocol_version {
                protocol_version.into()
            } else {
                esp_mqtt_protocol_ver_t_MQTT_PROTOCOL_UNDEFINED
            },
            client_id: cstrs.as_nptr(conf.client_id)?,

            refresh_connection_after_ms: conf.connection_refresh_interval.as_millis() as _,
            network_timeout_ms: conf.network_timeout.as_millis() as _,

            disable_clean_session: conf.disable_clean_session as _,

            task_prio: conf.task_prio as _,
            task_stack: conf.task_stack as _,
            buffer_size: conf.buffer_size as _,
            out_buffer_size: conf.out_buffer_size as _,

            username: cstrs.as_nptr(conf.username)?,
            password: cstrs.as_nptr(conf.password)?,

            use_global_ca_store: conf.use_global_ca_store,
            skip_cert_common_name_check: conf.skip_cert_common_name_check,
            crt_bundle_attach: conf.crt_bundle_attach,

            ..Default::default()
        };

        if let Some(keep_alive_interval) = conf.keep_alive_interval {
            c_conf.keepalive = keep_alive_interval.as_secs() as _;
            c_conf.disable_keepalive = false;
        } else {
            c_conf.disable_keepalive = true;
        }

        if let Some(reconnect_timeout) = conf.reconnect_timeout {
            c_conf.reconnect_timeout_ms = reconnect_timeout.as_millis() as _;
            c_conf.disable_auto_reconnect = false;
        } else {
            c_conf.disable_auto_reconnect = true;
        }

        if let Some(lwt) = conf.lwt.as_ref() {
            c_conf.lwt_topic = cstrs.as_ptr(lwt.topic)?;
            c_conf.lwt_msg = lwt.payload.as_ptr() as _;
            c_conf.lwt_msg_len = lwt.payload.len() as _;
            c_conf.lwt_qos = lwt.qos as _;
            c_conf.lwt_retain = lwt.retain as _;
        }

        if let Some(cert) = conf.server_certificate {
            c_conf.cert_pem = cert.as_esp_idf_raw_ptr() as _;
            c_conf.cert_len = cert.as_esp_idf_raw_len();
        }

        if let (Some(cert), Some(private_key)) = (conf.client_certificate, conf.private_key) {
            c_conf.client_cert_pem = cert.as_esp_idf_raw_ptr() as _;
            c_conf.client_cert_len = cert.as_esp_idf_raw_len();

            c_conf.client_key_pem = private_key.as_esp_idf_raw_ptr() as _;
            c_conf.client_key_len = private_key.as_esp_idf_raw_len();

            if let Some(pass) = conf.private_key_password {
                c_conf.clientkey_password = pass.as_ptr() as _;
                c_conf.clientkey_password_len = pass.len() as _;
            }
        }

        #[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
        let tls_psk_conf = conf.psk.as_ref().map(|psk| psk.try_into()).transpose()?;
        #[cfg(not(all(esp_idf_esp_tls_psk_verification, feature = "alloc")))]
        let tls_psk_conf = None;

        Ok((c_conf, cstrs, tls_psk_conf))
    }
}

#[allow(clippy::needless_update)]
#[cfg(not(esp_idf_version_major = "4"))]
impl<'a> TryFrom<&'a MqttClientConfiguration<'a>>
    for (esp_mqtt_client_config_t, RawCstrs, Option<TlsPsk>)
{
    type Error = EspError;

    fn try_from(conf: &'a MqttClientConfiguration<'a>) -> Result<Self, EspError> {
        let mut cstrs = RawCstrs::new();

        #[allow(clippy::needless_update)]
        let mut c_conf = esp_mqtt_client_config_t {
            broker: esp_mqtt_client_config_t_broker_t {
                verification: esp_mqtt_client_config_t_broker_t_verification_t {
                    use_global_ca_store: conf.use_global_ca_store,
                    skip_cert_common_name_check: conf.skip_cert_common_name_check,
                    crt_bundle_attach: conf.crt_bundle_attach,
                    ..Default::default()
                },
                ..Default::default()
            },
            credentials: esp_mqtt_client_config_t_credentials_t {
                client_id: cstrs.as_nptr(conf.client_id)?,
                set_null_client_id: conf.client_id.is_none(),
                username: cstrs.as_nptr(conf.username)?,
                authentication: esp_mqtt_client_config_t_credentials_t_authentication_t {
                    password: cstrs.as_nptr(conf.password)?,
                    ..Default::default()
                },
                ..Default::default()
            },
            session: esp_mqtt_client_config_t_session_t {
                protocol_ver: if let Some(protocol_version) = conf.protocol_version {
                    protocol_version.into()
                } else {
                    esp_mqtt_protocol_ver_t_MQTT_PROTOCOL_UNDEFINED
                },
                disable_clean_session: conf.disable_clean_session as _,
                ..Default::default()
            },
            network: esp_mqtt_client_config_t_network_t {
                refresh_connection_after_ms: conf.connection_refresh_interval.as_millis() as _,
                timeout_ms: conf.network_timeout.as_millis() as _,
                ..Default::default()
            },
            task: esp_mqtt_client_config_t_task_t {
                priority: conf.task_prio as _,
                stack_size: conf.task_stack as _,
                ..Default::default()
            },
            buffer: esp_mqtt_client_config_t_buffer_t {
                size: conf.buffer_size as _,
                out_size: conf.out_buffer_size as _,
                ..Default::default()
            },
            ..Default::default()
        };

        if let Some(keep_alive_interval) = conf.keep_alive_interval {
            c_conf.session.keepalive = keep_alive_interval.as_secs() as _;
            c_conf.session.disable_keepalive = false;
        } else {
            c_conf.session.disable_keepalive = true;
        }

        if let Some(reconnect_timeout) = conf.reconnect_timeout {
            c_conf.network.reconnect_timeout_ms = reconnect_timeout.as_millis() as _;
            c_conf.network.disable_auto_reconnect = false;
        } else {
            c_conf.network.disable_auto_reconnect = true;
        }

        if let Some(lwt) = conf.lwt.as_ref() {
            c_conf.session.last_will = esp_mqtt_client_config_t_session_t_last_will_t {
                topic: cstrs.as_ptr(lwt.topic)?,
                msg: lwt.payload.as_ptr() as _,
                msg_len: lwt.payload.len() as _,
                qos: lwt.qos as _,
                retain: lwt.retain as _,
                ..Default::default()
            };
        }

        if let Some(cert) = conf.server_certificate {
            c_conf.broker.verification.certificate = cert.as_esp_idf_raw_ptr() as _;
            c_conf.broker.verification.certificate_len = cert.as_esp_idf_raw_len();
        }

        if let (Some(cert), Some(private_key)) = (conf.client_certificate, conf.private_key) {
            c_conf.credentials.authentication.certificate = cert.as_esp_idf_raw_ptr() as _;
            c_conf.credentials.authentication.certificate_len = cert.as_esp_idf_raw_len();

            c_conf.credentials.authentication.key = private_key.as_esp_idf_raw_ptr() as _;
            c_conf.credentials.authentication.key_len = private_key.as_esp_idf_raw_len();

            if let Some(pass) = conf.private_key_password {
                c_conf.credentials.authentication.key_password = pass.as_ptr() as _;
                c_conf.credentials.authentication.key_password_len = pass.len() as _;
            }
        }

        #[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
        let tls_psk_conf = conf.psk.as_ref().map(|psk| psk.try_into()).transpose()?;
        #[cfg(not(all(esp_idf_esp_tls_psk_verification, feature = "alloc")))]
        let tls_psk_conf = None;

        Ok((c_conf, cstrs, tls_psk_conf))
    }
}

struct UnsafeCallback<'a>(*mut Box<dyn FnMut(esp_mqtt_event_handle_t) + Send + 'a>);

impl<'a> UnsafeCallback<'a> {
    fn from(boxed: &mut Box<Box<dyn FnMut(esp_mqtt_event_handle_t) + Send + 'a>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: esp_mqtt_event_handle_t) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

pub struct EspMqttClient<'a> {
    raw_client: esp_mqtt_client_handle_t,
    _boxed_raw_callback: Box<dyn FnMut(esp_mqtt_event_handle_t) + Send + 'a>,
    _tls_psk_conf: Option<TlsPsk>,
}

impl<'a> RawHandle for EspMqttClient<'a> {
    type Handle = esp_mqtt_client_handle_t;

    fn handle(&self) -> Self::Handle {
        self.raw_client
    }
}

impl EspMqttClient<'static> {
    pub fn new(
        url: &str,
        conf: &MqttClientConfiguration,
    ) -> Result<(Self, EspMqttConnection), EspError>
    where
        Self: Sized,
    {
        let (channel, receiver) = Channel::new();

        let sender = QuitOnDrop::new(channel);

        let conn = EspMqttConnection {
            receiver,
            given: false,
        };

        let client = Self::new_cb(url, conf, move |mut event| {
            let event: &mut EspMqttEvent<'static> = unsafe { core::mem::transmute(&mut event) };
            sender.channel().share(event);
        })?;

        Ok((client, conn))
    }

    pub fn new_cb<F>(
        url: &str,
        conf: &MqttClientConfiguration,
        callback: F,
    ) -> Result<Self, EspError>
    where
        F: for<'b> FnMut(EspMqttEvent<'b>) + Send + 'static,
        Self: Sized,
    {
        unsafe { Self::new_nonstatic_cb(url, conf, callback) }
    }
}

impl<'a> EspMqttClient<'a> {
    /// # Safety
    ///
    /// This method - in contrast to method `new_generic` - allows the user to pass
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
    pub unsafe fn new_nonstatic_cb<F>(
        url: &str,
        conf: &MqttClientConfiguration,
        mut callback: F,
    ) -> Result<Self, EspError>
    where
        F: for<'b> FnMut(EspMqttEvent<'b>) + Send + 'a,
        Self: Sized,
    {
        Self::new_raw(
            url,
            conf,
            Box::new(move |event_handle| {
                callback(EspMqttEvent::new(unsafe { event_handle.as_ref() }.unwrap()));
            }),
        )
    }

    fn new_raw(
        url: &str,
        conf: &MqttClientConfiguration,
        raw_callback: Box<dyn FnMut(esp_mqtt_event_handle_t) + Send + 'a>,
    ) -> Result<Self, EspError>
    where
        Self: Sized,
    {
        let mut boxed_raw_callback = Box::new(raw_callback);

        let unsafe_callback = UnsafeCallback::from(&mut boxed_raw_callback);

        let (mut c_conf, mut cstrs, tls_psk_conf) = conf.try_into()?;

        #[cfg(esp_idf_version_major = "4")]
        {
            c_conf.uri = cstrs.as_ptr(url)?;
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            c_conf.broker.address.uri = cstrs.as_ptr(url)?;
        }

        #[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
        {
            #[cfg(esp_idf_version_major = "4")]
            if let Some(ref conf) = tls_psk_conf {
                c_conf.psk_hint_key = &*conf.psk;
            }
            #[cfg(not(esp_idf_version_major = "4"))]
            if let Some(ref conf) = tls_psk_conf {
                c_conf.broker.verification.psk_hint_key = &*conf.psk;
            }
        }

        let raw_client = unsafe { esp_mqtt_client_init(&c_conf as *const _) };
        if raw_client.is_null() {
            return Err(EspError::from_infallible::<ESP_FAIL>());
        }

        let client = Self {
            raw_client,
            _boxed_raw_callback: boxed_raw_callback,
            _tls_psk_conf: tls_psk_conf,
        };

        esp!(unsafe {
            esp_mqtt_client_register_event(
                client.raw_client,
                esp_mqtt_event_id_t_MQTT_EVENT_ANY,
                Some(Self::handle),
                unsafe_callback.as_ptr(),
            )
        })?;

        esp!(unsafe { esp_mqtt_client_start(client.raw_client) })?;

        Ok(client)
    }

    pub fn subscribe(&mut self, topic: &str, qos: QoS) -> Result<MessageId, EspError> {
        self.subscribe_cstr(to_cstring_arg(topic)?.as_c_str(), qos)
    }

    pub fn unsubscribe(&mut self, topic: &str) -> Result<MessageId, EspError> {
        self.unsubscribe_cstr(to_cstring_arg(topic)?.as_c_str())
    }

    pub fn publish(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, EspError> {
        self.publish_cstr(to_cstring_arg(topic)?.as_c_str(), qos, retain, payload)
    }

    pub fn enqueue(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, EspError> {
        self.enqueue_cstr(to_cstring_arg(topic)?.as_c_str(), qos, retain, payload)
    }

    pub fn subscribe_cstr(
        &mut self,
        topic: &core::ffi::CStr,
        qos: QoS,
    ) -> Result<MessageId, EspError> {
        #[cfg(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(
                esp_idf_version_major = "5",
                esp_idf_version_minor = "1",
                any(esp_idf_version_patch = "0", esp_idf_version_patch = "1")
            )
        ))]
        let res = Self::check(unsafe {
            esp_mqtt_client_subscribe(self.raw_client, topic.as_ptr(), qos as _)
        });

        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
            all(
                esp_idf_version_major = "5",
                esp_idf_version_minor = "1",
                any(esp_idf_version_patch = "0", esp_idf_version_patch = "1")
            )
        )))]
        let res = Self::check(unsafe {
            esp_mqtt_client_subscribe_single(self.raw_client, topic.as_ptr(), qos as _)
        });

        res
    }

    pub fn unsubscribe_cstr(&mut self, topic: &core::ffi::CStr) -> Result<MessageId, EspError> {
        Self::check(unsafe { esp_mqtt_client_unsubscribe(self.raw_client, topic.as_ptr()) })
    }

    pub fn publish_cstr(
        &mut self,
        topic: &core::ffi::CStr,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, EspError> {
        let payload_ptr = match payload.len() {
            0 => core::ptr::null(),
            _ => payload.as_ptr(),
        };

        Self::check(unsafe {
            esp_mqtt_client_publish(
                self.raw_client,
                topic.as_ptr(),
                payload_ptr as _,
                payload.len() as _,
                qos as _,
                retain as _,
            )
        })
    }

    pub fn enqueue_cstr(
        &mut self,
        topic: &core::ffi::CStr,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, EspError> {
        let payload_ptr = match payload.len() {
            0 => core::ptr::null(),
            _ => payload.as_ptr(),
        };

        Self::check(unsafe {
            esp_mqtt_client_enqueue(
                self.raw_client,
                topic.as_ptr(),
                payload_ptr as _,
                payload.len() as _,
                qos as _,
                retain as _,
                true,
            )
        })
    }

    extern "C" fn handle(
        event_handler_arg: *mut c_void,
        _event_base: esp_event_base_t,
        _event_id: i32,
        event_data: *mut c_void,
    ) {
        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(event_data as _);
        }
    }

    fn check(result: i32) -> Result<MessageId, EspError> {
        match EspError::from(result) {
            Some(err) if result < 0 => Err(err),
            _ => Ok(result as _),
        }
    }
}

impl<'a> Drop for EspMqttClient<'a> {
    fn drop(&mut self) {
        unsafe {
            esp_mqtt_client_destroy(self.raw_client as _);
        }
    }
}

impl<'a> ErrorType for EspMqttClient<'a> {
    type Error = EspError;
}

impl<'a> Client for EspMqttClient<'a> {
    fn subscribe(&mut self, topic: &str, qos: QoS) -> Result<MessageId, Self::Error> {
        EspMqttClient::subscribe(self, topic, qos)
    }

    fn unsubscribe(&mut self, topic: &str) -> Result<MessageId, Self::Error> {
        EspMqttClient::unsubscribe(self, topic)
    }
}

impl<'a> Publish for EspMqttClient<'a> {
    fn publish(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, Self::Error> {
        EspMqttClient::publish(self, topic, qos, retain, payload)
    }
}

impl<'a> Enqueue for EspMqttClient<'a> {
    fn enqueue(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, Self::Error> {
        EspMqttClient::enqueue(self, topic, qos, retain, payload)
    }
}

unsafe impl<'a> Send for EspMqttClient<'a> {}

pub struct EspMqttConnection {
    receiver: Receiver<EspMqttEvent<'static>>,
    given: bool,
}

impl EspMqttConnection {
    #[allow(clippy::should_implement_trait)]
    pub fn next(&mut self) -> Result<&EspMqttEvent<'_>, EspError> {
        if self.given {
            self.receiver.done();
        }

        if let Some(event) = self.receiver.get_shared() {
            self.given = true;

            Ok(event)
        } else {
            self.given = false;

            Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
        }
    }
}

impl ErrorType for EspMqttConnection {
    type Error = EspError;
}

impl Connection for EspMqttConnection {
    type Event<'a> = &'a EspMqttEvent<'a>;

    fn next(&mut self) -> Result<Self::Event<'_>, Self::Error> {
        EspMqttConnection::next(self)
    }
}

#[derive(Copy, Clone, Debug)]
enum AsyncCommand {
    Subscribe { qos: QoS },
    Unsubscribe,
    Publish { qos: QoS, retain: bool },
}

#[derive(Debug)]
struct AsyncWork {
    command: AsyncCommand,
    topic: alloc::vec::Vec<u8>,
    payload: alloc::vec::Vec<u8>,
    result: Result<MessageId, EspError>,
}

pub struct EspAsyncMqttClient(Unblocker<AsyncWork>);

impl EspAsyncMqttClient {
    pub fn new(
        url: &str,
        conf: &MqttClientConfiguration<'_>,
    ) -> Result<(Self, EspAsyncMqttConnection), EspError> {
        let (channel, receiver) = Channel::new();
        let conn = EspAsyncMqttConnection {
            receiver,
            given: false,
        };

        let client = Self::wrap(EspMqttClient::new_cb(url, conf, move |mut event| {
            let event: &mut EspMqttEvent<'static> = unsafe { core::mem::transmute(&mut event) };
            channel.share(event);
        })?)?;

        Ok((client, conn))
    }

    fn wrap(client: EspMqttClient<'static>) -> Result<Self, EspError> {
        let unblocker = Unblocker::new(
            CStr::from_bytes_until_nul(b"MQTT Sending task\0").unwrap(),
            4096,
            None,
            None,
            move |channel| Self::work(channel, client),
        )?;

        Ok(Self(unblocker))
    }

    pub async fn subscribe(&mut self, topic: &str, qos: QoS) -> Result<MessageId, EspError> {
        self.execute(AsyncCommand::Subscribe { qos }, Some(topic), None)
            .await
    }

    pub async fn unsubscribe(&mut self, topic: &str) -> Result<MessageId, EspError> {
        self.execute(AsyncCommand::Unsubscribe, Some(topic), None)
            .await
    }

    pub async fn publish(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, EspError> {
        self.execute(
            AsyncCommand::Publish { qos, retain },
            Some(topic),
            Some(payload),
        )
        .await
    }

    async fn execute(
        &mut self,
        command: AsyncCommand,
        topic: Option<&str>,
        payload: Option<&[u8]>,
    ) -> Result<MessageId, EspError> {
        let work = self.0.exec_in_out().await.unwrap();

        work.command = command;

        if let Some(topic) = topic {
            work.topic.clear();
            work.topic.extend_from_slice(topic.as_bytes());
            work.topic.push(0);
        }

        if let Some(payload) = payload {
            work.payload.clear();
            work.payload.extend_from_slice(payload);
        }

        self.0.do_exec().await;

        let work = self.0.exec_in_out().await.unwrap();

        work.result
    }

    fn work(channel: Arc<Channel<AsyncWork>>, mut client: EspMqttClient) {
        let mut work = AsyncWork {
            command: AsyncCommand::Unsubscribe,
            topic: alloc::vec::Vec::new(),
            payload: alloc::vec::Vec::new(),
            result: Ok(0),
        };

        while channel.share(&mut work) {
            let topic = unsafe { core::ffi::CStr::from_bytes_with_nul_unchecked(&work.topic) };

            match work.command {
                AsyncCommand::Subscribe { qos } => {
                    work.result = client.subscribe_cstr(topic, qos);
                }
                AsyncCommand::Unsubscribe => {
                    work.result = client.unsubscribe_cstr(topic);
                }
                AsyncCommand::Publish { qos, retain } => {
                    work.result = client.publish_cstr(topic, qos, retain, &work.payload);
                }
            }
        }
    }
}

impl ErrorType for EspAsyncMqttClient {
    type Error = EspError;
}

impl asynch::Client for EspAsyncMqttClient {
    async fn subscribe(&mut self, topic: &str, qos: QoS) -> Result<MessageId, Self::Error> {
        EspAsyncMqttClient::subscribe(self, topic, qos).await
    }

    async fn unsubscribe(&mut self, topic: &str) -> Result<MessageId, Self::Error> {
        EspAsyncMqttClient::unsubscribe(self, topic).await
    }
}

impl asynch::Publish for EspAsyncMqttClient {
    async fn publish(
        &mut self,
        topic: &str,
        qos: QoS,
        retain: bool,
        payload: &[u8],
    ) -> Result<MessageId, Self::Error> {
        EspAsyncMqttClient::publish(self, topic, qos, retain, payload).await
    }
}

pub struct EspAsyncMqttConnection {
    receiver: Receiver<EspMqttEvent<'static>>,
    given: bool,
}

impl EspAsyncMqttConnection {
    pub async fn next(&mut self) -> Result<&EspMqttEvent<'_>, EspError> {
        if self.given {
            self.receiver.done();
        }

        if let Some(event) = self.receiver.get_shared_async().await {
            self.given = true;

            Ok(event)
        } else {
            self.given = false;

            Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
        }
    }
}

impl ErrorType for EspAsyncMqttConnection {
    type Error = EspError;
}

impl asynch::Connection for EspAsyncMqttConnection {
    type Event<'a> = &'a EspMqttEvent<'a>;

    async fn next(&mut self) -> Result<Self::Event<'_>, Self::Error> {
        EspAsyncMqttConnection::next(self).await
    }
}

static ERROR: EspError = EspError::from_infallible::<ESP_FAIL>();

pub struct EspMqttEvent<'a>(&'a esp_mqtt_event_t);

impl<'a> EspMqttEvent<'a> {
    const fn new(event: &'a esp_mqtt_event_t) -> Self {
        Self(event)
    }

    #[allow(non_upper_case_globals)]
    pub fn payload(&self) -> EventPayload<'_, EspError> {
        match self.0.event_id {
            esp_mqtt_event_id_t_MQTT_EVENT_ERROR => EventPayload::Error(&ERROR), // TODO
            esp_mqtt_event_id_t_MQTT_EVENT_BEFORE_CONNECT => EventPayload::BeforeConnect,
            esp_mqtt_event_id_t_MQTT_EVENT_CONNECTED => {
                EventPayload::Connected(self.0.session_present != 0)
            }
            esp_mqtt_event_id_t_MQTT_EVENT_DISCONNECTED => EventPayload::Disconnected,
            esp_mqtt_event_id_t_MQTT_EVENT_SUBSCRIBED => {
                EventPayload::Subscribed(self.0.msg_id as _)
            }
            esp_mqtt_event_id_t_MQTT_EVENT_UNSUBSCRIBED => {
                EventPayload::Unsubscribed(self.0.msg_id as _)
            }
            esp_mqtt_event_id_t_MQTT_EVENT_PUBLISHED => EventPayload::Published(self.0.msg_id as _),
            esp_mqtt_event_id_t_MQTT_EVENT_DATA => EventPayload::Received {
                id: self.0.msg_id as _,
                topic: {
                    let ptr = self.0.topic;

                    if ptr.is_null() {
                        None
                    } else {
                        let len = self.0.topic_len;

                        let topic = unsafe {
                            let slice = slice::from_raw_parts(ptr as _, len.try_into().unwrap());
                            core::str::from_utf8(slice).unwrap()
                        };

                        Some(topic)
                    }
                },
                data: if self.0.data_len > 0 {
                    unsafe {
                        slice::from_raw_parts(
                            (self.0.data as *const u8).as_ref().unwrap(),
                            self.0.data_len as _,
                        )
                    }
                } else {
                    &[]
                },
                details: {
                    if self.0.data_len < self.0.total_data_len {
                        if self.0.current_data_offset == 0 {
                            Details::InitialChunk(InitialChunkData {
                                total_data_size: self.0.total_data_len as _,
                            })
                        } else {
                            Details::SubsequentChunk(SubsequentChunkData {
                                current_data_offset: self.0.current_data_offset as _,
                                total_data_size: self.0.total_data_len as _,
                            })
                        }
                    } else {
                        Details::Complete
                    }
                },
            },
            esp_mqtt_event_id_t_MQTT_EVENT_DELETED => EventPayload::Deleted(self.0.msg_id as _),
            other => panic!("Unknown message type: {}", other),
        }
    }
}

unsafe impl<'a> Send for EspMqttEvent<'a> {}

impl<'a> ErrorType for EspMqttEvent<'a> {
    type Error = EspError;
}

impl<'a> Event for EspMqttEvent<'a> {
    fn payload(&self) -> EventPayload<'_, Self::Error> {
        EspMqttEvent::payload(self)
    }
}
