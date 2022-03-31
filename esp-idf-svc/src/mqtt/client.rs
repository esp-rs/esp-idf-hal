use core::convert::TryInto;
use core::fmt::{self, Debug};
use core::mem::ManuallyDrop;
use core::ptr;
use core::slice;
use core::time;

extern crate alloc;
use alloc::borrow::Cow;
use alloc::boxed::Box;
use alloc::sync::Arc;

use embedded_svc::{errors, mqtt::client, mqtt::client::Message};

use esp_idf_hal::mutex::{Condvar, Mutex};

use esp_idf_sys::*;

use crate::private::{common::Newtype, cstr::*};

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
    pub qos: client::QoS,
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
    #[cfg(not(esp_idf_version = "4.3"))]
    pub crt_bundle_attach: Option<unsafe extern "C" fn(conf: *mut c_types::c_void) -> esp_err_t>,
    // TODO: Future

    // pub cert_pem: &'a [u8],
    // pub client_cert_pem: &'a [u8],
    // pub client_key_pem: &'a [u8],

    // pub psk_hint_key: KeyHint,
    // pub alpn_protos: &'a [&'a str],

    // pub clientkey_password: &'a str,
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

            #[cfg(not(esp_idf_version = "4.3"))]
            crt_bundle_attach: Default::default(),
        }
    }
}

impl<'a> From<&'a MqttClientConfiguration<'a>> for (esp_mqtt_client_config_t, RawCstrs) {
    fn from(conf: &'a MqttClientConfiguration<'a>) -> Self {
        let mut cstrs = RawCstrs::new();

        let mut c_conf = esp_mqtt_client_config_t {
            protocol_ver: if let Some(protocol_version) = conf.protocol_version {
                protocol_version.into()
            } else {
                esp_mqtt_protocol_ver_t_MQTT_PROTOCOL_UNDEFINED
            },
            client_id: cstrs.as_nptr(conf.client_id),

            refresh_connection_after_ms: conf.connection_refresh_interval.as_millis() as _,
            network_timeout_ms: conf.network_timeout.as_millis() as _,

            disable_clean_session: conf.disable_clean_session as _,

            task_prio: conf.task_prio as _,
            task_stack: conf.task_stack as _,
            buffer_size: conf.buffer_size as _,
            out_buffer_size: conf.out_buffer_size as _,

            username: cstrs.as_nptr(conf.username),
            password: cstrs.as_nptr(conf.password),

            use_global_ca_store: conf.use_global_ca_store,
            skip_cert_common_name_check: conf.skip_cert_common_name_check,
            #[cfg(not(esp_idf_version = "4.3"))]
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
            c_conf.lwt_topic = cstrs.as_ptr(lwt.topic);
            c_conf.lwt_msg = lwt.payload.as_ptr() as _;
            c_conf.lwt_msg_len = lwt.payload.len() as _;
            c_conf.lwt_qos = lwt.qos as _;
            c_conf.lwt_retain = lwt.retain as _;
        }

        (c_conf, cstrs)
    }
}

struct UnsafeCallback(*mut Box<dyn FnMut(esp_mqtt_event_handle_t)>);

impl UnsafeCallback {
    fn from(boxed: &mut Box<Box<dyn FnMut(esp_mqtt_event_handle_t)>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: esp_mqtt_event_handle_t) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

pub struct EspMqttClient(
    esp_mqtt_client_handle_t,
    Box<dyn FnMut(esp_mqtt_event_handle_t)>,
);

impl EspMqttClient {
    pub fn new<'a>(
        url: impl AsRef<str>,
        conf: &'a MqttClientConfiguration<'a>,
    ) -> Result<(Self, EspMqttConnection), EspError>
    where
        Self: Sized,
    {
        let state = Arc::new(EspMqttConnectionState {
            message: Mutex::new(None),
            posted: Condvar::new(),
            processed: Condvar::new(),
        });

        let connection = EspMqttConnection(state);
        let client_connection = connection.clone();

        let client = Self::new_with_raw_callback(
            url,
            conf,
            Box::new(move |event_handle| EspMqttConnection::post(&client_connection, event_handle)),
        )?;

        Ok((client, connection))
    }

    pub fn new_with_callback<'a>(
        url: impl AsRef<str>,
        conf: &'a MqttClientConfiguration<'a>,
        mut callback: impl for<'b> FnMut(&'b Option<Result<client::Event<EspMqttMessage<'b>>, EspError>>)
            + 'static,
    ) -> Result<Self, EspError>
    where
        Self: Sized,
    {
        Self::new_with_raw_callback(
            url,
            conf,
            Box::new(move |event_handle| {
                let event = unsafe { event_handle.as_ref() };

                if let Some(event) = event {
                    let mut event = ManuallyDrop::new(Some(EspMqttMessage::new_event(event, None)));

                    callback(&mut event);

                    unsafe {
                        ManuallyDrop::drop(&mut event);
                    }
                } else {
                    callback(&None)
                }
            }),
        )
    }

    fn new_with_raw_callback<'a>(
        url: impl AsRef<str> + 'a,
        conf: &'a MqttClientConfiguration<'a>,
        raw_callback: Box<dyn FnMut(esp_mqtt_event_handle_t)>,
    ) -> Result<Self, EspError>
    where
        Self: Sized,
    {
        let mut boxed_raw_callback = Box::new(raw_callback);

        let unsafe_callback = UnsafeCallback::from(&mut boxed_raw_callback);

        let (c_conf, _cstrs) = conf.into();

        let client = unsafe { esp_mqtt_client_init(&c_conf as *const _) };
        if client.is_null() {
            esp!(ESP_FAIL)?;
        }

        let client = Self(client, boxed_raw_callback);

        let c_url = CString::new(url.as_ref()).unwrap();

        esp!(unsafe { esp_mqtt_client_set_uri(client.0, c_url.as_ptr()) })?;

        esp!(unsafe {
            esp_mqtt_client_register_event(
                client.0,
                esp_mqtt_event_id_t_MQTT_EVENT_ANY,
                Some(Self::handle),
                unsafe_callback.as_ptr(),
            )
        })?;

        esp!(unsafe { esp_mqtt_client_start(client.0) })?;

        Ok(client)
    }

    extern "C" fn handle(
        event_handler_arg: *mut c_types::c_void,
        _event_base: esp_event_base_t,
        _event_id: i32,
        event_data: *mut c_types::c_void,
    ) {
        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(event_data as _);
        }
    }

    fn check(result: i32) -> Result<client::MessageId, EspError> {
        if result < 0 {
            esp!(result)?;
        }

        Ok(result as _)
    }
}

#[cfg(feature = "experimental")]
impl EspMqttClient {
    pub fn new_async<'a, U, S>(
        url: S,
        conf: /*TODO: issue with the compiler: &'a*/ MqttClientConfiguration<'a>,
    ) -> Result<
        (
            embedded_svc::utils::asyncify::mqtt::client::AsyncClient<U, Mutex<Self>>,
            embedded_svc::utils::asyncify::mqtt::client::AsyncConnection<
                Condvar,
                EspMqttMessage,
                EspError,
            >,
        ),
        EspError,
    >
    where
        U: embedded_svc::unblocker::asyncs::Unblocker,
        S: AsRef<str> + 'a,
    {
        let connection = embedded_svc::utils::asyncify::mqtt::client::AsyncConnection::<
            Condvar,
            _,
            EspError,
        >::new();

        let cb_connection = connection.clone();

        let client = Self::new_with_callback(url, &conf, move |event| cb_connection.post(event))?;

        Ok((
            embedded_svc::utils::asyncify::mqtt::client::AsyncClient::new(client),
            connection,
        ))
    }
}

impl Drop for EspMqttClient {
    fn drop(&mut self) {
        esp!(unsafe { esp_mqtt_client_disconnect(self.0) }).unwrap();
        esp!(unsafe { esp_mqtt_client_stop(self.0) }).unwrap();
        esp!(unsafe { esp_mqtt_client_destroy(self.0) }).unwrap();

        (self.1)(ptr::null_mut() as *mut _);
    }
}

impl errors::Errors for EspMqttClient {
    type Error = EspError;
}

impl client::Client for EspMqttClient {
    fn subscribe<'a, S>(
        &'a mut self,
        topic: S,
        qos: client::QoS,
    ) -> Result<client::MessageId, Self::Error>
    where
        S: Into<Cow<'a, str>>,
    {
        let c_topic = CString::new(topic.into().as_ref()).unwrap();

        Self::check(unsafe { esp_mqtt_client_subscribe(self.0, c_topic.as_ptr(), qos as _) })
    }

    fn unsubscribe<'a, S>(&'a mut self, topic: S) -> Result<client::MessageId, Self::Error>
    where
        S: Into<Cow<'a, str>>,
    {
        let c_topic = CString::new(topic.into().as_ref()).unwrap();

        Self::check(unsafe { esp_mqtt_client_unsubscribe(self.0, c_topic.as_ptr()) })
    }
}

impl client::Publish for EspMqttClient {
    fn publish<'a, S, V>(
        &'a mut self,
        topic: S,
        qos: client::QoS,
        retain: bool,
        payload: V,
    ) -> Result<client::MessageId, Self::Error>
    where
        S: Into<Cow<'a, str>>,
        V: Into<Cow<'a, [u8]>>,
    {
        let c_topic = CString::new(topic.into().as_ref()).unwrap();

        let payload = payload.into();

        Self::check(unsafe {
            esp_mqtt_client_publish(
                self.0,
                c_topic.as_ptr(),
                payload.as_ref().as_ptr() as _,
                payload.as_ref().len() as _,
                qos as _,
                retain as _,
            )
        })
    }
}

impl client::Enqueue for EspMqttClient {
    fn enqueue<'a, S, V>(
        &'a mut self,
        topic: S,
        qos: client::QoS,
        retain: bool,
        payload: V,
    ) -> Result<client::MessageId, Self::Error>
    where
        S: Into<Cow<'a, str>>,
        V: Into<Cow<'a, [u8]>>,
    {
        let c_topic = CString::new(topic.into().as_ref()).unwrap();

        let payload = payload.into();

        Self::check(unsafe {
            esp_mqtt_client_enqueue(
                self.0,
                c_topic.as_ptr(),
                payload.as_ref().as_ptr() as _,
                payload.as_ref().len() as _,
                qos as _,
                retain as _,
                true,
            )
        })
    }
}

unsafe impl Send for EspMqttClient {}

pub struct EspMqttMessage<'a> {
    event: &'a esp_mqtt_event_t,
    details: client::Details,
    connection: Option<Arc<EspMqttConnectionState>>,
}

impl<'a> EspMqttMessage<'a> {
    #[allow(non_upper_case_globals)]
    fn new_event(
        event: &'a esp_mqtt_event_t,
        connection: Option<&Arc<EspMqttConnectionState>>,
    ) -> Result<client::Event<EspMqttMessage<'a>>, EspError> {
        match event.event_id {
            esp_mqtt_event_id_t_MQTT_EVENT_ERROR => Err(EspError::from(ESP_FAIL).unwrap()), // TODO
            esp_mqtt_event_id_t_MQTT_EVENT_BEFORE_CONNECT => Ok(client::Event::BeforeConnect),
            esp_mqtt_event_id_t_MQTT_EVENT_CONNECTED => {
                Ok(client::Event::Connected(event.session_present != 0))
            }
            esp_mqtt_event_id_t_MQTT_EVENT_DISCONNECTED => Ok(client::Event::Disconnected),
            esp_mqtt_event_id_t_MQTT_EVENT_SUBSCRIBED => {
                Ok(client::Event::Subscribed(event.msg_id as _))
            }
            esp_mqtt_event_id_t_MQTT_EVENT_UNSUBSCRIBED => {
                Ok(client::Event::Unsubscribed(event.msg_id as _))
            }
            esp_mqtt_event_id_t_MQTT_EVENT_PUBLISHED => {
                Ok(client::Event::Published(event.msg_id as _))
            }
            esp_mqtt_event_id_t_MQTT_EVENT_DATA => Ok(client::Event::Received(
                EspMqttMessage::new(event, connection.cloned()),
            )),
            esp_mqtt_event_id_t_MQTT_EVENT_DELETED => Ok(client::Event::Deleted(event.msg_id as _)),
            other => panic!("Unknown message type: {}", other),
        }
    }

    fn new(event: &'a esp_mqtt_event_t, connection: Option<Arc<EspMqttConnectionState>>) -> Self {
        let mut message = Self {
            event,
            details: client::Details::Complete(unsafe { client::TopicToken::new() }),
            connection,
        };

        message.fill_chunk_details();

        message
    }

    fn fill_chunk_details(&mut self) {
        if self.event.data_len < self.event.total_data_len {
            if self.event.current_data_offset == 0 {
                self.details = client::Details::InitialChunk(client::InitialChunkData {
                    topic_token: unsafe { client::TopicToken::new() },
                    total_data_size: self.event.total_data_len as _,
                });
            } else {
                self.details = client::Details::SubsequentChunk(client::SubsequentChunkData {
                    current_data_offset: self.event.current_data_offset as _,
                    total_data_size: self.event.total_data_len as _,
                });
            }
        }
    }
}

impl<'a> Debug for EspMqttMessage<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[id = {}, topic = {:?}, details = {:?}]",
            self.id(),
            self.retrieve_topic(),
            self.details()
        )
    }
}

impl<'a> Drop for EspMqttMessage<'a> {
    fn drop(&mut self) {
        if let Some(state) = self.connection.as_ref() {
            let mut message = state.message.lock();

            if message.is_some() {
                *message = None;
                state.processed.notify_all();
            }
        }
    }
}

impl<'a> client::Message for EspMqttMessage<'a> {
    fn id(&self) -> client::MessageId {
        self.event.msg_id as _
    }

    fn data(&self) -> Cow<'_, [u8]> {
        Cow::Borrowed(unsafe {
            slice::from_raw_parts(
                (self.event.data as *const u8).as_ref().unwrap(),
                self.event.data_len as _,
            )
        })
    }

    fn topic(&self, _topic_token: &client::TopicToken) -> Cow<'_, str> {
        let ptr = self.event.topic;
        let len = self.event.topic_len;

        unsafe {
            let slice = slice::from_raw_parts(ptr as _, len.try_into().unwrap());
            Cow::Borrowed(core::str::from_utf8(slice).unwrap())
        }
    }

    fn details(&self) -> &client::Details {
        &self.details
    }
}

#[cfg_attr(version("1.61"), allow(suspicious_auto_trait_impls))]
unsafe impl Send for Newtype<esp_mqtt_event_handle_t> {}

struct EspMqttConnectionState {
    message: Mutex<Option<Newtype<esp_mqtt_event_handle_t>>>,
    posted: Condvar,
    processed: Condvar,
}

#[derive(Clone)]
pub struct EspMqttConnection(Arc<EspMqttConnectionState>);

impl EspMqttConnection {
    fn post(&self, event: esp_mqtt_event_handle_t) {
        let mut message = self.0.message.lock();

        while message.is_some() {
            message = self.0.processed.wait(message);
        }

        *message = Some(Newtype(event));
        self.0.posted.notify_all();

        while message.is_some() {
            message = self.0.processed.wait(message);
        }
    }
}

unsafe impl Send for EspMqttConnection {}

impl errors::Errors for EspMqttConnection {
    type Error = EspError;
}

impl client::Connection for EspMqttConnection {
    type Message<'a> = EspMqttMessage<'a>;

    fn next(&mut self) -> Option<Result<client::Event<Self::Message<'_>>, Self::Error>> {
        let mut message = self.0.message.lock();

        while message.is_none() {
            message = self.0.posted.wait(message);
        }

        let event = unsafe { message.as_ref().unwrap().0.as_ref() };
        if let Some(event) = event {
            let event = EspMqttMessage::new_event(event, Some(&self.0));

            if let Ok(client::Event::Received(_)) = event.as_ref() {
                Some(event)
            } else {
                *message = None;
                self.0.processed.notify_all();

                Some(event)
            }
        } else {
            None
        }
    }
}
