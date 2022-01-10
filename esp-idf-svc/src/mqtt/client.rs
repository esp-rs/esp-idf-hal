use core::ptr;
use core::slice;
use core::time;

extern crate alloc;
use alloc::borrow::Cow;
use alloc::sync::Arc;

use embedded_svc::mqtt::client;

use esp_idf_sys::*;

use crate::private::cstr::*;

// !!! NOTE: WORK IN PROGRESS

#[derive(Debug)]
pub struct LwtConfiguration<'a> {
    pub topic: &'a str,
    pub payload: &'a [u8],
    pub qos: client::QoS,
    pub retain: bool,
}

#[derive(Debug, Default)]
pub struct Configuration<'a> {
    // pub protocol_version: ProtocolVersion,
    pub client_id: Option<&'a str>,

    pub connection_refresh_interval: time::Duration,
    pub keep_alive_interval: Option<time::Duration>,
    pub reconnect_timeout: time::Duration,
    pub network_timeout: time::Duration,

    pub lwt: Option<LwtConfiguration<'a>>,

    pub disable_clean_session: bool,
    pub disable_auto_reconnect: bool,

    pub task_prio: u8,
    pub task_stack: usize,
    pub buffer_size: usize,
    pub out_buffer_size: usize,
    // pub cert_pem: &'a [u8],
    // pub client_cert_pem: &'a [u8],
    // pub client_key_pem: &'a [u8],

    // pub psk_hint_key: KeyHint,
    // pub use_global_ca_store: bool,
    // //esp_err_t (*crt_bundle_attach)(void *conf); /*!< Pointer to ESP x509 Certificate Bundle attach function for the usage of certification bundles in mqtts */
    // pub alpn_protos: &'a [&'a str],

    // pub clientkey_password: &'a str,
    // pub skip_cert_common_name_check: bool,
    // pub use_secure_element: bool,

    // void *ds_data;                          /*!< carrier of handle for digital signature parameters */
}

impl<'a> From<&Configuration<'a>> for (esp_mqtt_client_config_t, RawCstrs) {
    fn from(conf: &Configuration<'a>) -> Self {
        let mut cstrs = RawCstrs::new();

        let c_conf = esp_mqtt_client_config_t {
            client_id: cstrs.as_nptr(conf.client_id),
            // refresh_connection: time::Duration,
            // reconnect_timeout: time::Duration,
            // network_timeout: time::Duration,
            // keepalive: Option<time::Duration>,
            ..Default::default()
        };

        (c_conf, cstrs)
    }
}

pub struct EspMqttClient(
    esp_mqtt_client_handle_t,
    Box<dyn Fn(esp_mqtt_event_handle_t)>,
);

impl EspMqttClient {
    pub fn new<'a>(
        url: impl AsRef<str>,
        conf: &'a Configuration<'a>,
    ) -> Result<(Self, EspConnection), EspError>
    where
        Self: Sized,
    {
        let queue = unsafe {
            xQueueGenericCreate(1, core::mem::size_of::<&esp_mqtt_event_handle_t>() as _, 0)
        };
        if queue.is_null() {
            esp!(ESP_FAIL)?;
        }

        let queue = Arc::new(Queue(queue));

        let connection = EspConnection(queue.clone());

        let client = Self::new_with_raw_callback(
            url,
            conf,
            Box::new(move |event_handle| EspConnection::post(&queue, event_handle)),
        )?;

        Ok((client, connection))
    }

    pub fn new_with_callback<'a>(
        url: impl AsRef<str>,
        conf: &'a Configuration<'a>,
        callback: impl for<'b> Fn(Result<client::Event<EspMessage<'b>>, EspError>) + 'static,
    ) -> Result<Self, EspError>
    where
        Self: Sized,
    {
        Self::new_with_raw_callback(
            url,
            conf,
            Box::new(move |event_handle| {
                let event = unsafe { event_handle.as_ref().unwrap() };

                callback(EspMessage::new_event(event, None))
            }),
        )
    }

    fn new_with_raw_callback<'a>(
        url: impl AsRef<str>,
        conf: &'a Configuration<'a>,
        raw_callback: Box<dyn Fn(esp_mqtt_event_handle_t)>,
    ) -> Result<Self, EspError>
    where
        Self: Sized,
    {
        let (c_conf, _cstrs) = conf.into();

        let client = unsafe { esp_mqtt_client_init(&c_conf as *const _) };
        if client.is_null() {
            esp!(ESP_FAIL)?;
        }

        let client = Self(client, Box::new(raw_callback));

        let c_url = CString::new(url.as_ref()).unwrap();

        esp!(unsafe { esp_mqtt_client_set_uri(client.0, c_url.as_ptr()) })?;

        esp!(unsafe {
            esp_mqtt_client_register_event(
                client.0,
                esp_mqtt_event_id_t_MQTT_EVENT_ANY,
                Some(Self::handle),
                &*client.1 as *const _ as *mut _,
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
        let handler_ptr = event_handler_arg as *mut Box<dyn Fn(esp_mqtt_event_handle_t)>;

        let handler = unsafe { handler_ptr.as_ref() }.unwrap();

        (handler)(event_data as _);
    }

    fn check(result: i32) -> Result<client::MessageId, EspError> {
        if result < 0 {
            esp!(result)?;
        }

        Ok(result as _)
    }
}

impl Drop for EspMqttClient {
    fn drop(&mut self) {
        esp!(unsafe { esp_mqtt_client_disconnect(self.0) }).unwrap();
        esp!(unsafe { esp_mqtt_client_stop(self.0) }).unwrap();
        esp!(unsafe { esp_mqtt_client_destroy(self.0) }).unwrap();
    }
}

impl client::Client for EspMqttClient {
    type Error = EspError;

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

unsafe impl Send for EspMqttClient {}

pub struct EspMessage<'a> {
    event: &'a esp_mqtt_event_t,
    details: client::Details,
    queue: Option<Arc<Queue>>,
}

impl<'a> EspMessage<'a> {
    #[allow(non_upper_case_globals)]
    fn new_event(
        event: &esp_mqtt_event_t,
        queue: Option<Arc<Queue>>,
    ) -> Result<client::Event<EspMessage<'_>>, EspError> {
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
            esp_mqtt_event_id_t_MQTT_EVENT_DATA => {
                Ok(client::Event::Received(EspMessage::new(event, queue)))
            }
            esp_mqtt_event_id_t_MQTT_EVENT_DELETED => Ok(client::Event::Deleted(event.msg_id as _)),
            other => panic!("Unknown message type: {}", other),
        }
    }

    fn new(event: &'a esp_mqtt_event_t, queue: Option<Arc<Queue>>) -> Self {
        let mut message = Self {
            event,
            details: client::Details::Complete(unsafe { client::TopicToken::new() }),
            queue,
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

impl<'a> Drop for EspMessage<'a> {
    fn drop(&mut self) {
        if let Some(queue) = self.queue.as_ref() {
            let mut conn_data: esp_mqtt_event_handle_t = ptr::null_mut();

            esp!(unsafe {
                xQueueReceive(
                    queue.0,
                    &mut conn_data as *mut _ as *mut _,
                    TickType_t::max_value(),
                )
            })
            .unwrap();
        }
    }
}

impl<'a> client::Message for EspMessage<'a> {
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
        Cow::Owned(from_cstr_ptr(self.event.topic).into_owned())
    }

    fn details(&self) -> &client::Details {
        &self.details
    }
}

struct Queue(QueueHandle_t);

impl Drop for Queue {
    fn drop(&mut self) {
        unsafe {
            vQueueDelete(self.0);
        }
    }
}

pub struct EspConnection(Arc<Queue>);

impl EspConnection {
    fn post(queue: &Queue, event: esp_mqtt_event_handle_t) {
        esp!(unsafe {
            xQueueGenericSend(
                queue.0,
                &event as *const _ as *const _,
                TickType_t::max_value(),
                0_i32,
            )
        })
        .unwrap();
    }
}

unsafe impl Send for EspConnection {}

impl client::Connection for EspConnection {
    type Error = EspError;

    type Message<'a> = EspMessage<'a>;

    fn next(&mut self) -> Option<Result<client::Event<Self::Message<'_>>, Self::Error>> {
        let mut conn_data: esp_mqtt_event_handle_t = ptr::null_mut();

        esp!(unsafe {
            xQueuePeek(
                self.0 .0,
                &mut conn_data as *mut _ as *mut _,
                TickType_t::max_value(),
            )
        })
        .unwrap();

        let conn_data = unsafe { conn_data.as_ref() };

        conn_data.map(|event| EspMessage::new_event(event, Some(self.0.clone())))
    }
}
