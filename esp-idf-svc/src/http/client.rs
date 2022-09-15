extern crate alloc;
use core::cell::UnsafeCell;

use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::string::ToString;

use ::log::*;

use embedded_svc::http::client::*;
use embedded_svc::http::*;
use embedded_svc::io::{Io, Read, Write};

use esp_idf_sys::*;

use uncased::{Uncased, UncasedStr};

use crate::errors::EspIOError;
use crate::handle::RawHandle;
use crate::private::common::Newtype;
use crate::private::cstr::*;

impl From<Method> for Newtype<(esp_http_client_method_t, ())> {
    fn from(method: Method) -> Self {
        Self((
            match method {
                Method::Get => esp_http_client_method_t_HTTP_METHOD_GET,
                Method::Post => esp_http_client_method_t_HTTP_METHOD_POST,
                Method::Delete => esp_http_client_method_t_HTTP_METHOD_DELETE,
                Method::Head => esp_http_client_method_t_HTTP_METHOD_HEAD,
                Method::Put => esp_http_client_method_t_HTTP_METHOD_PUT,
                Method::Options => esp_http_client_method_t_HTTP_METHOD_OPTIONS,
                Method::Copy => esp_http_client_method_t_HTTP_METHOD_COPY,
                Method::Lock => esp_http_client_method_t_HTTP_METHOD_LOCK,
                Method::MkCol => esp_http_client_method_t_HTTP_METHOD_MKCOL,
                Method::Move => esp_http_client_method_t_HTTP_METHOD_MOVE,
                Method::Propfind => esp_http_client_method_t_HTTP_METHOD_PROPFIND,
                Method::Proppatch => esp_http_client_method_t_HTTP_METHOD_PROPPATCH,
                Method::Unlock => esp_http_client_method_t_HTTP_METHOD_UNLOCK,
                Method::Notify => esp_http_client_method_t_HTTP_METHOD_NOTIFY,
                Method::Subscribe => esp_http_client_method_t_HTTP_METHOD_SUBSCRIBE,
                Method::Unsubscribe => esp_http_client_method_t_HTTP_METHOD_UNSUBSCRIBE,
                Method::Patch => esp_http_client_method_t_HTTP_METHOD_PATCH,
                method => panic!("Method {:?} is not supported", method),
            },
            (),
        ))
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "std", derive(Hash))]
pub enum FollowRedirectsPolicy {
    FollowNone,
    FollowGetHead,
    FollowAll,
}

impl Default for FollowRedirectsPolicy {
    fn default() -> Self {
        Self::FollowGetHead
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Configuration {
    pub buffer_size: Option<usize>,
    pub buffer_size_tx: Option<usize>,
    pub follow_redirects_policy: FollowRedirectsPolicy,

    pub use_global_ca_store: bool,
    #[cfg(not(esp_idf_version = "4.3"))]
    pub crt_bundle_attach: Option<unsafe extern "C" fn(conf: *mut c_types::c_void) -> esp_err_t>,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum State {
    New,
    Request,
    Response,
}

#[allow(clippy::type_complexity)]
pub struct EspHttpConnection {
    raw_client: esp_http_client_handle_t,
    follow_redirects_policy: FollowRedirectsPolicy,
    event_handler: Box<Option<Box<dyn Fn(&esp_http_client_event_t) -> esp_err_t>>>,
    state: State,
    request_content_len: u64,
    follow_redirects: bool,
    headers: BTreeMap<Uncased<'static>, String>,
    content_len_header: UnsafeCell<Option<Option<String>>>,
}

impl EspHttpConnection {
    pub fn new(configuration: &Configuration) -> Result<Self, EspError> {
        let event_handler = Box::new(None);

        let mut native_config = esp_http_client_config_t {
            // The ESP-IDF HTTP client is really picky on being initialized with a valid URL
            // So we set something here, which will be changed later anyway, in the request() method
            url: b"http://127.0.0.1\0".as_ptr() as *const _,
            event_handler: Some(Self::on_events),
            user_data: &*event_handler as *const _ as *mut c_types::c_void,

            use_global_ca_store: configuration.use_global_ca_store,
            #[cfg(not(esp_idf_version = "4.3"))]
            crt_bundle_attach: configuration.crt_bundle_attach,

            ..Default::default()
        };

        if let Some(buffer_size) = configuration.buffer_size {
            native_config.buffer_size = buffer_size as _;
        };

        if let Some(buffer_size_tx) = configuration.buffer_size_tx {
            native_config.buffer_size_tx = buffer_size_tx as _;
        }

        let raw_client = unsafe { esp_http_client_init(&native_config) };
        if raw_client.is_null() {
            Err(EspError::from(ESP_FAIL).unwrap())
        } else {
            Ok(Self {
                raw_client,
                follow_redirects_policy: configuration.follow_redirects_policy,
                event_handler,
                state: State::New,
                request_content_len: 0,
                follow_redirects: false,
                headers: BTreeMap::new(),
                content_len_header: UnsafeCell::new(None),
            })
        }
    }

    pub fn status(&self) -> u16 {
        unsafe { esp_http_client_get_status_code(self.raw_client) as _ }
    }

    pub fn status_message(&self) -> Option<&str> {
        None
    }

    pub fn header(&self, name: &str) -> Option<&str> {
        if name.eq_ignore_ascii_case("Content-Length") {
            if let Some(content_len_opt) =
                unsafe { self.content_len_header.get().as_mut().unwrap() }.as_ref()
            {
                content_len_opt.as_ref().map(|s| s.as_str())
            } else {
                let content_len = unsafe { esp_http_client_get_content_length(self.raw_client) };
                *unsafe { self.content_len_header.get().as_mut().unwrap() } = if content_len >= 0 {
                    Some(Some(content_len.to_string()))
                } else {
                    None
                };

                unsafe { self.content_len_header.get().as_mut().unwrap() }
                    .as_ref()
                    .and_then(|s| s.as_ref().map(|s| s.as_ref()))
            }
        } else {
            self.headers.get(UncasedStr::new(name)).map(|s| s.as_str())
        }
    }

    pub fn initiate_request<'a>(
        &'a mut self,
        method: Method,
        uri: &'a str,
        headers: &'a [(&'a str, &'a str)],
    ) -> Result<(), EspError> {
        let c_uri = CString::new(uri).unwrap();

        esp!(unsafe { esp_http_client_set_url(self.raw_client, c_uri.as_ptr() as _) })?;
        esp!(unsafe {
            esp_http_client_set_method(
                self.raw_client,
                Newtype::<(esp_http_client_method_t, ())>::from(method).0 .0,
            )
        })?;

        let mut content_len = None;

        for (name, value) in headers {
            if name.eq_ignore_ascii_case("Content-Length") {
                if let Ok(len) = value.parse::<u64>() {
                    content_len = Some(len);
                }
            }

            let c_name = CString::new(*name).unwrap();

            // TODO: Replace with a proper conversion from UTF8 to ISO-8859-1
            let c_value = CString::new(*value).unwrap();

            esp!(unsafe {
                esp_http_client_set_header(
                    self.raw_client,
                    c_name.as_ptr() as _,
                    c_value.as_ptr() as _,
                )
            })?;
        }

        self.follow_redirects = match self.follow_redirects_policy {
            FollowRedirectsPolicy::FollowAll => true,
            FollowRedirectsPolicy::FollowGetHead => method == Method::Get || method == Method::Head,
            _ => false,
        };

        self.request_content_len = content_len.unwrap_or(0);

        esp!(unsafe { esp_http_client_open(self.raw_client, self.request_content_len as _) })?;

        self.state = State::Request;

        Ok(())
    }

    pub fn assert_request(&mut self) -> Result<(), EspError> {
        if self.state == State::Request {
            Ok(())
        } else {
            Err(EspError::from(ESP_FAIL).unwrap().into())
        }
    }

    pub fn initiate_response(&mut self) -> Result<(), EspError> {
        self.fetch_headers()?;

        self.state = State::Response;

        Ok(())
    }

    pub fn split(&mut self) -> Result<(&EspHttpConnection, &mut Self), EspError> {
        if self.state == State::Response {
            let headers_ptr: *const EspHttpConnection = self as *const _;

            let headers = unsafe { headers_ptr.as_ref().unwrap() };

            Ok((headers, self))
        } else {
            Err(EspError::from(ESP_FAIL).unwrap())
        }
    }

    pub fn headers(&self) -> Result<&Self, EspError> {
        if self.state == State::Response {
            Ok(self)
        } else {
            Err(EspError::from(ESP_FAIL).unwrap())
        }
    }

    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
        if self.state == State::Response {
            let result = unsafe {
                esp_http_client_read_response(
                    self.raw_client,
                    buf.as_mut_ptr() as _,
                    buf.len() as _,
                )
            };
            if result < 0 {
                esp!(result)?;
            }

            Ok(result as _)
        } else {
            Err(EspError::from(ESP_FAIL).unwrap())
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspError> {
        if self.state == State::Request {
            let result = unsafe {
                esp_http_client_write(self.raw_client, buf.as_ptr() as _, buf.len() as _)
            };
            if result < 0 {
                esp!(result)?;
            }

            Ok(result as _)
        } else {
            Err(EspError::from(ESP_FAIL).unwrap().into())
        }
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        if self.state == State::Request {
            Ok(())
        } else {
            Err(EspError::from(ESP_FAIL).unwrap())
        }
    }

    extern "C" fn on_events(event: *mut esp_http_client_event_t) -> esp_err_t {
        match unsafe { event.as_mut() } {
            Some(event) => {
                let handler = event.user_data
                    as *const Option<Box<dyn Fn(&esp_http_client_event_t) -> esp_err_t>>;
                if let Some(handler) = unsafe { handler.as_ref() } {
                    if let Some(handler) = handler.as_ref() {
                        return handler(event);
                    }
                }

                ESP_OK as _
            }
            None => ESP_FAIL as _,
        }
    }

    fn fetch_headers(&mut self) -> Result<(), EspError> {
        self.headers.clear();
        *self.content_len_header.get_mut() = None;

        loop {
            // TODO: Implement a mechanism where the client can declare in which header it is interested
            let headers_ptr = &mut self.headers as *mut BTreeMap<Uncased, String>;

            let handler = move |event: &esp_http_client_event_t| {
                if event.event_id == esp_http_client_event_id_t_HTTP_EVENT_ON_HEADER {
                    unsafe {
                        // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8

                        headers_ptr.as_mut().unwrap().insert(
                            Uncased::from(from_cstr_ptr(event.header_key).to_string()),
                            from_cstr_ptr(event.header_value).to_string(),
                        );
                    }
                }

                ESP_OK as esp_err_t
            };

            self.register_handler(handler);

            let result = unsafe { esp_http_client_fetch_headers(self.raw_client) };

            self.deregister_handler();

            if result < 0 {
                esp!(result)?;
            }

            trace!("Fetched headers: {:?}", self.headers);

            if self.follow_redirects {
                let status = unsafe { esp_http_client_get_status_code(self.raw_client) as u16 };

                if status::REDIRECT.contains(&status) {
                    info!("Got response {}, about to follow redirect", status);

                    let mut len = 0_i32;
                    esp!(unsafe { esp_http_client_flush_response(self.raw_client, &mut len) })?;
                    esp!(unsafe {
                        esp_http_client_set_method(
                            self.raw_client,
                            esp_http_client_method_t_HTTP_METHOD_GET,
                        )
                    })?;
                    esp!(unsafe { esp_http_client_set_redirection(self.raw_client) })?;
                    esp!(unsafe {
                        esp_http_client_open(self.raw_client, self.request_content_len as _)
                    })?;

                    self.headers.clear();

                    continue;
                }
            }

            break;
        }

        Ok(())
    }

    fn register_handler(
        &mut self,
        handler: impl Fn(&esp_http_client_event_t) -> esp_err_t + 'static,
    ) {
        *self.event_handler = Some(Box::new(handler));
    }

    fn deregister_handler(&mut self) {
        *self.event_handler = None;
    }
}

impl Drop for EspHttpConnection {
    fn drop(&mut self) {
        esp!(unsafe { esp_http_client_cleanup(self.raw_client) })
            .expect("Unable to stop the client cleanly");
    }
}

impl RawHandle for EspHttpConnection {
    type Handle = esp_http_client_handle_t;

    unsafe fn handle(&self) -> Self::Handle {
        self.raw_client
    }
}

impl Status for EspHttpConnection {
    fn status(&self) -> u16 {
        EspHttpConnection::status(self)
    }

    fn status_message(&self) -> Option<&str> {
        EspHttpConnection::status_message(self)
    }
}

impl Headers for EspHttpConnection {
    fn header(&self, name: &str) -> Option<&str> {
        EspHttpConnection::header(self, name)
    }
}

impl Io for EspHttpConnection {
    type Error = EspIOError;
}

impl Read for EspHttpConnection {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let size = EspHttpConnection::read(self, buf)?;

        Ok(size)
    }
}

impl Write for EspHttpConnection {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let size = EspHttpConnection::write(self, buf)?;

        Ok(size)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        EspHttpConnection::flush(self).map_err(EspIOError)
    }
}

impl Connection for EspHttpConnection {
    type Headers = Self;

    type Read = Self;

    type RawConnectionError = EspIOError;

    type RawConnection = Self;

    fn initiate_request<'a>(
        &'a mut self,
        method: Method,
        uri: &'a str,
        headers: &'a [(&'a str, &'a str)],
    ) -> Result<(), Self::Error> {
        EspHttpConnection::initiate_request(self, method, uri, headers).map_err(EspIOError)
    }

    fn assert_request(&mut self) -> Result<(), Self::Error> {
        EspHttpConnection::assert_request(self).map_err(EspIOError)
    }

    fn initiate_response(&mut self) -> Result<(), Self::Error> {
        EspHttpConnection::initiate_response(self).map_err(EspIOError)
    }

    fn split(&mut self) -> Result<(&Self::Headers, &mut Self::Read), Self::Error> {
        EspHttpConnection::split(self).map_err(EspIOError)
    }

    fn headers(&self) -> Result<&Self::Headers, Self::Error> {
        EspHttpConnection::headers(self).map_err(EspIOError)
    }

    fn raw_connection(&mut self) -> Result<&mut Self::RawConnection, Self::Error> {
        Err(EspError::from(ESP_FAIL).unwrap().into())
    }
}
