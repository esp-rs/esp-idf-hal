extern crate alloc;
use alloc::borrow::Cow;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::string::ToString;

use ::log::*;

use embedded_svc::http::client::*;
use embedded_svc::http::*;
use embedded_svc::io::{Read, Write};

use esp_idf_sys::*;

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
pub struct EspHttpClientConfiguration {
    pub buffer_size: Option<usize>,
    pub follow_redirects_policy: FollowRedirectsPolicy,
}

#[allow(clippy::type_complexity)]
pub struct EspHttpClient {
    raw: esp_http_client_handle_t,
    follow_redirects_policy: FollowRedirectsPolicy,
    event_handler: Box<Option<Box<dyn Fn(&esp_http_client_event_t) -> esp_err_t>>>,
}

impl EspHttpClient {
    pub fn new_default() -> Result<Self, EspError> {
        Self::new(&Default::default())
    }

    pub fn new(configuration: &EspHttpClientConfiguration) -> Result<Self, EspError> {
        let event_handler = Box::new(None);

        let mut native_config = esp_http_client_config_t {
            // The ESP-IDF HTTP client is really picky on being initialized with a valid URL
            // So we set something here, which will be changed later anyway, in the request() method
            url: b"http://127.0.0.1\0".as_ptr() as *const _,
            event_handler: Some(Self::on_events),
            user_data: &*event_handler as *const _ as *mut c_types::c_void,
            ..Default::default()
        };

        if let Some(buffer_size) = configuration.buffer_size {
            native_config.buffer_size = buffer_size as _;
        };

        let raw = unsafe { esp_http_client_init(&native_config) };
        if raw.is_null() {
            Err(EspError::from(ESP_FAIL).unwrap())
        } else {
            Ok(Self {
                raw,
                follow_redirects_policy: configuration.follow_redirects_policy,
                event_handler,
            })
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
}

impl Drop for EspHttpClient {
    fn drop(&mut self) {
        esp!(unsafe { esp_http_client_cleanup(self.raw) })
            .expect("Unable to stop the client cleanly");
    }
}

impl Client for EspHttpClient {
    type Request<'a> = EspHttpRequest<'a>;

    type Error = EspError;

    fn request(
        &mut self,
        method: Method,
        url: impl AsRef<str>,
    ) -> Result<Self::Request<'_>, Self::Error> {
        let c_url = CString::new(url.as_ref()).unwrap();

        esp!(unsafe { esp_http_client_set_url(self.raw, c_url.as_ptr() as _) })?;
        esp!(unsafe {
            esp_http_client_set_method(
                self.raw,
                Newtype::<(esp_http_client_method_t, ())>::from(method).0 .0,
            )
        })?;

        let follow_redirects = match self.follow_redirects_policy {
            FollowRedirectsPolicy::FollowAll => true,
            FollowRedirectsPolicy::FollowGetHead => method == Method::Get || method == Method::Head,
            _ => false,
        };

        Ok(EspHttpRequest {
            client: self,
            follow_redirects,
        })
    }
}

pub struct EspHttpRequest<'a> {
    client: &'a mut EspHttpClient,
    follow_redirects: bool,
}

impl<'a> Request<'a> for EspHttpRequest<'a> {
    type Write<'b> = EspHttpRequestWrite<'b>;

    type Error = EspError;

    fn into_writer(self, size: usize) -> Result<Self::Write<'a>, Self::Error> {
        esp!(unsafe { esp_http_client_open(self.client.raw, size as _) })?;

        Ok(Self::Write::<'a> {
            client: self.client,
            follow_redirects: self.follow_redirects,
            size,
        })
    }
}

impl<'a> SendHeaders<'a> for EspHttpRequest<'a> {
    fn set_header<H, V>(&mut self, name: H, value: V) -> &mut Self
    where
        H: Into<Cow<'a, str>>,
        V: Into<Cow<'a, str>>,
    {
        let c_name = CString::new(name.into().as_ref()).unwrap();

        // TODO: Replace with a proper conversion from UTF8 to ISO-8859-1
        let c_value = CString::new(value.into().as_ref()).unwrap();

        esp!(unsafe {
            esp_http_client_set_header(self.client.raw, c_name.as_ptr() as _, c_value.as_ptr() as _)
        })
        .unwrap();

        self
    }
}

pub struct EspHttpRequestWrite<'a> {
    client: &'a mut EspHttpClient,
    follow_redirects: bool,
    size: usize,
}

impl<'a> EspHttpRequestWrite<'a> {
    fn fetch_headers(&mut self) -> Result<BTreeMap<String, String>, EspError> {
        let mut headers = BTreeMap::new();

        loop {
            // TODO: Implement a mechanism where the client can declare in which header it is interested
            let headers_ptr = &mut headers as *mut BTreeMap<String, String>;

            let handler = move |event: &esp_http_client_event_t| {
                if event.event_id == esp_http_client_event_id_t_HTTP_EVENT_ON_HEADER {
                    unsafe {
                        // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8

                        headers_ptr.as_mut().unwrap().insert(
                            from_cstr_ptr(event.header_key).into_owned(),
                            from_cstr_ptr(event.header_value).into_owned(),
                        );
                    }
                }

                ESP_OK as esp_err_t
            };

            self.register_handler(handler);

            let result = unsafe { esp_http_client_fetch_headers(self.client.raw) };

            self.deregister_handler();

            if result < 0 {
                esp!(result)?;
            }

            trace!("Fetched headers: {:?}", headers);

            if self.follow_redirects {
                let status = unsafe { esp_http_client_get_status_code(self.client.raw) as u16 };

                if status::REDIRECT.contains(&status) {
                    info!("Got response {}, about to follow redirect", status);

                    let mut len = 0_i32;
                    esp!(unsafe { esp_http_client_flush_response(self.client.raw, &mut len) })?;
                    esp!(unsafe {
                        esp_http_client_set_method(
                            self.client.raw,
                            esp_http_client_method_t_HTTP_METHOD_GET,
                        )
                    })?;
                    esp!(unsafe { esp_http_client_set_redirection(self.client.raw) })?;
                    esp!(unsafe { esp_http_client_open(self.client.raw, self.size as _) })?;

                    headers.clear();

                    continue;
                }
            }

            break;
        }

        Ok(headers)
    }

    fn register_handler(
        &mut self,
        handler: impl Fn(&esp_http_client_event_t) -> esp_err_t + 'static,
    ) {
        *self.client.event_handler = Some(Box::new(handler));
    }

    fn deregister_handler(&mut self) {
        *self.client.event_handler = None;
    }
}

impl<'a> RequestWrite<'a> for EspHttpRequestWrite<'a> {
    type Response = EspHttpResponse<'a>;

    fn into_response(mut self) -> Result<Self::Response, Self::Error> {
        let headers = self.fetch_headers()?;

        Ok(EspHttpResponse {
            client: self.client,
            headers,
        })
    }
}

impl<'a> Write for EspHttpRequestWrite<'a> {
    type Error = EspError;

    fn do_write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let result =
            unsafe { esp_http_client_write(self.client.raw, buf.as_ptr() as _, buf.len() as _) };
        if result < 0 {
            esp!(result)?;
        }

        Ok(result as _)
    }
}

pub struct EspHttpResponse<'a> {
    client: &'a mut EspHttpClient,
    headers: BTreeMap<String, String>,
}

impl<'a> Response for EspHttpResponse<'a> {
    type Read<'b>
    where
        'a: 'b,
    = &'b EspHttpResponse<'a>;

    type Error = EspError;

    fn reader(&self) -> Self::Read<'_> {
        self
    }
}

impl<'a> Headers for EspHttpResponse<'a> {
    fn header(&self, name: impl AsRef<str>) -> Option<Cow<'_, str>> {
        if name.as_ref().eq_ignore_ascii_case("Content-Length") {
            self.content_len().map(|l| Cow::Owned(l.to_string()))
        } else {
            self.headers
                .get(name.as_ref())
                .map(|s| Cow::Borrowed(s.as_str()))
        }
    }

    fn content_len(&self) -> Option<usize> {
        let content_length = unsafe { esp_http_client_get_content_length(self.client.raw) };

        if content_length >= 0 {
            Some(content_length as usize)
        } else {
            None
        }
    }
}

impl<'a> Status for EspHttpResponse<'a> {
    fn status(&self) -> u16 {
        unsafe { esp_http_client_get_status_code(self.client.raw) as _ }
    }

    fn status_message(&self) -> Option<Cow<'_, str>> {
        None
    }
}

impl<'a> Read for &EspHttpResponse<'a> {
    type Error = EspError;

    fn do_read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let result = unsafe {
            esp_http_client_read_response(self.client.raw, buf.as_mut_ptr() as _, buf.len() as _)
        };
        if result < 0 {
            esp!(result)?;
        }

        Ok(result as _)
    }
}
