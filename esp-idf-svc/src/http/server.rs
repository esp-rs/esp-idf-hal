use core::{cell::RefCell, fmt, marker::PhantomData, ptr, time::*};

extern crate alloc;
use alloc::borrow::Cow;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;

use log::info;

use crate::private::cstr::CString;

use embedded_svc::http::server::{
    attr, middleware, registry::*, session, Completion, Request, Response, ResponseWrite, Session,
};
use embedded_svc::http::*;
use embedded_svc::io::{Read, Write};

use esp_idf_sys::*;

use crate::private::common::Newtype;

#[derive(Copy, Clone, Debug)]
pub struct Configuration {
    pub http_port: u16,
    pub https_port: u16,
    pub max_sessions: usize,
    pub session_timeout: Duration,
    pub stack_size: usize,
    pub max_open_sockets: usize,
    pub max_uri_handlers: usize,
    pub max_resp_handlers: usize,
}

impl Default for Configuration {
    fn default() -> Self {
        Configuration {
            http_port: 80,
            https_port: 443,
            max_sessions: 16,
            session_timeout: Duration::from_secs(20 * 60),
            stack_size: 10240,
            max_open_sockets: 8,
            max_uri_handlers: 32,
            max_resp_handlers: 4,
        }
    }
}

impl From<&Configuration> for Newtype<httpd_config_t> {
    fn from(conf: &Configuration) -> Self {
        Self(httpd_config_t {
            task_priority: 5,
            stack_size: conf.stack_size as _,
            core_id: std::i32::MAX,
            server_port: conf.http_port,
            ctrl_port: 32768,
            max_open_sockets: conf.max_open_sockets as _,
            max_uri_handlers: conf.max_uri_handlers as _,
            max_resp_headers: conf.max_resp_handlers as _,
            backlog_conn: 5,
            lru_purge_enable: conf.https_port != 0,
            recv_wait_timeout: 5,
            send_wait_timeout: 5,
            global_user_ctx: ptr::null_mut(),
            global_user_ctx_free_fn: None,
            global_transport_ctx: ptr::null_mut(),
            global_transport_ctx_free_fn: None,
            open_fn: None,
            close_fn: None,
            uri_match_fn: None,
        })
    }
}

impl From<Method> for Newtype<c_types::c_uint> {
    fn from(method: Method) -> Self {
        Self(match method {
            Method::Get => esp_idf_sys::http_method_HTTP_GET,
            Method::Post => esp_idf_sys::http_method_HTTP_POST,
            Method::Delete => esp_idf_sys::http_method_HTTP_DELETE,
            Method::Head => esp_idf_sys::http_method_HTTP_HEAD,
            Method::Put => esp_idf_sys::http_method_HTTP_PUT,
            Method::Connect => esp_idf_sys::http_method_HTTP_CONNECT,
            Method::Options => esp_idf_sys::http_method_HTTP_OPTIONS,
            Method::Trace => esp_idf_sys::http_method_HTTP_TRACE,
            Method::Copy => esp_idf_sys::http_method_HTTP_COPY,
            Method::Lock => esp_idf_sys::http_method_HTTP_LOCK,
            Method::MkCol => esp_idf_sys::http_method_HTTP_MKCOL,
            Method::Move => esp_idf_sys::http_method_HTTP_MOVE,
            Method::Propfind => esp_idf_sys::http_method_HTTP_PROPFIND,
            Method::Proppatch => esp_idf_sys::http_method_HTTP_PROPPATCH,
            Method::Search => esp_idf_sys::http_method_HTTP_SEARCH,
            Method::Unlock => esp_idf_sys::http_method_HTTP_UNLOCK,
            Method::Bind => esp_idf_sys::http_method_HTTP_BIND,
            Method::Rebind => esp_idf_sys::http_method_HTTP_REBIND,
            Method::Unbind => esp_idf_sys::http_method_HTTP_UNBIND,
            Method::Acl => esp_idf_sys::http_method_HTTP_ACL,
            Method::Report => esp_idf_sys::http_method_HTTP_REPORT,
            Method::MkActivity => esp_idf_sys::http_method_HTTP_MKACTIVITY,
            Method::Checkout => esp_idf_sys::http_method_HTTP_CHECKOUT,
            Method::Merge => esp_idf_sys::http_method_HTTP_MERGE,
            Method::MSearch => esp_idf_sys::http_method_HTTP_MSEARCH,
            Method::Notify => esp_idf_sys::http_method_HTTP_NOTIFY,
            Method::Subscribe => esp_idf_sys::http_method_HTTP_SUBSCRIBE,
            Method::Unsubscribe => esp_idf_sys::http_method_HTTP_UNSUBSCRIBE,
            Method::Patch => esp_idf_sys::http_method_HTTP_PATCH,
            Method::Purge => esp_idf_sys::http_method_HTTP_PURGE,
            Method::MkCalendar => esp_idf_sys::http_method_HTTP_MKCALENDAR,
            Method::Link => esp_idf_sys::http_method_HTTP_LINK,
            Method::Unlink => esp_idf_sys::http_method_HTTP_UNLINK,
        })
    }
}

type EspSessionMutex = EspMutex<session::SessionState>;
type EspSessionsMutex = EspMutex<BTreeMap<String, Arc<EspSessionMutex>>>;
type EspSessions = session::Sessions<EspSessionsMutex, EspSessionMutex>;
type EspRequestScopedSession = session::RequestScopedSession<EspSessionsMutex, EspSessionMutex>;

pub struct EspHttpServer {
    sd: esp_idf_sys::httpd_handle_t,
    registrations: Vec<(CString, esp_idf_sys::httpd_uri_t)>,
    sessions: Arc<EspSessions>,
}

impl EspHttpServer {
    pub fn new(conf: &Configuration) -> Result<Self, EspError> {
        let config: Newtype<esp_idf_sys::httpd_config_t> = conf.into();

        let mut handle: esp_idf_sys::httpd_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe { esp_idf_sys::httpd_start(handle_ref, &config.0 as *const _) })?;

        info!("Started Httpd server with config {:?}", conf);

        Ok(EspHttpServer {
            sd: handle,
            registrations: vec![],
            sessions: Arc::new(EspSessions::new(
                Self::get_random,
                Self::get_current_time,
                conf.max_sessions,
                conf.session_timeout,
            )),
        })
    }

    fn get_random() -> [u8; 16] {
        let mut result = [0; 16];

        unsafe {
            esp_fill_random(
                &mut result as *mut _ as *mut c_types::c_void,
                result.len() as _,
            )
        }

        result
    }

    fn get_current_time() -> Duration {
        Duration::from_micros(unsafe { esp_timer_get_time() } as _)
    }

    fn unregister(&mut self, uri: CString, conf: esp_idf_sys::httpd_uri_t) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::httpd_unregister_uri_handler(
                self.sd,
                uri.as_ptr() as _,
                conf.method
            ))?;

            let _drop = Box::from_raw(conf.user_ctx as *mut _);
        };

        info!(
            "Unregistered Httpd server handler {:?} for URI \"{}\"",
            conf.method,
            uri.to_str().unwrap()
        );

        Ok(())
    }

    fn stop(&mut self) -> Result<(), EspError> {
        if !self.sd.is_null() {
            while !self.registrations.is_empty() {
                let (uri, registration) = self.registrations.pop().unwrap();

                self.unregister(uri, registration)?;
            }

            esp!(unsafe { esp_idf_sys::httpd_stop(self.sd) })?;

            self.sd = ptr::null_mut();
        }

        info!("Httpd server stopped");

        Ok(())
    }

    fn handle_request<'a, H, E>(
        req: EspHttpRequest<'a>,
        resp: EspHttpResponse<'a>,
        handler: &H,
    ) -> Result<(), E>
    where
        H: for<'b> Fn(EspHttpRequest<'b>, EspHttpResponse<'b>) -> Result<Completion, E>,
        E: fmt::Display + fmt::Debug,
    {
        info!("About to handle query string {:?}", req.query_string());

        handler(req, resp)?;

        Ok(())
    }

    fn handle_error<'a, E>(
        raw_req: *mut httpd_req_t,
        response_state: ResponseState,
        error: E,
    ) -> c_types::c_int
    where
        E: fmt::Display + fmt::Debug,
    {
        if response_state != ResponseState::New {
            info!("About to handle error {:?}, request is complete", error);

            1
        } else {
            info!("About to handle error {:?}, request is not complete", error);

            1 // TODO
        }
    }

    fn to_native_handler<H, E>(&self, handler: H) -> Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>
    where
        H: for<'a> Fn(EspHttpRequest<'a>, EspHttpResponse<'a>) -> Result<Completion, E> + 'static,
        E: fmt::Display + fmt::Debug,
    {
        let sessions = self.sessions.clone();

        Box::new(move |raw_req| {
            let mut response_state = ResponseState::New;

            let result = Self::handle_request(
                EspHttpRequest::new(raw_req, sessions.clone()),
                EspHttpResponse::new(raw_req, &mut response_state),
                &handler,
            );

            match result {
                Ok(()) => 0,
                Err(e) => Self::handle_error(raw_req, response_state, e),
            }
        })
    }

    extern "C" fn handle(raw_req: *mut httpd_req_t) -> c_types::c_int {
        let handler_ptr =
            (unsafe { *raw_req }).user_ctx as *mut Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>;

        let handler = unsafe { handler_ptr.as_ref() }.unwrap();

        (handler)(raw_req)
    }
}

impl Drop for EspHttpServer {
    fn drop(&mut self) {
        self.stop().expect("Unable to stop the server cleanly");
    }
}

impl Registry for EspHttpServer {
    type Request<'a> = EspHttpRequest<'a>;
    type Response<'a> = EspHttpResponse<'a>;
    type Error = EspError;
    type Root = Self;
    type MiddlewareRegistry<'q, M>
    where
        Self: 'q,
        M: middleware::Middleware<Self::Root> + Clone + 'static + 'q,
    = middleware::MiddlewareRegistry<'q, Self::Root, M>;

    fn with_middleware<M>(&mut self, middleware: M) -> Self::MiddlewareRegistry<'_, M>
    where
        M: middleware::Middleware<Self::Root> + Clone + 'static,
        M::Error: 'static,
        Self: Sized,
    {
        middleware::MiddlewareRegistry::new(self, middleware)
    }

    fn set_inline_handler<H, E>(
        &mut self,
        uri: &str,
        method: Method,
        handler: H,
    ) -> Result<&mut Self, Self::Error>
    where
        H: for<'a> Fn(Self::Request<'a>, Self::Response<'a>) -> Result<Completion, E> + 'static,
        E: fmt::Display + fmt::Debug,
    {
        let c_str = CString::new(uri).unwrap();

        let conf = esp_idf_sys::httpd_uri_t {
            uri: c_str.as_ptr() as _,
            method: Newtype::<c_types::c_uint>::from(method).0,
            user_ctx: Box::into_raw(self.to_native_handler(handler)) as *mut _,
            handler: Some(EspHttpServer::handle),
        };

        esp!(unsafe { esp_idf_sys::httpd_register_uri_handler(self.sd, &conf) })?;

        info!(
            "Registered Httpd server handler {:?} for URI \"{}\"",
            method,
            c_str.to_str().unwrap()
        );

        self.registrations.push((c_str, conf));

        Ok(self)
    }
}

pub struct EspHttpRequest<'a> {
    raw_req: *mut httpd_req_t,
    _ptr: PhantomData<&'a httpd_req_t>,
    attributes: RefCell<attr::RequestScopedAttributes>,
    session: RefCell<EspRequestScopedSession>,
}

impl<'a> EspHttpRequest<'a> {
    fn new(raw_req: *mut httpd_req_t, sessions: Arc<EspSessions>) -> Self {
        let session = RefCell::new(EspRequestScopedSession::new(
            sessions,
            EspSessions::get_session_id(Self::header(raw_req, "cookies")),
        ));

        Self {
            raw_req,
            _ptr: PhantomData,
            attributes: RefCell::new(attr::RequestScopedAttributes::new()),
            session,
        }
    }

    fn header<'b>(raw_req: *mut httpd_req_t, name: impl AsRef<str>) -> Option<Cow<'b, str>> {
        let c_name = CString::new(name.as_ref()).unwrap();

        unsafe {
            match esp_idf_sys::httpd_req_get_hdr_value_len(raw_req, c_name.as_ptr() as _) as usize {
                0 => None,
                len => {
                    // TODO: Would've been much more effective, if ESP-IDF was capable of returning a
                    // pointer to the header value that is in the scratch buffer
                    //
                    // Check if we can implement it ourselves vy traversing the scratch buffer manually

                    let mut buf: vec::Vec<u8> = Vec::with_capacity(len + 1);

                    esp_nofail!(esp_idf_sys::httpd_req_get_hdr_value_str(
                        raw_req,
                        c_name.as_ptr() as _,
                        buf.as_mut_ptr() as *mut _,
                        (len + 1) as esp_idf_sys::size_t
                    ));

                    buf.set_len(len + 1);

                    // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8
                    Some(String::from_utf8_lossy(&buf[..len]).into_owned().into())
                }
            }
        }
    }
}

impl<'a> Request<'a> for EspHttpRequest<'a> {
    type Read<'b>
    where
        'a: 'b,
    = &'b EspHttpRequest<'a>;

    type Attributes<'b>
    where
        Self: 'b,
    = attr::RequestScopedAttributesReference<'b>;

    type Session<'b>
    where
        Self: 'b,
    = session::RequestScopedSessionReference<'b, EspSessionsMutex, EspSessionMutex>;

    type Error = EspError;

    fn query_string(&self) -> Cow<'a, str> {
        unsafe {
            match esp_idf_sys::httpd_req_get_url_query_len(self.raw_req) as usize {
                0 => "".into(),
                len => {
                    // TODO: Would've been much more effective, if ESP-IDF was capable of returning a
                    // pointer to the header value that is in the scratch buffer
                    //
                    // Check if we can implement it ourselves vy traversing the scratch buffer manually

                    let mut buf: vec::Vec<u8> = Vec::with_capacity(len + 1);

                    esp_nofail!(esp_idf_sys::httpd_req_get_url_query_str(
                        self.raw_req,
                        buf.as_mut_ptr() as *mut _,
                        (len + 1) as esp_idf_sys::size_t
                    ));

                    buf.set_len(len + 1);

                    // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8
                    String::from_utf8_lossy(&buf[..len]).into_owned().into()
                }
            }
        }
    }

    fn attrs<'r>(&'r self) -> Self::Attributes<'r> {
        Self::Attributes::<'r>::new(&self.attributes)
    }

    fn session<'r>(&'r self) -> Self::Session<'r> {
        Self::Session::<'r>::new(&self.session)
    }

    fn reader(&self) -> Self::Read<'_> {
        self
    }
}

impl<'a> Read for &EspHttpRequest<'a> {
    type Error = EspError;

    fn do_read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        unsafe {
            let len = esp_idf_sys::httpd_req_recv(
                self.raw_req,
                buf.as_mut_ptr() as *mut _,
                buf.len() as esp_idf_sys::size_t,
            );

            if len < 0 {
                esp!(len)?;
            }

            Ok(len as usize)
        }
    }
}

impl<'a> Headers for EspHttpRequest<'a> {
    fn header(&self, name: impl AsRef<str>) -> Option<Cow<'a, str>> {
        EspHttpRequest::header(self.raw_req, name)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum ResponseState {
    New,
    HeadersSent,
    Opened,
    Closed,
}

pub struct EspHttpResponse<'a> {
    raw_req: *mut httpd_req_t,
    _ptr: PhantomData<&'a httpd_req_t>,
    state: &'a mut ResponseState,
    status: u16,
    status_message: Option<Cow<'a, str>>,
    headers: BTreeMap<Cow<'a, str>, Cow<'a, str>>,
    c_headers: Option<Vec<(CString, CString)>>,
    c_content_type: Option<CString>,
    c_status: Option<CString>,
}

impl<'a> EspHttpResponse<'a> {
    fn new(raw_req: *mut httpd_req_t, state: &'a mut ResponseState) -> Self {
        *state = ResponseState::New;

        Self {
            raw_req,
            _ptr: PhantomData,
            state,
            status: 200,
            status_message: None,
            headers: BTreeMap::new(),
            c_headers: None,
            c_content_type: None,
            c_status: None,
        }
    }

    fn send_headers(&mut self, session_id: Option<impl AsRef<str>>) -> Result<(), EspError> {
        *self.state = ResponseState::HeadersSent;

        // TODO: Would be much more effective if we are serializing the status line and headers directly
        // Consider implementing this on top of http_resp_send() - even though that would require implementing
        // chunking in Rust

        if let Some(session_id) = session_id {
            self.headers.insert(
                Cow::Borrowed("cookies"),
                Cow::Owned(EspSessions::insert_session_cookie(
                    self.headers.get("cookies"),
                    session_id,
                )),
            );
        }

        let status = if let Some(ref status_message) = self.status_message {
            format!("{} {}", self.status, status_message)
        } else {
            self.status.to_string()
        };

        let c_status = CString::new(status.as_str()).unwrap();

        esp!(unsafe { esp_idf_sys::httpd_resp_set_status(self.raw_req, c_status.as_ptr() as _) })?;

        let mut c_headers: Vec<(CString, CString)> = vec![];
        let mut c_content_type: Option<CString> = None;
        //let content_len: Option<usize> = None;

        for (key, value) in &self.headers {
            if key.as_ref().eq_ignore_ascii_case("Content-Type") {
                c_content_type = Some(CString::new(value.as_ref()).unwrap());
            } else if key.as_ref().eq_ignore_ascii_case("Content-Length") {
                // TODO: Skip this header for now, as we are doing a chunked delivery anyway
                // content_len = Some(
                //     value
                //         .as_ref()
                //         .parse::<usize>()
                //         .map_err(|_| EspError::from(ESP_ERR_INVALID_ARG as _).unwrap())?,
                // );
            } else {
                c_headers.push((
                    CString::new(key.as_ref()).unwrap(),
                    // TODO: Replace with a proper conversion from UTF8 to ISO-8859-1
                    CString::new(value.as_ref()).unwrap(),
                ))
            }
        }

        if let Some(c_content_type) = c_content_type.as_ref() {
            esp!(unsafe {
                esp_idf_sys::httpd_resp_set_type(self.raw_req, c_content_type.as_ptr())
            })?
        }

        for (c_field, c_value) in &c_headers {
            esp!(unsafe {
                esp_idf_sys::httpd_resp_set_hdr(
                    self.raw_req,
                    c_field.as_ptr() as _,
                    c_value.as_ptr() as _,
                )
            })?;
        }

        self.c_headers = Some(c_headers);
        self.c_content_type = c_content_type;
        self.c_status = Some(c_status);

        Ok(())
    }
}

impl<'a> SendStatus<'a> for EspHttpResponse<'a> {
    fn set_status(&mut self, status: u16) -> &mut Self {
        self.status = status;
        self
    }

    fn set_status_message<M>(&mut self, message: M) -> &mut Self
    where
        M: Into<Cow<'a, str>>,
    {
        self.status_message = Some(message.into());
        self
    }
}

impl<'a> SendHeaders<'a> for EspHttpResponse<'a> {
    fn set_header<H, V>(&mut self, name: H, value: V) -> &mut Self
    where
        H: Into<Cow<'a, str>>,
        V: Into<Cow<'a, str>>,
    {
        // TODO: Optimize; convert everything to lower case (or make the map case insensitive)
        *self.headers.entry(name.into()).or_insert(Cow::Borrowed("")) = value.into();
        self
    }
}

impl<'a> Response<'a> for EspHttpResponse<'a> {
    type Write<'b> = EspHttpResponse<'b>;
    type Error = EspError;

    fn into_writer(mut self, request: impl Request<'a>) -> Result<Self::Write<'a>, Self::Error> {
        let session = request.session();
        let valid = session.is_valid();

        let session_id = session
            .id()
            .map(|session_id| session_id.to_owned()) // FIXME
            .and_then(|session_id| if valid { Some(session_id) } else { None });

        self.send_headers(session_id)?;

        Ok(self)
    }
}

impl<'a> ResponseWrite<'a> for EspHttpResponse<'a> {
    fn complete(self) -> Result<Completion, Self::Error> {
        esp!(unsafe {
            esp_idf_sys::httpd_resp_send_chunk(self.raw_req, core::ptr::null() as *const _, 0)
        })?;

        *self.state = ResponseState::Closed;

        Ok(unsafe { Completion::internal_new() })
    }
}

impl<'a> Write for EspHttpResponse<'a> {
    type Error = EspError;

    fn do_write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !buf.is_empty() {
            esp!(unsafe {
                esp_idf_sys::httpd_resp_send_chunk(
                    self.raw_req,
                    buf.as_ptr() as *const _,
                    buf.len() as esp_idf_sys::ssize_t,
                )
            })?;

            *self.state = ResponseState::Opened;
        }

        Ok(buf.len())
    }
}
