use core::cell::UnsafeCell;
use core::fmt::{Debug, Display, Write as _};
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::*;

extern crate alloc;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use log::{info, warn};

use embedded_svc::http::server::{registry::Registry, Handler, HandlerError, Request, Response};
use embedded_svc::http::*;
use embedded_svc::io::{Io, Read, Write};

use esp_idf_hal::mutex;

use esp_idf_sys::*;

use uncased::{Uncased, UncasedStr};

use crate::errors::EspIOError;
use crate::private::common::Newtype;
use crate::private::cstr::CString;

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
    pub lru_purge_enable: bool,
    pub session_cookie_name: &'static str,
}

impl Default for Configuration {
    fn default() -> Self {
        Configuration {
            http_port: 80,
            https_port: 443,
            max_sessions: 16,
            session_timeout: Duration::from_secs(20 * 60),
            stack_size: 6144,
            max_open_sockets: 4,
            max_uri_handlers: 32,
            max_resp_handlers: 8,
            lru_purge_enable: true,
            session_cookie_name: "SESSIONID",
        }
    }
}

impl From<&Configuration> for Newtype<httpd_config_t> {
    fn from(conf: &Configuration) -> Self {
        Self(httpd_config_t {
            task_priority: 5,
            stack_size: conf.stack_size as _,
            core_id: core::i32::MAX,
            server_port: conf.http_port,
            ctrl_port: 32768,
            max_open_sockets: conf.max_open_sockets as _,
            max_uri_handlers: conf.max_uri_handlers as _,
            max_resp_headers: conf.max_resp_handlers as _,
            backlog_conn: 5,
            lru_purge_enable: conf.lru_purge_enable,
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
            Method::Get => http_method_HTTP_GET,
            Method::Post => http_method_HTTP_POST,
            Method::Delete => http_method_HTTP_DELETE,
            Method::Head => http_method_HTTP_HEAD,
            Method::Put => http_method_HTTP_PUT,
            Method::Connect => http_method_HTTP_CONNECT,
            Method::Options => http_method_HTTP_OPTIONS,
            Method::Trace => http_method_HTTP_TRACE,
            Method::Copy => http_method_HTTP_COPY,
            Method::Lock => http_method_HTTP_LOCK,
            Method::MkCol => http_method_HTTP_MKCOL,
            Method::Move => http_method_HTTP_MOVE,
            Method::Propfind => http_method_HTTP_PROPFIND,
            Method::Proppatch => http_method_HTTP_PROPPATCH,
            Method::Search => http_method_HTTP_SEARCH,
            Method::Unlock => http_method_HTTP_UNLOCK,
            Method::Bind => http_method_HTTP_BIND,
            Method::Rebind => http_method_HTTP_REBIND,
            Method::Unbind => http_method_HTTP_UNBIND,
            Method::Acl => http_method_HTTP_ACL,
            Method::Report => http_method_HTTP_REPORT,
            Method::MkActivity => http_method_HTTP_MKACTIVITY,
            Method::Checkout => http_method_HTTP_CHECKOUT,
            Method::Merge => http_method_HTTP_MERGE,
            Method::MSearch => http_method_HTTP_MSEARCH,
            Method::Notify => http_method_HTTP_NOTIFY,
            Method::Subscribe => http_method_HTTP_SUBSCRIBE,
            Method::Unsubscribe => http_method_HTTP_UNSUBSCRIBE,
            Method::Patch => http_method_HTTP_PATCH,
            Method::Purge => http_method_HTTP_PURGE,
            Method::MkCalendar => http_method_HTTP_MKCALENDAR,
            Method::Link => http_method_HTTP_LINK,
            Method::Unlink => http_method_HTTP_UNLINK,
        })
    }
}

static OPEN_SESSIONS: mutex::Mutex<BTreeMap<(u32, c_types::c_int), Arc<AtomicBool>>> =
    mutex::Mutex::new(BTreeMap::new());
#[allow(clippy::type_complexity)]
static mut CLOSE_HANDLERS: mutex::Mutex<BTreeMap<u32, Vec<Box<dyn Fn(c_types::c_int)>>>> =
    mutex::Mutex::new(BTreeMap::new());

pub struct EspHttpServer {
    sd: httpd_handle_t,
    registrations: Vec<(CString, esp_idf_sys::httpd_uri_t)>,
}

impl EspHttpServer {
    pub fn new(conf: &Configuration) -> Result<Self, EspIOError> {
        let mut config: Newtype<httpd_config_t> = conf.into();
        config.0.close_fn = Some(Self::close_fn);

        let mut handle: httpd_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe { httpd_start(handle_ref, &config.0 as *const _) })?;

        info!("Started Httpd server with config {:?}", conf);

        let server = EspHttpServer {
            sd: handle,
            registrations: Vec::new(),
        };

        unsafe {
            CLOSE_HANDLERS.lock().insert(server.sd as _, Vec::new());
        }

        Ok(server)
    }

    fn unregister(&mut self, uri: CString, conf: httpd_uri_t) -> Result<(), EspIOError> {
        unsafe {
            esp!(httpd_unregister_uri_handler(
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

    fn stop(&mut self) -> Result<(), EspIOError> {
        if !self.sd.is_null() {
            while !self.registrations.is_empty() {
                let (uri, registration) = self.registrations.pop().unwrap();

                self.unregister(uri, registration)?;
            }

            esp!(unsafe { esp_idf_sys::httpd_stop(self.sd) })?;

            unsafe { CLOSE_HANDLERS.lock() }.remove(&(self.sd as u32));

            self.sd = ptr::null_mut();
        }

        info!("Httpd server stopped");

        Ok(())
    }

    fn handle_request<'a, H>(
        req: EspHttpRequest<'a>,
        resp: EspHttpResponse<'a>,
        handler: &H,
    ) -> Result<(), HandlerError>
    where
        H: Handler<EspHttpRequest<'a>, EspHttpResponse<'a>>,
    {
        info!("About to handle query string {:?}", req.query_string());

        handler.handle(req, resp)?;

        Ok(())
    }

    fn complete(
        raw_req: *mut httpd_req_t,
        headers: &mut Option<EspHttpResponseHeaders>,
    ) -> Result<(), HandlerError> {
        let buf = &[];

        if let Some(headers) = headers.as_mut() {
            headers.send(raw_req)?;

            esp!(unsafe { httpd_resp_send(raw_req, buf.as_ptr() as *const _, 0) })?;
        } else {
            esp!(unsafe { httpd_resp_send_chunk(raw_req, buf.as_ptr() as *const _, 0) })?;
        }

        Ok(())
    }

    fn handle_error<E>(raw_req: *mut httpd_req_t, response_sent: bool, error: E) -> c_types::c_int
    where
        E: Display,
    {
        if !response_sent {
            info!(
                "About to handle internal error [{}], response not sent yet",
                &error
            );

            if let Err(error2) = Self::render_error(raw_req, &error) {
                warn!(
                    "Internal error[{}] while rendering another internal error:\n{}",
                    error2, error
                );
            }
        } else {
            warn!(
                "Unhandled internal error [{}], response is already sent",
                error
            );
        }

        ESP_OK as _
    }

    fn render_error<E>(raw_req: *mut httpd_req_t, error: E) -> Result<(), EspIOError>
    where
        E: Display,
    {
        let mut headers = Some(EspHttpResponseHeaders::new());

        let mut writer = EspHttpResponse::new(raw_req, &mut headers)
            .status(500)
            .content_type("text/html")
            .into_writer()?;

        writer.write_all(
            format!(
                r#"
                    <!DOCTYPE html5>
                    <html>
                        <body style="font-family: Verdana, Sans;">
                            <h1>INTERNAL ERROR</h1>
                            <hr>
                            <pre>{}</pre>
                        <body>
                    </html>
                "#,
                error
            )
            .as_bytes(),
        )?;

        let buf = &[];

        esp!(unsafe { httpd_resp_send_chunk(raw_req, buf.as_ptr() as *const _, 0) })?;

        Ok(())
    }

    fn to_native_handler<H>(&self, handler: H) -> Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>
    where
        H: for<'a> Handler<EspHttpRequest<'a>, EspHttpResponse<'a>> + 'static,
    {
        Box::new(move |raw_req| {
            let mut headers = Some(EspHttpResponseHeaders::new());

            let result = Self::handle_request(
                EspHttpRequest::new(raw_req),
                EspHttpResponse::new(raw_req, &mut headers),
                &handler,
            )
            .and_then(|_| Self::complete(raw_req, &mut headers));

            match result {
                Ok(()) => ESP_OK as _,
                Err(e) => Self::handle_error(raw_req, headers.is_none(), e),
            }
        })
    }

    extern "C" fn handle_req(raw_req: *mut httpd_req_t) -> c_types::c_int {
        let handler_ptr =
            (unsafe { *raw_req }).user_ctx as *mut Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>;

        let handler = unsafe { handler_ptr.as_ref() }.unwrap();

        (handler)(raw_req)
    }

    extern "C" fn close_fn(sd: httpd_handle_t, sockfd: c_types::c_int) {
        {
            let mut sessions = OPEN_SESSIONS.lock();

            if let Some(closed) = sessions.remove(&(sd as u32, sockfd)) {
                closed.store(true, Ordering::SeqCst);
            }
        }

        let all_close_handlers = unsafe { CLOSE_HANDLERS.lock() };

        let close_handlers = all_close_handlers.get(&(sd as u32)).unwrap();

        for close_handler in &*close_handlers {
            (close_handler)(sockfd);
        }
    }
}

impl Drop for EspHttpServer {
    fn drop(&mut self) {
        self.stop().expect("Unable to stop the server cleanly");
    }
}

impl Registry for EspHttpServer {
    type Error = EspError;
    type IOError = EspIOError;

    type Request<'a> = EspHttpRequest<'a>;
    type Response<'a> = EspHttpResponse<'a>;

    fn set_handler<H>(
        &mut self,
        uri: &str,
        method: Method,
        handler: H,
    ) -> Result<&mut Self, Self::Error>
    where
        H: for<'a> Handler<Self::Request<'a>, Self::Response<'a>> + 'static,
    {
        let c_str = CString::new(uri).unwrap();

        #[allow(clippy::needless_update)]
        let conf = httpd_uri_t {
            uri: c_str.as_ptr() as _,
            method: Newtype::<c_types::c_uint>::from(method).0,
            user_ctx: Box::into_raw(Box::new(self.to_native_handler(handler))) as *mut _,
            handler: Some(EspHttpServer::handle_req),
            ..Default::default()
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
    id: UnsafeCell<Option<heapless::String<32>>>,
    query_string: UnsafeCell<Option<String>>,
    headers: UnsafeCell<BTreeMap<Uncased<'static>, String>>,
    _ref: PhantomData<&'a ()>,
}

impl<'a> EspHttpRequest<'a> {
    fn new(raw_req: *mut httpd_req_t) -> Self {
        Self {
            raw_req,
            id: UnsafeCell::new(None),
            query_string: UnsafeCell::new(None),
            headers: UnsafeCell::new(BTreeMap::new()),
            _ref: PhantomData,
        }
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
}

impl<'a> Io for EspHttpRequest<'a> {
    type Error = EspIOError;
}

impl<'a> Request for EspHttpRequest<'a> {
    type Read<'b>
    where
        'a: 'b,
    = &'b mut EspHttpRequest<'a>;

    fn get_request_id(&self) -> &'_ str {
        if let Some(id) = unsafe { self.id.get().as_ref().unwrap() }.as_ref() {
            id.as_ref()
        } else {
            let mut id = heapless::String::<32>::new();
            let buf = Self::get_random();

            write!(
                &mut id,
                "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9],
                buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]).unwrap();

            *unsafe { self.id.get().as_mut().unwrap() } = Some(id);

            unsafe { self.id.get().as_ref().unwrap() }.as_ref().unwrap()
        }
    }

    fn query_string(&self) -> &str {
        if let Some(query_string) = unsafe { self.query_string.get().as_ref().unwrap() }.as_ref() {
            query_string.as_ref()
        } else {
            match unsafe { httpd_req_get_url_query_len(self.raw_req) } as usize {
                0 => "",
                len => {
                    // TODO: Would've been much more effective, if ESP-IDF was capable of returning a
                    // pointer to the header value that is in the scratch buffer
                    //
                    // Check if we can implement it ourselves vy traversing the scratch buffer manually

                    let mut buf: Vec<u8> = Vec::with_capacity(len + 1);

                    esp_nofail!(unsafe {
                        httpd_req_get_url_query_str(
                            self.raw_req,
                            buf.as_mut_ptr() as *mut _,
                            (len + 1) as size_t,
                        )
                    });

                    unsafe {
                        buf.set_len(len + 1);
                    }

                    // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8
                    let query_string = String::from_utf8_lossy(&buf[..len]).into_owned();

                    *unsafe { self.query_string.get().as_mut().unwrap() } = Some(query_string);

                    unsafe { self.query_string.get().as_ref().unwrap() }
                        .as_ref()
                        .map(|s| s.as_ref())
                        .unwrap()
                }
            }
        }
    }

    fn reader(&mut self) -> Self::Read<'_> {
        self
    }
}

impl<'a> Read for &mut EspHttpRequest<'a> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        unsafe {
            let len = httpd_req_recv(
                self.raw_req,
                buf.as_mut_ptr() as *mut _,
                buf.len() as size_t,
            );

            if len < 0 {
                esp!(len)?;
            }

            Ok(len as usize)
        }
    }
}

impl<'a> Headers for EspHttpRequest<'a> {
    fn header(&self, name: &str) -> Option<&str> {
        if let Some(value) =
            unsafe { self.headers.get().as_ref().unwrap() }.get(UncasedStr::new(name))
        {
            Some(value.as_ref())
        } else {
            let c_name = CString::new(name).unwrap();

            match unsafe { httpd_req_get_hdr_value_len(self.raw_req, c_name.as_ptr() as _) }
                as usize
            {
                0 => None,
                len => {
                    // TODO: Would've been much more effective, if ESP-IDF was capable of returning a
                    // pointer to the header value that is in the scratch buffer
                    //
                    // Check if we can implement it ourselves vy traversing the scratch buffer manually

                    let mut buf: Vec<u8> = Vec::with_capacity(len + 1);

                    esp_nofail!(unsafe {
                        httpd_req_get_hdr_value_str(
                            self.raw_req,
                            c_name.as_ptr() as _,
                            buf.as_mut_ptr() as *mut _,
                            (len + 1) as size_t,
                        )
                    });

                    unsafe {
                        buf.set_len(len + 1);
                    }

                    // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8
                    let value = String::from_utf8_lossy(&buf[..len]).into_owned();
                    unsafe { self.headers.get().as_mut().unwrap() }
                        .insert(Uncased::from(name.to_owned()), value);

                    unsafe { self.headers.get().as_ref().unwrap() }
                        .get(UncasedStr::new(name))
                        .map(|s| s.as_ref())
                }
            }
        }
    }
}

pub struct EspHttpResponse<'a> {
    raw_req: *mut httpd_req_t,
    headers: &'a mut Option<EspHttpResponseHeaders>,
    _ref: PhantomData<&'a ()>,
}

struct EspHttpResponseHeaders {
    status: u16,
    status_message: Option<String>,
    headers: BTreeMap<Uncased<'static>, CString>,
    names: Vec<CString>,
}

impl<'a> EspHttpResponseHeaders {
    fn new() -> Self {
        Self {
            status: 200,
            status_message: None,
            headers: BTreeMap::new(),
            names: Vec::new(),
        }
    }

    fn send(&mut self, raw_req: *mut httpd_req_t) -> Result<(), EspIOError> {
        // TODO: Would be much more effective if we are serializing the status line and headers directly
        // Consider implementing this on top of http_resp_send() - even though that would require implementing
        // chunking in Rust

        let status = if let Some(ref status_message) = self.status_message {
            format!("{} {}", self.status, status_message)
        } else {
            self.status.to_string()
        };

        let c_status = CString::new(status.as_str()).unwrap();

        esp!(unsafe { httpd_resp_set_status(raw_req, c_status.as_ptr() as _) })?;

        for (key, value) in &self.headers {
            if key == "Content-Type" {
                esp!(unsafe { httpd_resp_set_type(raw_req, value.as_ptr()) })?;
            } else if key == "Content-Length" {
                // TODO
            } else {
                let name = CString::new(key.as_str()).unwrap();

                esp!(unsafe {
                    httpd_resp_set_hdr(raw_req, name.as_ptr() as _, value.as_ptr() as _)
                })?;

                self.names.push(name);
            }
        }

        Ok(())
    }
}

impl<'a> EspHttpResponse<'a> {
    fn new(raw_req: *mut httpd_req_t, headers: &'a mut Option<EspHttpResponseHeaders>) -> Self {
        Self {
            raw_req,
            headers,
            _ref: PhantomData,
        }
    }
}

impl<'a> SendStatus for EspHttpResponse<'a> {
    fn set_status(&mut self, status: u16) -> &mut Self {
        self.headers.as_mut().unwrap().status = status;
        self
    }

    fn set_status_message(&mut self, message: &str) -> &mut Self {
        self.headers.as_mut().unwrap().status_message = Some(message.to_owned());
        self
    }
}

impl<'a> SendHeaders for EspHttpResponse<'a> {
    fn set_header(&mut self, name: &str, value: &str) -> &mut Self {
        self.headers
            .as_mut()
            .unwrap()
            .headers
            .get_mut(UncasedStr::new(name))
            .map(|entry| *entry = CString::new(value).unwrap())
            .unwrap_or_else(|| {
                self.headers
                    .as_mut()
                    .unwrap()
                    .headers
                    .insert(Uncased::from(name.to_owned()), CString::new(value).unwrap());
            });

        self
    }
}

impl<'a> Io for EspHttpResponse<'a> {
    type Error = EspIOError;
}

impl<'a> Response for EspHttpResponse<'a> {
    type Write = EspHttpResponseWrite<'a>;

    fn into_writer(self) -> Result<EspHttpResponseWrite<'a>, EspIOError> {
        Ok(EspHttpResponseWrite::<'a> {
            raw_req: self.raw_req,
            headers: self.headers,
            _ref: self._ref,
        })
    }
}

pub struct EspHttpResponseWrite<'a> {
    raw_req: *mut httpd_req_t,
    headers: &'a mut Option<EspHttpResponseHeaders>,
    _ref: PhantomData<&'a ()>,
}

impl<'a> EspHttpResponseWrite<'a> {}

impl<'a> Io for EspHttpResponseWrite<'a> {
    type Error = EspIOError;
}

impl<'a> Write for EspHttpResponseWrite<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !buf.is_empty() {
            if let Some(headers) = self.headers {
                headers.send(self.raw_req)?;
            }

            esp!(unsafe {
                httpd_resp_send_chunk(self.raw_req, buf.as_ptr() as *const _, buf.len() as ssize_t)
            })?;

            *self.headers = None;
        }

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[cfg(esp_idf_httpd_ws_support)]
pub mod ws {
    use core::fmt::Debug;
    use core::ptr;
    use core::sync::atomic::{AtomicBool, Ordering};

    extern crate alloc;
    use alloc::sync::Arc;

    use embedded_svc::ws::server::registry::Registry;
    use log::*;

    use embedded_svc::http::Method;
    use embedded_svc::ws::server::*;
    use embedded_svc::ws::*;

    use esp_idf_sys::*;

    use esp_idf_hal::mutex::{Condvar, Mutex};

    use crate::private::common::Newtype;
    use crate::private::cstr::CString;

    use super::EspHttpServer;
    use super::CLOSE_HANDLERS;
    use super::OPEN_SESSIONS;

    pub enum EspHttpWsSender {
        Open(httpd_handle_t, *mut httpd_req_t),
        Closed(c_types::c_int),
    }

    impl EspHttpWsSender {
        fn create_raw_frame(frame_type: FrameType, frame_data: Option<&[u8]>) -> httpd_ws_frame_t {
            httpd_ws_frame_t {
                type_: match frame_type {
                    FrameType::Text(_) => httpd_ws_type_t_HTTPD_WS_TYPE_TEXT,
                    FrameType::Binary(_) => httpd_ws_type_t_HTTPD_WS_TYPE_BINARY,
                    FrameType::Ping => httpd_ws_type_t_HTTPD_WS_TYPE_PING,
                    FrameType::Pong => httpd_ws_type_t_HTTPD_WS_TYPE_PONG,
                    FrameType::Close => httpd_ws_type_t_HTTPD_WS_TYPE_CLOSE,
                    FrameType::Continue(_) => httpd_ws_type_t_HTTPD_WS_TYPE_CONTINUE,
                    FrameType::SocketClose => panic!("Cannot send SocketClose as a frame"),
                },
                final_: frame_type.is_final(),
                fragmented: frame_type.is_fragmented(),
                payload: frame_data
                    .map(|frame_data| frame_data.as_ptr() as *const _ as *mut _)
                    .unwrap_or(ptr::null_mut()),
                len: frame_data
                    .map(|frame_data| frame_data.len() as _)
                    .unwrap_or(0),
            }
        }
    }

    impl ErrorType for EspHttpWsSender {
        type Error = EspError;
    }

    impl Sender for EspHttpWsSender {
        fn send(
            &mut self,
            frame_type: FrameType,
            frame_data: Option<&[u8]>,
        ) -> Result<(), Self::Error> {
            match self {
                Self::Open(_, raw_req) => {
                    let raw_frame = Self::create_raw_frame(frame_type, frame_data);

                    esp!(unsafe {
                        httpd_ws_send_frame(*raw_req, &raw_frame as *const _ as *mut _)
                    })?;

                    Ok(())
                }
                Self::Closed(_) => {
                    esp!(ESP_FAIL)?;

                    Ok(())
                }
            }
        }
    }

    impl SenderFactory for EspHttpWsSender {
        type Sender = EspHttpWsDetachedSender;

        fn create(&self) -> Result<Self::Sender, Self::Error> {
            match self {
                Self::Open(sd, raw_req) => {
                    let fd = unsafe { httpd_req_to_sockfd(*raw_req) };

                    let mut sessions = OPEN_SESSIONS.lock();

                    let closed = sessions
                        .entry((*sd as u32, fd))
                        .or_insert_with(|| Arc::new(AtomicBool::new(false)));

                    Ok(EspHttpWsDetachedSender::new(*sd, fd, closed.clone()))
                }
                Self::Closed(_) => Err(EspError::from(ESP_FAIL).unwrap().into()),
            }
        }
    }

    impl SessionProvider for EspHttpWsSender {
        type Session = c_types::c_int;

        fn session(&self) -> Self::Session {
            match self {
                Self::Open(_, raw_req) => unsafe { httpd_req_to_sockfd(*raw_req) },
                Self::Closed(fd) => *fd,
            }
        }

        fn is_new(&self) -> bool {
            false
        }

        fn is_closed(&self) -> bool {
            matches!(self, Self::Closed(_))
        }
    }

    pub enum EspHttpWsReceiver {
        New(*mut httpd_req_t),
        Open(*mut httpd_req_t),
        Closed(c_types::c_int),
    }

    impl EspHttpWsReceiver {
        #[allow(non_upper_case_globals)]
        fn create_frame_type(raw_frame: &httpd_ws_frame_t) -> (FrameType, usize) {
            match raw_frame.type_ {
                httpd_ws_type_t_HTTPD_WS_TYPE_TEXT => (
                    FrameType::Text(raw_frame.fragmented),
                    raw_frame.len as usize + 1,
                ),
                httpd_ws_type_t_HTTPD_WS_TYPE_BINARY => {
                    (FrameType::Binary(raw_frame.fragmented), raw_frame.len as _)
                }
                httpd_ws_type_t_HTTPD_WS_TYPE_CONTINUE => {
                    (FrameType::Continue(raw_frame.final_), raw_frame.len as _)
                }
                httpd_ws_type_t_HTTPD_WS_TYPE_PING => (FrameType::Ping, 0),
                httpd_ws_type_t_HTTPD_WS_TYPE_PONG => (FrameType::Pong, 0),
                httpd_ws_type_t_HTTPD_WS_TYPE_CLOSE => (FrameType::Close, 0),
                _ => panic!("Unknown frame type: {}", raw_frame.type_),
            }
        }
    }

    impl ErrorType for EspHttpWsReceiver {
        type Error = EspError;
    }

    impl Receiver for EspHttpWsReceiver {
        fn recv(&mut self, frame_data_buf: &mut [u8]) -> Result<(FrameType, usize), Self::Error> {
            match self {
                Self::New(_) => Err(EspError::from(ESP_FAIL).unwrap().into()),
                Self::Open(raw_req) => {
                    let mut raw_frame: httpd_ws_frame_t = Default::default();

                    esp!(unsafe { httpd_ws_recv_frame(*raw_req, &mut raw_frame as *mut _, 0) })?;

                    let (frame_type, len) = Self::create_frame_type(&raw_frame);

                    if frame_data_buf.len() >= len {
                        raw_frame.payload = frame_data_buf.as_mut_ptr() as *mut _;
                        esp!(unsafe {
                            httpd_ws_recv_frame(*raw_req, &mut raw_frame as *mut _, len as _)
                        })?;
                    }

                    Ok((frame_type, len))
                }
                Self::Closed(_) => Ok((FrameType::SocketClose, 0)),
            }
        }
    }

    impl SessionProvider for EspHttpWsReceiver {
        type Session = c_types::c_int;

        fn session(&self) -> Self::Session {
            match self {
                Self::New(raw_req) | Self::Open(raw_req) => unsafe {
                    httpd_req_to_sockfd(*raw_req)
                },
                Self::Closed(fd) => *fd,
            }
        }

        fn is_new(&self) -> bool {
            matches!(self, Self::New(_))
        }

        fn is_closed(&self) -> bool {
            matches!(self, Self::Closed(_))
        }
    }

    pub struct EspWsDetachedSendRequest {
        sd: httpd_handle_t,
        fd: c_types::c_int,

        closed: Arc<AtomicBool>,

        raw_frame: *const httpd_ws_frame_t,

        error_code: Mutex<Option<u32>>,
        condvar: Condvar,
    }

    pub struct EspHttpWsDetachedSender {
        sd: httpd_handle_t,
        fd: c_types::c_int,
        closed: Arc<AtomicBool>,
    }

    impl EspHttpWsDetachedSender {
        fn new(sd: httpd_handle_t, fd: c_types::c_int, closed: Arc<AtomicBool>) -> Self {
            Self { sd, fd, closed }
        }

        extern "C" fn enqueue(arg: *mut c_types::c_void) {
            let request = unsafe { (arg as *const EspWsDetachedSendRequest).as_ref().unwrap() };

            let ret = if !request.closed.load(Ordering::SeqCst) {
                unsafe {
                    httpd_ws_send_frame_async(
                        request.sd,
                        request.fd,
                        request.raw_frame as *const _ as *mut _,
                    )
                }
            } else {
                ESP_FAIL
            };

            *request.error_code.lock() = Some(ret as _);
            request.condvar.notify_all();
        }
    }

    unsafe impl Send for EspHttpWsDetachedSender {}

    impl Clone for EspHttpWsDetachedSender {
        fn clone(&self) -> Self {
            Self {
                sd: self.sd.clone(),
                fd: self.fd.clone(),
                closed: self.closed.clone(),
            }
        }
    }

    impl ErrorType for EspHttpWsDetachedSender {
        type Error = EspError;
    }

    impl Sender for EspHttpWsDetachedSender {
        fn send(
            &mut self,
            frame_type: FrameType,
            frame_data: Option<&[u8]>,
        ) -> Result<(), Self::Error> {
            if !self.closed.load(Ordering::SeqCst) {
                let raw_frame = EspHttpWsSender::create_raw_frame(frame_type, frame_data);

                let send_request = EspWsDetachedSendRequest {
                    sd: self.sd,
                    fd: self.fd,

                    closed: self.closed.clone(),

                    raw_frame: &raw_frame as *const _,

                    error_code: Mutex::new(None),
                    condvar: Condvar::new(),
                };

                esp!(unsafe {
                    httpd_queue_work(
                        self.sd,
                        Some(Self::enqueue),
                        &send_request as *const _ as *mut _,
                    )
                })?;

                let mut guard = send_request.error_code.lock();

                while guard.is_none() {
                    guard = send_request.condvar.wait(guard);
                }

                esp!((*guard).unwrap())?;
            } else {
                esp!(ESP_FAIL)?;
            }

            Ok(())
        }
    }

    impl SessionProvider for EspHttpWsDetachedSender {
        type Session = c_types::c_int;

        fn session(&self) -> Self::Session {
            self.fd
        }

        fn is_new(&self) -> bool {
            false
        }

        fn is_closed(&self) -> bool {
            self.closed.load(Ordering::SeqCst)
        }
    }

    impl Registry for EspHttpServer {
        type Error = EspError;

        type SendReceiveError = EspError;

        type Receiver = EspHttpWsReceiver;
        type Sender = EspHttpWsSender;

        fn handle_ws<H, E>(&mut self, uri: &str, handler: H) -> Result<&mut Self, EspError>
        where
            H: for<'a> Fn(&'a mut EspHttpWsReceiver, &'a mut EspHttpWsSender) -> Result<(), E>
                + 'static,
            E: Debug,
        {
            let c_str = CString::new(uri).unwrap();

            let (req_handler, close_handler) = self.to_native_ws_handler(self.sd.clone(), handler);

            let conf = httpd_uri_t {
                uri: c_str.as_ptr() as _,
                method: Newtype::<c_types::c_uint>::from(Method::Get).0,
                user_ctx: Box::into_raw(Box::new(req_handler)) as *mut _,
                handler: Some(EspHttpServer::handle_req),
                is_websocket: true,
                // TODO: Expose as a parameter in future: handle_ws_control_frames: true,
                ..Default::default()
            };

            esp!(unsafe { esp_idf_sys::httpd_register_uri_handler(self.sd, &conf) })?;

            {
                let mut all_close_handlers = unsafe { CLOSE_HANDLERS.lock() };

                let close_handlers = all_close_handlers.get_mut(&(self.sd as u32)).unwrap();

                close_handlers.push(close_handler);
            }

            info!(
                "Registered Httpd server WS handler for URI \"{}\"",
                c_str.to_str().unwrap()
            );

            self.registrations.push((c_str, conf));

            Ok(self)
        }
    }

    impl EspHttpServer {
        fn handle_ws_request<H, E>(
            receiver: &mut EspHttpWsReceiver,
            sender: &mut EspHttpWsSender,
            handler: &H,
        ) -> Result<(), E>
        where
            H: for<'b> Fn(&'b mut EspHttpWsReceiver, &'b mut EspHttpWsSender) -> Result<(), E>,
            E: Debug,
        {
            handler(receiver, sender)?;

            Ok(())
        }

        fn handle_ws_error<'a, E>(error: E) -> c_types::c_int
        where
            E: Debug,
        {
            warn!("Unhandled internal error [{:?}]:\n{:?}", error, error);

            ESP_OK as _
        }

        fn to_native_ws_handler<H, E>(
            &self,
            server_handle: httpd_handle_t,
            handler: H,
        ) -> (
            Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>,
            Box<dyn Fn(c_types::c_int)>,
        )
        where
            H: for<'a> Fn(&'a mut EspHttpWsReceiver, &'a mut EspHttpWsSender) -> Result<(), E>
                + 'static,
            E: Debug,
        {
            let boxed_handler = Arc::new(
                move |mut receiver: EspHttpWsReceiver, mut sender: EspHttpWsSender| {
                    let result = Self::handle_ws_request(&mut receiver, &mut sender, &handler);

                    match result {
                        Ok(()) => ESP_OK as _,
                        Err(e) => Self::handle_ws_error(e),
                    }
                },
            );

            let req_handler = {
                let boxed_handler = boxed_handler.clone();

                Box::new(move |raw_req: *mut httpd_req_t| {
                    let req = unsafe { raw_req.as_ref() }.unwrap();

                    (boxed_handler)(
                        if req.method == http_method_HTTP_GET as i32 {
                            EspHttpWsReceiver::New(raw_req)
                        } else {
                            EspHttpWsReceiver::Open(raw_req)
                        },
                        EspHttpWsSender::Open(server_handle.clone(), raw_req),
                    )
                })
            };

            let close_handler = Box::new(move |fd| {
                (boxed_handler)(EspHttpWsReceiver::Closed(fd), EspHttpWsSender::Closed(fd));
            });

            (req_handler, close_handler)
        }
    }

    #[cfg(feature = "experimental")]
    pub mod asynch {
        use embedded_svc::utils::asyncify::ws::server::{AsyncAcceptor, Processor};

        use super::{EspHttpWsDetachedSender, EspHttpWsReceiver, EspHttpWsSender};

        pub type EspHttpWsProcessor<const N: usize, const F: usize> =
            Processor<esp_idf_hal::mutex::Condvar, EspHttpWsSender, EspHttpWsReceiver, N, F>;

        pub type EspHttpWsAcceptor<U> =
            AsyncAcceptor<U, esp_idf_hal::mutex::Condvar, EspHttpWsDetachedSender>;
    }
}
