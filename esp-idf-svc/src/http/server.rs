use core::cell::UnsafeCell;
use core::fmt::{Debug, Display};
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::*;
use core::{mem, ptr};

extern crate alloc;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use ::log::{info, warn};

use embedded_svc::http::headers::content_type;
use embedded_svc::http::server::{
    handler, Connection, Handler, HandlerError, HandlerResult, Request,
};
use embedded_svc::http::*;
use embedded_svc::io::{Io, Read, Write};
use embedded_svc::utils::http::server::registration::{ChainHandler, ChainRoot};

use esp_idf_sys::*;

use uncased::{Uncased, UncasedStr};

use crate::errors::EspIOError;
use crate::handle::RawHandle;
use crate::private::common::Newtype;
use crate::private::cstr::{CStr, CString};
use crate::private::mutex::{Mutex, RawMutex};

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
    #[cfg(esp_idf_esp_https_server_enable)]
    pub server_certificate: Option<&'static str>,
    #[cfg(esp_idf_esp_https_server_enable)]
    pub private_key: Option<&'static str>,
}

impl Default for Configuration {
    fn default() -> Self {
        Configuration {
            http_port: 80,
            https_port: 443,
            max_sessions: 16,
            session_timeout: Duration::from_secs(20 * 60),
            #[cfg(not(esp_idf_esp_https_server_enable))]
            stack_size: 6144,
            #[cfg(esp_idf_esp_https_server_enable)]
            stack_size: 10240,
            max_open_sockets: 4,
            max_uri_handlers: 32,
            max_resp_handlers: 8,
            lru_purge_enable: true,
            #[cfg(esp_idf_esp_https_server_enable)]
            server_certificate: None,
            #[cfg(esp_idf_esp_https_server_enable)]
            private_key: None,
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

#[allow(non_upper_case_globals)]
impl From<Newtype<c_types::c_uint>> for Method {
    fn from(method: Newtype<c_types::c_uint>) -> Self {
        match method.0 {
            http_method_HTTP_GET => Method::Get,
            http_method_HTTP_POST => Method::Post,
            http_method_HTTP_DELETE => Method::Delete,
            http_method_HTTP_HEAD => Method::Head,
            http_method_HTTP_PUT => Method::Put,
            http_method_HTTP_CONNECT => Method::Connect,
            http_method_HTTP_OPTIONS => Method::Options,
            http_method_HTTP_TRACE => Method::Trace,
            http_method_HTTP_COPY => Method::Copy,
            http_method_HTTP_LOCK => Method::Lock,
            http_method_HTTP_MKCOL => Method::MkCol,
            http_method_HTTP_MOVE => Method::Move,
            http_method_HTTP_PROPFIND => Method::Propfind,
            http_method_HTTP_PROPPATCH => Method::Proppatch,
            http_method_HTTP_SEARCH => Method::Search,
            http_method_HTTP_UNLOCK => Method::Unlock,
            http_method_HTTP_BIND => Method::Bind,
            http_method_HTTP_REBIND => Method::Rebind,
            http_method_HTTP_UNBIND => Method::Unbind,
            http_method_HTTP_ACL => Method::Acl,
            http_method_HTTP_REPORT => Method::Report,
            http_method_HTTP_MKACTIVITY => Method::MkActivity,
            http_method_HTTP_CHECKOUT => Method::Checkout,
            http_method_HTTP_MERGE => Method::Merge,
            http_method_HTTP_MSEARCH => Method::MSearch,
            http_method_HTTP_NOTIFY => Method::Notify,
            http_method_HTTP_SUBSCRIBE => Method::Subscribe,
            http_method_HTTP_UNSUBSCRIBE => Method::Unsubscribe,
            http_method_HTTP_PATCH => Method::Patch,
            http_method_HTTP_PURGE => Method::Purge,
            http_method_HTTP_MKCALENDAR => Method::MkCalendar,
            http_method_HTTP_LINK => Method::Link,
            http_method_HTTP_UNLINK => Method::Unlink,
            _ => unreachable!(),
        }
    }
}

#[cfg(esp_idf_esp_https_server_enable)]
impl From<&Configuration> for Newtype<httpd_ssl_config_t> {
    fn from(conf: &Configuration) -> Self {
        let http_config: Newtype<httpd_config_t> = conf.into();
        // start in insecure mode if no certificates are set
        let transport_mode = match (conf.server_certificate, conf.private_key) {
            (Some(_), Some(_)) => httpd_ssl_transport_mode_t_HTTPD_SSL_TRANSPORT_SECURE,
            _ => {
                warn!("Starting server in insecure mode because no certificates were set in the http config.");
                httpd_ssl_transport_mode_t_HTTPD_SSL_TRANSPORT_INSECURE
            }
        };
        // Default values taken from: https://github.com/espressif/esp-idf/blob/master/components/esp_https_server/include/esp_https_server.h#L114
        Self(httpd_ssl_config_t {
            httpd: http_config.0,
            session_tickets: false,
            port_secure: conf.https_port,
            port_insecure: conf.http_port,
            transport_mode,
            cacert_pem: ptr::null(),
            cacert_len: 0,
            prvtkey_pem: ptr::null(),
            prvtkey_len: 0,
            client_verify_cert_pem: ptr::null(),
            client_verify_cert_len: 0,
            user_cb: None,
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

#[allow(clippy::type_complexity)]
static OPEN_SESSIONS: Mutex<BTreeMap<(u32, c_types::c_int), Arc<AtomicBool>>> =
    Mutex::wrap(RawMutex::new(), BTreeMap::new());
#[allow(clippy::type_complexity)]
static mut CLOSE_HANDLERS: Mutex<BTreeMap<u32, Vec<Box<dyn Fn(c_types::c_int)>>>> =
    Mutex::wrap(RawMutex::new(), BTreeMap::new());

pub struct EspHttpServer {
    sd: httpd_handle_t,
    registrations: Vec<(CString, esp_idf_sys::httpd_uri_t)>,
}

impl EspHttpServer {
    pub fn new(conf: &Configuration) -> Result<Self, EspIOError> {
        let mut handle: httpd_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        #[cfg(not(esp_idf_esp_https_server_enable))]
        {
            let mut config: Newtype<httpd_config_t> = conf.into();
            config.0.close_fn = Some(Self::close_fn);
            esp!(unsafe { httpd_start(handle_ref, &config.0 as *const _) })?;
        }

        #[cfg(esp_idf_esp_https_server_enable)]
        {
            let mut config: Newtype<httpd_ssl_config_t> = conf.into();
            config.0.httpd.close_fn = Some(Self::close_fn);

            if let (Some(cert), Some(private_key)) = (conf.server_certificate, conf.private_key) {
                // Converting into CString has to happen in the same scope as calling http_ssl_start,
                // since the pointer passed to http_conf_t is valid only as long as the underlying bytes are in scope and valid.
                // Passing an invalid pointer causes the null byte to be overwritten, which mbedtls requires to parse the certificate.
                let cert = CString::new(cert).unwrap().into_bytes_with_nul();
                let private_key = CString::new(private_key).unwrap().into_bytes_with_nul();

                config.0.cacert_pem = cert.as_ptr();
                config.0.cacert_len = cert.len() as u32;

                config.0.prvtkey_pem = private_key.as_ptr() as _;
                config.0.prvtkey_len = private_key.len() as u32;

                esp!(unsafe { httpd_ssl_start(handle_ref, &mut config.0) })?;
            } else {
                esp!(unsafe { httpd_ssl_start(handle_ref, &mut config.0) })?;
            }
        }

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
            // Maybe its better to always call httpd_stop because httpd_ssl_stop directly wraps httpd_stop anyways
            // https://github.com/espressif/esp-idf/blob/e6fda46a02c41777f1d116a023fbec6a1efaffb9/components/esp_https_server/src/https_server.c#L268
            #[cfg(not(esp_idf_esp_https_server_enable))]
            esp!(unsafe { esp_idf_sys::httpd_stop(self.sd) })?;
            // httpd_ssl_stop doesn't return EspErr for some reason. It returns void.
            #[cfg(all(esp_idf_esp_https_server_enable, esp_idf_version_major = "4"))]
            unsafe {
                esp_idf_sys::httpd_ssl_stop(self.sd)
            };
            // esp-idf version 5 does return EspErr
            #[cfg(all(esp_idf_esp_https_server_enable, not(esp_idf_version_major = "4")))]
            esp!(unsafe { esp_idf_sys::httpd_ssl_stop(self.sd) })?;

            unsafe { CLOSE_HANDLERS.lock() }.remove(&(self.sd as u32));

            self.sd = ptr::null_mut();
        }

        info!("Httpd server stopped");

        Ok(())
    }

    pub fn handler_chain<C>(&mut self, chain: C) -> Result<&mut Self, EspError>
    where
        C: EspHttpTraversableChain,
    {
        chain.accept(self)?;

        Ok(self)
    }

    pub fn handler<H>(
        &mut self,
        uri: &str,
        method: Method,
        handler: H,
    ) -> Result<&mut Self, EspError>
    where
        H: for<'a, 'b> Handler<&'a mut EspHttpConnection<'b>> + 'static,
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

    pub fn fn_handler_chain<C>(&mut self, chain: C) -> Result<&mut Self, EspError>
    where
        C: EspHttpFnTraversableChain,
    {
        chain.accept(self)?;

        Ok(self)
    }

    pub fn fn_handler<F>(&mut self, uri: &str, method: Method, f: F) -> Result<&mut Self, EspError>
    where
        F: for<'a> Fn(Request<&mut EspHttpConnection<'a>>) -> HandlerResult + Send + 'static,
    {
        self.handler(uri, method, handler(f))
    }

    fn to_native_handler<H>(&self, handler: H) -> Box<dyn Fn(*mut httpd_req_t) -> c_types::c_int>
    where
        H: for<'a, 'b> Handler<&'a mut EspHttpConnection<'b>> + 'static,
    {
        Box::new(move |raw_req| {
            let mut connection = EspHttpConnection::new(unsafe { raw_req.as_mut().unwrap() });

            let mut result = EspHttpConnection::handle(&mut connection, &handler);

            if result.is_ok() {
                result = connection.complete();
            }

            if let Err(e) = result {
                connection.handle_error(e);
            }

            ESP_OK as _
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
        esp_nofail!(unsafe { close(sockfd) });
    }
}

impl Drop for EspHttpServer {
    fn drop(&mut self) {
        self.stop().expect("Unable to stop the server cleanly");
    }
}

impl RawHandle for EspHttpServer {
    type Handle = httpd_handle_t;

    fn handle(&self) -> Self::Handle {
        self.sd
    }
}

pub trait EspHttpTraversableChain {
    fn accept(self, server: &mut EspHttpServer) -> Result<(), EspError>;
}

impl EspHttpTraversableChain for ChainRoot {
    fn accept(self, _server: &mut EspHttpServer) -> Result<(), EspError> {
        Ok(())
    }
}

impl<H, N> EspHttpTraversableChain for ChainHandler<H, N>
where
    H: for<'a, 'b> Handler<&'a mut EspHttpConnection<'b>> + 'static,
    N: EspHttpTraversableChain,
{
    fn accept(self, server: &mut EspHttpServer) -> Result<(), EspError> {
        self.next.accept(server)?;

        server.handler(self.path, self.method, self.handler)?;

        Ok(())
    }
}

pub trait EspHttpFnTraversableChain {
    fn accept(self, server: &mut EspHttpServer) -> Result<(), EspError>;
}

impl EspHttpFnTraversableChain for ChainRoot {
    fn accept(self, _server: &mut EspHttpServer) -> Result<(), EspError> {
        Ok(())
    }
}

impl<F, N> EspHttpFnTraversableChain for ChainHandler<F, N>
where
    F: for<'a> Fn(Request<&mut EspHttpConnection<'a>>) -> HandlerResult + Send + 'static,
    N: EspHttpFnTraversableChain,
{
    fn accept(self, server: &mut EspHttpServer) -> Result<(), EspError> {
        self.next.accept(server)?;

        server.fn_handler(self.path, self.method, self.handler)?;

        Ok(())
    }
}

pub struct EspHttpRequest<'a>(&'a mut httpd_req_t);

impl<'a> RawHandle for EspHttpRequest<'a> {
    type Handle = *mut httpd_req_t;

    fn handle(&self) -> Self::Handle {
        self.0 as *const _ as *mut _
    }
}

impl<'a> Io for EspHttpRequest<'a> {
    type Error = EspIOError;
}

impl<'a> Read for EspHttpRequest<'a> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if !buf.is_empty() {
            let fd = unsafe { httpd_req_to_sockfd(self.0) };
            let len = unsafe { esp_idf_sys::read(fd, buf.as_ptr() as *mut _, buf.len() as _) };

            Ok(len as _)
        } else {
            Ok(0)
        }
    }
}

impl<'a> Write for EspHttpRequest<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if !buf.is_empty() {
            let fd = unsafe { httpd_req_to_sockfd(self.0) };
            let len = unsafe { esp_idf_sys::write(fd, buf.as_ptr() as *const _, buf.len() as _) };

            Ok(len as _)
        } else {
            Ok(0)
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

type EspHttpHeaders = BTreeMap<Uncased<'static>, String>;

pub struct EspHttpConnection<'a> {
    request: EspHttpRequest<'a>,
    headers: Option<UnsafeCell<EspHttpHeaders>>,
    response_headers: Option<Vec<CString>>,
}

impl<'a> EspHttpConnection<'a> {
    fn new(raw_req: &'a mut httpd_req_t) -> Self {
        Self {
            request: EspHttpRequest(raw_req),
            headers: Some(UnsafeCell::new(EspHttpHeaders::new())),
            response_headers: None,
        }
    }

    pub fn uri(&self) -> &str {
        self.assert_request();

        let c_uri = unsafe {
            CStr::from_bytes_with_nul_unchecked(mem::transmute(self.request.0.uri.as_slice()))
        };

        c_uri.to_str().unwrap()
    }

    pub fn method(&self) -> Method {
        self.assert_request();

        Method::from(Newtype(self.request.0.method as u32))
    }

    pub fn header(&self, name: &str) -> Option<&str> {
        self.assert_request();

        let headers = self.headers.as_ref().unwrap();

        if let Some(value) = unsafe { headers.get().as_ref().unwrap() }.get(UncasedStr::new(name)) {
            Some(value.as_ref())
        } else {
            let raw_req = self.request.0 as *const httpd_req_t as *mut httpd_req_t;

            let c_name = CString::new(name).unwrap();

            match unsafe { httpd_req_get_hdr_value_len(raw_req, c_name.as_ptr() as _) } as usize {
                0 => None,
                len => {
                    // TODO: Would've been much more effective, if ESP-IDF was capable of returning a
                    // pointer to the header value that is in the scratch buffer
                    //
                    // Check if we can implement it ourselves vy traversing the scratch buffer manually

                    let mut buf: Vec<u8> = Vec::with_capacity(len + 1);

                    esp_nofail!(unsafe {
                        httpd_req_get_hdr_value_str(
                            raw_req,
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
                    unsafe { headers.get().as_mut().unwrap() }
                        .insert(Uncased::from(name.to_owned()), value);

                    unsafe { headers.get().as_ref().unwrap() }
                        .get(UncasedStr::new(name))
                        .map(|s| s.as_ref())
                }
            }
        }
    }

    pub fn split(&mut self) -> (&EspHttpConnection<'a>, &mut Self) {
        self.assert_request();

        let headers_ptr: *const EspHttpConnection<'a> = self as *const _;

        let headers = unsafe { headers_ptr.as_ref().unwrap() };

        (headers, self)
    }

    pub fn initiate_response<'b>(
        &'b mut self,
        status: u16,
        message: Option<&'b str>,
        headers: &'b [(&'b str, &'b str)],
    ) -> Result<(), EspError> {
        self.assert_request();

        let mut c_headers = Vec::new();

        let status = if let Some(message) = message {
            format!("{} {}", status, message)
        } else {
            status.to_string()
        };

        let c_status = CString::new(status.as_str()).unwrap();
        esp!(unsafe { httpd_resp_set_status(self.request.0, c_status.as_ptr() as _) })?;

        c_headers.push(c_status);

        for (key, value) in headers {
            if key.eq_ignore_ascii_case("Content-Type") {
                let c_type = CString::new(*value).unwrap();

                esp!(unsafe { httpd_resp_set_type(self.request.0, c_type.as_c_str().as_ptr()) })?;

                c_headers.push(c_type);
            } else if key.eq_ignore_ascii_case("Content-Length") {
                let c_len = CString::new(*value).unwrap();

                //esp!(unsafe { httpd_resp_set_len(self.raw_req, c_len.as_c_str().as_ptr()) })?;

                c_headers.push(c_len);
            } else {
                let name = CString::new(*key).unwrap();
                let value = CString::new(*value).unwrap();

                esp!(unsafe {
                    httpd_resp_set_hdr(
                        self.request.0,
                        name.as_c_str().as_ptr() as _,
                        value.as_c_str().as_ptr() as _,
                    )
                })?;

                c_headers.push(name);
                c_headers.push(value);
            }
        }

        self.response_headers = Some(c_headers);
        self.headers = None;

        Ok(())
    }

    pub fn is_response_initiated(&self) -> bool {
        self.headers.is_none()
    }

    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
        self.assert_request();

        unsafe {
            let len = httpd_req_recv(
                self.request.0,
                buf.as_mut_ptr() as *mut _,
                buf.len() as size_t,
            );

            if len < 0 {
                esp!(len)?;
            }

            Ok(len as usize)
        }
    }

    pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspError> {
        self.assert_response();

        if !buf.is_empty() {
            esp!(unsafe {
                httpd_resp_send_chunk(
                    self.request.0,
                    buf.as_ptr() as *const _,
                    buf.len() as ssize_t,
                )
            })?;

            self.response_headers = None;
        }

        Ok(buf.len())
    }

    pub fn flush(&mut self) -> Result<(), EspError> {
        self.assert_response();

        Ok(())
    }

    pub fn raw_connection(&mut self) -> Result<&mut EspHttpRequest<'a>, EspError> {
        Ok(&mut self.request)
    }

    fn handle<'b, H>(&'b mut self, handler: &'b H) -> Result<(), HandlerError>
    where
        H: Handler<&'b mut Self>,
    {
        // TODO info!("About to handle query string {:?}", self.query_string());

        handler.handle(self)?;

        Ok(())
    }

    fn complete(&mut self) -> Result<(), HandlerError> {
        let buf = &[];

        if self.response_headers.is_some() {
            esp!(unsafe { httpd_resp_send(self.request.0, buf.as_ptr() as *const _, 0) })?;
        } else {
            esp!(unsafe { httpd_resp_send_chunk(self.request.0, buf.as_ptr() as *const _, 0) })?;
        }

        self.response_headers = None;

        Ok(())
    }

    fn handle_error<E>(&mut self, error: E)
    where
        E: Display,
    {
        if self.response_headers.is_some() {
            info!(
                "About to handle internal error [{}], response not sent yet",
                &error
            );

            if let Err(error2) = self.render_error(&error) {
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
    }

    fn render_error<E>(&mut self, error: E) -> Result<(), EspIOError>
    where
        E: Display,
    {
        self.initiate_response(500, Some("Internal Error"), &[content_type("text/html")])?;

        self.write_all(
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

        Ok(())
    }

    fn assert_request(&self) {
        if self.headers.is_none() {
            panic!("connection is not in request phase");
        }
    }

    fn assert_response(&self) {
        if self.headers.is_some() {
            panic!("connection is not in response phase");
        }
    }
}

impl<'a> RawHandle for EspHttpConnection<'a> {
    type Handle = *mut httpd_req_t;

    fn handle(&self) -> Self::Handle {
        self.request.handle()
    }
}

impl<'a> Query for EspHttpConnection<'a> {
    fn uri(&self) -> &str {
        EspHttpConnection::uri(self)
    }

    fn method(&self) -> Method {
        EspHttpConnection::method(self)
    }
}

impl<'a> Headers for EspHttpConnection<'a> {
    fn header(&self, name: &str) -> Option<&str> {
        EspHttpConnection::header(self, name)
    }
}

impl<'a> Io for EspHttpConnection<'a> {
    type Error = EspIOError;
}

impl<'a> Read for EspHttpConnection<'a> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        EspHttpConnection::read(self, buf).map_err(EspIOError)
    }
}

impl<'a> Write for EspHttpConnection<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        EspHttpConnection::write(self, buf).map_err(EspIOError)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        EspHttpConnection::flush(self).map_err(EspIOError)
    }
}

impl<'b> Connection for EspHttpConnection<'b> {
    type Headers = Self;

    type Read = Self;

    type RawConnectionError = EspIOError;

    type RawConnection = EspHttpRequest<'b>;

    fn split(&mut self) -> (&Self::Headers, &mut Self::Read) {
        EspHttpConnection::split(self)
    }

    fn initiate_response<'a>(
        &'a mut self,
        status: u16,
        message: Option<&'a str>,
        headers: &'a [(&'a str, &'a str)],
    ) -> Result<(), Self::Error> {
        EspHttpConnection::initiate_response(self, status, message, headers).map_err(EspIOError)
    }

    fn is_response_initiated(&self) -> bool {
        EspHttpConnection::is_response_initiated(self)
    }

    fn raw_connection(&mut self) -> Result<&mut Self::RawConnection, Self::Error> {
        EspHttpConnection::raw_connection(self).map_err(EspIOError)
    }
}

#[cfg(esp_idf_httpd_ws_support)]
pub mod ws {
    use core::fmt::Debug;
    use core::sync::atomic::{AtomicBool, Ordering};

    extern crate alloc;
    use alloc::sync::Arc;

    use ::log::*;

    use embedded_svc::http::Method;
    use embedded_svc::utils::mutex::{Condvar, Mutex};
    use embedded_svc::ws::callback_server::*;

    use esp_idf_sys::*;

    use crate::private::common::Newtype;
    use crate::private::cstr::CString;
    use crate::private::mutex::{RawCondvar, RawMutex};

    use super::EspHttpServer;
    use super::CLOSE_HANDLERS;
    use super::OPEN_SESSIONS;

    #[cfg(all(feature = "nightly", feature = "experimental"))]
    pub use asyncify::*;

    pub enum EspHttpWsConnection {
        New(httpd_handle_t, *mut httpd_req_t),
        Receiving(httpd_handle_t, *mut httpd_req_t),
        Closed(c_types::c_int),
    }

    impl EspHttpWsConnection {
        pub fn session(&self) -> i32 {
            match self {
                Self::New(_, raw_req) | Self::Receiving(_, raw_req) => unsafe {
                    httpd_req_to_sockfd(*raw_req)
                },
                Self::Closed(fd) => *fd,
            }
        }

        pub fn is_new(&self) -> bool {
            matches!(self, Self::New(_, _))
        }

        pub fn is_closed(&self) -> bool {
            matches!(self, Self::Closed(_))
        }

        pub fn create_detached_sender(&self) -> Result<EspHttpWsDetachedSender, EspError> {
            match self {
                Self::New(sd, raw_req) | Self::Receiving(sd, raw_req) => {
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

        pub fn send(&mut self, frame_type: FrameType, frame_data: &[u8]) -> Result<(), EspError> {
            match self {
                Self::New(_, raw_req) | Self::Receiving(_, raw_req) => {
                    let raw_frame = Self::create_raw_frame(frame_type, frame_data);

                    esp!(unsafe {
                        httpd_ws_send_frame(*raw_req, &raw_frame as *const _ as *mut _)
                    })?;

                    Ok(())
                }
                _ => {
                    esp!(ESP_FAIL)?;

                    Ok(())
                }
            }
        }

        pub fn recv(&mut self, frame_data_buf: &mut [u8]) -> Result<(FrameType, usize), EspError> {
            match self {
                Self::New(_, _) => Err(EspError::from(ESP_FAIL).unwrap().into()),
                Self::Receiving(_, raw_req) => {
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

        fn create_raw_frame(frame_type: FrameType, frame_data: &[u8]) -> httpd_ws_frame_t {
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
                payload: frame_data.as_ptr() as *const _ as *mut _,
                len: frame_data.len() as _,
            }
        }

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

    impl ErrorType for EspHttpWsConnection {
        type Error = EspError;
    }

    impl Sender for EspHttpWsConnection {
        fn send(&mut self, frame_type: FrameType, frame_data: &[u8]) -> Result<(), Self::Error> {
            EspHttpWsConnection::send(self, frame_type, frame_data)
        }
    }

    impl Receiver for EspHttpWsConnection {
        fn recv(&mut self, frame_data_buf: &mut [u8]) -> Result<(FrameType, usize), Self::Error> {
            EspHttpWsConnection::recv(self, frame_data_buf)
        }
    }

    impl SenderFactory for EspHttpWsConnection {
        type Sender = EspHttpWsDetachedSender;

        fn create(&self) -> Result<Self::Sender, Self::Error> {
            EspHttpWsConnection::create_detached_sender(self)
        }
    }

    impl SessionProvider for EspHttpWsConnection {
        type Session = c_types::c_int;

        fn session(&self) -> Self::Session {
            EspHttpWsConnection::session(self)
        }

        fn is_new(&self) -> bool {
            EspHttpWsConnection::is_new(self)
        }

        fn is_closed(&self) -> bool {
            EspHttpWsConnection::is_closed(self)
        }
    }

    pub struct EspWsDetachedSendRequest {
        sd: httpd_handle_t,
        fd: c_types::c_int,

        closed: Arc<AtomicBool>,

        raw_frame: *const httpd_ws_frame_t,

        error_code: Mutex<RawMutex, Option<u32>>,
        condvar: Condvar<RawCondvar>,
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

        pub fn session(&self) -> i32 {
            self.fd
        }

        pub fn is_new(&self) -> bool {
            false
        }

        pub fn is_closed(&self) -> bool {
            self.closed.load(Ordering::SeqCst)
        }

        pub fn send(&mut self, frame_type: FrameType, frame_data: &[u8]) -> Result<(), EspError> {
            if !self.closed.load(Ordering::SeqCst) {
                let raw_frame = EspHttpWsConnection::create_raw_frame(frame_type, frame_data);

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
        fn send(&mut self, frame_type: FrameType, frame_data: &[u8]) -> Result<(), Self::Error> {
            EspHttpWsDetachedSender::send(self, frame_type, frame_data)
        }
    }

    impl SessionProvider for EspHttpWsDetachedSender {
        type Session = c_types::c_int;

        fn session(&self) -> Self::Session {
            EspHttpWsDetachedSender::session(self)
        }

        fn is_new(&self) -> bool {
            EspHttpWsDetachedSender::is_new(self)
        }

        fn is_closed(&self) -> bool {
            EspHttpWsDetachedSender::is_closed(self)
        }
    }

    impl EspHttpServer {
        pub fn ws_handler<H, E>(&mut self, uri: &str, handler: H) -> Result<&mut Self, EspError>
        where
            H: for<'a> Fn(&'a mut EspHttpWsConnection) -> Result<(), E> + 'static,
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

        fn handle_ws_request<H, E>(
            connection: &mut EspHttpWsConnection,
            handler: &H,
        ) -> Result<(), E>
        where
            H: for<'b> Fn(&'b mut EspHttpWsConnection) -> Result<(), E>,
            E: Debug,
        {
            handler(connection)?;

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
            H: for<'a> Fn(&'a mut EspHttpWsConnection) -> Result<(), E> + 'static,
            E: Debug,
        {
            let boxed_handler = Arc::new(move |mut connection: EspHttpWsConnection| {
                let result = Self::handle_ws_request(&mut connection, &handler);

                match result {
                    Ok(()) => ESP_OK as _,
                    Err(e) => Self::handle_ws_error(e),
                }
            });

            let req_handler = {
                let boxed_handler = boxed_handler.clone();

                Box::new(move |raw_req: *mut httpd_req_t| {
                    let req = unsafe { raw_req.as_ref() }.unwrap();

                    (boxed_handler)(if req.method == http_method_HTTP_GET as i32 {
                        EspHttpWsConnection::New(server_handle.clone(), raw_req)
                    } else {
                        EspHttpWsConnection::Receiving(server_handle.clone(), raw_req)
                    })
                })
            };

            let close_handler = Box::new(move |fd| {
                (boxed_handler)(EspHttpWsConnection::Closed(fd));
            });

            (req_handler, close_handler)
        }
    }

    #[cfg(all(feature = "nightly", feature = "experimental"))]
    pub mod asyncify {
        use embedded_svc::utils::asyncify::ws::server::{
            AsyncAcceptor, AsyncReceiver, AsyncSender, Processor,
        };
        use esp_idf_sys::EspError;

        pub type EspHttpWsProcessor<const N: usize, const F: usize> =
            Processor<N, F, crate::private::mutex::RawCondvar, super::EspHttpWsConnection>;

        pub type EspHttpWsAsyncAcceptor<U> =
            AsyncAcceptor<U, crate::private::mutex::RawCondvar, super::EspHttpWsDetachedSender>;

        pub type EspHttpWsAsyncSender<U> = AsyncSender<U, super::EspHttpWsDetachedSender>;

        pub type EspHttpWsAsyncReceiver =
            AsyncReceiver<crate::private::mutex::RawCondvar, EspError>;
    }
}
