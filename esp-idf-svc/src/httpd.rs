use std::{cell::RefCell, ffi::CString, io, io::Read, marker::PhantomData, ptr, sync::Arc, sync::Mutex, sync::RwLock, vec};

use anyhow;

use embedded_svc::httpd;

use esp_idf_sys::c_types::*;
use esp_idf_sys::esp;
use esp_idf_sys::esp_nofail;

pub struct Request<'r, S, A> {
    rd: *mut esp_idf_sys::httpd_req_t,
    session: Option<Arc<RwLock<S>>>,
    app: Arc<RwLock<A>>,
    _rmarker: PhantomData<&'r esp_idf_sys::httpd_req_t>,
}

impl<'r, S, A> Request<'r, S, A> {
    fn send(
        &mut self, 
        status: u16, 
        status_message: Option<&String>,
        mut headers: std::collections::HashMap<String, String>,
        body: httpd::Body) -> anyhow::Result<()> {

        let mut status_string = status.to_string();
        if let Some(message) = status_message {
            status_string.push_str(" ");
            status_string.push_str(message.as_str());
        }

        let c_status = CString::new(status_string.as_str()).unwrap();
    
        esp!(unsafe {esp_idf_sys::httpd_resp_set_status(self.rd, c_status.as_ptr())})?;
    
        if let Some(cl) = body.len() {
            headers.insert("content-length".into(), cl.to_string());
        }
    
        let mut c_headers: std::vec::Vec<(CString, CString)> = vec! [];
    
        for (key, value) in headers {
            c_headers.push((
                CString::new(key.as_str()).unwrap(),
                CString::new(value.as_str()).unwrap()
            ))
        }
    
        for (c_field, c_value) in &c_headers {
            esp!(unsafe {esp_idf_sys::httpd_resp_set_hdr(self.rd, c_field.as_ptr(), c_value.as_ptr())})?;
        }
    
        match body {
            httpd::Body::Empty => self.send_body_bytes(&[]),
            httpd::Body::Bytes(vec) => self.send_body_bytes(vec.as_slice()),
            httpd::Body::Read(_, r) => self.send_body_read(r)
        }
    }

    fn send_body_bytes(&mut self, data: &[u8]) -> anyhow::Result<()> {
        esp!(unsafe {esp_idf_sys::httpd_resp_send(
            self.rd, 
            data.as_ptr() as *const c_char, 
            data.len() as esp_idf_sys::ssize_t)}).map_err(Into::into)
    }
    
    fn send_body_read(&mut self, r: Box<RefCell<dyn Read>>) -> anyhow::Result<()> {
        let mut buf: [u8; 256] = [0; 256];
    
        Ok(loop {
            let len = r.borrow_mut().read(&mut buf)?;
    
            esp!(unsafe {esp_idf_sys::httpd_resp_send_chunk(
                self.rd, 
                buf.as_ptr() as *const c_char, 
                len as esp_idf_sys::ssize_t)})?;
    
            if len == 0 {
                break
            }
        })
    }
}

impl<'r, S, A> httpd::Request<S, A> for Request<'r, S, A> {
    fn header(&self, name: impl AsRef<str>) -> Option<String> {
        let c_str = CString::new(name.as_ref()).unwrap();
        let c_name: *const c_char = c_str.as_ptr() as *const c_char;

        unsafe {
            let len = esp_idf_sys::httpd_req_get_hdr_value_len(self.rd, c_name) as usize + 1;
            match len {
                0 => None,
                len => {
                    let mut buf: vec::Vec<u8> = std::vec![0; len];
                    
                    esp_nofail!(esp_idf_sys::httpd_req_get_hdr_value_str(
                        self.rd, 
                        c_name, 
                        buf.as_mut_slice().as_ptr() as *mut i8,
                        len as esp_idf_sys::size_t));
        
                    Some(std::str::from_utf8_unchecked(&buf.as_slice()[..len - 1]).into())
                }
            }
        }
    }

    fn url(&self) -> String {
        unsafe {
            let len = esp_idf_sys::httpd_req_get_url_query_len(self.rd) as usize + 1;

            let mut buf: vec::Vec<u8> = std::vec![0; len];

            esp_nofail!(esp_idf_sys::httpd_req_get_url_query_str(
                self.rd, 
                buf.as_mut_slice().as_ptr() as *mut i8,
                len as esp_idf_sys::size_t));
        
            std::str::from_utf8_unchecked(&buf.as_slice()[..len - 1]).into()
        }
    }

    fn with_session<Q>(&self, f: impl FnOnce(Option<&S>) -> Q) -> Q {
        if let Some(p) = self.session.as_ref() {
            f(Some(&p.read().unwrap()))
        } else {
            f(None)
        }
    }

    fn with_session_mut<Q>(&self, f: impl FnOnce(Option<&mut S>) -> Q) -> Q {
        if let Some(p) = self.session.as_ref() {
            f(Some(&mut p.write().unwrap()))
        } else {
            f(None)
        }
    }

    fn with_app<Q>(&self, f: impl FnOnce(&A) -> Q) -> Q {
        f(&self.app.read().unwrap())
    }

    fn with_app_mut<Q>(&self, f: impl FnOnce(&mut A) -> Q) -> Q {
        f(&mut self.app.write().unwrap())
    }
}

impl<'r, S, A> io::Read for Request<'r, S, A> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, io::Error> {
        unsafe {
            let len = esp_idf_sys::httpd_req_recv(
                self.rd, 
                buf.as_ptr() as *mut c_char, 
                buf.len() as esp_idf_sys::size_t);
            
            if len < 0 {
                Err(match len {
                    esp_idf_sys::HTTPD_SOCK_ERR_INVALID => io::ErrorKind::InvalidInput,
                    esp_idf_sys::HTTPD_SOCK_ERR_TIMEOUT => io::ErrorKind::TimedOut,
                    esp_idf_sys::HTTPD_SOCK_ERR_FAIL => io::ErrorKind::Other,
                    _ => io::ErrorKind::Other,
                }.into())
            } else {
                Ok(len as usize)
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Configuration {
    pub http_port: u16,
    pub https_port: u16,
    pub max_sessions: usize,
}

impl Default for Configuration {
    fn default() -> Self {
        Configuration {
            http_port: 80,
            https_port: 443,
            max_sessions: 10,
        }
    }
}

pub struct Server<S: 'static = (), A: 'static = ()> {
    sd: esp_idf_sys::httpd_handle_t,
    registrations: Vec<(CString, esp_idf_sys::httpd_uri_t)>,
    sessions: Arc<Mutex<httpd::sessions_impl::Sessions<Arc<RwLock<S>>>>>,
    app: Arc<RwLock<A>>,
}

impl<S: 'static, A: 'static> Server<S, A> {
    pub fn default_new(app: Arc<RwLock<A>>) -> anyhow::Result<Self> {
        Server::new(&Default::default(), app)
    }

    pub fn new(conf: &Configuration, app: Arc<RwLock<A>>) -> anyhow::Result<Self> {
        let config = default_configuration(conf.http_port, conf.https_port);

        let mut handle: esp_idf_sys::httpd_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe {esp_idf_sys::httpd_start(handle_ref, &config as *const esp_idf_sys::httpd_config_t)})?;

        Ok(Server {
            sd: handle,
            registrations: vec! [],
            sessions: Arc::new(Mutex::new(httpd::sessions_impl::Sessions::new(conf.max_sessions))),
            app,
        })
    }

    fn unregister(&mut self, uri: CString, conf: esp_idf_sys::httpd_uri_t) -> anyhow::Result<()> {
        esp!(unsafe {esp_idf_sys::httpd_unregister_uri_handler(
                self.sd, 
                uri.as_ptr(), 
                conf.method)})
            .map_err(Into::into)
    }

    fn stop(&mut self) -> anyhow::Result<()> {
        if self.sd != ptr::null_mut() {
            while !self.registrations.is_empty() {
                let (uri, registration) = self.registrations.pop().unwrap();
           
                self.unregister(uri, registration)?;
            }

            esp!(unsafe {esp_idf_sys::httpd_stop(self.sd)})?;

            self.sd = ptr::null_mut();
        }

        Ok(())
    }

    fn handle<'r, F: Fn(&mut Request<'r, S, A>) -> anyhow::Result<httpd::Response<S>>>(
            rd: *mut esp_idf_sys::httpd_req_t,
            sessions: Arc<Mutex<httpd::sessions_impl::Sessions<Arc<RwLock<S>>>>>,
            app: Arc<RwLock<A>>,
            handler: F) -> c_int {
        let mut internal_request = Request {
            rd,
            app: app.clone(),
            session: None,
            _rmarker: PhantomData
        };

        let response = sessions.lock().unwrap().update(
            &internal_request, 
            handler(
                &mut Request {
                    rd,
                    app: app.clone(),
                    session: sessions
                        .lock()
                        .unwrap()
                        .get(&internal_request)
                        .as_ref()
                        .map(Clone::clone),
                    _rmarker: PhantomData,
                })
                .unwrap_or_else(Into::into),
            |s| Arc::new(RwLock::new(s)));

        let result = internal_request.send(
            response.status, 
            response.status_message.as_ref(), 
            response.headers, 
            response.body);

        match result {
            Result::Ok(_) => 0,
            Result::Err(_) => 1, // TODO: For  now
        }
    }
}

impl<'r, S, A> httpd::registry::Registry<Request<'r, S, A>, S> for Server<S, A> {
    fn register(&mut self, registration: httpd::Registration<Request<'r, S, A>, S>) -> anyhow::Result<()> {
        let sessions = self.sessions.clone();
        let app = self.app.clone();

        let method = registration.method();

        let c_str = CString::new(registration.uri().as_ref()).unwrap();
    
        let registration_handler: Box<dyn Fn(&mut Request<'r, S, A>) -> anyhow::Result<httpd::Response<S>>> = registration.handler();

        let handler = move |handle| Server::handle(
            handle, 
            sessions, 
            app,
            registration_handler);

        let conf = esp_idf_sys::httpd_uri_t {
            uri: c_str.as_ptr(),
            method: get_httpd_method(method),
            user_ctx: Box::into_raw(Box::new(handler)) as *mut _,
            handler: Some(call_closure),
        };

        esp!(unsafe {esp_idf_sys::httpd_register_uri_handler(self.sd, &conf)})?;

        self.registrations.push((c_str, conf));

        Ok(())
    }
}

impl<S, A> Drop for Server<S, A> {
    fn drop(&mut self) {
        self.stop().expect("Unable to stop the server cleanly");
    }
}

unsafe extern "C" fn call_closure(handle: *mut esp_idf_sys::httpd_req_t) -> c_int {
    let callback_ptr = (*handle).user_ctx as *mut Box<dyn Fn(*mut esp_idf_sys::httpd_req_t) -> c_int>;
    let callback = &mut *callback_ptr;

    callback(handle)
}

// TODO: No way to call this from the IDF handlers' deregistration APIs
unsafe extern "C" fn drop_closure(user_ctx: *mut c_void) {
    Box::from_raw(user_ctx);
}

fn get_httpd_method(m: httpd::Method) -> c_uint {
    match m {
        httpd::Method::Get => esp_idf_sys::http_method_HTTP_GET,
        httpd::Method::Post => esp_idf_sys::http_method_HTTP_POST,
        httpd::Method::Delete => esp_idf_sys::http_method_HTTP_DELETE,
        httpd::Method::Head => esp_idf_sys::http_method_HTTP_HEAD,
        httpd::Method::Put => esp_idf_sys::http_method_HTTP_PUT,
        httpd::Method::Connect => esp_idf_sys::http_method_HTTP_CONNECT,
        httpd::Method::Options => esp_idf_sys::http_method_HTTP_OPTIONS,
        httpd::Method::Trace => esp_idf_sys::http_method_HTTP_TRACE,
        httpd::Method::Copy => esp_idf_sys::http_method_HTTP_COPY,
        httpd::Method::Lock => esp_idf_sys::http_method_HTTP_LOCK,
        httpd::Method::MkCol => esp_idf_sys::http_method_HTTP_MKCOL,
        httpd::Method::Move => esp_idf_sys::http_method_HTTP_MOVE,
        httpd::Method::Propfind => esp_idf_sys::http_method_HTTP_PROPFIND,
        httpd::Method::Proppatch => esp_idf_sys::http_method_HTTP_PROPPATCH,
        httpd::Method::Search => esp_idf_sys::http_method_HTTP_SEARCH,
        httpd::Method::Unlock => esp_idf_sys::http_method_HTTP_UNLOCK,
        httpd::Method::Bind => esp_idf_sys::http_method_HTTP_BIND,
        httpd::Method::Rebind => esp_idf_sys::http_method_HTTP_REBIND,
        httpd::Method::Unbind => esp_idf_sys::http_method_HTTP_UNBIND,
        httpd::Method::Acl => esp_idf_sys::http_method_HTTP_ACL,
        httpd::Method::Report => esp_idf_sys::http_method_HTTP_REPORT,
        httpd::Method::MkActivity => esp_idf_sys::http_method_HTTP_MKACTIVITY,
        httpd::Method::Checkout => esp_idf_sys::http_method_HTTP_CHECKOUT,
        httpd::Method::Merge => esp_idf_sys::http_method_HTTP_MERGE,
        httpd::Method::MSearch => esp_idf_sys::http_method_HTTP_MSEARCH,
        httpd::Method::Notify => esp_idf_sys::http_method_HTTP_NOTIFY,
        httpd::Method::Subscribe => esp_idf_sys::http_method_HTTP_SUBSCRIBE,
        httpd::Method::Unsubscribe => esp_idf_sys::http_method_HTTP_UNSUBSCRIBE,
        httpd::Method::Patch => esp_idf_sys::http_method_HTTP_PATCH,
        httpd::Method::Purge => esp_idf_sys::http_method_HTTP_PURGE,
        httpd::Method::MkCalendar => esp_idf_sys::http_method_HTTP_MKCALENDAR,
        httpd::Method::Link => esp_idf_sys::http_method_HTTP_LINK,
        httpd::Method::Unlink => esp_idf_sys::http_method_HTTP_UNLINK,
    }
}

/// Copied from the definition of HTTPD_DEFAULT_CONFIG() in http_server.h/https_server.h
fn default_configuration(http_port: u16, https_port: u16) -> esp_idf_sys::httpd_config_t {
    esp_idf_sys::httpd_config_t {
        task_priority:      5,
        stack_size:         if https_port != 0 {10240} else {4096},
        core_id:            std::i32::MAX,
        server_port:        http_port,
        ctrl_port:          32768,
        max_open_sockets:   if https_port != 0 {4} else {7},
        max_uri_handlers:   8,
        max_resp_headers:   8,
        backlog_conn:       5,
        lru_purge_enable:   https_port != 0,
        recv_wait_timeout:  5,
        send_wait_timeout:  5,
        global_user_ctx:    ptr::null_mut(),
        global_user_ctx_free_fn: None,
        global_transport_ctx: ptr::null_mut(),
        global_transport_ctx_free_fn: None,
        open_fn:            None,
        close_fn:           None,
        uri_match_fn:       None,
    }
}
