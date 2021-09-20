use core::ptr;

extern crate alloc;
use alloc::borrow::Cow;

#[cfg(feature = "std")]
pub use std::ffi::{CStr, CString};

#[cfg(not(feature = "std"))]
pub use cstr_core::{CStr, CString};

use embedded_svc::http::client::*;
use embedded_svc::http::*;
use embedded_svc::io::{Read, Write};

use esp_idf_sys::*;

use crate::private::common::Newtype;

impl From<Method> for Newtype<((), esp_http_client_method_t)> {
    fn from(method: Method) -> Self {
        Self((
            (),
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
        ))
    }
}

pub struct EspClient(esp_http_client_handle_t);

impl EspClient {
    pub fn new() -> Self {
        Self(unsafe { esp_http_client_init(&Default::default()) })
    }
}

impl Default for EspClient {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for EspClient {
    fn drop(&mut self) {
        esp!(unsafe { esp_http_client_cleanup(self.0) })
            .expect("Unable to stop the client cleanly");
    }
}

impl Client for EspClient {
    type Request<'a> = EspRequest<'a>;

    type Error = EspError;

    fn request(
        &mut self,
        method: Method,
        url: impl AsRef<str>,
    ) -> Result<Self::Request<'_>, Self::Error> {
        let c_url = CString::new(url.as_ref()).unwrap();

        esp!(unsafe { esp_http_client_set_url(self.0, c_url.as_ptr() as _) })?;
        esp!(unsafe {
            esp_http_client_set_method(self.0, Newtype::<esp_http_client_method_t>::from(method).0)
        })?;

        Ok(EspRequest(self))
    }
}

pub struct EspRequest<'a>(&'a mut EspClient);

impl<'a> Request<'a> for EspRequest<'a> {
    type Response<'b> = EspResponse<'b>;

    type Write<'b> = Self;

    type Error = EspError;

    fn set_follow_redirects(&mut self, _follow_redirects: bool) -> &mut Self {
        todo!()
    }

    fn send(
        mut self,
        size: usize,
        f: impl FnOnce(&mut Self::Write<'a>) -> Result<(), Self::Error>,
    ) -> Result<Self::Response<'a>, Self::Error> {
        esp!(unsafe { esp_http_client_open(self.0 .0, size as _) })?;

        f(&mut self)?;

        let result = unsafe { esp_http_client_fetch_headers(self.0 .0) };
        if result < 0 {
            esp!(result)?;
        }

        Ok(EspResponse(self.0))
    }
}

impl<'a> SendHeaders<'a> for EspRequest<'a> {
    fn set_header<H, V>(&mut self, name: H, value: V) -> &mut Self
    where
        H: Into<Cow<'a, str>>,
        V: Into<Cow<'a, str>>,
    {
        let c_name = CString::new(name.into().as_ref()).unwrap();

        // TODO: Replace with a proper conversion from UTF8 to ISO-8859-1
        let c_value = CString::new(value.into().as_ref()).unwrap();

        esp!(unsafe {
            esp_http_client_set_header(self.0 .0, c_name.as_ptr() as _, c_value.as_ptr() as _)
        })
        .unwrap();

        self
    }
}

impl<'a> Write for EspRequest<'a> {
    type Error = EspError;

    fn do_write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        esp!(unsafe { esp_http_client_write(self.0 .0, buf.as_ptr() as _, buf.len() as _) })?;

        Ok(buf.len())
    }
}

pub struct EspResponse<'a>(&'a mut EspClient);

impl<'a> Response<'a> for EspResponse<'a> {
    type Read<'b> = Self;

    type Error = EspError;

    fn payload(&mut self) -> &mut Self {
        self
    }

    fn into_payload(self) -> Self::Read<'a> {
        self
    }
}

impl<'a> Headers for EspResponse<'a> {
    fn header(&self, name: impl AsRef<str>) -> Option<std::borrow::Cow<'_, str>> {
        let c_name = CString::new(name.as_ref()).unwrap();

        let mut handle: *mut c_types::c_char = ptr::null_mut();
        let handle_ref = &mut handle;

        unsafe {
            esp_nofail!(esp_http_client_get_header(
                self.0 .0,
                c_name.as_ptr() as _,
                handle_ref
            ));
        }

        if handle.is_null() {
            None
        } else {
            // TODO: Replace with a proper conversion from ISO-8859-1 to UTF8
            Some(String::from_utf8_lossy(
                unsafe { CStr::from_ptr(handle as _) }.to_bytes(),
            ))
        }
    }
}

impl<'a> Status for EspResponse<'a> {
    fn status(&self) -> u16 {
        unsafe { esp_http_client_get_status_code(self.0 .0) as _ }
    }

    fn status_message(&self) -> Option<std::borrow::Cow<'_, str>> {
        None
    }
}

impl<'a> Read for EspResponse<'a> {
    type Error = EspError;

    fn do_read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let result =
            unsafe { esp_http_client_read(self.0 .0, buf.as_mut_ptr() as _, buf.len() as _) };
        if result < 0 {
            esp!(result)?;
        }

        Ok(result as _)
    }
}
