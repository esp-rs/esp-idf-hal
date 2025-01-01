//! Type safe abstraction for esp-tls

#[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
use core::convert::TryFrom;
use core::fmt::Debug;

use crate::private::cstr::{c_char, CStr};
#[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
use crate::sys::EspError;

#[cfg(all(
    esp_idf_comp_esp_tls_enabled,
    any(esp_idf_esp_tls_using_mbedtls, esp_idf_esp_tls_using_wolfssl)
))]
pub use self::esptls::*;

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct Psk<'a> {
    pub key: &'a [u8],
    pub hint: &'a str,
}

impl Debug for Psk<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        f.debug_struct("Psk")
            .field("hint", &self.hint)
            .finish_non_exhaustive()
    }
}

/// Helper for holding PSK data for lately initialized TLS connections.
///
/// It could be easily converted from the public `Psk` configuration and holds the `psk_hint_key_t`
/// along with its (string) data as this data typically needs to be around after initializing a TLS
/// client until it has been started.
#[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
pub(crate) struct TlsPsk {
    pub(crate) psk: alloc::boxed::Box<crate::hal::sys::psk_hint_key_t>,
    pub(crate) _cstrs: crate::private::cstr::RawCstrs,
}
/// Dummy for maintaining the same internal interface whether TLS PSK support is enabled or not.
#[cfg(not(all(esp_idf_esp_tls_psk_verification, feature = "alloc")))]
#[allow(dead_code)]
pub(crate) struct TlsPsk {}

#[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
impl Debug for TlsPsk {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        f.debug_struct("TlsPsk")
            .field("psk", &self.psk)
            .finish_non_exhaustive()
    }
}

#[cfg(all(esp_idf_esp_tls_psk_verification, feature = "alloc"))]
impl<'a> TryFrom<&'a Psk<'a>> for TlsPsk {
    type Error = EspError;

    fn try_from(conf: &Psk) -> Result<Self, EspError> {
        let mut cstrs = crate::private::cstr::RawCstrs::new();
        let psk = alloc::boxed::Box::new(crate::hal::sys::psk_hint_key_t {
            key: conf.key.as_ptr(),
            key_size: conf.key.len(),
            hint: cstrs.as_ptr(conf.hint)?,
        });

        Ok(TlsPsk { psk, _cstrs: cstrs })
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub struct X509<'a>(&'a [u8]);

impl<'a> X509<'a> {
    pub fn pem(cstr: &'a CStr) -> Self {
        Self(cstr.to_bytes_with_nul())
    }

    pub const fn pem_until_nul(bytes: &'a [u8]) -> Self {
        // TODO: replace with `CStr::from_bytes_until_nul` when stabilized
        let mut nul_pos = 0;
        while nul_pos < bytes.len() {
            if bytes[nul_pos] == 0 {
                // TODO: replace with `<[u8]>::split_at(nul_pos + 1)` when const stabilized
                let slice = unsafe { core::slice::from_raw_parts(bytes.as_ptr(), nul_pos + 1) };
                return Self(slice);
            }
            nul_pos += 1;
        }
        panic!("PEM certificates should end with a NIL (`\\0`) ASCII character.")
    }

    pub const fn der(bytes: &'a [u8]) -> Self {
        Self(bytes)
    }

    pub fn data(&self) -> &[u8] {
        self.0
    }

    #[allow(unused)]
    pub(crate) fn as_esp_idf_raw_ptr(&self) -> *const c_char {
        self.data().as_ptr().cast()
    }

    #[allow(unused)]
    pub(crate) fn as_esp_idf_raw_len(&self) -> usize {
        self.data().len()
    }
}

impl Debug for X509<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        f.debug_struct("X509").finish_non_exhaustive()
    }
}

#[cfg(all(
    esp_idf_comp_esp_tls_enabled,
    any(esp_idf_esp_tls_using_mbedtls, esp_idf_esp_tls_using_wolfssl)
))]
mod esptls {
    use core::ffi::c_char;
    #[cfg(esp_idf_esp_tls_server_cert_select_hook)]
    use core::ffi::c_int;
    use core::task::{Context, Poll};
    use core::time::Duration;
    #[allow(unused_imports)]
    use core::{pin::Pin, task::ready};

    use embedded_svc::io;

    use super::X509;

    use crate::{
        io::EspIOError,
        private::cstr::{cstr_arr_from_str_slice, cstr_from_str_truncating, CStr},
        sys::{
            self, EspError, ESP_ERR_NO_MEM, ESP_FAIL, ESP_TLS_ERR_SSL_WANT_READ,
            ESP_TLS_ERR_SSL_WANT_WRITE, EWOULDBLOCK,
        },
    };

    /// see https://www.ietf.org/rfc/rfc3280.txt ub-common-name-length
    const MAX_COMMON_NAME_LENGTH: usize = 64;

    pub struct Config<'a> {
        /// up to 9 ALPNs allowed, with avg 10 bytes for each name
        pub alpn_protos: Option<&'a [&'a str]>,
        pub ca_cert: Option<X509<'a>>,
        pub client_cert: Option<X509<'a>>,
        pub client_key: Option<X509<'a>>,
        pub client_key_password: Option<&'a str>,
        pub non_block: bool,
        pub use_secure_element: bool,
        pub timeout_ms: u32,
        pub use_global_ca_store: bool,
        pub common_name: Option<&'a str>,
        pub skip_common_name: bool,
        pub keep_alive_cfg: Option<KeepAliveConfig>,
        pub psk_hint_key: Option<PskHintKey<'a>>,
        /// whether to use esp_crt_bundle_attach, see https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/protocols/esp_crt_bundle.html
        #[cfg(esp_idf_mbedtls_certificate_bundle)]
        pub use_crt_bundle_attach: bool,
        // TODO ds_data not implemented
        pub is_plain_tcp: bool,
    }

    impl Config<'_> {
        pub const fn new() -> Self {
            Self {
                alpn_protos: None,
                ca_cert: None,
                client_cert: None,
                client_key: None,
                client_key_password: None,
                non_block: false,
                use_secure_element: false,
                timeout_ms: 4000,
                use_global_ca_store: false,
                common_name: None,
                skip_common_name: false,
                keep_alive_cfg: None,
                psk_hint_key: None,
                #[cfg(esp_idf_mbedtls_certificate_bundle)]
                use_crt_bundle_attach: true,
                is_plain_tcp: false,
            }
        }

        fn try_into_raw(&self, bufs: &mut RawConfigBufs) -> Result<sys::esp_tls_cfg, EspError> {
            let mut rcfg: sys::esp_tls_cfg = Default::default();

            if let Some(ca_cert) = self.ca_cert {
                rcfg.__bindgen_anon_1.cacert_buf = ca_cert.data().as_ptr();
                rcfg.__bindgen_anon_2.cacert_bytes = ca_cert.data().len() as u32;
            }

            if let Some(client_cert) = self.client_cert {
                rcfg.__bindgen_anon_3.clientcert_buf = client_cert.data().as_ptr();
                rcfg.__bindgen_anon_4.clientcert_bytes = client_cert.data().len() as u32;
            }

            if let Some(client_key) = self.client_key {
                rcfg.__bindgen_anon_5.clientkey_buf = client_key.data().as_ptr();
                rcfg.__bindgen_anon_6.clientkey_bytes = client_key.data().len() as u32;
            }

            if let Some(ckp) = self.client_key_password {
                rcfg.clientkey_password = ckp.as_ptr();
                rcfg.clientkey_password_len = ckp.len() as u32;
            }

            // allow up to 9 protocols
            if let Some(protos) = self.alpn_protos {
                bufs.alpn_protos = cstr_arr_from_str_slice(protos, &mut bufs.alpn_protos_cbuf)?;
                rcfg.alpn_protos = bufs.alpn_protos.as_mut_ptr();
            }

            rcfg.non_block = self.non_block;
            rcfg.use_secure_element = self.use_secure_element;
            rcfg.timeout_ms = self.timeout_ms as i32;
            rcfg.use_global_ca_store = self.use_global_ca_store;

            if let Some(common_name) = self.common_name {
                rcfg.common_name =
                    cstr_from_str_truncating(common_name, &mut bufs.common_name_buf).as_ptr();
            }

            rcfg.skip_common_name = self.skip_common_name;

            let mut raw_kac: sys::tls_keep_alive_cfg;
            if let Some(kac) = &self.keep_alive_cfg {
                raw_kac = sys::tls_keep_alive_cfg {
                    keep_alive_enable: kac.enable,
                    keep_alive_idle: kac.idle.as_secs() as i32,
                    keep_alive_interval: kac.interval.as_secs() as i32,
                    keep_alive_count: kac.count as i32,
                };
                rcfg.keep_alive_cfg = &mut raw_kac as *mut _;
            }

            #[cfg(any(
                esp_idf_esp_tls_psk_verification,
                esp_idf_version_major = "4",
                esp_idf_version = "5.0",
                esp_idf_version = "5.1",
                esp_idf_version = "5.2",
                esp_idf_version = "5.3",
                esp_idf_version = "5.4",
            ))]
            {
                let mut raw_psk: sys::psk_key_hint;
                if let Some(psk) = &self.psk_hint_key {
                    raw_psk = sys::psk_key_hint {
                        key: psk.key.as_ptr(),
                        key_size: psk.key.len(),
                        hint: psk.hint.as_ptr(),
                    };
                    rcfg.psk_hint_key = &mut raw_psk as *mut _;
                }
            }

            #[cfg(esp_idf_mbedtls_certificate_bundle)]
            if self.use_crt_bundle_attach {
                rcfg.crt_bundle_attach = Some(sys::esp_crt_bundle_attach);
            }

            rcfg.is_plain_tcp = self.is_plain_tcp;

            #[cfg(esp_idf_comp_lwip_enabled)]
            {
                rcfg.if_name = core::ptr::null_mut();
            }

            Ok(rcfg)
        }
    }

    impl Default for Config<'_> {
        fn default() -> Self {
            Self::new()
        }
    }

    struct RawConfigBufs {
        alpn_protos: [*const c_char; 10],
        alpn_protos_cbuf: [u8; 99],
        common_name_buf: [u8; MAX_COMMON_NAME_LENGTH + 1],
    }

    unsafe impl Send for RawConfigBufs {}

    impl Default for RawConfigBufs {
        fn default() -> Self {
            RawConfigBufs {
                alpn_protos: [core::ptr::null(); 10],
                alpn_protos_cbuf: [0; 99],
                common_name_buf: [0; MAX_COMMON_NAME_LENGTH + 1],
            }
        }
    }

    type AlpnBuf = [u8; 16];

    #[derive(Clone, Default)]
    pub struct CompletedHandshake {
        alpn: AlpnBuf,
    }

    impl CompletedHandshake {
        pub fn alpn_proto(&self) -> Option<&str> {
            let p = CStr::from_bytes_until_nul(self.alpn.as_slice()).unwrap();
            // Safety: the bytes always come from a user supplied &str.
            let p = unsafe { core::str::from_utf8_unchecked(p.to_bytes()) };

            // A valid protocol is never empty.
            if !p.is_empty() {
                Some(p)
            } else {
                None
            }
        }

        // Safety: Must be called while the configured ALPN protocol strings are valid.
        unsafe fn extract(raw: *mut sys::esp_tls) -> CompletedHandshake {
            CompletedHandshake {
                alpn: unsafe { Self::extract_alpn(raw) }.unwrap_or_default(),
            }
        }

        #[cfg(not(all(
            not(esp_idf_version_major = "4"),
            esp_idf_comp_esp_tls_enabled,
            esp_idf_esp_tls_using_mbedtls,
            esp_idf_mbedtls_ssl_alpn
        )))]
        unsafe fn extract_alpn(_raw: *mut sys::esp_tls) -> Option<AlpnBuf> {
            None
        }

        #[cfg(all(
            not(esp_idf_version_major = "4"),
            esp_idf_comp_esp_tls_enabled,
            esp_idf_esp_tls_using_mbedtls,
            esp_idf_mbedtls_ssl_alpn
        ))]
        #[warn(unsafe_op_in_unsafe_fn)]
        unsafe fn extract_alpn(raw: *mut sys::esp_tls) -> Option<AlpnBuf> {
            let raw: *mut sys::mbedtls_ssl_context =
                unsafe { sys::esp_tls_get_ssl_context(raw) }.cast();

            if raw.is_null() {
                return None;
            }

            let chosen = unsafe { sys::mbedtls_ssl_get_alpn_protocol(raw) };
            if chosen.is_null() {
                return None;
            }

            let mut proto = AlpnBuf::default();
            let chosen = unsafe { CStr::from_ptr(chosen) };
            let chosen_bytes = chosen.to_bytes_with_nul();
            if chosen_bytes.len() > proto.len() {
                return None;
            }

            proto[..chosen_bytes.len()].copy_from_slice(chosen_bytes);

            Some(proto)
        }
    }

    #[derive(Clone, Debug)]
    pub struct KeepAliveConfig {
        /// Enable keep-alive timeout
        pub enable: bool,
        /// Keep-alive idle time (second)
        pub idle: Duration,
        /// Keep-alive interval time (second)
        pub interval: Duration,
        /// Keep-alive packet retry send count
        pub count: u32,
    }

    pub struct PskHintKey<'a> {
        pub key: &'a [u8],
        pub hint: &'a CStr,
    }

    #[cfg(esp_idf_esp_tls_server)]
    pub struct ServerConfig<'a> {
        /// up to 9 ALPNs allowed, with avg 10 bytes for each name
        pub alpn_protos: Option<&'a [&'a str]>,
        pub ca_cert: Option<X509<'a>>,
        pub server_cert: Option<X509<'a>>,
        pub server_key: Option<X509<'a>>,
        pub server_key_password: Option<&'a str>,
        pub use_secure_element: bool,
        #[cfg(esp_idf_esp_tls_server_cert_select_hook)]
        pub handshake_callback: Option<extern "C" fn(*mut sys::mbedtls_ssl_context) -> c_int>,
    }

    #[cfg(esp_idf_esp_tls_server)]
    impl<'a> ServerConfig<'a> {
        pub const fn new() -> Self {
            Self {
                alpn_protos: None,
                ca_cert: None,
                server_cert: None,
                server_key: None,
                server_key_password: None,
                use_secure_element: false,
                #[cfg(esp_idf_esp_tls_server_cert_select_hook)]
                handshake_callback: None,
            }
        }

        fn try_into_raw(
            &self,
            bufs: &mut RawConfigBufs,
        ) -> Result<sys::esp_tls_cfg_server, EspError> {
            let mut rcfg: sys::esp_tls_cfg_server = Default::default();

            if let Some(ca_cert) = self.ca_cert {
                rcfg.__bindgen_anon_1.cacert_buf = ca_cert.data().as_ptr();
                rcfg.__bindgen_anon_2.cacert_bytes = ca_cert.data().len() as u32;
            }

            if let Some(server_cert) = self.server_cert {
                rcfg.__bindgen_anon_3.servercert_buf = server_cert.data().as_ptr();
                rcfg.__bindgen_anon_4.servercert_bytes = server_cert.data().len() as u32;
            }

            if let Some(server_key) = self.server_key {
                rcfg.__bindgen_anon_5.serverkey_buf = server_key.data().as_ptr();
                rcfg.__bindgen_anon_6.serverkey_bytes = server_key.data().len() as u32;
            }

            if let Some(ckp) = self.server_key_password {
                rcfg.serverkey_password = ckp.as_ptr();
                rcfg.serverkey_password_len = ckp.len() as u32;
            }

            // allow up to 9 protocols
            if let Some(protos) = self.alpn_protos {
                bufs.alpn_protos = cstr_arr_from_str_slice(protos, &mut bufs.alpn_protos_cbuf)?;
                rcfg.alpn_protos = bufs.alpn_protos.as_mut_ptr();
            }

            rcfg.use_secure_element = self.use_secure_element;

            #[cfg(esp_idf_esp_tls_server_cert_select_hook)]
            if let Some(cb) = self.handshake_callback {
                rcfg.cert_select_cb = cb;
            }

            Ok(rcfg)
        }
    }

    #[cfg(esp_idf_esp_tls_server)]
    impl<'a> Default for ServerConfig<'a> {
        fn default() -> Self {
            Self::new()
        }
    }

    pub trait Socket {
        /// Returns the integer FD.
        fn handle(&self) -> i32;
        /// This is called before cleaning up the the tls context and is responsible
        /// for essentially giving up ownership of the socket such that it can safely
        /// be closed by the ESP IDF.
        fn release(&mut self) -> Result<(), EspError>;
    }

    pub trait PollableSocket: Socket {
        fn poll_readable(&self, ctx: &mut Context) -> Poll<Result<(), EspError>>;
        fn poll_writable(&self, ctx: &mut Context) -> Poll<Result<(), EspError>>;
    }

    pub struct InternalSocket(());

    impl Socket for InternalSocket {
        fn handle(&self) -> i32 {
            unreachable!()
        }

        fn release(&mut self) -> Result<(), EspError> {
            Ok(())
        }
    }

    /// Wrapper for `esp-tls` module. Only supports synchronous operation for now.
    pub struct EspTls<S>
    where
        S: Socket,
    {
        raw: *mut sys::esp_tls,
        socket: S,
        #[cfg(esp_idf_esp_tls_server)]
        server_session: bool,
    }

    // A single Mbed TLS context itself is safe to send across threads.
    // Require the threading implementation to be enabled since a shared context such as RSA or X509 could be used by multiple threads at once.
    // See https://mbed-tls.readthedocs.io/en/latest/kb/development/thread-safety-and-multi-threading/
    #[cfg(all(
        esp_idf_comp_esp_tls_enabled,
        esp_idf_esp_tls_using_mbedtls,
        esp_idf_mbedtls_threading_c
    ))]
    unsafe impl<S> Send for EspTls<S> where S: Send + Socket {}

    impl EspTls<InternalSocket> {
        /// Create a new `EspTls` instance using internally-managed socket.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_NO_MEM` if not enough memory to create the TLS connection
        pub fn new() -> Result<Self, EspError> {
            let raw = unsafe { sys::esp_tls_init() };
            if !raw.is_null() {
                Ok(Self {
                    raw,
                    socket: InternalSocket(()),
                    #[cfg(esp_idf_esp_tls_server)]
                    server_session: false,
                })
            } else {
                Err(EspError::from_infallible::<ESP_ERR_NO_MEM>())
            }
        }

        /// Establish a TLS/SSL connection with the specified host and port, using an internally-managed socket.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_INVALID_SIZE` if `cfg.alpn_protos` exceeds 9 elements or avg 10 bytes/ALPN
        /// * `ESP_FAIL` if connection could not be established
        /// * `ESP_TLS_ERR_SSL_WANT_READ` if the socket is in non-blocking mode and it is not ready for reading
        /// * `ESP_TLS_ERR_SSL_WANT_WRITE` if the socket is in non-blocking mode and it is not ready for writing
        /// * `EWOULDBLOCK` if the socket is in non-blocking mode and it is not ready either for reading or writing (a peculiarity/bug of the `esp-tls` C module)
        pub fn connect(
            &mut self,
            host: &str,
            port: u16,
            cfg: &Config,
        ) -> Result<CompletedHandshake, EspError> {
            let mut bufs = RawConfigBufs::default();
            let rcfg = cfg.try_into_raw(&mut bufs)?;

            let res = self.internal_connect(host, port, cfg.non_block, &rcfg);

            // Make sure buffers are held long enough
            #[allow(clippy::drop_non_drop)]
            drop(bufs);

            res
        }
    }

    impl<S> EspTls<S>
    where
        S: Socket,
    {
        /// Create a new `EspTls` instance adopting the supplied socket.
        /// The socket should be in a connected state.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_NO_MEM` if not enough memory to create the TLS connection
        #[cfg(all(
            not(esp_idf_version_major = "4"),
            any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
        ))]
        pub fn adopt(socket: S) -> Result<Self, EspError> {
            let raw = unsafe { sys::esp_tls_init() };
            if !raw.is_null() {
                sys::esp!(unsafe { sys::esp_tls_set_conn_sockfd(raw, socket.handle()) })?;

                sys::esp!(unsafe {
                    sys::esp_tls_set_conn_state(raw, sys::esp_tls_conn_state_ESP_TLS_CONNECTING)
                })?;

                Ok(Self {
                    raw,
                    socket,
                    #[cfg(esp_idf_esp_tls_server)]
                    server_session: false,
                })
            } else {
                Err(EspError::from_infallible::<ESP_ERR_NO_MEM>())
            }
        }

        /// Establish a TLS/SSL connection using the adopted socket.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_INVALID_SIZE` if `cfg.alpn_protos` exceeds 9 elements or avg 10 bytes/ALPN
        /// * `ESP_FAIL` if connection could not be established
        /// * `ESP_TLS_ERR_SSL_WANT_READ` if the socket is in non-blocking mode and it is not ready for reading
        /// * `ESP_TLS_ERR_SSL_WANT_WRITE` if the socket is in non-blocking mode and it is not ready for writing
        /// * `EWOULDBLOCK` if the socket is in non-blocking mode and it is not ready either for reading or writing (a peculiarity/bug of the `esp-tls` C module)
        #[cfg(all(
            not(esp_idf_version_major = "4"),
            any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
        ))]
        pub fn negotiate(
            &mut self,
            host: &str,
            cfg: &Config,
        ) -> Result<CompletedHandshake, EspError> {
            let mut bufs = RawConfigBufs::default();
            let rcfg = cfg.try_into_raw(&mut bufs)?;

            let res = self.internal_connect(host, 0, cfg.non_block, &rcfg);

            // Make sure buffers are held long enough
            #[allow(clippy::drop_non_drop)]
            drop(bufs);

            res
        }

        /// Establish a TLS/SSL connection using the adopted connection, acting as the server.
        ///
        /// # Errors
        ///
        /// * `ESP_FAIL` if connection could not be established
        #[cfg(esp_idf_esp_tls_server)]
        pub fn negotiate_server(&mut self, cfg: &ServerConfig) -> Result<(), EspError> {
            let mut bufs = RawConfigBufs::default();
            let mut rcfg = cfg.try_into_raw(&mut bufs)?;

            unsafe {
                let error =
                    sys::esp_tls_server_session_create(&mut rcfg, self.socket.handle(), self.raw);
                if error != 0 {
                    log::error!("failed to create tls server session (error {error})");
                    return Err(EspError::from_infallible::<ESP_FAIL>());
                }
            }
            self.server_session = true;

            // Make sure buffers are held long enough
            #[allow(clippy::drop_non_drop)]
            drop(bufs);

            Ok(())
        }

        #[allow(clippy::unnecessary_cast)]
        fn internal_connect(
            &mut self,
            host: &str,
            port: u16,
            asynch: bool,
            cfg: &sys::esp_tls_cfg,
        ) -> Result<CompletedHandshake, EspError> {
            let ret = unsafe {
                if asynch {
                    sys::esp_tls_conn_new_async(
                        host.as_bytes().as_ptr() as *const c_char,
                        host.len() as i32,
                        port as i32,
                        cfg,
                        self.raw,
                    )
                } else {
                    sys::esp_tls_conn_new_sync(
                        host.as_bytes().as_ptr() as *const c_char,
                        host.len() as i32,
                        port as i32,
                        cfg,
                        self.raw,
                    )
                }
            };

            match ret {
                1 => Ok(unsafe { CompletedHandshake::extract(self.raw) }),
                ESP_TLS_ERR_SSL_WANT_READ => Err(EspError::from_infallible::<
                    { ESP_TLS_ERR_SSL_WANT_READ as i32 },
                >()),
                ESP_TLS_ERR_SSL_WANT_WRITE => Err(EspError::from_infallible::<
                    { ESP_TLS_ERR_SSL_WANT_WRITE as i32 },
                >()),
                0 => Err(EspError::from_infallible::<{ EWOULDBLOCK as i32 }>()),
                _ => Err(EspError::from_infallible::<ESP_FAIL>()),
            }
        }

        /// Read in the supplied buffer. Returns the number of bytes read.
        ///
        ///
        /// # Errors
        /// * `ESP_TLS_ERR_SSL_WANT_READ` if the socket is in non-blocking mode and it is not ready for reading
        /// * `ESP_TLS_ERR_SSL_WANT_WRITE` if the socket is in non-blocking mode and it is not ready for writing
        /// * Any other `EspError` for a general error
        pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
            if buf.is_empty() {
                return Ok(0);
            }

            let ret = self.read_raw(buf);
            // ESP docs treat 0 as error, but in Rust it's common to return 0 from `Read::read` to indicate eof
            if ret >= 0 {
                Ok(ret as usize)
            } else {
                Err(EspError::from(ret as i32).unwrap())
            }
        }

        #[cfg(esp_idf_version_major = "4")]
        fn read_raw(&mut self, buf: &mut [u8]) -> isize {
            // cannot call esp_tls_conn_read bc it's inline in v4
            let esp_tls = unsafe { core::ptr::read_unaligned(self.raw) };
            let read_func = esp_tls.read.unwrap();
            unsafe { read_func(self.raw, buf.as_mut_ptr() as *mut c_char, buf.len()) }
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        fn read_raw(&mut self, buf: &mut [u8]) -> isize {
            use core::ffi::c_void;

            unsafe { sys::esp_tls_conn_read(self.raw, buf.as_mut_ptr() as *mut c_void, buf.len()) }
        }

        /// Write the supplied buffer. Returns the number of bytes written.
        ///
        /// # Errors
        /// * `ESP_TLS_ERR_SSL_WANT_READ` if the socket is in non-blocking mode and it is not ready for reading
        /// * `ESP_TLS_ERR_SSL_WANT_WRITE` if the socket is in non-blocking mode and it is not ready for writing
        /// * Any other `EspError` for a general error
        pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspError> {
            if buf.is_empty() {
                return Ok(0);
            }

            let ret = self.write_raw(buf);
            if ret >= 0 {
                Ok(ret as usize)
            } else {
                Err(EspError::from(ret as i32).unwrap())
            }
        }

        pub fn write_all(&mut self, buf: &[u8]) -> Result<(), EspError> {
            let mut buf = buf;

            while !buf.is_empty() {
                match self.write(buf) {
                    Ok(0) => panic!("zero-length write."),
                    Ok(n) => buf = &buf[n..],
                    Err(e) => return Err(e),
                }
            }

            Ok(())
        }

        #[cfg(esp_idf_version_major = "4")]
        fn write_raw(&mut self, buf: &[u8]) -> isize {
            // cannot call esp_tls_conn_write bc it's inline
            let esp_tls = unsafe { core::ptr::read_unaligned(self.raw) };
            let write_func = esp_tls.write.unwrap();
            unsafe { write_func(self.raw, buf.as_ptr() as *const c_char, buf.len()) }
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        fn write_raw(&mut self, buf: &[u8]) -> isize {
            use core::ffi::c_void;

            unsafe { sys::esp_tls_conn_write(self.raw, buf.as_ptr() as *const c_void, buf.len()) }
        }

        pub fn context_handle(&self) -> *mut sys::esp_tls {
            self.raw
        }
    }

    impl<S> Drop for EspTls<S>
    where
        S: Socket,
    {
        fn drop(&mut self) {
            let _ = self.socket.release();

            unsafe {
                // use esp_tls_conn_destroy for both client and server
                sys::esp_tls_conn_destroy(self.raw);
            }
        }
    }

    impl<S> io::ErrorType for EspTls<S>
    where
        S: Socket,
    {
        type Error = EspIOError;
    }

    impl<S> io::Read for EspTls<S>
    where
        S: Socket,
    {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspIOError> {
            EspTls::read(self, buf).map_err(EspIOError)
        }
    }

    impl<S> io::Write for EspTls<S>
    where
        S: Socket,
    {
        fn write(&mut self, buf: &[u8]) -> Result<usize, EspIOError> {
            EspTls::write(self, buf).map_err(EspIOError)
        }

        fn flush(&mut self) -> Result<(), EspIOError> {
            Ok(())
        }
    }
    #[cfg(all(
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    pub struct EspAsyncTls<S>(crate::private::mutex::Mutex<EspTls<S>>)
    where
        S: PollableSocket;

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        /// Create a new `AsyncEspTls` instance adopting the supplied socket.
        /// The socket should be in a connected state.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_NO_MEM` if not enough memory to create the TLS connection
        pub fn adopt(socket: S) -> Result<Self, EspError> {
            Ok(Self(crate::private::mutex::Mutex::new(EspTls::adopt(
                socket,
            )?)))
        }

        /// Establish a TLS/SSL connection using the adopted socket.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_INVALID_SIZE` if `cfg.alpn_protos` exceeds 9 elements or avg 10 bytes/ALPN
        /// * `ESP_FAIL` if connection could not be established
        pub async fn negotiate(
            &mut self,
            hostname: &str,
            cfg: &Config<'_>,
        ) -> Result<CompletedHandshake, EspError> {
            struct AssertSend<T>(T);
            unsafe impl<T> Send for AssertSend<T> {}

            let mut bufs = RawConfigBufs::default();
            let mut rcfg: AssertSend<sys::esp_tls_cfg> = AssertSend(cfg.try_into_raw(&mut bufs)?);

            // It is a bit unintuitive, but when an async socket is being adopted, `non_block` should be set to false.
            //
            // Background:
            // `non_block = true` is only used at one place in the ESP IDF code and that is to run
            // a check - with `select` - whether the socket is really connected.
            // However, we want to avoid the `select()` call, as
            // (a) It won't work, because we jump directly into the ESP_TLS_CONNECTING state as we adopt a socket.
            //.    As a side effect, the select() call is not properly initialized.
            // (b) The adopted socket might be registered in a select() loop already.
            //
            // Avoiding the connectivity check with `select()` should be fine, as the adopted socket
            // must be already connected anyway (API requirement).
            rcfg.0.non_block = false;

            let res = loop {
                let res = self
                    .0
                    .get_mut()
                    .internal_connect(hostname, 0, true, &rcfg.0);

                match res {
                    Err(e) => self.wait(e).await?,
                    other => break other,
                }
            };

            // Make sure buffers are held long enough
            #[allow(clippy::drop_non_drop)]
            drop(bufs);

            res
        }

        // TODO: Create upstream support for async server negotiation
        // Establish a TLS/SSL connection using the adopted connection, acting as the server.
        //
        // # Errors
        //
        // * `ESP_FAIL` if connection could not be established
        // #[cfg(esp_idf_esp_tls_server)]
        // pub async fn negotiate_server(&mut self, cfg: &ServerConfig<'_>) -> Result<(), EspError> {
        //     // FIXME: this isn't actually async, but esp-idf does not expose anything else.
        //     // we would have to use various hacks to call mbedtls_ssl_handshake by ourself
        //     self.0.borrow_mut().negotiate_server(cfg)
        // }

        /// Read in the supplied buffer. Returns the number of bytes read.
        pub async fn read(&self, buf: &mut [u8]) -> Result<usize, EspError> {
            core::future::poll_fn(|ctx| self.poll_read(ctx, buf)).await
        }

        pub fn poll_read(
            &self,
            ctx: &mut Context<'_>,
            buf: &mut [u8],
        ) -> Poll<Result<usize, EspError>> {
            loop {
                let res = self.0.lock().read(buf);

                match res {
                    Err(e) => ready!(self.poll_wait(ctx, e))?,
                    Ok(n) => break Poll::Ready(Ok(n)),
                }
            }
        }

        /// Write the supplied buffer. Returns the number of bytes written.
        pub async fn write(&self, buf: &[u8]) -> Result<usize, EspError> {
            core::future::poll_fn(|ctx| self.poll_write(ctx, buf)).await
        }

        pub fn poll_write(
            &self,
            ctx: &mut Context<'_>,
            buf: &[u8],
        ) -> Poll<Result<usize, EspError>> {
            loop {
                let res = self.0.lock().write(buf);

                match res {
                    Err(e) => ready!(self.poll_wait(ctx, e))?,
                    Ok(n) => break Poll::Ready(Ok(n)),
                }
            }
        }

        pub async fn write_all(&self, buf: &[u8]) -> Result<(), EspError> {
            let mut buf = buf;

            while !buf.is_empty() {
                match self.write(buf).await {
                    Ok(0) => panic!("zero-length write."),
                    Ok(n) => buf = &buf[n..],
                    Err(e) => return Err(e),
                }
            }

            Ok(())
        }

        fn poll_wait(&self, ctx: &mut Context<'_>, error: EspError) -> Poll<Result<(), EspError>> {
            const EWOULDBLOCK_I32: i32 = EWOULDBLOCK as i32;

            match error.code() {
                // EWOULDBLOCK models the "0" return code of esp_mbedtls_handshake() which does not allow us
                // to figure out whether we need the socket to become readable or writable
                // The code below is therefore a hack which just waits with a timeout for the socket to (eventually)
                // become readable as we actually don't even know if that's what esp_tls wants
                EWOULDBLOCK_I32 => {
                    let res = self.0.lock().socket.poll_writable(ctx);
                    crate::hal::delay::FreeRtos::delay_ms(0);
                    res
                }
                ESP_TLS_ERR_SSL_WANT_READ => self.0.lock().socket.poll_readable(ctx),
                ESP_TLS_ERR_SSL_WANT_WRITE => self.0.lock().socket.poll_writable(ctx),
                _ => Poll::Ready(Err(error)),
            }
        }

        async fn wait(&self, error: EspError) -> Result<(), EspError> {
            core::future::poll_fn(|ctx| self.poll_wait(ctx, error)).await
        }

        pub fn context_handle(&self) -> *mut sys::esp_tls {
            self.0.lock().context_handle()
        }
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> io::ErrorType for EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        type Error = EspIOError;
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> io::asynch::Read for EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            EspAsyncTls::read(self, buf).await.map_err(EspIOError)
        }
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> io::asynch::Write for EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            EspAsyncTls::write(self, buf).await.map_err(EspIOError)
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[cfg(all(
        feature = "std",
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> futures_io::AsyncRead for EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        fn poll_read(
            self: Pin<&mut Self>,
            ctx: &mut Context<'_>,
            buf: &mut [u8],
        ) -> Poll<std::io::Result<usize>> {
            self.as_ref()
                .poll_read(ctx, buf)
                .map_err(std::io::Error::other)
        }
    }

    #[cfg(all(
        feature = "std",
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> futures_io::AsyncRead for &EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        fn poll_read(
            self: Pin<&mut Self>,
            ctx: &mut Context<'_>,
            buf: &mut [u8],
        ) -> Poll<std::io::Result<usize>> {
            self.as_ref()
                .poll_read(ctx, buf)
                .map_err(std::io::Error::other)
        }
    }

    #[cfg(all(
        feature = "std",
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> futures_io::AsyncWrite for EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        fn poll_write(
            self: Pin<&mut Self>,
            ctx: &mut Context<'_>,
            buf: &[u8],
        ) -> Poll<std::io::Result<usize>> {
            self.as_ref()
                .poll_write(ctx, buf)
                .map_err(std::io::Error::other)
        }

        fn poll_flush(self: Pin<&mut Self>, _ctx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }

        fn poll_close(self: Pin<&mut Self>, _ctx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }
    }

    #[cfg(all(
        feature = "std",
        not(esp_idf_version_major = "4"),
        any(not(esp_idf_version_major = "5"), not(esp_idf_version_minor = "0"))
    ))]
    impl<S> futures_io::AsyncWrite for &EspAsyncTls<S>
    where
        S: PollableSocket,
    {
        fn poll_write(
            self: Pin<&mut Self>,
            ctx: &mut Context<'_>,
            buf: &[u8],
        ) -> Poll<std::io::Result<usize>> {
            self.as_ref()
                .poll_write(ctx, buf)
                .map_err(std::io::Error::other)
        }

        fn poll_flush(self: Pin<&mut Self>, _ctx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }

        fn poll_close(self: Pin<&mut Self>, _ctx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }
    }
}
