//! Type safe abstraction for esp-tls

use core::fmt::Debug;

use crate::private::cstr::{c_char, CStr};

#[cfg(esp_idf_comp_esp_tls_enabled)]
pub use self::esptls::*;

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

impl<'a> Debug for X509<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        write!(f, "X509(...)")
    }
}

#[cfg(esp_idf_comp_esp_tls_enabled)]
mod esptls {
    use core::time::Duration;

    use embedded_svc::io;

    use super::X509;
    use crate::{
        errors::EspIOError,
        private::cstr::{cstr_arr_from_str_slice, cstr_from_str_truncating, CStr},
        sys::{self, EspError, ESP_ERR_NO_MEM, ESP_FAIL},
    };

    /// see https://www.ietf.org/rfc/rfc3280.txt ub-common-name-length
    const MAX_COMMON_NAME_LENGTH: usize = 64;

    /// Wrapper for `esp-tls` module. Only supports synchronous operation for now.
    pub struct EspTls {
        reader: EspTlsRead,
        writer: EspTlsWrite,
    }

    impl EspTls {
        /// Create a new blocking TLS/SSL connection.
        ///
        /// This function establishes a TLS/SSL connection with the specified host in a blocking manner.
        ///
        /// # Errors
        ///
        /// * `ESP_ERR_INVALID_SIZE` if `cfg.alpn_protos` exceeds 9 elements or avg 10 bytes/ALPN
        /// * `ESP_ERR_NO_MEM` if TLS context could not be allocated
        /// * `ESP_FAIL` if connection could not be established
        pub fn new(host: &str, port: u16, cfg: &Config) -> Result<Self, EspError> {
            let mut rcfg: sys::esp_tls_cfg = unsafe { core::mem::zeroed() };

            if let Some(ca_cert) = cfg.ca_cert {
                rcfg.__bindgen_anon_1.cacert_buf = ca_cert.data().as_ptr();
                rcfg.__bindgen_anon_2.cacert_bytes = ca_cert.data().len() as u32;
            }

            if let Some(client_cert) = cfg.client_cert {
                rcfg.__bindgen_anon_3.clientcert_buf = client_cert.data().as_ptr();
                rcfg.__bindgen_anon_4.clientcert_bytes = client_cert.data().len() as u32;
            }

            if let Some(client_key) = cfg.client_key {
                rcfg.__bindgen_anon_5.clientkey_buf = client_key.data().as_ptr();
                rcfg.__bindgen_anon_6.clientkey_bytes = client_key.data().len() as u32;
            }

            if let Some(ckp) = cfg.client_key_password {
                rcfg.clientkey_password = ckp.as_ptr();
                rcfg.clientkey_password_len = ckp.len() as u32;
            }

            // allow up to 9 protocols
            let mut alpn_protos: [*const i8; 10];
            let mut alpn_protos_cbuf = [0u8; 99];
            if let Some(protos) = cfg.alpn_protos {
                alpn_protos = cstr_arr_from_str_slice(protos, &mut alpn_protos_cbuf)?;
                rcfg.alpn_protos = alpn_protos.as_mut_ptr();
            }

            rcfg.non_block = cfg.non_block;
            rcfg.use_secure_element = cfg.use_secure_element;
            rcfg.timeout_ms = cfg.timeout_ms as i32;
            rcfg.use_global_ca_store = cfg.use_global_ca_store;

            if let Some(common_name) = cfg.common_name {
                let mut common_name_buf = [0; MAX_COMMON_NAME_LENGTH + 1];
                rcfg.common_name =
                    cstr_from_str_truncating(common_name, &mut common_name_buf).as_ptr();
            }

            rcfg.skip_common_name = cfg.skip_common_name;

            let mut raw_kac: sys::tls_keep_alive_cfg;
            if let Some(kac) = &cfg.keep_alive_cfg {
                raw_kac = sys::tls_keep_alive_cfg {
                    keep_alive_enable: kac.enable,
                    keep_alive_idle: kac.idle.as_secs() as i32,
                    keep_alive_interval: kac.interval.as_secs() as i32,
                    keep_alive_count: kac.count as i32,
                };
                rcfg.keep_alive_cfg = &mut raw_kac as *mut _;
            }

            let mut raw_psk: sys::psk_key_hint;
            if let Some(psk) = &cfg.psk_hint_key {
                raw_psk = sys::psk_key_hint {
                    key: psk.key.as_ptr(),
                    key_size: psk.key.len(),
                    hint: psk.hint.as_ptr(),
                };
                rcfg.psk_hint_key = &mut raw_psk as *mut _;
            }

            #[cfg(esp_idf_mbedtls_certificate_bundle)]
            if cfg.use_crt_bundle_attach {
                rcfg.crt_bundle_attach = Some(sys::esp_crt_bundle_attach);
            }

            rcfg.is_plain_tcp = cfg.is_plain_tcp;
            rcfg.if_name = core::ptr::null_mut();

            let tls = unsafe { sys::esp_tls_init() };
            if tls.is_null() {
                return Err(EspError::from_infallible::<ESP_ERR_NO_MEM>());
            }
            let ret = unsafe {
                sys::esp_tls_conn_new_sync(
                    host.as_bytes().as_ptr() as *const i8,
                    host.len() as i32,
                    port as i32,
                    &rcfg,
                    tls,
                )
            };

            if ret == 1 {
                Ok(EspTls {
                    reader: EspTlsRead { raw: tls },
                    writer: EspTlsWrite { raw: tls },
                })
            } else {
                unsafe {
                    sys::esp_tls_conn_destroy(tls);
                }

                Err(EspError::from_infallible::<ESP_FAIL>())
            }
        }

        pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspIOError> {
            self.reader.read(buf)
        }

        pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspIOError> {
            self.writer.write(buf)
        }

        pub fn split(&mut self) -> (&mut EspTlsRead, &mut EspTlsWrite) {
            (&mut self.reader, &mut self.writer)
        }
    }

    #[cfg(feature = "std")]
    impl std::os::fd::AsRawFd for EspTls {
        fn as_raw_fd(&self) -> std::os::fd::RawFd {
            let mut fd = -1;
            let _ = unsafe { sys::esp_tls_get_conn_sockfd(self.reader.raw, &mut fd) };

            fd
        }
    }

    impl io::Io for EspTls {
        type Error = EspIOError;
    }

    impl io::Read for EspTls {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspIOError> {
            self.read(buf)
        }
    }

    impl io::Write for EspTls {
        fn write(&mut self, buf: &[u8]) -> Result<usize, EspIOError> {
            self.write(buf)
        }

        fn flush(&mut self) -> Result<(), EspIOError> {
            Ok(())
        }
    }

    pub struct EspTlsRead {
        raw: *mut sys::esp_tls,
    }

    impl EspTlsRead {
        pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspIOError> {
            if buf.is_empty() {
                return Ok(0);
            }

            let ret = self.read_raw(buf);
            // ESP docs treat 0 as error, but in Rust it's common to return 0 from `Read::read` to indicate eof
            if ret >= 0 {
                Ok(ret as usize)
            } else {
                Err(EspIOError(EspError::from(ret as i32).unwrap()))
            }
        }

        #[cfg(esp_idf_version_major = "4")]
        fn read_raw(&mut self, buf: &mut [u8]) -> isize {
            // cannot call esp_tls_conn_read bc it's inline in v4
            let esp_tls = unsafe { core::ptr::read_unaligned(self.raw) };
            let read_func = esp_tls.read.unwrap();
            unsafe { read_func(self.raw, buf.as_mut_ptr() as *mut i8, buf.len()) }
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        fn read_raw(&mut self, buf: &mut [u8]) -> isize {
            use core::ffi::c_void;

            unsafe { sys::esp_tls_conn_read(self.raw, buf.as_mut_ptr() as *mut c_void, buf.len()) }
        }
    }

    impl io::Io for EspTlsRead {
        type Error = EspIOError;
    }

    impl io::Read for EspTlsRead {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, EspIOError> {
            self.read(buf)
        }
    }

    pub struct EspTlsWrite {
        raw: *mut sys::esp_tls,
    }

    impl EspTlsWrite {
        pub fn write(&mut self, buf: &[u8]) -> Result<usize, EspIOError> {
            if buf.is_empty() {
                return Ok(0);
            }

            let ret = self.write_raw(buf);
            if ret >= 0 {
                Ok(ret as usize)
            } else {
                Err(EspIOError(EspError::from(ret as i32).unwrap()))
            }
        }

        #[cfg(esp_idf_version_major = "4")]
        fn write_raw(&mut self, buf: &[u8]) -> isize {
            // cannot call esp_tls_conn_write bc it's inline
            let esp_tls = unsafe { core::ptr::read_unaligned(self.raw) };
            let write_func = esp_tls.write.unwrap();
            unsafe { write_func(self.raw, buf.as_ptr() as *const i8, buf.len()) }
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        fn write_raw(&mut self, buf: &[u8]) -> isize {
            use core::ffi::c_void;

            unsafe { sys::esp_tls_conn_write(self.raw, buf.as_ptr() as *const c_void, buf.len()) }
        }
    }

    impl io::Io for EspTlsWrite {
        type Error = EspIOError;
    }

    impl io::Write for EspTlsWrite {
        fn write(&mut self, buf: &[u8]) -> Result<usize, EspIOError> {
            self.write(buf)
        }

        fn flush(&mut self) -> Result<(), EspIOError> {
            Ok(())
        }
    }

    impl Drop for EspTls {
        fn drop(&mut self) {
            unsafe {
                sys::esp_tls_conn_destroy(self.reader.raw);
            }
        }
    }

    #[derive(Default)]
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
        #[cfg(esp_idf_comp_lwip_enabled)]
        pub if_name: sys::ifreq,
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
}
