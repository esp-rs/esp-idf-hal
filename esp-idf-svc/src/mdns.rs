//! mDNS Service

use core::time::Duration;

extern crate alloc;
use alloc::string::String;
use alloc::string::ToString;
use alloc::vec::Vec;

use ::log::info;

#[cfg(any(esp_idf_lwip_ipv4, esp_idf_lwip_ipv6))]
use embedded_svc::ipv4::IpAddr;
#[cfg(esp_idf_lwip_ipv4)]
use embedded_svc::ipv4::Ipv4Addr;
#[cfg(esp_idf_lwip_ipv6)]
use embedded_svc::ipv4::Ipv6Addr;

use crate::sys::*;

use crate::private::cstr::to_cstring_arg;
use crate::private::cstr::CStr;
use crate::private::mutex::Mutex;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Interface {
    STA,
    AP,
    ETH,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Protocol {
    #[cfg(esp_idf_lwip_ipv4)]
    V4,
    #[cfg(esp_idf_lwip_ipv6)]
    V6,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Type {
    #[cfg(esp_idf_lwip_ipv4)]
    A = MDNS_TYPE_A as _,
    #[cfg(esp_idf_lwip_ipv6)]
    AAAA = MDNS_TYPE_AAAA as _,
    ANY = MDNS_TYPE_ANY as _,
    NSEC = MDNS_TYPE_NSEC as _,
    OPT = MDNS_TYPE_OPT as _,
    PTR = MDNS_TYPE_PTR as _,
    SRV = MDNS_TYPE_SRV as _,
    TXT = MDNS_TYPE_TXT as _,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QueryResult {
    pub instance_name: Option<String>,
    pub hostname: Option<String>,
    pub port: u16,
    pub txt: Vec<(String, String)>,
    pub addr: Vec<IpAddr>,
    pub interface: Interface,
    pub ip_protocol: Protocol,
}

impl From<mdns_result_t> for QueryResult {
    #[allow(non_upper_case_globals)]
    fn from(result: mdns_result_t) -> Self {
        let instance_name = unsafe { result.instance_name.as_ref() }
            .map(|p| unsafe { CStr::from_ptr(p) }.to_str().unwrap().to_string());
        let hostname = unsafe { result.hostname.as_ref() }
            .map(|p| unsafe { CStr::from_ptr(p) }.to_str().unwrap().to_string());
        let port = result.port;

        let mut txt = Vec::with_capacity(result.txt_count);
        for i in 0..result.txt_count as _ {
            let p = unsafe { result.txt.offset(i) };
            let key = unsafe { CStr::from_ptr((*p).key) }
                .to_str()
                .unwrap()
                .to_string();
            let value = unsafe { (*p).value.as_ref() }
                .map_or(Default::default(), |p| {
                    unsafe { CStr::from_ptr(p) }.to_str().unwrap()
                })
                .to_string();
            txt.push((key, value));
        }

        let mut addr = Vec::new();
        let mut p = result.addr;
        while !p.is_null() {
            let a = unsafe { (*p).addr };
            let a = match a.type_ as _ {
                #[cfg(esp_idf_lwip_ipv4)]
                ESP_IPADDR_TYPE_V4 => IpAddr::V4(from_esp_ip4_addr_t(unsafe { &a.u_addr.ip4 })),
                #[cfg(esp_idf_lwip_ipv6)]
                ESP_IPADDR_TYPE_V6 => IpAddr::V6(from_esp_ip6_addr_t(unsafe { &a.u_addr.ip6 })),
                _ => unreachable!(),
            };
            addr.push(a);

            p = unsafe { (*p).next };
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        let interface = match unsafe { CStr::from_ptr(esp_netif_get_desc(result.esp_netif)) }
            .to_bytes_with_nul()
        {
            b"sta\0" => Interface::STA,
            b"ap\0" => Interface::AP,
            b"eth\0" => Interface::ETH,
            // TODO: the above are only the default descriptions, and can be overridden, and there are several more
            // interfaces. Is there a way in `esp_netif` to get an enumerated IF type?
            _ => todo!("unknown interface type"),
        };

        #[cfg(esp_idf_version_major = "4")]
        let interface = match result.tcpip_if {
            mdns_if_internal_MDNS_IF_STA => Interface::STA,
            mdns_if_internal_MDNS_IF_AP => Interface::AP,
            mdns_if_internal_MDNS_IF_ETH => Interface::ETH,
            _ => unreachable!(),
        };

        let ip_protocol = match result.ip_protocol {
            #[cfg(esp_idf_lwip_ipv4)]
            mdns_ip_protocol_t_MDNS_IP_PROTOCOL_V4 => Protocol::V4,
            #[cfg(esp_idf_lwip_ipv6)]
            mdns_ip_protocol_t_MDNS_IP_PROTOCOL_V6 => Protocol::V6,
            _ => unreachable!(),
        };

        QueryResult {
            instance_name,
            hostname,
            port,
            txt,
            addr,
            interface,
            ip_protocol,
        }
    }
}

static TAKEN: Mutex<bool> = Mutex::new(false);

pub struct EspMdns(());

impl EspMdns {
    pub fn take() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        info!("Initializing MDNS");
        esp!(unsafe { mdns_init() })?;

        *taken = true;

        Ok(Self(()))
    }

    pub fn set_hostname(&mut self, hostname: impl AsRef<str>) -> Result<(), EspError> {
        let hostname = to_cstring_arg(hostname.as_ref())?;

        esp!(unsafe { mdns_hostname_set(hostname.as_ptr()) })
    }

    pub fn set_instance_name(&mut self, instance_name: impl AsRef<str>) -> Result<(), EspError> {
        let instance_name = to_cstring_arg(instance_name.as_ref())?;

        esp!(unsafe { mdns_instance_name_set(instance_name.as_ptr()) })
    }

    pub fn add_service(
        &mut self,
        instance_name: Option<&str>,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        port: u16,
        txt: &[(&str, &str)],
    ) -> Result<(), EspError> {
        let instance_name = if let Some(instance_name) = instance_name {
            Some(to_cstring_arg(instance_name)?)
        } else {
            None
        };
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let mut txtcstr = Vec::with_capacity(txt.len());
        let mut txtptr = Vec::with_capacity(txt.len());
        for e in txt.iter() {
            let key = to_cstring_arg(e.0)?;
            let value = to_cstring_arg(e.1)?;
            txtptr.push(mdns_txt_item_t {
                key: key.as_ptr(),
                value: value.as_ptr(),
            });
            txtcstr.push((key, value));
        }

        esp!(unsafe {
            mdns_service_add(
                instance_name
                    .as_ref()
                    .map_or(core::ptr::null(), |x| x.as_ptr()),
                service_type.as_ptr(),
                proto.as_ptr(),
                port,
                txtptr.as_mut_ptr(),
                txtptr.len() as _,
            )
        })
    }

    pub fn set_service_port(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        port: u16,
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;

        esp!(unsafe { mdns_service_port_set(service_type.as_ptr(), proto.as_ptr(), port) })
    }

    pub fn set_service_instance_name(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        instance_name: impl AsRef<str>,
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let instance_name = to_cstring_arg(instance_name.as_ref())?;

        esp!(unsafe {
            mdns_service_instance_name_set(
                service_type.as_ptr(),
                proto.as_ptr(),
                instance_name.as_ptr(),
            )
        })
    }

    pub fn set_service_txt_item(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        key: impl AsRef<str>,
        value: impl AsRef<str>,
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let key = to_cstring_arg(key.as_ref())?;
        let value = to_cstring_arg(value.as_ref())?;

        esp!(unsafe {
            mdns_service_txt_item_set(
                service_type.as_ptr(),
                proto.as_ptr(),
                key.as_ptr(),
                value.as_ptr(),
            )
        })
    }

    pub fn remove_service_txt_item(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        key: impl AsRef<str>,
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let key = to_cstring_arg(key.as_ref())?;

        esp!(unsafe {
            mdns_service_txt_item_remove(service_type.as_ptr(), proto.as_ptr(), key.as_ptr())
        })
    }

    pub fn set_service_txt(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        txt: &[(&str, &str)],
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;

        let mut txtcstr = Vec::with_capacity(txt.len());
        let mut txtptr = Vec::with_capacity(txt.len());

        for e in txt.iter() {
            let key = to_cstring_arg(e.0)?;
            let value = to_cstring_arg(e.1)?;
            txtptr.push(mdns_txt_item_t {
                key: key.as_ptr(),
                value: value.as_ptr(),
            });
            txtcstr.push((key, value));
        }

        esp!(unsafe {
            mdns_service_txt_set(
                service_type.as_ptr(),
                proto.as_ptr(),
                txtptr.as_mut_ptr(),
                txtptr.len() as _,
            )
        })
    }

    pub fn remove_service(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
    ) -> Result<(), EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;

        esp!(unsafe { mdns_service_remove(service_type.as_ptr(), proto.as_ptr()) })
    }

    pub fn remove_services(&mut self) -> Result<(), EspError> {
        esp!(unsafe { mdns_service_remove_all() })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn query(
        &self,
        name: Option<&str>,
        service_type: Option<&str>,
        proto: Option<&str>,
        mdns_type: Type,
        timeout: Duration,
        max_results: usize,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let name = if let Some(name) = name {
            Some(to_cstring_arg(name)?)
        } else {
            None
        };
        let service_type = if let Some(service_type) = service_type {
            Some(to_cstring_arg(service_type)?)
        } else {
            None
        };
        let proto = if let Some(proto) = proto {
            Some(to_cstring_arg(proto)?)
        } else {
            None
        };
        let mut result = core::ptr::null_mut();
        esp!(unsafe {
            mdns_query(
                name.as_ref().map_or(core::ptr::null(), |x| x.as_ptr()),
                service_type
                    .as_ref()
                    .map_or(core::ptr::null(), |x| x.as_ptr()),
                proto.as_ref().map_or(core::ptr::null(), |x| x.as_ptr()),
                mdns_type as _,
                timeout.as_millis() as _,
                max_results as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(result, results))
    }

    #[cfg(esp_idf_lwip_ipv4)]
    pub fn query_a(
        &self,
        hostname: impl AsRef<str>,
        timeout: Duration,
    ) -> Result<Ipv4Addr, EspError> {
        let hostname = to_cstring_arg(hostname.as_ref())?;
        let mut addr: esp_ip4_addr_t = Default::default();

        esp!(unsafe { mdns_query_a(hostname.as_ptr(), timeout.as_millis() as _, &mut addr) })?;

        Ok(from_esp_ip4_addr_t(&addr))
    }

    #[cfg(esp_idf_lwip_ipv6)]
    pub fn query_aaaa(
        &self,
        hostname: impl AsRef<str>,
        timeout: Duration,
    ) -> Result<Ipv6Addr, EspError> {
        let hostname = to_cstring_arg(hostname.as_ref())?;
        let mut addr: esp_ip6_addr_t = Default::default();

        esp!(unsafe { mdns_query_aaaa(hostname.as_ptr(), timeout.as_millis() as _, &mut addr) })?;

        Ok(from_esp_ip6_addr_t(&addr))
    }

    pub fn query_txt(
        &self,
        instance_name: impl AsRef<str>,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        timeout: Duration,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let instance_name = to_cstring_arg(instance_name.as_ref())?;
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let mut result = core::ptr::null_mut();

        esp!(unsafe {
            mdns_query_txt(
                instance_name.as_ptr(),
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(result, results))
    }

    pub fn query_srv(
        &self,
        instance_name: impl AsRef<str>,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        timeout: Duration,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let instance_name = to_cstring_arg(instance_name.as_ref())?;
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let mut result = core::ptr::null_mut();

        esp!(unsafe {
            mdns_query_srv(
                instance_name.as_ptr(),
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(result, results))
    }

    pub fn query_ptr(
        &self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        timeout: Duration,
        max_results: usize,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let service_type = to_cstring_arg(service_type.as_ref())?;
        let proto = to_cstring_arg(proto.as_ref())?;
        let mut result = core::ptr::null_mut();

        esp!(unsafe {
            mdns_query_ptr(
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                max_results as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(result, results))
    }
}

impl Drop for EspMdns {
    fn drop(&mut self) {
        let mut taken = TAKEN.lock();

        unsafe { mdns_free() };

        *taken = false;
    }
}

fn copy_query_results(src: *mut mdns_result_t, dst: &mut [QueryResult]) -> usize {
    if !src.is_null() {
        let mut p = src;
        let mut i = 0;
        while !p.is_null() && i < dst.len() {
            dst[i] = QueryResult::from(unsafe { *p });
            p = unsafe { (*p).next };
            i += 1;
        }

        unsafe { mdns_query_results_free(src) };

        i
    } else {
        0
    }
}

#[cfg(esp_idf_lwip_ipv4)]
fn from_esp_ip4_addr_t(addr: &esp_ip4_addr_t) -> Ipv4Addr {
    Ipv4Addr::from(addr.addr.to_le_bytes())
}

#[cfg(esp_idf_lwip_ipv6)]
fn from_esp_ip6_addr_t(addr: &esp_ip6_addr_t) -> Ipv6Addr {
    let mut buf = [0u8; 16];
    let mut i = 0;
    for e in addr.addr.iter() {
        for e in e.to_le_bytes().iter() {
            buf[i] = *e;
            i += 1;
        }
    }
    Ipv6Addr::from(buf)
}
