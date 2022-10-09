use core::time::Duration;

extern crate alloc;
use alloc::string::String;
use alloc::vec::Vec;

use ::log::info;

use embedded_svc::ipv4::Ipv4Addr;
use std::net::{IpAddr, Ipv6Addr};

use esp_idf_sys::*;

use crate::private::cstr::{CStr, CString};
use crate::private::mutex::{Mutex, RawMutex};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Interface {
    STA,
    AP,
    ETH,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Protocol {
    V4,
    V6,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Type {
    A = MDNS_TYPE_A as _,
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

        let mut txt = Vec::with_capacity(result.txt_count as usize);
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
                ESP_IPADDR_TYPE_V4 => IpAddr::V4(from_esp_ip4_addr_t(unsafe { &a.u_addr.ip4 })),
                ESP_IPADDR_TYPE_V6 => IpAddr::V6(from_esp_ip6_addr_t(unsafe { &a.u_addr.ip6 })),
                _ => unreachable!(),
            };
            addr.push(a);

            p = unsafe { (*p).next };
        }

        let interface = match result.tcpip_if {
            mdns_if_internal_MDNS_IF_STA => Interface::STA,
            mdns_if_internal_MDNS_IF_AP => Interface::AP,
            mdns_if_internal_MDNS_IF_ETH => Interface::ETH,
            _ => unreachable!(),
        };

        let ip_protocol = match result.ip_protocol {
            mdns_ip_protocol_t_MDNS_IP_PROTOCOL_V4 => Protocol::V4,
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

static TAKEN: Mutex<bool> = Mutex::wrap(RawMutex::new(), false);

pub struct EspMdns(());

impl EspMdns {
    pub fn take() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE)?;
        }

        info!("Initializing MDNS");
        esp!(unsafe { mdns_init() })?;

        *taken = true;

        Ok(Self(()))
    }

    pub fn set_hostname(&mut self, hostname: impl AsRef<str>) -> Result<(), EspError> {
        let hostname = CString::new(hostname.as_ref()).unwrap();

        esp!(unsafe { mdns_hostname_set(hostname.as_ptr()) })
    }

    pub fn set_instance_name(&mut self, instance_name: impl AsRef<str>) -> Result<(), EspError> {
        let instance_name = CString::new(instance_name.as_ref()).unwrap();

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
        let instance_name = instance_name.map(|x| CString::new(x.to_string()).unwrap());
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let mut txtcstr = Vec::with_capacity(txt.len());
        let mut txtptr = Vec::with_capacity(txt.len());
        for e in txt.iter() {
            let key = CString::new(e.0.as_bytes()).unwrap();
            let value = CString::new(e.1.as_bytes()).unwrap();
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
                    .map_or(std::ptr::null(), |x| x.as_ptr()),
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
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();

        esp!(unsafe { mdns_service_port_set(service_type.as_ptr(), proto.as_ptr(), port) })
    }

    pub fn set_service_instance_name(
        &mut self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        instance_name: impl AsRef<str>,
    ) -> Result<(), EspError> {
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let instance_name = CString::new(instance_name.as_ref()).unwrap();

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
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let key = CString::new(key.as_ref()).unwrap();
        let value = CString::new(value.as_ref()).unwrap();

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
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let key = CString::new(key.as_ref()).unwrap();

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
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();

        let mut txtcstr = Vec::with_capacity(txt.len());
        let mut txtptr = Vec::with_capacity(txt.len());

        for e in txt.iter() {
            let key = CString::new(e.0.as_bytes()).unwrap();
            let value = CString::new(e.1.as_bytes()).unwrap();
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
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();

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
        let name = name.map(|x| CString::new(x.to_string()).unwrap());
        let service_type = service_type.map(|x| CString::new(x.to_string()).unwrap());
        let proto = proto.map(|x| CString::new(x.to_string()).unwrap());

        let mut result = std::ptr::null_mut();
        esp!(unsafe {
            mdns_query(
                name.as_ref().map_or(std::ptr::null(), |x| x.as_ptr()),
                service_type
                    .as_ref()
                    .map_or(std::ptr::null(), |x| x.as_ptr()),
                proto.as_ref().map_or(std::ptr::null(), |x| x.as_ptr()),
                mdns_type as _,
                timeout.as_millis() as _,
                max_results as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(
            unsafe { Box::from_raw(result) },
            results,
        ))
    }

    pub fn query_a(
        &self,
        hostname: impl AsRef<str>,
        timeout: Duration,
    ) -> Result<Ipv4Addr, EspError> {
        let hostname = CString::new(hostname.as_ref()).unwrap();
        let mut addr: esp_ip4_addr_t = Default::default();

        esp!(unsafe { mdns_query_a(hostname.as_ptr(), timeout.as_millis() as _, &mut addr) })?;

        Ok(from_esp_ip4_addr_t(&addr))
    }

    pub fn query_aaaa(
        &self,
        hostname: impl AsRef<str>,
        timeout: Duration,
    ) -> Result<Ipv6Addr, EspError> {
        let hostname = CString::new(hostname.as_ref()).unwrap();
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
        let instance_name = CString::new(instance_name.as_ref()).unwrap();
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let mut result = std::ptr::null_mut();

        esp!(unsafe {
            mdns_query_txt(
                instance_name.as_ptr(),
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(
            unsafe { Box::from_raw(result) },
            results,
        ))
    }

    pub fn query_srv(
        &self,
        instance_name: impl AsRef<str>,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        timeout: Duration,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let instance_name = CString::new(instance_name.as_ref()).unwrap();
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let mut result = std::ptr::null_mut();

        esp!(unsafe {
            mdns_query_srv(
                instance_name.as_ptr(),
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(
            unsafe { Box::from_raw(result) },
            results,
        ))
    }

    pub fn query_ptr(
        &self,
        service_type: impl AsRef<str>,
        proto: impl AsRef<str>,
        timeout: Duration,
        max_results: usize,
        results: &mut [QueryResult],
    ) -> Result<usize, EspError> {
        let service_type = CString::new(service_type.as_ref()).unwrap();
        let proto = CString::new(proto.as_ref()).unwrap();
        let mut result = std::ptr::null_mut();

        esp!(unsafe {
            mdns_query_ptr(
                service_type.as_ptr(),
                proto.as_ptr(),
                timeout.as_millis() as _,
                max_results as _,
                &mut result,
            )
        })?;

        Ok(copy_query_results(
            unsafe { Box::from_raw(result) },
            results,
        ))
    }
}

impl Drop for EspMdns {
    fn drop(&mut self) {
        let mut taken = TAKEN.lock();

        unsafe { mdns_free() };

        *taken = false;
    }
}

fn copy_query_results(src: Box<mdns_result_t>, dst: &mut [QueryResult]) -> usize {
    let src = Box::into_raw(src);
    let mut p = src;
    let mut i = 0;
    while !p.is_null() && i < dst.len() {
        dst[i] = QueryResult::from(unsafe { *p });
        p = unsafe { (*p).next };
        i += 1;
    }

    unsafe { mdns_query_results_free(src) };

    i
}

fn from_esp_ip4_addr_t(addr: &esp_ip4_addr_t) -> Ipv4Addr {
    Ipv4Addr::from(addr.addr.to_le_bytes())
}

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
