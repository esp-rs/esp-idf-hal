use core::convert::{TryFrom, TryInto};

use crate::sys::*;
use embedded_svc::ipv4::{self, Mask};

use crate::private::common::*;

impl From<ipv4::Ipv4Addr> for Newtype<esp_ip4_addr_t> {
    fn from(ip: ipv4::Ipv4Addr) -> Self {
        Newtype(esp_ip4_addr_t {
            addr: u32::to_be(u32::from_be_bytes(ip.octets())),
        })
    }
}

impl From<Newtype<esp_ip4_addr_t>> for ipv4::Ipv4Addr {
    fn from(ip: Newtype<esp_ip4_addr_t>) -> Self {
        ipv4::Ipv4Addr::from(u32::from_be(ip.0.addr))
    }
}

impl From<ipv4::Ipv4Addr> for Newtype<ip4_addr_t> {
    fn from(ip: ipv4::Ipv4Addr) -> Self {
        let result: Newtype<esp_ip4_addr_t> = ip.into();

        Newtype(ip4_addr_t {
            addr: result.0.addr,
        })
    }
}

impl From<Newtype<ip4_addr_t>> for ipv4::Ipv4Addr {
    fn from(ip: Newtype<ip4_addr_t>) -> Self {
        Newtype(esp_ip4_addr_t { addr: ip.0.addr }).into()
    }
}

impl From<Mask> for Newtype<esp_ip4_addr_t> {
    fn from(mask: Mask) -> Self {
        let ip: ipv4::Ipv4Addr = mask.into();

        ip.into()
    }
}

impl TryFrom<Newtype<esp_ip4_addr_t>> for Mask {
    type Error = EspError;

    fn try_from(esp_ip: Newtype<esp_ip4_addr_t>) -> Result<Self, Self::Error> {
        let ip: ipv4::Ipv4Addr = esp_ip.into();

        ip.try_into()
            .map_err(|_| EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
    }
}

impl From<Mask> for Newtype<ip4_addr_t> {
    fn from(mask: Mask) -> Self {
        let ip: ipv4::Ipv4Addr = mask.into();

        ip.into()
    }
}

impl TryFrom<Newtype<ip4_addr_t>> for Mask {
    type Error = EspError;

    fn try_from(esp_ip: Newtype<ip4_addr_t>) -> Result<Self, Self::Error> {
        let ip: ipv4::Ipv4Addr = esp_ip.into();

        ip.try_into()
            .map_err(|_| EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
    }
}

impl From<ipv4::IpInfo> for Newtype<esp_netif_ip_info_t> {
    fn from(ip_info: ipv4::IpInfo) -> Self {
        Newtype(esp_netif_ip_info_t {
            ip: Newtype::<esp_ip4_addr_t>::from(ip_info.ip).0,
            netmask: Newtype::<esp_ip4_addr_t>::from(ip_info.subnet.mask).0,
            gw: Newtype::<esp_ip4_addr_t>::from(ip_info.subnet.gateway).0,
        })
    }
}

impl From<Newtype<esp_netif_ip_info_t>> for ipv4::IpInfo {
    fn from(ip_info: Newtype<esp_netif_ip_info_t>) -> Self {
        ipv4::IpInfo {
            ip: ipv4::Ipv4Addr::from(Newtype(ip_info.0.ip)),
            subnet: ipv4::Subnet {
                gateway: ipv4::Ipv4Addr::from(Newtype(ip_info.0.gw)),
                mask: Newtype(ip_info.0.netmask).try_into().unwrap(),
            },
            dns: None,
            secondary_dns: None,
        }
    }
}

impl From<Newtype<esp_ip6_addr_t>> for core::net::Ipv6Addr {
    fn from(value: Newtype<esp_ip6_addr_t>) -> Self {
        let mut out = [0; 16];
        value
            .0
            .addr
            .into_iter()
            .map(u32::from_be)
            .flat_map(u32::to_be_bytes)
            .zip(out.iter_mut())
            .for_each(|(i, o)| *o = i);

        core::net::Ipv6Addr::from(out)
    }
}
