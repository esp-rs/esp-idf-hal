use core::convert::{TryFrom, TryInto};

use embedded_svc::ipv4::{self, Mask};
use esp_idf_sys::*;

use crate::private::common::*;

impl From<ipv4::Ipv4Addr> for Newtype<esp_ip4_addr_t> {
    fn from(ip: ipv4::Ipv4Addr) -> Self {
        let octets = ip.octets();

        Newtype(esp_ip4_addr_t {
            addr: ((octets[3] as u32 & 0xff) << 24)
                | ((octets[2] as u32 & 0xff) << 16)
                | ((octets[1] as u32 & 0xff) << 8)
                | (octets[0] as u32 & 0xff),
        })
    }
}

impl From<Newtype<esp_ip4_addr_t>> for ipv4::Ipv4Addr {
    fn from(ip: Newtype<esp_ip4_addr_t>) -> Self {
        let (d, c, b, a) = (
            ((ip.0.addr >> 24) & 0xff) as u8,
            ((ip.0.addr >> 16) & 0xff) as u8,
            ((ip.0.addr >> 8) & 0xff) as u8,
            (ip.0.addr & 0xff) as u8,
        );

        ipv4::Ipv4Addr::new(a, b, c, d)
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
            .map_err(|_| EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
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
            .map_err(|_| EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap())
    }
}
