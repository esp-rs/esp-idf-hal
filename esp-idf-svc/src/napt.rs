use mutex_trait::Mutex;

use embedded_svc::ipv4;

use esp_idf_sys::*;

use crate::private::common::*;

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspNapt(PrivateData);

pub enum Protocol {
    UDP,
    TCP,
}

impl Protocol {
    fn get_num_proto(&self) -> u8 {
        match self {
            Self::UDP => 17,
            Self::TCP => 6,
        }
    }
}

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

impl EspNapt {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    *taken = true;
                    Ok(Self(PrivateData))
                }
            })
        }
    }

    pub fn add_portmap(
        protocol: Protocol,
        external_ip: ipv4::Ipv4Addr,
        external_port: u16,
        internal_ip: ipv4::Ipv4Addr,
        internal_port: u16,
    ) -> bool {
        unsafe {
            ip_portmap_add(
                protocol.get_num_proto(),
                Newtype::<esp_ip4_addr_t>::from(external_ip).0.addr,
                external_port,
                Newtype::<esp_ip4_addr_t>::from(internal_ip).0.addr,
                internal_port,
            ) != 0
        }
    }

    pub fn remove_portmap(protocol: Protocol, external_port: u16) -> bool {
        unsafe { ip_portmap_remove(protocol.get_num_proto(), external_port) != 0 }
    }
}

impl Drop for EspNapt {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                *taken = false;
            });
        }
    }
}
