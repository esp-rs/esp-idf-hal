use core::ptr;

extern crate alloc;
use alloc::borrow::Cow;
use alloc::string::String;
use alloc::sync::Arc;

use ::log::*;
use cstr_core::CString;

use mutex_trait::*;

use embedded_svc::ipv4;

use esp_idf_sys::*;

use crate::private::common::*;
use crate::private::cstr::*;

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "std", derive(Hash))]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub enum InterfaceStack {
    Sta,
    Ap,
    Eth,
    #[cfg(esp_idf_ppp_support)]
    Ppp,
    #[cfg(esp_idf_slip_support)]
    Slip,
}

impl InterfaceStack {
    pub fn get_default_configuration(&self) -> InterfaceConfiguration {
        match self {
            Self::Sta => InterfaceConfiguration::wifi_default_client(),
            Self::Ap => InterfaceConfiguration::wifi_default_router(),
            Self::Eth => InterfaceConfiguration::eth_default_client(),
            #[cfg(esp_idf_ppp_support)]
            Self::Ppp => InterfaceConfiguration::ppp_default_client(),
            #[cfg(esp_idf_slip_support)]
            Self::Slip => InterfaceConfiguration::slip_default_client(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub enum InterfaceIpConfiguration {
    Client(ipv4::ClientConfiguration),
    Router(ipv4::RouterConfiguration),
}

#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub struct InterfaceConfiguration {
    pub key: String,
    pub description: String,
    pub route_priority: u32,
    pub ip_configuration: InterfaceIpConfiguration,
    pub interface_stack: InterfaceStack,
}

impl Default for InterfaceConfiguration {
    fn default() -> Self {
        Self::wifi_default_client()
    }
}

impl InterfaceConfiguration {
    pub fn eth_default_client() -> Self {
        Self {
            key: "ETH_CL_DEF".into(),
            description: "eth".into(),
            route_priority: 60,
            ip_configuration: InterfaceIpConfiguration::Client(Default::default()),
            interface_stack: InterfaceStack::Eth,
        }
    }

    pub fn eth_default_router() -> Self {
        Self {
            key: "ETH_RT_DEF".into(),
            description: "ethrt".into(),
            route_priority: 50,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Eth,
        }
    }

    pub fn wifi_default_client() -> Self {
        Self {
            key: "WIFI_STA_DEF".into(),
            description: "sta".into(),
            route_priority: 100,
            ip_configuration: InterfaceIpConfiguration::Client(Default::default()),
            interface_stack: InterfaceStack::Sta,
        }
    }

    pub fn wifi_default_router() -> Self {
        Self {
            key: "WIFI_AP_DEF".into(),
            description: "ap".into(),
            route_priority: 10,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Ap,
        }
    }

    #[cfg(esp_idf_ppp_support)]
    pub fn ppp_default_client() -> Self {
        Self {
            key: "PPP_CL_DEF".into(),
            description: "ppp".into(),
            route_priority: 30,
            ip_configuration: InterfaceIpConfiguration::Client(Default::default()),
            interface_stack: InterfaceStack::Ppp,
        }
    }

    #[cfg(esp_idf_ppp_support)]
    pub fn ppp_default_router() -> Self {
        Self {
            key: "PPP_RT_DEF".into(),
            description: "ppprt".into(),
            route_priority: 20,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Ppp,
        }
    }

    #[cfg(esp_idf_slip_support)]
    pub fn slip_default_client() -> Self {
        Self {
            key: "SLIP_CL_DEF".into(),
            description: "slip".into(),
            route_priority: 35,
            ip_configuration: InterfaceIpConfiguration::Client(Default::default()),
            interface_stack: InterfaceStack::Slip,
        }
    }

    #[cfg(esp_idf_slip_support)]
    pub fn slip_default_router() -> Self {
        Self {
            key: "SLIP_RT_DEF".into(),
            description: "sliprt".into(),
            route_priority: 25,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Slip,
        }
    }
}

static mut TAKEN: EspMutex<(bool, bool)> = EspMutex::new((false, false));

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspNetifStack(PrivateData);

impl EspNetifStack {
    pub fn new() -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if taken.0 {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    if !taken.1 {
                        esp!(esp_netif_init())?;
                    }

                    *taken = (true, true);
                    Ok(Self(PrivateData))
                }
            })
        }
    }
}

impl Drop for EspNetifStack {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                // ESP netif does not support deinitialization yet, so we only flag that it is no longer owned
                *taken = (false, true);
            });
        }

        info!("Dropped");
    }
}

#[derive(Debug)]
pub struct EspNetif(Arc<EspNetifStack>, pub(crate) *mut esp_netif_t);

impl EspNetif {
    pub fn new(
        netif_stack: Arc<EspNetifStack>,
        conf: &InterfaceConfiguration,
    ) -> Result<Self, EspError> {
        let c_if_key = CString::new(conf.key.as_str()).unwrap();
        let c_if_description = CString::new(conf.description.as_str()).unwrap();

        let (mut esp_inherent_config, ip_info, dhcps, dns, secondary_dns) = match conf
            .ip_configuration
        {
            InterfaceIpConfiguration::Client(ref ip_conf) => (
                esp_netif_inherent_config_t {
                    flags: match ip_conf {
                        ipv4::ClientConfiguration::DHCP => {
                            esp_netif_flags_ESP_NETIF_DHCP_CLIENT
                                | esp_netif_flags_ESP_NETIF_FLAG_GARP
                                | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED
                        }
                        ipv4::ClientConfiguration::Fixed(_) => {
                            esp_netif_flags_ESP_NETIF_FLAG_AUTOUP
                        }
                    },
                    mac: [0; 6],
                    ip_info: ptr::null(),
                    get_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP => {
                            if conf.interface_stack == InterfaceStack::Sta {
                                ip_event_t_IP_EVENT_STA_GOT_IP
                            } else {
                                0
                            }
                        }
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    lost_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP => {
                            if conf.interface_stack == InterfaceStack::Sta {
                                ip_event_t_IP_EVENT_STA_LOST_IP
                            } else {
                                0
                            }
                        }
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    if_key: c_if_key.as_c_str().as_ptr() as _,
                    if_desc: c_if_description.as_c_str().as_ptr() as _,
                    route_prio: conf.route_priority as _,
                },
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => Some(esp_netif_ip_info_t {
                        ip: Newtype::<esp_ip4_addr_t>::from(fixed_conf.ip).0,
                        netmask: Newtype::<esp_ip4_addr_t>::from(fixed_conf.subnet.mask).0,
                        gw: Newtype::<esp_ip4_addr_t>::from(fixed_conf.subnet.gateway).0,
                    }),
                },
                false,
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => fixed_conf.dns,
                },
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => fixed_conf.secondary_dns,
                },
            ),
            InterfaceIpConfiguration::Router(ref ip_conf) => (
                esp_netif_inherent_config_t {
                    flags: (if ip_conf.dhcp_enabled {
                        esp_netif_flags_ESP_NETIF_DHCP_SERVER
                    } else {
                        0
                    }) | esp_netif_flags_ESP_NETIF_FLAG_AUTOUP,
                    mac: [0; 6],
                    ip_info: ptr::null(),
                    get_ip_event: 0,
                    lost_ip_event: 0,
                    if_key: c_if_key.as_c_str().as_ptr() as _,
                    if_desc: c_if_description.as_c_str().as_ptr() as _,
                    route_prio: conf.route_priority as _,
                },
                Some(esp_netif_ip_info_t {
                    ip: Newtype::<esp_ip4_addr_t>::from(ip_conf.subnet.gateway).0,
                    netmask: Newtype::<esp_ip4_addr_t>::from(ip_conf.subnet.mask).0,
                    gw: Newtype::<esp_ip4_addr_t>::from(ip_conf.subnet.gateway).0,
                }),
                ip_conf.dhcp_enabled,
                ip_conf.dns,
                None, /* For APs, ESP-IDF supports setting a primary DNS only ip_conf.secondary_dns */
            ),
        };

        if let Some(ip_info) = ip_info.as_ref() {
            esp_inherent_config.ip_info = ip_info;
        }

        let cfg = esp_netif_config_t {
            base: &esp_inherent_config,
            driver: ptr::null(),
            stack: unsafe {
                match conf.interface_stack {
                    InterfaceStack::Sta => _g_esp_netif_netstack_default_wifi_sta,
                    InterfaceStack::Ap => _g_esp_netif_netstack_default_wifi_ap,
                    InterfaceStack::Eth => _g_esp_netif_netstack_default_eth,
                    #[cfg(esp_idf_ppp_support)]
                    InterfaceStack::Ppp => _g_esp_netif_netstack_default_ppp,
                    #[cfg(esp_idf_slip_support)]
                    InterfaceStack::Slip => _g_esp_netif_netstack_default_slip,
                }
            },
        };

        let mut netif = Self(netif_stack, unsafe { esp_netif_new(&cfg) });

        if let Some(dns) = dns {
            netif.set_dns(dns);

            if dhcps {
                let mut dhcps_dns_value: dhcps_offer_t = dhcps_offer_option_OFFER_DNS as _;

                esp!(unsafe {
                    esp_netif_dhcps_option(
                        netif.1,
                        esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
                        esp_netif_dhcp_option_id_t_ESP_NETIF_DOMAIN_NAME_SERVER,
                        &mut dhcps_dns_value as *mut _ as *mut _,
                        core::mem::size_of::<dhcps_offer_t>() as u32,
                    )
                })?;
            }
        }

        if let Some(secondary_dns) = secondary_dns {
            netif.set_secondary_dns(secondary_dns);
        }

        Ok(netif)
    }

    pub fn get_key(&self) -> Cow<'_, str> {
        from_cstr_ptr(unsafe { esp_netif_get_ifkey(self.1) })
    }

    pub fn get_index(&self) -> u32 {
        unsafe { esp_netif_get_netif_impl_index(self.1) as _ }
    }

    pub fn get_name(&self) -> Cow<'_, str> {
        let mut netif_name = [0u8; 7];

        esp!(unsafe { esp_netif_get_netif_impl_name(self.1, netif_name.as_mut_ptr() as *mut _) })
            .unwrap();

        Cow::Owned(from_cstr(&netif_name).into_owned())
    }

    pub fn get_dns(&self) -> ipv4::Ipv4Addr {
        let mut dns_info = Default::default();

        unsafe {
            esp!(esp_netif_get_dns_info(
                self.1,
                esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
                &mut dns_info
            ))
            .unwrap();

            Newtype(dns_info.ip.u_addr.ip4).into()
        }
    }

    pub fn get_secondary_dns(&self) -> ipv4::Ipv4Addr {
        let mut dns_info = Default::default();

        unsafe {
            esp!(esp_netif_get_dns_info(
                self.1,
                esp_netif_dns_type_t_ESP_NETIF_DNS_BACKUP,
                &mut dns_info
            ))
            .unwrap();

            Newtype(dns_info.ip.u_addr.ip4).into()
        }
    }

    pub fn set_dns(&mut self, dns: ipv4::Ipv4Addr) {
        let mut dns_info: esp_netif_dns_info_t = Default::default();

        unsafe {
            dns_info.ip.u_addr.ip4 = Newtype::<esp_ip4_addr_t>::from(dns).0;

            esp!(esp_netif_set_dns_info(
                self.1,
                esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
                &mut dns_info
            ))
            .unwrap();
        }
    }

    pub fn set_secondary_dns(&mut self, secondary_dns: ipv4::Ipv4Addr) {
        let mut dns_info: esp_netif_dns_info_t = Default::default();

        unsafe {
            dns_info.ip.u_addr.ip4 = Newtype::<esp_ip4_addr_t>::from(secondary_dns).0;

            esp!(esp_netif_set_dns_info(
                self.1,
                esp_netif_dns_type_t_ESP_NETIF_DNS_BACKUP,
                &mut dns_info
            ))
            .unwrap();
        }
    }

    #[cfg(esp_idf_config_lwip_ipv4_napt)]
    pub fn enable_napt(&mut self, enable: bool) {
        unsafe {
            esp_idf_sys::ip_napt_enable_no(
                (esp_netif_get_netif_impl_index(self.1) - 1) as u8,
                if enable { 1 } else { 0 },
            )
        };
    }
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        unsafe { esp_netif_destroy(self.1) };
    }
}
