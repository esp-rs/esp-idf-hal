use core::convert::TryInto;
use core::ptr;

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use cstr_core::CString;

use embedded_svc::ipv4;

use esp_idf_hal::mutex;

use esp_idf_sys::*;

use crate::eventloop::{EspTypedEventDeserializer, EspTypedEventSource};
use crate::private::common::*;
use crate::private::cstr::*;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    pub key: heapless::String<32>,
    pub description: heapless::String<8>,
    pub route_priority: u32,
    pub ip_configuration: InterfaceIpConfiguration,
    pub interface_stack: InterfaceStack,
    pub custom_mac: Option<[u8; 6]>,
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
            custom_mac: None,
        }
    }

    pub fn eth_default_router() -> Self {
        Self {
            key: "ETH_RT_DEF".into(),
            description: "ethrt".into(),
            route_priority: 50,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Eth,
            custom_mac: None,
        }
    }

    pub fn wifi_default_client() -> Self {
        Self {
            key: "WIFI_STA_DEF".into(),
            description: "sta".into(),
            route_priority: 100,
            ip_configuration: InterfaceIpConfiguration::Client(Default::default()),
            interface_stack: InterfaceStack::Sta,
            custom_mac: None,
        }
    }

    pub fn wifi_default_router() -> Self {
        Self {
            key: "WIFI_AP_DEF".into(),
            description: "ap".into(),
            route_priority: 10,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            interface_stack: InterfaceStack::Ap,
            custom_mac: None,
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
            custom_mac: None,
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
            custom_mac: None,
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
            custom_mac: None,
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
            custom_mac: None,
        }
    }
}

static TAKEN: mutex::Mutex<(bool, bool)> = mutex::Mutex::new((false, false));

#[derive(Debug)]
struct PrivateData;

#[derive(Debug)]
pub struct EspNetifStack(PrivateData);

impl EspNetifStack {
    pub fn new() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if taken.0 {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        if !taken.1 {
            esp!(unsafe { esp_netif_init() })?;
        }

        *taken = (true, true);
        Ok(Self(PrivateData))
    }
}

impl Drop for EspNetifStack {
    fn drop(&mut self) {
        // ESP netif does not support deinitialization yet, so we only flag that it is no longer owned
        *TAKEN.lock() = (false, true);

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

        let initial_mac = if let Some(custom_mac) = conf.custom_mac {
            custom_mac
        } else {
            let mut mac = [0; 6];
            match conf.interface_stack {
                InterfaceStack::Sta => esp!(unsafe {
                    esp_read_mac(mac.as_mut_ptr() as *mut _, esp_mac_type_t_ESP_MAC_WIFI_STA)
                })?,
                InterfaceStack::Ap => esp!(unsafe {
                    esp_read_mac(
                        mac.as_mut_ptr() as *mut _,
                        esp_mac_type_t_ESP_MAC_WIFI_SOFTAP,
                    )
                })?,
                InterfaceStack::Eth => esp!(unsafe {
                    esp_read_mac(mac.as_mut_ptr() as *mut _, esp_mac_type_t_ESP_MAC_ETH)
                })?,
                #[cfg(esp_idf_slip_support)]
                #[cfg(esp_idf_ppp_support)]
                _ => {}
            };
            mac
        };

        let (mut esp_inherent_config, ip_info, dhcps, dns, secondary_dns, hostname) = match conf
            .ip_configuration
        {
            InterfaceIpConfiguration::Client(ref ip_conf) => (
                esp_netif_inherent_config_t {
                    flags: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
                            esp_netif_flags_ESP_NETIF_DHCP_CLIENT
                                | esp_netif_flags_ESP_NETIF_FLAG_GARP
                                | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED
                        }
                        ipv4::ClientConfiguration::Fixed(_) => {
                            esp_netif_flags_ESP_NETIF_FLAG_AUTOUP
                        }
                    },
                    mac: initial_mac,
                    ip_info: ptr::null(),
                    get_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
                            if conf.interface_stack == InterfaceStack::Sta {
                                ip_event_t_IP_EVENT_STA_GOT_IP
                            } else {
                                0
                            }
                        }
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    lost_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
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
                    ipv4::ClientConfiguration::DHCP(_) => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => Some(esp_netif_ip_info_t {
                        ip: Newtype::<esp_ip4_addr_t>::from(fixed_conf.ip).0,
                        netmask: Newtype::<esp_ip4_addr_t>::from(fixed_conf.subnet.mask).0,
                        gw: Newtype::<esp_ip4_addr_t>::from(fixed_conf.subnet.gateway).0,
                    }),
                },
                false,
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP(_) => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => fixed_conf.dns,
                },
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP(_) => None,
                    ipv4::ClientConfiguration::Fixed(ref fixed_conf) => fixed_conf.secondary_dns,
                },
                match ip_conf {
                    ipv4::ClientConfiguration::DHCP(ref dhcp_conf) => dhcp_conf.hostname.as_ref(),
                    ipv4::ClientConfiguration::Fixed(_) => None,
                },
            ),
            InterfaceIpConfiguration::Router(ref ip_conf) => (
                esp_netif_inherent_config_t {
                    flags: (if ip_conf.dhcp_enabled {
                        esp_netif_flags_ESP_NETIF_DHCP_SERVER
                    } else {
                        0
                    }) | esp_netif_flags_ESP_NETIF_FLAG_AUTOUP,
                    mac: initial_mac,
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
                None,
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
                #[cfg(esp_idf_version_major = "4")]
                let mut dhcps_dns_value: dhcps_offer_t = dhcps_offer_option_OFFER_DNS as _;

                // Strangely dhcps_offer_t and dhcps_offer_option_* are not included in ESP-IDF V5's bindings
                #[cfg(esp_idf_version_major = "5")]
                let mut dhcps_dns_value: u8 = 2_u8;

                esp!(unsafe {
                    esp_netif_dhcps_option(
                        netif.1,
                        esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
                        esp_netif_dhcp_option_id_t_ESP_NETIF_DOMAIN_NAME_SERVER,
                        &mut dhcps_dns_value as *mut _ as *mut _,
                        core::mem::size_of_val(&dhcps_dns_value) as u32,
                    )
                })?;
            }
        }

        if let Some(secondary_dns) = secondary_dns {
            netif.set_secondary_dns(secondary_dns);
        }

        if let Some(hostname) = hostname {
            netif.set_hostname(hostname)?;
        }

        Ok(netif)
    }

    pub fn get_key(&self) -> heapless::String<32> {
        from_cstr_ptr(unsafe { esp_netif_get_ifkey(self.1) }).into()
    }

    pub fn get_index(&self) -> u32 {
        unsafe { esp_netif_get_netif_impl_index(self.1) as _ }
    }

    pub fn get_name(&self) -> heapless::String<6> {
        let mut netif_name = [0u8; 7];

        esp!(unsafe { esp_netif_get_netif_impl_name(self.1, netif_name.as_mut_ptr() as *mut _) })
            .unwrap();

        from_cstr(&netif_name).into()
    }

    pub fn get_mac(&self) -> Result<[u8; 6], EspError> {
        let mut mac = [0u8; 6];

        esp!(unsafe { esp_netif_get_mac(self.1, mac.as_mut_ptr() as *mut _) })?;
        Ok(mac)
    }

    pub fn set_mac(&mut self, mac: &[u8; 6]) -> Result<(), EspError> {
        esp!(unsafe { esp_netif_set_mac(self.1, mac.as_ptr() as *mut _) })?;
        Ok(())
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

    #[cfg(esp_idf_lwip_ipv4_napt)]
    pub fn enable_napt(&mut self, enable: bool) {
        unsafe {
            esp_idf_sys::ip_napt_enable_no(
                (esp_netif_get_netif_impl_index(self.1) - 1) as u8,
                if enable { 1 } else { 0 },
            )
        };
    }

    pub fn get_hostname(&self) -> Result<heapless::String<30>, EspError> {
        let mut ptr: *const c_types::c_char = core::ptr::null();
        esp!(unsafe { esp_netif_get_hostname(self.1, &mut ptr) })?;

        Ok(from_cstr_ptr(ptr).into())
    }

    pub fn set_hostname(&self, hostname: &str) -> Result<(), EspError> {
        if let Ok(hostname) = CString::new(hostname) {
            esp!(unsafe { esp_netif_set_hostname(self.1, hostname.as_ptr() as *const _) })?;
        } else {
            esp!(ESP_ERR_INVALID_ARG)?;
        }

        Ok(())
    }
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        unsafe { esp_netif_destroy(self.1) };
    }
}

pub type NetifHandle = *const core::ffi::c_void;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ApStaIpAssignment {
    pub ip: ipv4::Ipv4Addr,
    #[cfg(not(esp_idf_version_major = "4"))]
    pub mac: [u8; 6],
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct DhcpIpAssignment {
    pub netif_handle: NetifHandle,
    pub ip_settings: ipv4::ClientSettings,
    pub ip_changed: bool,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct DhcpIp6Assignment {
    pub netif_handle: NetifHandle,
    pub ip: [u32; 4],
    pub ip_zone: u8,
    pub ip_index: u32,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum IpEvent {
    ApStaIpAssigned(ApStaIpAssignment),
    DhcpIpAssigned(DhcpIpAssignment),
    DhcpIp6Assigned(DhcpIp6Assignment),
    DhcpIpDeassigned(NetifHandle),
}

impl IpEvent {
    pub fn handle(&self) -> Option<NetifHandle> {
        match self {
            Self::ApStaIpAssigned(_) => None,
            Self::DhcpIpAssigned(assignment) => Some(assignment.netif_handle),
            Self::DhcpIp6Assigned(assignment) => Some(assignment.netif_handle),
            Self::DhcpIpDeassigned(handle) => Some(*handle),
        }
    }
}

impl EspTypedEventSource for IpEvent {
    fn source() -> *const c_types::c_char {
        unsafe { IP_EVENT }
    }
}

impl EspTypedEventDeserializer<IpEvent> for IpEvent {
    #[allow(non_upper_case_globals, non_snake_case)]
    fn deserialize<R>(
        data: &crate::eventloop::EspEventFetchData,
        f: &mut impl for<'a> FnMut(&'a IpEvent) -> R,
    ) -> R {
        let event_id = data.event_id as u32;

        let event = if event_id == ip_event_t_IP_EVENT_AP_STAIPASSIGNED {
            let event = unsafe {
                (data.payload as *const ip_event_ap_staipassigned_t)
                    .as_ref()
                    .unwrap()
            };

            IpEvent::ApStaIpAssigned(ApStaIpAssignment {
                ip: ipv4::Ipv4Addr::from(Newtype(event.ip)),
                #[cfg(not(esp_idf_version_major = "4"))]
                mac: event.mac,
            })
        } else if event_id == ip_event_t_IP_EVENT_STA_GOT_IP
            || event_id == ip_event_t_IP_EVENT_ETH_GOT_IP
            || event_id == ip_event_t_IP_EVENT_PPP_GOT_IP
        {
            let event = unsafe { (data.payload as *const ip_event_got_ip_t).as_ref().unwrap() };

            IpEvent::DhcpIpAssigned(DhcpIpAssignment {
                netif_handle: event.esp_netif as _,
                ip_settings: ipv4::ClientSettings {
                    ip: ipv4::Ipv4Addr::from(Newtype(event.ip_info.ip)),
                    subnet: ipv4::Subnet {
                        gateway: ipv4::Ipv4Addr::from(Newtype(event.ip_info.gw)),
                        mask: Newtype(event.ip_info.netmask).try_into().unwrap(),
                    },
                    dns: None,           // TODO
                    secondary_dns: None, // TODO
                },
                ip_changed: event.ip_changed,
            })
        } else if event_id == ip_event_t_IP_EVENT_GOT_IP6 {
            let event = unsafe {
                (data.payload as *const ip_event_got_ip6_t)
                    .as_ref()
                    .unwrap()
            };

            IpEvent::DhcpIp6Assigned(DhcpIp6Assignment {
                netif_handle: event.esp_netif as _,
                ip: event.ip6_info.ip.addr,
                ip_zone: event.ip6_info.ip.zone,
                ip_index: event.ip_index as _,
            })
        } else if event_id == ip_event_t_IP_EVENT_STA_LOST_IP
            || event_id == ip_event_t_IP_EVENT_PPP_LOST_IP
        {
            let netif_handle_ref = unsafe { (data.payload as *const *mut esp_netif_obj).as_ref() };

            IpEvent::DhcpIpDeassigned(*netif_handle_ref.unwrap() as _)
        } else {
            panic!("Unknown event ID: {}", event_id);
        };

        f(&event)
    }
}
