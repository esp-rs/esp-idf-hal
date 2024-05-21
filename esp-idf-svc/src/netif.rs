//! Network abstraction
//!
//! The purpose of ESP-NETIF library is twofold:
//!
//! - It provides an abstraction layer for the application on top of the TCP/IP
//!   stack. This will allow applications to choose between IP stacks in the
//!   future.
//! - The APIs it provides are thread safe, even if the underlying TCP/IP
//!   stack APIs are not.

use core::{ffi, fmt, ptr};

use crate::ipv4;
use crate::sys::*;

use ::log::info;

use crate::eventloop::{EspEventDeserializer, EspEventSource};
use crate::handle::RawHandle;
use crate::private::common::*;
use crate::private::cstr::*;
use crate::private::mutex;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
pub enum NetifStack {
    /// Station mode (WiFi client)
    Sta,
    #[cfg(esp_idf_esp_wifi_softap_support)]
    /// Access point mode (WiFi router)
    Ap,
    /// Ethernet
    Eth,
    #[cfg(esp_idf_lwip_ppp_support)]
    /// Point-to-Point Protocol (PPP)
    Ppp,
    #[cfg(esp_idf_lwip_slip_support)]
    /// Serial Line Internet Protocol (SLIP)
    Slip,
}

impl NetifStack {
    pub fn default_configuration(&self) -> NetifConfiguration {
        match self {
            Self::Sta => NetifConfiguration::wifi_default_client(),
            #[cfg(esp_idf_esp_wifi_softap_support)]
            Self::Ap => NetifConfiguration::wifi_default_router(),
            Self::Eth => NetifConfiguration::eth_default_client(),
            #[cfg(esp_idf_lwip_ppp_support)]
            Self::Ppp => NetifConfiguration::ppp_default_client(),
            #[cfg(esp_idf_lwip_slip_support)]
            Self::Slip => NetifConfiguration::slip_default_client(),
        }
    }

    fn default_mac(&self) -> Result<Option<[u8; 6]>, EspError> {
        if let Some(mac_type) = self.default_mac_raw_type() {
            let mut mac = [0; 6];
            esp!(unsafe { esp_read_mac(mac.as_mut_ptr() as *mut _, mac_type) })?;

            Ok(Some(mac))
        } else {
            Ok(None)
        }
    }

    fn default_mac_raw_type(&self) -> Option<esp_mac_type_t> {
        match self {
            Self::Sta => Some(esp_mac_type_t_ESP_MAC_WIFI_STA),
            #[cfg(esp_idf_esp_wifi_softap_support)]
            Self::Ap => Some(esp_mac_type_t_ESP_MAC_WIFI_SOFTAP),
            Self::Eth => Some(esp_mac_type_t_ESP_MAC_ETH),
            #[cfg(any(esp_idf_lwip_slip_support, esp_idf_lwip_ppp_support))]
            _ => None,
        }
    }

    fn default_raw_stack(&self) -> *const esp_netif_netstack_config_t {
        unsafe {
            match self {
                Self::Sta => _g_esp_netif_netstack_default_wifi_sta,
                #[cfg(esp_idf_esp_wifi_softap_support)]
                Self::Ap => _g_esp_netif_netstack_default_wifi_ap,
                Self::Eth => _g_esp_netif_netstack_default_eth,
                #[cfg(esp_idf_lwip_ppp_support)]
                Self::Ppp => _g_esp_netif_netstack_default_ppp,
                #[cfg(esp_idf_lwip_slip_support)]
                Self::Slip => _g_esp_netif_netstack_default_slip,
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NetifConfiguration {
    pub key: heapless::String<32>,
    pub description: heapless::String<8>,
    pub route_priority: u32,
    pub ip_configuration: ipv4::Configuration,
    pub stack: NetifStack,
    pub custom_mac: Option<[u8; 6]>,
}

impl NetifConfiguration {
    pub fn eth_default_client() -> Self {
        Self {
            key: "ETH_CL_DEF".try_into().unwrap(),
            description: "eth".try_into().unwrap(),
            route_priority: 60,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Eth,
            custom_mac: None,
        }
    }

    pub fn eth_default_router() -> Self {
        Self {
            key: "ETH_RT_DEF".try_into().unwrap(),
            description: "ethrt".try_into().unwrap(),
            route_priority: 50,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Eth,
            custom_mac: None,
        }
    }

    pub fn wifi_default_client() -> Self {
        Self {
            key: "WIFI_STA_DEF".try_into().unwrap(),
            description: "sta".try_into().unwrap(),
            route_priority: 100,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Sta,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_esp_wifi_softap_support)]
    pub fn wifi_default_router() -> Self {
        Self {
            key: "WIFI_AP_DEF".try_into().unwrap(),
            description: "ap".try_into().unwrap(),
            route_priority: 10,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Ap,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_lwip_ppp_support)]
    pub fn ppp_default_client() -> Self {
        Self {
            key: "PPP_CL_DEF".try_into().unwrap(),
            description: "ppp".try_into().unwrap(),
            route_priority: 30,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Ppp,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_lwip_ppp_support)]
    pub fn ppp_default_router() -> Self {
        Self {
            key: "PPP_RT_DEF".try_into().unwrap(),
            description: "ppprt".try_into().unwrap(),
            route_priority: 20,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Ppp,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_lwip_slip_support)]
    pub fn slip_default_client() -> Self {
        Self {
            key: "SLIP_CL_DEF".try_into().unwrap(),
            description: "slip".try_into().unwrap(),
            route_priority: 35,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Slip,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_lwip_slip_support)]
    pub fn slip_default_router() -> Self {
        Self {
            key: "SLIP_RT_DEF".try_into().unwrap(),
            description: "sliprt".try_into().unwrap(),
            route_priority: 25,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Slip,
            custom_mac: None,
        }
    }
}

static INITALIZED: mutex::Mutex<bool> = mutex::Mutex::new(false);

fn initialize_netif_stack() -> Result<(), EspError> {
    let mut guard = INITALIZED.lock();

    if !*guard {
        esp!(unsafe { esp_netif_init() })?;

        *guard = true;
    }

    Ok(())
}

#[derive(Debug)]
pub struct EspNetif(*mut esp_netif_t);

impl EspNetif {
    pub fn new(stack: NetifStack) -> Result<Self, EspError> {
        Self::new_with_conf(&stack.default_configuration())
    }

    pub fn new_with_conf(conf: &NetifConfiguration) -> Result<Self, EspError> {
        initialize_netif_stack()?;

        let c_if_key = to_cstring_arg(conf.key.as_str())?;
        let c_if_description = to_cstring_arg(conf.description.as_str())?;

        let initial_mac = if let Some(custom_mac) = conf.custom_mac {
            custom_mac
        } else {
            conf.stack.default_mac()?.unwrap_or([0; 6])
        };

        let (mut esp_inherent_config, ip_info, dhcps, dns, secondary_dns, hostname) = match conf
            .ip_configuration
        {
            ipv4::Configuration::Client(ref ip_conf) => (
                esp_netif_inherent_config_t {
                    flags: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
                            esp_netif_flags_ESP_NETIF_DHCP_CLIENT
                                | esp_netif_flags_ESP_NETIF_FLAG_GARP
                                | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED
                        }
                        ipv4::ClientConfiguration::Fixed(_) => {
                            esp_netif_flags_ESP_NETIF_FLAG_GARP
                                | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED
                        }
                    },
                    mac: initial_mac,
                    ip_info: ptr::null(),
                    get_ip_event: ip_event_t_IP_EVENT_STA_GOT_IP,
                    lost_ip_event: ip_event_t_IP_EVENT_STA_LOST_IP,
                    if_key: c_if_key.as_c_str().as_ptr() as _,
                    if_desc: c_if_description.as_c_str().as_ptr() as _,
                    route_prio: conf.route_priority as _,
                    #[cfg(not(esp_idf_version_major = "4"))]
                    bridge_info: ptr::null_mut(),
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
            ipv4::Configuration::Router(ref ip_conf) => (
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
                    #[cfg(not(esp_idf_version_major = "4"))]
                    bridge_info: ptr::null_mut(),
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
            stack: conf.stack.default_raw_stack(),
        };

        let mut handle = Self(
            unsafe { esp_netif_new(&cfg).as_mut() }
                .ok_or(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?,
        );

        if let Some(dns) = dns {
            handle.set_dns(dns);

            if dhcps {
                #[cfg(esp_idf_version_major = "4")]
                let mut dhcps_dns_value: dhcps_offer_t = dhcps_offer_option_OFFER_DNS as _;

                // Strangely dhcps_offer_t and dhcps_offer_option_* are not included in ESP-IDF V5's bindings
                #[cfg(not(esp_idf_version_major = "4"))]
                let mut dhcps_dns_value: u8 = 2_u8;

                esp!(unsafe {
                    esp_netif_dhcps_option(
                        handle.0,
                        esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
                        esp_netif_dhcp_option_id_t_ESP_NETIF_DOMAIN_NAME_SERVER,
                        &mut dhcps_dns_value as *mut _ as *mut _,
                        core::mem::size_of_val(&dhcps_dns_value) as u32,
                    )
                })?;
            }
        }

        if let Some(secondary_dns) = secondary_dns {
            handle.set_secondary_dns(secondary_dns);
        }

        if let Some(hostname) = hostname {
            handle.set_hostname(hostname)?;
        }

        Ok(handle)
    }

    pub fn is_up(&self) -> Result<bool, EspError> {
        if !unsafe { esp_netif_is_netif_up(self.0) } {
            Ok(false)
        } else {
            let mut ip_info = Default::default();
            unsafe { esp!(esp_netif_get_ip_info(self.0, &mut ip_info)) }?;

            Ok(ipv4::IpInfo::from(Newtype(ip_info)).ip != ipv4::Ipv4Addr::new(0, 0, 0, 0))
        }
    }

    pub fn get_ip_info(&self) -> Result<ipv4::IpInfo, EspError> {
        let mut ip_info = Default::default();
        unsafe { esp!(esp_netif_get_ip_info(self.0, &mut ip_info)) }?;

        Ok(ipv4::IpInfo {
            // Get the DNS information
            dns: Some(self.get_dns()),
            secondary_dns: Some(self.get_secondary_dns()),
            ..Newtype(ip_info).into()
        })
    }

    pub fn get_key(&self) -> heapless::String<32> {
        unsafe { from_cstr_ptr(esp_netif_get_ifkey(self.0)) }
            .try_into()
            .unwrap()
    }

    pub fn get_index(&self) -> u32 {
        unsafe { esp_netif_get_netif_impl_index(self.0) as _ }
    }

    pub fn get_name(&self) -> heapless::String<6> {
        let mut netif_name = [0u8; 7];

        esp!(unsafe { esp_netif_get_netif_impl_name(self.0, netif_name.as_mut_ptr() as *mut _) })
            .unwrap();

        from_cstr(&netif_name).try_into().unwrap()
    }

    pub fn get_mac(&self) -> Result<[u8; 6], EspError> {
        let mut mac = [0u8; 6];

        esp!(unsafe { esp_netif_get_mac(self.0, mac.as_mut_ptr() as *mut _) })?;
        Ok(mac)
    }

    pub fn set_mac(&mut self, mac: &[u8; 6]) -> Result<(), EspError> {
        esp!(unsafe { esp_netif_set_mac(self.0, mac.as_ptr() as *mut _) })?;
        Ok(())
    }

    pub fn get_dns(&self) -> ipv4::Ipv4Addr {
        let mut dns_info = Default::default();

        unsafe {
            esp!(esp_netif_get_dns_info(
                self.0,
                esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
                &mut dns_info
            ))
            .unwrap();

            Newtype(dns_info.ip.u_addr.ip4).into()
        }
    }

    fn set_dns(&mut self, dns: ipv4::Ipv4Addr) {
        let mut dns_info: esp_netif_dns_info_t = Default::default();

        unsafe {
            dns_info.ip.u_addr.ip4 = Newtype::<esp_ip4_addr_t>::from(dns).0;

            esp!(esp_netif_set_dns_info(
                self.0,
                esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
                &mut dns_info
            ))
            .unwrap();
        }
    }

    pub fn get_secondary_dns(&self) -> ipv4::Ipv4Addr {
        let mut dns_info = Default::default();

        unsafe {
            esp!(esp_netif_get_dns_info(
                self.0,
                esp_netif_dns_type_t_ESP_NETIF_DNS_BACKUP,
                &mut dns_info
            ))
            .unwrap();

            Newtype(dns_info.ip.u_addr.ip4).into()
        }
    }

    fn set_secondary_dns(&mut self, secondary_dns: ipv4::Ipv4Addr) {
        let mut dns_info: esp_netif_dns_info_t = Default::default();

        unsafe {
            dns_info.ip.u_addr.ip4 = Newtype::<esp_ip4_addr_t>::from(secondary_dns).0;

            esp!(esp_netif_set_dns_info(
                self.0,
                esp_netif_dns_type_t_ESP_NETIF_DNS_BACKUP,
                &mut dns_info
            ))
            .unwrap();
        }
    }

    pub fn get_hostname(&self) -> Result<heapless::String<30>, EspError> {
        let mut ptr: *const ffi::c_char = ptr::null();
        esp!(unsafe { esp_netif_get_hostname(self.0, &mut ptr) })?;

        Ok(unsafe { from_cstr_ptr(ptr) }.try_into().unwrap())
    }

    fn set_hostname(&mut self, hostname: &str) -> Result<(), EspError> {
        let hostname = to_cstring_arg(hostname)?;

        esp!(unsafe { esp_netif_set_hostname(self.0, hostname.as_ptr() as *const _) })?;

        Ok(())
    }

    #[cfg(esp_idf_lwip_ipv4_napt)]
    pub fn enable_napt(&mut self, enable: bool) {
        unsafe {
            crate::sys::ip_napt_enable_no(
                (esp_netif_get_netif_impl_index(self.0) - 1) as u8,
                if enable { 1 } else { 0 },
            )
        };
    }
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        unsafe { esp_netif_destroy(self.0) };

        info!("Dropped");
    }
}

unsafe impl Send for EspNetif {}

impl RawHandle for EspNetif {
    type Handle = *mut esp_netif_t;

    fn handle(&self) -> Self::Handle {
        self.0
    }
}

#[derive(Copy, Clone)]
pub struct ApStaIpAssignment<'a>(&'a ip_event_ap_staipassigned_t);

impl ApStaIpAssignment<'_> {
    pub fn ip(&self) -> ipv4::Ipv4Addr {
        ipv4::Ipv4Addr::from(Newtype(self.0.ip))
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    pub fn mac(&self) -> [u8; 6] {
        self.0.mac
    }
}

impl fmt::Debug for ApStaIpAssignment<'_> {
    #[cfg(esp_idf_version_major = "4")]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ApStaIpAssignment")
            .field("ip", &self.ip())
            .finish()
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ApStaIpAssignment")
            .field("ip", &self.ip())
            .field("mac", &self.mac())
            .finish()
    }
}

#[derive(Copy, Clone)]
pub struct DhcpIpAssignment<'a>(&'a ip_event_got_ip_t);

impl<'a> DhcpIpAssignment<'a> {
    pub fn netif_handle(&self) -> *mut esp_netif_t {
        self.0.esp_netif
    }

    pub fn ip(&self) -> ipv4::Ipv4Addr {
        ipv4::Ipv4Addr::from(Newtype(self.0.ip_info.ip))
    }

    pub fn gateway(&self) -> ipv4::Ipv4Addr {
        ipv4::Ipv4Addr::from(Newtype(self.0.ip_info.gw))
    }

    pub fn mask(&self) -> ipv4::Mask {
        Newtype(self.0.ip_info.netmask).try_into().unwrap()
    }

    pub fn ip_info(&self) -> ipv4::IpInfo {
        ipv4::IpInfo {
            ip: self.ip(),
            subnet: ipv4::Subnet {
                gateway: self.gateway(),
                mask: self.mask(),
            },
            dns: None,           // TODO
            secondary_dns: None, // TODO
        }
    }

    pub fn is_ip_changed(&self) -> bool {
        self.0.ip_changed
    }
}

impl<'a> fmt::Debug for DhcpIpAssignment<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DhcpIpAssignment")
            .field("netif_handle", &self.netif_handle())
            .field("ip", &self.ip())
            .field("gateway", &self.gateway())
            .field("mask", &self.mask())
            .field("is_ip_changed", &self.is_ip_changed())
            .finish()
    }
}

#[derive(Copy, Clone)]
pub struct DhcpIp6Assignment<'a>(&'a ip_event_got_ip6_t);

impl<'a> DhcpIp6Assignment<'a> {
    pub fn netif_handle(&self) -> *mut esp_netif_t {
        self.0.esp_netif
    }

    pub fn ip(&self) -> [u32; 4] {
        self.0.ip6_info.ip.addr
    }

    pub fn ip_zone(&self) -> u8 {
        self.0.ip6_info.ip.zone
    }

    pub fn ip_index(&self) -> u32 {
        self.0.ip_index as _
    }
}

impl<'a> fmt::Debug for DhcpIp6Assignment<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DhcpIp6Assignment")
            .field("netif_handle", &self.netif_handle())
            .field("ip", &self.ip())
            .field("ip_zone", &self.ip_zone())
            .field("ip_index", &self.ip_index())
            .finish()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum IpEvent<'a> {
    ApStaIpAssigned(ApStaIpAssignment<'a>),
    DhcpIpAssigned(DhcpIpAssignment<'a>),
    DhcpIp6Assigned(DhcpIp6Assignment<'a>),
    DhcpIpDeassigned(*mut esp_netif_t),
}

unsafe impl<'a> Send for IpEvent<'a> {}

impl<'a> IpEvent<'a> {
    pub fn is_for(&self, raw_handle: &impl RawHandle<Handle = *mut esp_netif_t>) -> bool {
        self.is_for_handle(raw_handle.handle())
    }

    pub fn is_for_handle(&self, handle: *mut esp_netif_t) -> bool {
        self.handle()
            .map(|event_handle| event_handle == handle)
            .unwrap_or(false)
    }

    pub fn handle(&self) -> Option<*mut esp_netif_t> {
        match self {
            Self::ApStaIpAssigned(_) => None,
            Self::DhcpIpAssigned(assignment) => Some(assignment.netif_handle()),
            Self::DhcpIp6Assigned(assignment) => Some(assignment.netif_handle()),
            Self::DhcpIpDeassigned(handle) => Some(*handle),
        }
    }
}

unsafe impl<'a> EspEventSource for IpEvent<'a> {
    fn source() -> Option<&'static ffi::CStr> {
        Some(unsafe { CStr::from_ptr(IP_EVENT) })
    }
}

impl<'a> EspEventDeserializer for IpEvent<'a> {
    type Data<'d> = IpEvent<'d>;

    #[allow(non_upper_case_globals, non_snake_case)]
    fn deserialize<'d>(data: &crate::eventloop::EspEvent<'d>) -> IpEvent<'d> {
        let event_id = data.event_id as u32;

        if event_id == ip_event_t_IP_EVENT_AP_STAIPASSIGNED {
            let event = unsafe {
                (data.payload.unwrap() as *const _ as *const ip_event_ap_staipassigned_t)
                    .as_ref()
                    .unwrap()
            };

            IpEvent::ApStaIpAssigned(ApStaIpAssignment(event))
        } else if event_id == ip_event_t_IP_EVENT_STA_GOT_IP
            || event_id == ip_event_t_IP_EVENT_ETH_GOT_IP
            || event_id == ip_event_t_IP_EVENT_PPP_GOT_IP
        {
            let event = unsafe {
                (data.payload.unwrap() as *const _ as *const ip_event_got_ip_t)
                    .as_ref()
                    .unwrap()
            };

            IpEvent::DhcpIpAssigned(DhcpIpAssignment(event))
        } else if event_id == ip_event_t_IP_EVENT_GOT_IP6 {
            let event = unsafe {
                (data.payload.unwrap() as *const _ as *const ip_event_got_ip6_t)
                    .as_ref()
                    .unwrap()
            };

            IpEvent::DhcpIp6Assigned(DhcpIp6Assignment(event))
        } else if event_id == ip_event_t_IP_EVENT_STA_LOST_IP
            || event_id == ip_event_t_IP_EVENT_PPP_LOST_IP
        {
            let netif_handle_mut = unsafe {
                (data.payload.unwrap() as *const _ as *mut esp_netif_t)
                    .as_mut()
                    .unwrap()
            };

            IpEvent::DhcpIpDeassigned(netif_handle_mut as *mut _)
        } else {
            panic!("Unknown event ID: {}", event_id);
        }
    }
}

pub trait NetifStatus {
    fn is_up(&self) -> Result<bool, EspError>;
}

impl<T> NetifStatus for &T
where
    T: NetifStatus,
{
    fn is_up(&self) -> Result<bool, EspError> {
        (**self).is_up()
    }
}

impl<T> NetifStatus for &mut T
where
    T: NetifStatus,
{
    fn is_up(&self) -> Result<bool, EspError> {
        (**self).is_up()
    }
}

impl NetifStatus for EspNetif {
    fn is_up(&self) -> Result<bool, EspError> {
        EspNetif::is_up(self)
    }
}

const UP_TIMEOUT: core::time::Duration = core::time::Duration::from_secs(15);

pub struct BlockingNetif<T> {
    netif: T,
    event_loop: crate::eventloop::EspSystemEventLoop,
}

impl<T> BlockingNetif<T>
where
    T: NetifStatus,
{
    pub fn wrap(netif: T, event_loop: crate::eventloop::EspSystemEventLoop) -> Self {
        Self { netif, event_loop }
    }

    pub fn is_up(&self) -> Result<bool, EspError> {
        self.netif.is_up()
    }

    pub fn wait_netif_up(&self) -> Result<(), EspError> {
        self.ip_wait_while(|| self.netif.is_up().map(|s| !s), Some(UP_TIMEOUT))
    }

    pub fn ip_wait_while<F: Fn() -> Result<bool, EspError>>(
        &self,
        matcher: F,
        timeout: Option<core::time::Duration>,
    ) -> Result<(), EspError> {
        let wait = crate::eventloop::Wait::new::<IpEvent>(&self.event_loop)?;

        wait.wait_while(matcher, timeout)
    }
}

impl<T> NetifStatus for BlockingNetif<T>
where
    T: NetifStatus,
{
    fn is_up(&self) -> Result<bool, EspError> {
        BlockingNetif::is_up(self)
    }
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub struct AsyncNetif<T> {
    netif: T,
    event_loop: crate::eventloop::EspSystemEventLoop,
    timer_service: crate::timer::EspTaskTimerService,
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
impl<T> AsyncNetif<T>
where
    T: NetifStatus,
{
    pub fn wrap(
        netif: T,
        event_loop: crate::eventloop::EspSystemEventLoop,
        timer_service: crate::timer::EspTaskTimerService,
    ) -> Self {
        Self {
            netif,
            event_loop,
            timer_service,
        }
    }

    pub fn is_up(&self) -> Result<bool, EspError> {
        self.netif.is_up()
    }

    pub async fn wait_netif_up(&mut self) -> Result<(), EspError> {
        self.ip_wait_while(|this| this.netif.is_up().map(|s| !s), Some(UP_TIMEOUT))
            .await
    }

    pub async fn ip_wait_while<F: FnMut(&mut Self) -> Result<bool, EspError>>(
        &mut self,
        mut matcher: F,
        timeout: Option<core::time::Duration>,
    ) -> Result<(), EspError> {
        let mut wait =
            crate::eventloop::AsyncWait::<IpEvent, _>::new(&self.event_loop, &self.timer_service)?;

        wait.wait_while(|| matcher(self), timeout).await
    }
}

pub mod asynch {
    use crate::sys::EspError;

    pub trait NetifStatus {
        async fn is_up(&self) -> Result<bool, EspError>;
    }

    impl<T> NetifStatus for &T
    where
        T: NetifStatus,
    {
        async fn is_up(&self) -> Result<bool, EspError> {
            (**self).is_up().await
        }
    }

    impl<T> NetifStatus for &mut T
    where
        T: NetifStatus,
    {
        async fn is_up(&self) -> Result<bool, EspError> {
            (**self).is_up().await
        }
    }

    impl NetifStatus for super::EspNetif {
        async fn is_up(&self) -> Result<bool, EspError> {
            super::EspNetif::is_up(self)
        }
    }

    #[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
    impl<T> NetifStatus for super::AsyncNetif<T>
    where
        T: super::NetifStatus,
    {
        async fn is_up(&self) -> Result<bool, EspError> {
            super::AsyncNetif::is_up(self)
        }
    }
}
