use core::convert::TryInto;
use core::ptr;

use ::log::*;

use embedded_svc::ipv4;

use esp_idf_hal::mutex;
use esp_idf_hal::peripheral::Peripheral;

use esp_idf_sys::*;

use crate::eventloop::{EspTypedEventDeserializer, EspTypedEventSource};
use crate::handle::RawHandle;
use crate::private::common::*;
use crate::private::cstr::*;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub enum NetifStack {
    Sta,
    Ap,
    Eth,
    #[cfg(esp_idf_ppp_support)]
    Ppp,
    #[cfg(esp_idf_slip_support)]
    Slip,
}

impl NetifStack {
    pub fn default_configuration(&self) -> NetifConfiguration {
        match self {
            Self::Sta => NetifConfiguration::wifi_default_client(),
            Self::Ap => NetifConfiguration::wifi_default_router(),
            Self::Eth => NetifConfiguration::eth_default_client(),
            #[cfg(esp_idf_ppp_support)]
            Self::Ppp => NetifConfiguration::ppp_default_client(),
            #[cfg(esp_idf_slip_support)]
            Self::Slip => NetifConfiguration::slip_default_client(),
        }
    }

    fn default_mac(&self) -> Result<Option<[u8; 6]>, EspError> {
        if let Some(mac_type) = self.get_default_mac_raw_type() {
            let mut mac = [0; 6];
            Ok(Some(esp!(unsafe {
                esp_read_mac(mac.as_mut_ptr() as *mut _, mac_type)
            })?))
        } else {
            Ok(None)
        }
    }

    fn default_mac_raw_type(&self) -> Option<esp_mac_type_t> {
        match Self {
            Self::Sta => Some(esp_mac_type_t_ESP_MAC_WIFI_STA),
            Self::Ap => Some(esp_mac_type_t_ESP_MAC_WIFI_SOFTAP),
            Self::Eth => Some(esp_mac_type_t_ESP_MAC_ETH),
            #[cfg(esp_idf_slip_support)]
            #[cfg(esp_idf_ppp_support)]
            _ => None,
        }
    }

    fn default_raw_stack(&self) -> *mut esp_netif_netstack_config_t {
        unsafe {
            match Self {
                Self::Sta => _g_esp_netif_netstack_default_wifi_sta,
                Self::Ap => _g_esp_netif_netstack_default_wifi_ap,
                Self::Eth => _g_esp_netif_netstack_default_eth,
                #[cfg(esp_idf_ppp_support)]
                Self::Ppp => _g_esp_netif_netstack_default_ppp,
                #[cfg(esp_idf_slip_support)]
                Self::Slip => _g_esp_netif_netstack_default_slip,
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
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
            key: "ETH_CL_DEF".into(),
            description: "eth".into(),
            route_priority: 60,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Eth,
            custom_mac: None,
        }
    }

    pub fn eth_default_router() -> Self {
        Self {
            key: "ETH_RT_DEF".into(),
            description: "ethrt".into(),
            route_priority: 50,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Eth,
            custom_mac: None,
        }
    }

    pub fn wifi_default_client() -> Self {
        Self {
            key: "WIFI_STA_DEF".into(),
            description: "sta".into(),
            route_priority: 100,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Sta,
            custom_mac: None,
        }
    }

    pub fn wifi_default_router() -> Self {
        Self {
            key: "WIFI_AP_DEF".into(),
            description: "ap".into(),
            route_priority: 10,
            ip_configuration: InterfaceIpConfiguration::Router(Default::default()),
            stack: NetifStack::Ap,
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
            stack: NetifStack::Ppp,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_ppp_support)]
    pub fn ppp_default_router() -> Self {
        Self {
            key: "PPP_RT_DEF".into(),
            description: "ppprt".into(),
            route_priority: 20,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Ppp,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_slip_support)]
    pub fn slip_default_client() -> Self {
        Self {
            key: "SLIP_CL_DEF".into(),
            description: "slip".into(),
            route_priority: 35,
            ip_configuration: ipv4::Configuration::Client(Default::default()),
            stack: NetifStack::Slip,
            custom_mac: None,
        }
    }

    #[cfg(esp_idf_slip_support)]
    pub fn slip_default_router() -> Self {
        Self {
            key: "SLIP_RT_DEF".into(),
            description: "sliprt".into(),
            route_priority: 25,
            ip_configuration: ipv4::Configuration::Router(Default::default()),
            stack: NetifStack::Slip,
            custom_mac: None,
        }
    }
}

static INITALIZED: mutex::Mutex<bool> = mutex::Mutex::wrap(mutex::RawMutex::new(), false);

fn initialize_netif_stack() -> Result<EspError, ()> {
    let mut guard = INITALIZED.lock();

    if !*guard {
        esp!(unsafe { esp_netif_init() })?;

        *guard = true;
    }

    Ok(())
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Status {
    None,
    DhcpAssigned,
    DhcpDeassigned,
}

#[derive(Debug)]
pub struct EspNetif(esp_netif_t);

impl EspNetif {
    pub fn new(stack: NetifStack) -> Result<Self, EspError> {
        Self::new_with_conf(&stack.default_configuration())
    }

    pub fn new_with_conf(conf: &NetifConfiguration) -> Result<Self, EspError> {
        initialize_netif_stack()?;

        let c_if_key = CString::new(conf.key.as_str()).unwrap();
        let c_if_description = CString::new(conf.description.as_str()).unwrap();

        let initial_mac = if let Some(custom_mac) = conf.custom_mac {
            custom_mac
        } else {
            stack.default_mac()?.unwrap_or([0; 6])
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
                            esp_netif_flags_ESP_NETIF_FLAG_AUTOUP
                        }
                    },
                    mac: initial_mac,
                    ip_info: ptr::null(),
                    get_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
                            if conf.stack == NetifStack::Sta {
                                ip_event_t_IP_EVENT_STA_GOT_IP
                            } else {
                                0
                            }
                        }
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    lost_ip_event: match ip_conf {
                        ipv4::ClientConfiguration::DHCP(_) => {
                            if conf.stack == NetifStack::Sta {
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

        let mut handle = Self(unsafe { esp_netif_new(&cfg).as_mut() }.unwrap());

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
                        handle.1,
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

    pub fn is_up(&self) -> bool {
        // TODO
        todo!()
    }

    pub fn get_ip_info(&self) -> Result<ipv4::IpInfo, EspError> {
        // TODO
        todo!()
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

    fn set_dns(&mut self, dns: ipv4::Ipv4Addr) {
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

    fn set_secondary_dns(&mut self, secondary_dns: ipv4::Ipv4Addr) {
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

    pub fn get_hostname(&self) -> Result<heapless::String<30>, EspError> {
        let mut ptr: *const c_types::c_char = core::ptr::null();
        esp!(unsafe { esp_netif_get_hostname(self.1, &mut ptr) })?;

        Ok(from_cstr_ptr(ptr).into())
    }

    fn set_hostname(&mut self, hostname: &str) -> Result<(), EspError> {
        if let Ok(hostname) = CString::new(hostname) {
            esp!(unsafe { esp_netif_set_hostname(self.1, hostname.as_ptr() as *const _) })?;
        } else {
            esp!(ESP_ERR_INVALID_ARG)?;
        }

        Ok(())
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
}

impl Drop for EspNetif {
    fn drop(&mut self) {
        unsafe { esp_netif_destroy(self.1) };
    }
}

impl RawHandle for EspNetif {
    type Handle = esp_netif_t;

    unsafe fn handle(&self) -> Handle {
        self.0
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
    pub fn is_for(&self, raw_handle: &impl RawHandle<netif_handle_t>) -> bool {
        self.handle()
            .map(|handle| handle == unsafe { raw_handle.handle() })
            .unwrap_or(false)
    }

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
                ip_settings: ipv4::IpInfo {
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

pub struct EspNetifStatus<B>
where
    B: Borrow<EspNetif>,
{
    netif: B,
    status: Waitable<Status>,
    _subscription: EspSubscription<System>,
}

impl<B> EspNetifStatus<B>
where
    B: Borrow<EspNetif>,
{
    pub fn wait_status(&self, matcher: impl Fn(&Status) -> bool) {
        info!("About to wait for status");

        self.waitable.wait_while(|status| !matcher(&status));

        info!("Waiting for status done - success");
    }

    pub fn wait_status_with_timeout(
        &self,
        dur: Duration,
        matcher: impl Fn(&Status) -> bool,
    ) -> Result<(), Status> {
        info!("About to wait {:?} for status", dur);

        let (timeout, status) = self.waitable.wait_timeout_while_and_get(
            dur,
            |status| !matcher(status),
            |status| status.clone(),
        );

        if !timeout {
            info!("Waiting for status done - success");
            Ok(())
        } else {
            info!("Timeout while waiting for status");
            Err(status)
        }
    }

    fn on_ip_event(
        handle: netif_handle_t,
        waitable: &Waitable<Status>,
        event: &IpEvent,
    ) -> Result<bool, EspError> {
        if event.handle() == handle as _ {
            info!("Got IP event: {:?}", event);

            let status = match event {
                IpEvent::DhcpIpAssigned(_) => Some(Status::DhcpAssigned),
                IpEvent::DhcpIpDeassigned(_) => Some(Status::DhcpDeassigned),
                _ => None,
            };

            if let Some(status) = status {
                let mut guard = waitable.state.lock();

                if *guard != status {
                    *guard = status;

                    info!("IP event {:?} handled, set status: {:?}", event, status);

                    return Ok(true);
                }
            }

            info!("IP event {:?} skipped", event);

            Ok(false)
        }
    }
}

impl ErrorType for EspNetifStatus {
    type Error = EspError;
}

impl EventBus<()> for EspNetifStatus {
    type Subscription = EspSubscription<System>;

    fn subscribe(
        &self,
        callback: impl for<'a> FnMut(&'a ()) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        let handle = self.handle;
        let waitable = self.waitable.clone();
        let cb = Arc::new(UnsafeCellSendSync(UnsafeCell::new(callback)));

        let subscription = self.sys_loop.subscribe(move |event: &IpEvent| {
            let notify = self.on_ip_event(handle, &waitable, event);

            if notify {
                let cb_ref = unsafe { cb.0.get().as_mut().unwrap() };

                (cb_ref)(&());
            }
        })?;

        Ok(subscription)
    }
}
