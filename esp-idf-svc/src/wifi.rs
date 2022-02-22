use core::cell::UnsafeCell;
use core::cmp;
use core::ptr;
use core::time::Duration;

extern crate alloc;
use alloc::borrow::ToOwned;
use alloc::sync::Arc;
use alloc::vec;

use ::log::*;

use enumset::*;

use embedded_svc::errors::Errors;
use embedded_svc::event_bus::EventBus;
use embedded_svc::ipv4;
use embedded_svc::wifi::*;

use esp_idf_hal::mutex;

use esp_idf_sys::*;

use crate::eventloop::{EspSubscription, EspTypedEventDeserializer, EspTypedEventSource, System};
use crate::netif::*;
use crate::nvs::EspDefaultNvs;
use crate::private::common::*;
use crate::private::cstr::*;
use crate::private::waitable::*;
use crate::sysloop::*;

#[cfg(feature = "experimental")]
pub use nonblocking::*;

const MAX_AP: usize = 20;

impl From<AuthMethod> for Newtype<wifi_auth_mode_t> {
    fn from(method: AuthMethod) -> Self {
        Newtype(match method {
            AuthMethod::None => wifi_auth_mode_t_WIFI_AUTH_OPEN,
            AuthMethod::WEP => wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::WPA => wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::WPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WPAWPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
            AuthMethod::WPA2Enterprise => wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE,
            AuthMethod::WPA3Personal => wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
            AuthMethod::WPA2WPA3Personal => wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK,
            AuthMethod::WAPIPersonal => wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK,
        })
    }
}

impl From<Newtype<wifi_auth_mode_t>> for AuthMethod {
    #[allow(non_upper_case_globals)]
    fn from(mode: Newtype<wifi_auth_mode_t>) -> Self {
        match mode.0 {
            wifi_auth_mode_t_WIFI_AUTH_OPEN => AuthMethod::None,
            wifi_auth_mode_t_WIFI_AUTH_WEP => AuthMethod::WEP,
            wifi_auth_mode_t_WIFI_AUTH_WPA_PSK => AuthMethod::WPA,
            wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK => AuthMethod::WPA2Personal,
            wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => AuthMethod::WPAWPA2Personal,
            wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => AuthMethod::WPA2Enterprise,
            wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthMethod::WPA3Personal,
            wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => AuthMethod::WPA2WPA3Personal,
            wifi_auth_mode_t_WIFI_AUTH_WAPI_PSK => AuthMethod::WAPIPersonal,
            _ => panic!(),
        }
    }
}

impl From<&ClientConfiguration> for Newtype<wifi_sta_config_t> {
    fn from(conf: &ClientConfiguration) -> Self {
        let bssid: [u8; 6] = match &conf.bssid {
            Some(bssid_ref) => *bssid_ref,
            None => [0; 6],
        };

        let mut result = wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
            bssid_set: conf.bssid.is_some(),
            bssid,
            channel: conf.channel.unwrap_or(0u8),
            listen_interval: 0,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {
                rssi: 127,
                authmode: Newtype::<wifi_auth_mode_t>::from(conf.auth_method).0,
            },
            pmf_cfg: wifi_pmf_config_t {
                capable: false,
                required: false,
            },
            ..Default::default()
        };

        set_str(&mut result.ssid, conf.ssid.as_str());
        set_str(&mut result.password, conf.password.as_str());

        Newtype(result)
    }
}

impl From<Newtype<wifi_sta_config_t>> for ClientConfiguration {
    fn from(conf: Newtype<wifi_sta_config_t>) -> Self {
        ClientConfiguration {
            ssid: from_cstr(&conf.0.ssid).into(),
            bssid: if conf.0.bssid_set {
                Some(conf.0.bssid)
            } else {
                None
            },
            auth_method: Newtype(conf.0.threshold.authmode).into(),
            password: from_cstr(&conf.0.password).into(),
            channel: if conf.0.channel != 0 {
                Some(conf.0.channel)
            } else {
                None
            },
            ip_conf: None, // This must be set at a later stage
        }
    }
}

impl From<&AccessPointConfiguration> for Newtype<wifi_ap_config_t> {
    fn from(conf: &AccessPointConfiguration) -> Self {
        let mut result = wifi_ap_config_t {
            ssid: [0; 32],
            password: [0; 64],
            ssid_len: conf.ssid.len() as u8,
            channel: conf.channel,
            authmode: Newtype::<wifi_auth_mode_t>::from(conf.auth_method).0,
            ssid_hidden: if conf.ssid_hidden { 1 } else { 0 },
            max_connection: cmp::max(conf.max_connections, 16) as u8,
            beacon_interval: 100,
            ..Default::default()
        };

        set_str(&mut result.ssid, conf.ssid.as_str());
        set_str(&mut result.password, conf.password.as_str());

        Newtype(result)
    }
}

impl From<Newtype<wifi_ap_config_t>> for AccessPointConfiguration {
    fn from(conf: Newtype<wifi_ap_config_t>) -> Self {
        AccessPointConfiguration {
            ssid: if conf.0.ssid_len == 0 {
                from_cstr(&conf.0.ssid).into()
            } else {
                unsafe {
                    core::str::from_utf8_unchecked(&conf.0.ssid[0..conf.0.ssid_len as usize])
                        .to_owned()
                }
            },
            ssid_hidden: conf.0.ssid_hidden != 0,
            channel: conf.0.channel,
            secondary_channel: None,
            auth_method: AuthMethod::from(Newtype(conf.0.authmode)),
            protocols: EnumSet::<Protocol>::empty(), // TODO
            password: from_cstr(&conf.0.password).into(),
            max_connections: conf.0.max_connection as u16,
            ip_conf: None, // This must be set at a later stage
        }
    }
}

impl From<Newtype<&wifi_ap_record_t>> for AccessPointInfo {
    #[allow(non_upper_case_globals)]
    fn from(ap_info: Newtype<&wifi_ap_record_t>) -> Self {
        let a = ap_info.0;

        Self {
            ssid: from_cstr(&a.ssid).into(),
            bssid: a.bssid,
            channel: a.primary,
            secondary_channel: match a.second {
                wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => SecondaryChannel::None,
                wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => SecondaryChannel::Above,
                wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => SecondaryChannel::Below,
                _ => panic!(),
            },
            signal_strength: a.rssi as u8,
            protocols: EnumSet::<Protocol>::empty(), // TODO
            auth_method: AuthMethod::from(Newtype::<wifi_auth_mode_t>(a.authmode)),
        }
    }
}

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

struct Shared {
    client_ip_conf: Option<ipv4::ClientConfiguration>,
    router_ip_conf: Option<ipv4::RouterConfiguration>,

    status: Status,
    operating: bool,

    sta_netif: Option<EspNetif>,
    ap_netif: Option<EspNetif>,
}

impl Shared {
    fn is_our_sta_ip_event(&self, event: &IpEvent) -> bool {
        self.sta_netif.is_some()
            && self.sta_netif.as_ref().map(|netif| netif.1)
                == event.handle().map(|handle| handle as _)
    }

    fn is_our_ap_ip_event(&self, event: &IpEvent) -> bool {
        self.ap_netif.is_some()
            && self.ap_netif.as_ref().map(|netif| netif.1)
                == event.handle().map(|handle| handle as _)
    }
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            client_ip_conf: None,
            router_ip_conf: None,
            status: Status(ClientStatus::Stopped, ApStatus::Stopped),
            operating: false,
            sta_netif: None,
            ap_netif: None,
        }
    }
}

unsafe impl Send for Shared {}

pub struct EspWifi {
    netif_stack: Arc<EspNetifStack>,
    sys_loop_stack: Arc<EspSysLoopStack>,
    _nvs: Arc<EspDefaultNvs>,

    waitable: Arc<Waitable<Shared>>,

    _wifi_subscription: EspSubscription<System>,
    _ip_subscription: EspSubscription<System>,
}

impl EspWifi {
    pub fn new(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        nvs: Arc<EspDefaultNvs>,
    ) -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        let wifi = Self::init(netif_stack, sys_loop_stack, nvs)?;

        *taken = true;
        Ok(wifi)
    }

    fn init(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        nvs: Arc<EspDefaultNvs>,
    ) -> Result<Self, EspError> {
        let cfg = wifi_init_config_t {
            event_handler: Some(esp_event_send_internal),
            osi_funcs: unsafe { &mut g_wifi_osi_funcs },
            wpa_crypto_funcs: unsafe { g_wifi_default_wpa_crypto_funcs },
            static_rx_buf_num: 10,
            dynamic_rx_buf_num: 32,
            tx_buf_type: 1,
            static_tx_buf_num: 0,
            dynamic_tx_buf_num: 32,
            csi_enable: 0,
            ampdu_rx_enable: 1,
            ampdu_tx_enable: 1,
            nvs_enable: 0,
            nano_enable: 0,
            //tx_ba_win: 6,
            rx_ba_win: 6,
            wifi_task_core_id: 0,
            beacon_max_len: 752,
            mgmt_sbuf_num: 32,
            feature_caps: 1, // CONFIG_FEATURE_WPA3_SAE_BIT
            magic: 0x1F2F3F4F,
            ..Default::default()
        };
        esp!(unsafe { esp_wifi_init(&cfg) })?;

        info!("Driver initialized");

        let waitable: Arc<Waitable<Shared>> = Arc::new(Waitable::new(Default::default()));

        let wifi_waitable = waitable.clone();
        let wifi_subscription =
            sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |event: &WifiEvent| {
                    let mut shared = wifi_waitable.state.lock();

                    if Self::on_wifi_event(&mut shared, event).unwrap() {
                        wifi_waitable.cvar.notify_all();
                    }
                })?;

        let ip_waitable = waitable.clone();
        let ip_subscription =
            sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |event: &IpEvent| {
                    let mut shared = ip_waitable.state.lock();

                    if Self::on_ip_event(&mut shared, event).unwrap() {
                        ip_waitable.cvar.notify_all();
                    }
                })?;

        info!("Event handlers registered");

        let wifi = Self {
            netif_stack,
            sys_loop_stack,
            _nvs: nvs,
            waitable,
            _wifi_subscription: wifi_subscription,
            _ip_subscription: ip_subscription,
        };

        info!("Initialization complete");

        Ok(wifi)
    }

    pub fn with_client_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        self.waitable.get(|shared| f(shared.sta_netif.as_ref()))
    }

    pub fn with_client_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        self.waitable.get_mut(|shared| f(shared.sta_netif.as_mut()))
    }

    pub fn with_router_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        self.waitable.get(|shared| f(shared.ap_netif.as_ref()))
    }

    pub fn with_router_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        self.waitable.get_mut(|shared| f(shared.ap_netif.as_mut()))
    }

    fn get_client_conf(&self) -> Result<ClientConfiguration, EspError> {
        let mut wifi_config: wifi_config_t = Default::default();
        esp!(unsafe { esp_wifi_get_config(wifi_interface_t_WIFI_IF_STA, &mut wifi_config) })?;

        let mut result: ClientConfiguration = unsafe { Newtype(wifi_config.sta).into() };
        result.ip_conf = self.waitable.get(|shared| shared.client_ip_conf.clone());

        info!("Providing STA configuration: {:?}", &result);

        Ok(result)
    }

    fn set_client_conf(&mut self, conf: &ClientConfiguration) -> Result<(), EspError> {
        info!("Setting STA configuration: {:?}", conf);

        let mut wifi_config = wifi_config_t {
            sta: Newtype::<wifi_sta_config_t>::from(conf).0,
        };

        esp!(unsafe { esp_wifi_set_config(wifi_interface_t_WIFI_IF_STA, &mut wifi_config) })?;

        self.set_client_ip_conf(&conf.ip_conf)?;

        info!("STA configuration done");

        Ok(())
    }

    fn get_ap_conf(&self) -> Result<AccessPointConfiguration, EspError> {
        let mut wifi_config: wifi_config_t = Default::default();
        esp!(unsafe { esp_wifi_get_config(wifi_interface_t_WIFI_IF_AP, &mut wifi_config) })?;

        let mut result: AccessPointConfiguration = unsafe { Newtype(wifi_config.ap).into() };
        result.ip_conf = self.waitable.get(|shared| shared.router_ip_conf);

        info!("Providing AP configuration: {:?}", &result);

        Ok(result)
    }

    fn set_ap_conf(&mut self, conf: &AccessPointConfiguration) -> Result<(), EspError> {
        info!("Setting AP configuration: {:?}", conf);

        let mut wifi_config = wifi_config_t {
            ap: Newtype::<wifi_ap_config_t>::from(conf).0,
        };

        esp!(unsafe { esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut wifi_config) })?;
        self.set_router_ip_conf(&conf.ip_conf)?;

        info!("AP configuration done");

        Ok(())
    }

    fn set_client_ip_conf(
        &mut self,
        conf: &Option<ipv4::ClientConfiguration>,
    ) -> Result<(), EspError> {
        {
            let mut shared = self.waitable.state.lock();
            Self::netif_unbind(shared.sta_netif.as_mut())?;
        }

        let netif = if let Some(conf) = conf {
            let mut iconf = InterfaceConfiguration::wifi_default_client();
            iconf.ip_configuration = InterfaceIpConfiguration::Client(conf.clone());

            info!("Setting STA interface configuration: {:?}", iconf);

            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach_wifi_station(netif.1) })?;
            esp!(unsafe { esp_wifi_set_default_wifi_sta_handlers() })?;

            info!("STA IP configuration done");

            Some(netif)
        } else {
            info!("Skipping STA IP configuration (not configured)");

            None
        };

        {
            let mut shared = self.waitable.state.lock();
            shared.client_ip_conf = conf.clone();
            shared.sta_netif = netif;
        }

        Ok(())
    }

    fn set_router_ip_conf(
        &mut self,
        conf: &Option<ipv4::RouterConfiguration>,
    ) -> Result<(), EspError> {
        {
            let mut shared = self.waitable.state.lock();
            Self::netif_unbind(shared.ap_netif.as_mut())?;
        }

        let netif = if let Some(conf) = conf {
            let mut iconf = InterfaceConfiguration::wifi_default_router();
            iconf.ip_configuration = InterfaceIpConfiguration::Router(*conf);

            info!("Setting AP interface configuration: {:?}", iconf);

            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach_wifi_ap(netif.1) })?;
            esp!(unsafe { esp_wifi_set_default_wifi_ap_handlers() })?;

            info!("AP IP configuration done");

            Some(netif)
        } else {
            info!("Skipping AP IP configuration (not configured)");

            None
        };

        {
            let mut shared = self.waitable.state.lock();
            shared.router_ip_conf = *conf;
            shared.ap_netif = netif;
        }

        Ok(())
    }

    pub fn wait_status(&self, matcher: impl Fn(&Status) -> bool) {
        info!("About to wait for status");

        self.waitable.wait_while(|shared| !matcher(&shared.status));

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
            |shared| !matcher(&shared.status),
            |shared| shared.status.clone(),
        );

        if !timeout {
            info!("Waiting for status done - success");
            Ok(())
        } else {
            info!("Timeout while waiting for status");
            Err(status)
        }
    }

    fn start(&mut self, status: Status, wait: Option<Duration>) -> Result<(), EspError> {
        info!("Starting with status: {:?}", status);

        {
            let mut shared = self.waitable.state.lock();

            shared.status = status.clone();
            shared.operating = status.is_operating();

            if status.is_operating() {
                info!("Status is of operating type, starting");

                esp!(unsafe { esp_wifi_start() })?;

                info!("Start requested");

                Self::netif_info("STA", shared.sta_netif.as_ref())?;
                Self::netif_info("AP", shared.ap_netif.as_ref())?;
            } else {
                info!("Status is NOT of operating type, not starting");
            }
        }

        if let Some(duration) = wait {
            let result = self.wait_status_with_timeout(duration, |s| !s.is_transitional());

            if result.is_err() {
                info!("Timeout while waiting for the requested state");

                return Err(EspError::from(ESP_ERR_TIMEOUT as i32).unwrap());
            }

            info!("Started");
        }

        Ok(())
    }

    fn stop(&mut self, wait: bool) -> Result<(), EspError> {
        info!("Stopping");

        {
            let mut shared = self.waitable.state.lock();

            shared.operating = false;

            esp!(unsafe { esp_wifi_disconnect() }).or_else(|err| {
                if err.code() == esp_idf_sys::ESP_ERR_WIFI_NOT_STARTED as esp_err_t {
                    Ok(())
                } else {
                    Err(err)
                }
            })?;
            info!("Disconnect requested");

            esp!(unsafe { esp_wifi_stop() })?;
            info!("Stop requested");
        }

        if wait {
            self.wait_status(|s| matches!(s, Status(ClientStatus::Stopped, ApStatus::Stopped)));
        }

        info!("Stopped");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop(true)?;

        let mut shared = self.waitable.state.lock();

        unsafe {
            Self::netif_unbind(shared.ap_netif.as_mut())?;
            shared.ap_netif = None;

            Self::netif_unbind(shared.sta_netif.as_mut())?;
            shared.sta_netif = None;

            info!("Event handlers deregistered");

            esp!(esp_wifi_deinit())?;

            info!("Driver deinitialized");
        }

        info!("Deinitialization complete");

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    fn do_scan(&mut self) -> Result<usize, EspError> {
        info!("About to scan for access points");

        self.stop(true)?;

        unsafe {
            esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;
            esp!(esp_wifi_start())?;

            esp!(esp_wifi_scan_start(ptr::null_mut(), true))?;
        }

        let mut found_ap: u16 = 0;
        esp!(unsafe { esp_wifi_scan_get_ap_num(&mut found_ap as *mut _) })?;

        info!("Found {} access points", found_ap);

        Ok(found_ap as usize)
    }

    #[allow(non_upper_case_globals)]
    fn do_get_scan_infos(
        &mut self,
        ap_infos_raw: &mut [wifi_ap_record_t],
    ) -> Result<usize, EspError> {
        info!("About to get info for found access points");

        let mut ap_count: u16 = ap_infos_raw.len() as u16;

        esp!(unsafe { esp_wifi_scan_get_ap_records(&mut ap_count, ap_infos_raw.as_mut_ptr(),) })?;

        info!("Got info for {} access points", ap_count);

        Ok(ap_count as usize)
    }

    fn netif_unbind(netif: Option<&mut EspNetif>) -> Result<(), EspError> {
        if let Some(netif) = netif {
            esp!(unsafe {
                esp_wifi_clear_default_wifi_driver_and_handlers(netif.1 as *mut c_types::c_void)
            })?;
        }

        Ok(())
    }

    fn netif_info(name: &'static str, netif: Option<&EspNetif>) -> Result<(), EspError> {
        if let Some(netif) = netif {
            info!(
                "{} netif status: {:?}, index: {}, name: {}, ifkey: {}",
                name,
                netif,
                netif.get_index(),
                netif.get_name(),
                netif.get_key()
            );
        } else {
            info!("{} netif is not allocated", name);
        }

        Ok(())
    }

    fn on_wifi_event(shared: &mut Shared, event: &WifiEvent) -> Result<bool, EspError> {
        info!("Got wifi event: {:?}", event);

        let (client_status, ap_status) = match event {
            WifiEvent::StaStarted => (Some(Self::reconnect_if_operating(shared.operating)?), None),
            WifiEvent::StaStopped => (Some(ClientStatus::Stopped), None),
            WifiEvent::StaConnected => (
                Some(ClientStatus::Started(ClientConnectionStatus::Connected(
                    match shared.client_ip_conf.as_ref() {
                        None => ClientIpStatus::Disabled,
                        Some(ipv4::ClientConfiguration::DHCP(_)) => ClientIpStatus::Waiting,
                        Some(ipv4::ClientConfiguration::Fixed(ref status)) => {
                            ClientIpStatus::Done(*status)
                        }
                    },
                ))),
                None,
            ),
            WifiEvent::StaDisconnected => {
                (Some(Self::reconnect_if_operating(shared.operating)?), None)
            }
            WifiEvent::ApStarted => (None, Some(ApStatus::Started(ApIpStatus::Done))),
            WifiEvent::ApStopped => (None, Some(ApStatus::Stopped)),
            _ => (None, None),
        };

        if client_status.is_some() || ap_status.is_some() {
            if let Some(client_status) = client_status {
                shared.status.0 = client_status;
            }

            if let Some(ap_status) = ap_status {
                shared.status.1 = ap_status;
            }

            info!(
                "STA event {:?} handled, set status: {:?}",
                event, shared.status
            );

            Ok(true)
        } else {
            info!("STA event {:?} skipped", event);

            Ok(false)
        }
    }

    fn on_ip_event(shared: &mut Shared, event: &IpEvent) -> Result<bool, EspError> {
        if shared.is_our_sta_ip_event(event) {
            info!("Got STA IP event: {:?}", event);

            let status = match event {
                IpEvent::DhcpIpAssigned(assignment) => Some(ClientStatus::Started(
                    ClientConnectionStatus::Connected(ClientIpStatus::Done(assignment.ip_settings)),
                )),
                IpEvent::DhcpIpDeassigned(_) => Some(ClientStatus::Started(
                    ClientConnectionStatus::Connected(ClientIpStatus::Waiting),
                )),
                _ => None,
            };

            if let Some(status) = status {
                if matches!(event, IpEvent::DhcpIpDeassigned(_)) {
                    shared.status.0 = Self::reconnect_if_operating(shared.operating)?;
                } else {
                    shared.status.0 = status;
                }

                info!(
                    "STA IP event {:?} handled, set status: {:?}",
                    event, shared.status
                );

                Ok(true)
            } else {
                info!("STA IP event {:?} skipped", event);

                Ok(false)
            }
        } else if shared.is_our_ap_ip_event(event) {
            info!("Got AP IP event: {:?}", event);

            info!("AP IP event {:?} skipped", event);

            Ok(false)
        } else {
            Ok(false)
        }
    }

    fn reconnect_if_operating(operating: bool) -> Result<ClientStatus, EspError> {
        Ok(if operating {
            info!("Reconnecting");

            esp_nofail!(unsafe { esp_wifi_connect() });

            ClientStatus::Started(ClientConnectionStatus::Connecting)
        } else {
            ClientStatus::Started(ClientConnectionStatus::Disconnected)
        })
    }
}

impl Drop for EspWifi {
    fn drop(&mut self) {
        {
            let mut taken = TAKEN.lock();

            self.clear_all().unwrap();
            *taken = false;
        }

        info!("Dropped");
    }
}

impl Errors for EspWifi {
    type Error = EspError;
}

impl Wifi for EspWifi {
    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        let caps = Capability::Client | Capability::AccessPoint | Capability::Mixed;

        info!("Providing capabilities: {:?}", caps);

        Ok(caps)
    }

    fn get_status(&self) -> Status {
        let status = self.waitable.get(|shared| shared.status.clone());

        info!("Providing status: {:?}", status);

        status
    }

    #[allow(non_upper_case_globals)]
    fn scan_fill(&mut self, ap_infos: &mut [AccessPointInfo]) -> Result<usize, Self::Error> {
        let total_count = self.do_scan()?;

        if !ap_infos.is_empty() {
            let mut ap_infos_raw: [wifi_ap_record_t; MAX_AP] = Default::default();

            let real_count = self.do_get_scan_infos(&mut ap_infos_raw)?;

            for i in 0..real_count {
                if ap_infos.len() == i {
                    break;
                }

                let ap_info = Newtype(&ap_infos_raw[i]).into();
                info!("Found access point {:?}", ap_info);

                ap_infos[i] = ap_info;
            }
        }

        Ok(cmp::min(total_count, MAX_AP))
    }

    #[allow(non_upper_case_globals)]
    fn scan(&mut self) -> Result<vec::Vec<AccessPointInfo>, Self::Error> {
        let total_count = self.do_scan()?;

        let mut ap_infos_raw: vec::Vec<wifi_ap_record_t> =
            vec::Vec::with_capacity(total_count as usize);
        #[allow(clippy::uninit_vec)]
        // ... because we are filling it in on the next line and only reading the initialized members
        unsafe {
            ap_infos_raw.set_len(total_count as usize)
        };

        let real_count = self.do_get_scan_infos(&mut ap_infos_raw)?;

        let mut result = vec::Vec::with_capacity(real_count);
        for ap_info_raw in ap_infos_raw.iter().take(real_count) {
            let ap_info: AccessPointInfo = Newtype(ap_info_raw).into();
            info!("Found access point {:?}", ap_info);

            result.push(ap_info);
        }

        Ok(result)
    }

    #[allow(non_upper_case_globals)]
    fn get_configuration(&self) -> Result<Configuration, Self::Error> {
        info!("Getting configuration");

        unsafe {
            let mut mode: wifi_mode_t = 0;

            esp!(esp_wifi_get_mode(&mut mode))?;

            let conf = match mode {
                wifi_mode_t_WIFI_MODE_NULL => Configuration::None,
                wifi_mode_t_WIFI_MODE_AP => Configuration::AccessPoint(self.get_ap_conf()?),
                wifi_mode_t_WIFI_MODE_STA => Configuration::Client(self.get_client_conf()?),
                wifi_mode_t_WIFI_MODE_APSTA => {
                    Configuration::Mixed(self.get_client_conf()?, self.get_ap_conf()?)
                }
                _ => panic!(),
            };

            info!("Configuration gotten: {:?}", &conf);

            Ok(conf)
        }
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        info!("Setting configuration: {:?}", conf);

        self.stop(false)?;

        let status = unsafe {
            match conf {
                Configuration::None => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;
                    info!("Wifi mode NULL set");

                    Status(ClientStatus::Stopped, ApStatus::Stopped)
                }
                Configuration::AccessPoint(ap_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP))?;
                    info!("Wifi mode AP set");

                    self.set_ap_conf(ap_conf)?;
                    Status(ClientStatus::Stopped, ApStatus::Starting)
                }
                Configuration::Client(client_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;
                    info!("Wifi mode STA set");

                    self.set_client_conf(client_conf)?;
                    Status(ClientStatus::Starting, ApStatus::Stopped)
                }
                Configuration::Mixed(client_conf, ap_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_APSTA))?;
                    info!("Wifi mode APSTA set");

                    self.set_client_conf(client_conf)?;
                    self.set_ap_conf(ap_conf)?;
                    Status(ClientStatus::Starting, ApStatus::Starting)
                }
            }
        };

        self.start(status, None)?;

        info!("Configuration set");

        Ok(())
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum WifiEvent {
    Ready,

    ScanStarted,
    ScanDone,

    StaStarted,
    StaStopped,
    StaConnected,
    StaDisconnected,
    StaAuthmodeChanged,
    StaBssRssiLow,
    StaBeaconTimeout,
    StaWpsSuccess,
    StaWpsFailed,
    StaWpsTimeout,
    StaWpsPin,
    StaWpsPbcOverlap,

    ApStarted,
    ApStopped,
    ApStaConnected,
    ApStaDisconnected,
    ApProbeRequestReceived,

    FtmReport,
    ActionTxStatus,
    RocDone,
}

impl EspTypedEventSource for WifiEvent {
    fn source() -> *const c_types::c_char {
        unsafe { WIFI_EVENT }
    }
}

impl EspTypedEventDeserializer<WifiEvent> for WifiEvent {
    #[allow(non_upper_case_globals, non_snake_case)]
    fn deserialize<R>(
        data: &crate::eventloop::EspEventFetchData,
        f: &mut impl for<'a> FnMut(&'a WifiEvent) -> R,
    ) -> R {
        let event_id = data.event_id as u32;

        let event = if event_id == wifi_event_t_WIFI_EVENT_WIFI_READY {
            WifiEvent::Ready
        } else if event_id == wifi_event_t_WIFI_EVENT_SCAN_DONE {
            WifiEvent::ScanDone
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_START {
            WifiEvent::StaStarted
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_STOP {
            WifiEvent::StaStopped
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_CONNECTED {
            WifiEvent::StaConnected
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_DISCONNECTED {
            WifiEvent::StaDisconnected
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_AUTHMODE_CHANGE {
            WifiEvent::StaAuthmodeChanged
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_SUCCESS {
            WifiEvent::StaWpsSuccess
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_FAILED {
            WifiEvent::StaWpsFailed
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_TIMEOUT {
            WifiEvent::StaWpsTimeout
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_PIN {
            WifiEvent::StaWpsPin
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP {
            WifiEvent::StaWpsPbcOverlap
        } else if event_id == wifi_event_t_WIFI_EVENT_AP_START {
            WifiEvent::ApStarted
        } else if event_id == wifi_event_t_WIFI_EVENT_AP_STOP {
            WifiEvent::ApStopped
        } else if event_id == wifi_event_t_WIFI_EVENT_AP_STACONNECTED {
            WifiEvent::ApStaConnected
        } else if event_id == wifi_event_t_WIFI_EVENT_AP_STADISCONNECTED {
            WifiEvent::ApStaDisconnected
        } else if event_id == wifi_event_t_WIFI_EVENT_AP_PROBEREQRECVED {
            WifiEvent::ApProbeRequestReceived
        } else if event_id == wifi_event_t_WIFI_EVENT_FTM_REPORT {
            WifiEvent::FtmReport
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_BSS_RSSI_LOW {
            WifiEvent::StaBssRssiLow
        } else if event_id == wifi_event_t_WIFI_EVENT_ACTION_TX_STATUS {
            WifiEvent::ActionTxStatus
        } else if event_id == wifi_event_t_WIFI_EVENT_STA_BEACON_TIMEOUT {
            WifiEvent::StaBeaconTimeout
        } else {
            panic!("Unknown event ID: {}", event_id);
        };

        f(&event)
    }
}

impl EventBus<()> for EspWifi {
    type Subscription = (EspSubscription<System>, EspSubscription<System>);

    fn subscribe(
        &mut self,
        callback: impl for<'a> FnMut(&'a ()) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        let wifi_cb = Arc::new(UnsafeCellSendSync(UnsafeCell::new(callback)));
        let wifi_last_status = Arc::new(UnsafeCellSendSync(UnsafeCell::new(self.get_status())));
        let wifi_waitable = self.waitable.clone();

        let ip_cb = wifi_cb.clone();
        let ip_last_status = wifi_last_status.clone();
        let ip_waitable = wifi_waitable.clone();

        let subscription1 =
            self.sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |_event: &WifiEvent| {
                    let notify = {
                        let shared = wifi_waitable.state.lock();

                        let last_status_ref = unsafe { wifi_last_status.0.get().as_mut().unwrap() };

                        if *last_status_ref != shared.status {
                            *last_status_ref = shared.status.clone();

                            true
                        } else {
                            false
                        }
                    };

                    if notify {
                        let cb_ref = unsafe { wifi_cb.0.get().as_mut().unwrap() };

                        (cb_ref)(&());
                    }
                })?;

        let subscription2 =
            self.sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |event: &IpEvent| {
                    let notify = {
                        let shared = ip_waitable.state.lock();

                        if shared.is_our_sta_ip_event(event) || shared.is_our_ap_ip_event(event) {
                            let last_status_ref =
                                unsafe { ip_last_status.0.get().as_mut().unwrap() };

                            if *last_status_ref != shared.status {
                                *last_status_ref = shared.status.clone();

                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    };

                    if notify {
                        let cb_ref = unsafe { ip_cb.0.get().as_mut().unwrap() };

                        (cb_ref)(&());
                    }
                })?;

        Ok((subscription1, subscription2))
    }
}

#[cfg(feature = "experimental")]
mod nonblocking {
    use embedded_svc::utils::nonblocking::{event_bus::AsyncEventBus, Asyncify};

    use esp_idf_hal::mutex::Condvar;

    impl Asyncify for super::EspWifi {
        type AsyncWrapper<S> = AsyncEventBus<(), Condvar, S>;
    }
}
