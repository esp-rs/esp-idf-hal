use core::{cmp, convert::TryInto, mem, ptr, time::Duration};

extern crate alloc;
use alloc::sync::Arc;
use alloc::vec;

use enumset::*;
use log::*;

use mutex_trait::Mutex;

use embedded_svc::ipv4;
use embedded_svc::mutex::Mutex as ESVCMutex;
use embedded_svc::wifi::*;

use esp_idf_sys::*;

use crate::netif::*;
use crate::nvs::EspDefaultNvs;
use crate::sysloop::*;

use crate::private::common::*;
use crate::private::cstr::*;

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
        let has_bssid = match &conf.bssid {
            Some(_) => true,
            None => false,
        };

        let bssid: [u8; 6] = match &conf.bssid {
            Some(bssid_ref) => *bssid_ref,
            None => [0; 6],
        };

        let mut result = wifi_sta_config_t {
            ssid: [0; 32],
            password: [0; 64],
            scan_method: wifi_scan_method_t_WIFI_FAST_SCAN,
            bssid_set: has_bssid,
            bssid: bssid,
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

impl From<&Newtype<wifi_sta_config_t>> for ClientConfiguration {
    fn from(conf: &Newtype<wifi_sta_config_t>) -> Self {
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

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

struct Shared {
    client_ip_conf: Option<ipv4::ClientConfiguration>,
    router_ip_conf: Option<ipv4::RouterConfiguration>,

    status: Status,
    operating: bool,
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            client_ip_conf: None,
            router_ip_conf: None,
            status: Status(ClientStatus::Stopped, ApStatus::Stopped),
            operating: false,
        }
    }
}

pub struct EspWifi {
    netif_stack: Arc<EspNetifStack>,
    _sys_loop_stack: Arc<EspSysLoopStack>,
    _nvs: Arc<EspDefaultNvs>,

    sta_netif: Option<EspNetif>,
    ap_netif: Option<EspNetif>,

    shared: Box<EspMutex<Shared>>,
}

impl EspWifi {
    pub fn new(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        nvs: Arc<EspDefaultNvs>,
    ) -> Result<EspWifi, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let wifi = Self::init(netif_stack, sys_loop_stack, nvs)?;

                    *taken = true;
                    Ok(wifi)
                }
            })
        }
    }

    fn init(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        nvs: Arc<EspDefaultNvs>,
    ) -> Result<EspWifi, EspError> {
        let mut wifi = EspWifi {
            netif_stack: netif_stack,
            _sys_loop_stack: sys_loop_stack,
            _nvs: nvs,
            sta_netif: None,
            ap_netif: None,
            shared: Box::new(EspMutex::new(Default::default())),
        };

        unsafe {
            let cfg = wifi_init_config_t {
                event_handler: Some(esp_event_send_internal),
                osi_funcs: &mut g_wifi_osi_funcs,
                wpa_crypto_funcs: g_wifi_default_wpa_crypto_funcs,
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
            esp!(esp_wifi_init(&cfg))?;

            info!("Driver initialized");

            let shared_ref: *mut _ = &mut *wifi.shared;

            esp!(esp_event_handler_register(
                WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(EspWifi::event_handler),
                shared_ref as *mut c_types::c_void
            ))?;
            esp!(esp_event_handler_register(
                IP_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(EspWifi::event_handler),
                shared_ref as *mut c_types::c_void
            ))?;

            info!("Event handlers registered");
        }

        info!("Initialization complete");

        Ok(wifi)
    }

    pub fn with_client_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        f(self.sta_netif.as_ref())
    }

    pub fn with_client_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        f(self.sta_netif.as_mut())
    }

    pub fn with_router_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        f(self.ap_netif.as_ref())
    }

    pub fn with_router_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        f(self.ap_netif.as_mut())
    }

    fn get_client_conf(&self) -> Result<ClientConfiguration, EspError> {
        let mut wifi_config = [0 as u8; mem::size_of::<wifi_config_t>()];
        let wifi_config_ref: &mut wifi_config_t = unsafe { mem::transmute(&mut wifi_config) };

        esp!(unsafe { esp_wifi_get_config(wifi_interface_t_WIFI_IF_STA, wifi_config_ref) })?;

        let mut result: ClientConfiguration = unsafe { (&Newtype(wifi_config_ref.sta)).into() };
        result.ip_conf = self
            .shared
            .with_lock(|shared| shared.client_ip_conf.clone());

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
        result.ip_conf = self
            .shared
            .with_lock(|shared| shared.router_ip_conf.clone());

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
        Self::netif_unbind(self.sta_netif.as_mut())?;

        if let Some(conf) = conf {
            let mut iconf = InterfaceConfiguration::wifi_default_client();
            iconf.ip_configuration = InterfaceIpConfiguration::Client(conf.clone());

            info!("Setting STA interface configuration: {:?}", iconf);

            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach_wifi_station(netif.1) })?;
            esp!(unsafe { esp_wifi_set_default_wifi_sta_handlers() })?;

            self.sta_netif = Some(netif);

            info!("STA IP configuration done");
        } else {
            self.sta_netif = None;

            info!("Skipping STA IP configuration (not configured)");
        }

        self.shared
            .with_lock(|shared| shared.client_ip_conf = conf.clone());

        Ok(())
    }

    fn set_router_ip_conf(
        &mut self,
        conf: &Option<ipv4::RouterConfiguration>,
    ) -> Result<(), EspError> {
        Self::netif_unbind(self.ap_netif.as_mut())?;

        if let Some(conf) = conf {
            let mut iconf = InterfaceConfiguration::wifi_default_router();
            iconf.ip_configuration = InterfaceIpConfiguration::Router(conf.clone());

            info!("Setting AP interface configuration: {:?}", iconf);

            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach_wifi_ap(netif.1) })?;
            esp!(unsafe { esp_wifi_set_default_wifi_ap_handlers() })?;

            self.ap_netif = Some(netif);

            info!("AP IP configuration done");
        } else {
            self.ap_netif = None;

            info!("Skipping AP IP configuration (not configured)");
        }

        self.shared
            .with_lock(|shared| shared.router_ip_conf = conf.clone());

        Ok(())
    }

    fn wait_status<F: Fn(&Status) -> bool>(&self, waiter: F) -> Status {
        info!("About to wait for status");

        let result = loop {
            let status = self.get_status();

            if waiter(&status) {
                break status;
            }

            // TODO: Replace with waiting on a condvar that wakes up when an event is received
            unsafe { vTaskDelay(100) };
        };

        info!("Waiting for status done - success");

        result
    }

    fn wait_status_with_timeout<F: Fn(&Status) -> bool>(
        &self,
        timeout: Duration,
        waiter: F,
    ) -> Result<(), Status> {
        info!("About to wait for status with timeout {:?}", timeout);

        let mut accum = Duration::from_millis(0);

        loop {
            let status = self.get_status();

            if waiter(&status) {
                info!("Waiting for status done - success");

                break Ok(());
            }

            if accum > timeout {
                info!("Timeout while waiting for status");

                break Err(status);
            }

            // TODO: Replace with waiting on a condvar that wakes up when an event is received
            unsafe { vTaskDelay(500) };
            accum += Duration::from_millis(500);
        }
    }

    fn start(&mut self, status: Status) -> Result<(), EspError> {
        info!("Starting with status: {:?}", status);

        self.shared.with_lock(|shared| {
            shared.status = status.clone();
            shared.operating = status.is_operating();
        });

        if status.is_operating() {
            info!("Status is of operating type, starting");

            esp!(unsafe { esp_wifi_start() })?;

            info!("Start requested");

            let result =
                self.wait_status_with_timeout(Duration::from_secs(10), |s| !s.is_transitional());

            if let Err(_) = result {
                info!("Timeout while waiting for the requested state");

                return Err(EspError::from(ESP_ERR_TIMEOUT as i32).unwrap());
            }

            info!("Started");

            Self::netif_info("STA", self.sta_netif.as_ref())?;
            Self::netif_info("AP", self.ap_netif.as_ref())?;
        } else {
            info!("Status is NOT of operating type, not starting");
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<(), EspError> {
        info!("Stopping");

        self.shared.with_lock(|shared| shared.operating = false);

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

        self.wait_status(|s| match s {
            Status(ClientStatus::Stopped, ApStatus::Stopped) => true,
            _ => false,
        });

        info!("Stopped");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop()?;

        unsafe {
            Self::netif_unbind(self.ap_netif.as_mut())?;
            Self::netif_unbind(self.sta_netif.as_mut())?;

            esp!(esp_event_handler_unregister(
                WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(EspWifi::event_handler)
            ))?;
            esp!(esp_event_handler_unregister(
                IP_EVENT,
                ESP_EVENT_ANY_ID as i32,
                Option::Some(EspWifi::event_handler)
            ))?;

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

        self.stop()?;

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

    unsafe extern "C" fn event_handler(
        arg: *mut c_types::c_void,
        event_base: esp_event_base_t,
        event_id: c_types::c_int,
        event_data: *mut c_types::c_void,
    ) {
        let shared_ref = (arg as *mut mutex::EspMutex<Shared>).as_mut().unwrap();

        shared_ref.with_lock(|shared| {
            if event_base == WIFI_EVENT {
                Self::on_wifi_event(shared, event_id, event_data)
            } else if event_base == IP_EVENT {
                Self::on_ip_event(shared, event_id, event_data)
            } else {
                warn!("Got unknown event base");

                Ok(())
            }
            .unwrap()
        });
    }

    #[allow(non_upper_case_globals)]
    unsafe fn on_wifi_event(
        shared: &mut Shared,
        event_id: c_types::c_int,
        event_data: *mut c_types::c_void,
    ) -> Result<(), EspError> {
        info!("Got wifi event: {} ", event_id);

        shared.status = Status(
            match event_id as u32 {
                wifi_event_t_WIFI_EVENT_STA_START => {
                    EspWifi::reconnect_if_operating(shared.operating)?
                }
                wifi_event_t_WIFI_EVENT_STA_STOP => ClientStatus::Stopped,
                wifi_event_t_WIFI_EVENT_STA_CONNECTED => ClientStatus::Started(
                    ClientConnectionStatus::Connected(match shared.client_ip_conf.as_ref() {
                        None => ClientIpStatus::Disabled,
                        Some(ipv4::ClientConfiguration::DHCP) => ClientIpStatus::Waiting,
                        Some(ipv4::ClientConfiguration::Fixed(ref status)) => {
                            ClientIpStatus::Done(status.clone())
                        }
                    }),
                ),
                wifi_event_t_WIFI_EVENT_STA_DISCONNECTED => {
                    EspWifi::reconnect_if_operating(shared.operating)?
                }
                _ => shared.status.0.clone(),
            },
            match event_id as u32 {
                wifi_event_t_WIFI_EVENT_AP_START => ApStatus::Started(ApIpStatus::Done),
                wifi_event_t_WIFI_EVENT_AP_STOP => ApStatus::Stopped,
                wifi_event_t_WIFI_EVENT_AP_STACONNECTED => {
                    let event: *const wifi_event_ap_staconnected_t = mem::transmute(event_data);
                    info!("Station {:?} AID={} connected", (*event).mac, (*event).aid);
                    shared.status.1.clone()
                }
                wifi_event_t_WIFI_EVENT_AP_STADISCONNECTED => {
                    let event: *const wifi_event_ap_stadisconnected_t = mem::transmute(event_data);
                    info!(
                        "Station {:?} AID={} disconnected",
                        (*event).mac,
                        (*event).aid
                    );
                    shared.status.1.clone()
                }
                _ => shared.status.1.clone(),
            },
        );

        info!("Set status: {:?}", shared.status);

        info!("Wifi event {} handled", event_id);

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    unsafe fn on_ip_event(
        shared: &mut Shared,
        event_id: c_types::c_int,
        event_data: *mut c_types::c_void,
    ) -> Result<(), EspError> {
        info!("Got IP event: {}", event_id);

        shared.status = Status(
            match event_id as u32 {
                ip_event_t_IP_EVENT_STA_GOT_IP => {
                    let event: *const ip_event_got_ip_t = mem::transmute(event_data);

                    ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(
                        ipv4::ClientSettings {
                            ip: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.ip)),
                            subnet: ipv4::Subnet {
                                gateway: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.gw)),
                                mask: Newtype((*event).ip_info.netmask).try_into()?,
                            },
                            dns: None,           // TODO
                            secondary_dns: None, // TODO
                        },
                    )))
                }
                ip_event_t_IP_EVENT_STA_LOST_IP => {
                    EspWifi::reconnect_if_operating(shared.operating)?
                }
                _ => shared.status.0.clone(),
            },
            match event_id as u32 {
                ip_event_t_IP_EVENT_AP_STAIPASSIGNED => {
                    let event: *const ip_event_ap_staipassigned_t = mem::transmute(event_data);
                    info!(
                        "Station got IP {}",
                        ipv4::Ipv4Addr::from(Newtype((*event).ip))
                    );
                    shared.status.1.clone()
                }
                _ => shared.status.1.clone(),
            },
        );

        info!("Set status: {:?}", shared.status);

        info!("IP event {} handled", event_id);

        Ok(())
    }

    unsafe fn reconnect_if_operating(operating: bool) -> Result<ClientStatus, EspError> {
        Ok(if operating {
            info!("Reconnecting");

            esp_nofail!(esp_wifi_connect());

            ClientStatus::Started(ClientConnectionStatus::Connecting)
        } else {
            ClientStatus::Started(ClientConnectionStatus::Disconnected)
        })
    }
}

impl Drop for EspWifi {
    fn drop(&mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                self.clear_all().unwrap();
                *taken = false;
            });
        }

        info!("Dropped");
    }
}

impl Wifi for EspWifi {
    type Error = EspError;

    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        let caps = Capability::Client | Capability::AccessPoint | Capability::Mixed;

        info!("Providing capabilities: {:?}", caps);

        Ok(caps)
    }

    fn get_status(&self) -> Status {
        let status = self.shared.with_lock(|shared| shared.status.clone());

        info!("Providing status: {:?}", status);

        status
    }

    #[allow(non_upper_case_globals)]
    fn scan_fill(&mut self, ap_infos: &mut [AccessPointInfo]) -> Result<usize, Self::Error> {
        let total_count = self.do_scan()?;

        if ap_infos.len() > 0 {
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
        unsafe { ap_infos_raw.set_len(total_count as usize) };

        let real_count = self.do_get_scan_infos(&mut ap_infos_raw)?;

        let mut result = vec::Vec::with_capacity(real_count);
        for i in 0..real_count {
            let ap_info: AccessPointInfo = Newtype(&ap_infos_raw[i]).into();
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

        self.stop()?;

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

        self.start(status)?;

        info!("Configuration set");

        Ok(())
    }
}
