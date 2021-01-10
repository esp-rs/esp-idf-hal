use std::{cmp, collections, mem, ptr, sync::Arc, sync::Mutex, sync::RwLock, thread, time::{Duration, Instant}, vec};
use std::ffi::CStr;

use anyhow::*;

use embedded_svc::wifi::*;
use embedded_svc::ipv4;
use esp_idf_sys::*;

use crate::common::*;
use crate::netif::EspNetif;
use crate::sysloop::EspSysLoop;
use crate::nvs::EspDefaultNvs;

impl From<AuthMethod> for Newtype<wifi_auth_mode_t> {
    fn from(method: AuthMethod) -> Self {
        Newtype(match method {
            AuthMethod::None => wifi_auth_mode_t_WIFI_AUTH_OPEN,
            AuthMethod::WEP => wifi_auth_mode_t_WIFI_AUTH_WEP,
            AuthMethod::WPA => wifi_auth_mode_t_WIFI_AUTH_WPA_PSK,
            AuthMethod::WPA2Personal => wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            AuthMethod::WPA3Personal => wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
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
            wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK => AuthMethod::WPA3Personal,
            _ => panic!()
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
            channel: 0,
            listen_interval: 0,
            sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
            threshold: wifi_scan_threshold_t {rssi: 127, authmode: Newtype::<wifi_auth_mode_t>::from(conf.auth_method).0},
            pmf_cfg: wifi_pmf_config_t {capable: false, required: false},
        };

        set_str(&mut result.ssid, conf.ssid.as_str());
        set_str(&mut result.password, conf.password.as_str());

        Newtype(result)
    }
}

impl From<&Newtype<wifi_sta_config_t>> for ClientConfiguration {
    fn from(conf: &Newtype<wifi_sta_config_t>) -> Self {
        ClientConfiguration {
            ssid: from_cstr(&conf.0.ssid),
            bssid: if conf.0.bssid_set {Some(conf.0.bssid)} else {None},
            auth_method: Newtype(conf.0.threshold.authmode).into(),
            password: from_cstr(&conf.0.password),
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
            ssid_hidden: if conf.ssid_hidden {1} else {0},
            max_connection: cmp::max(conf.max_connections, 16) as u8,
            beacon_interval: 100,
        };

        set_str(&mut result.ssid, conf.ssid.as_str());
        set_str(&mut result.password, conf.password.as_str());

        Newtype(result)
    }
}

impl From<Newtype<wifi_ap_config_t>> for AccessPointConfiguration {
    fn from(conf: Newtype<wifi_ap_config_t>) -> Self {
        AccessPointConfiguration {
            ssid: from_cstr(&conf.0.ssid),
            ssid_hidden: conf.0.ssid_hidden != 0,
            channel: conf.0.channel,
            secondary_channel: None,
            auth_method: AuthMethod::from(Newtype(conf.0.authmode)),
            protocols: collections::HashSet::new(), // TODO
            password: from_cstr(&conf.0.password),
            max_connections: conf.0.max_connection as u16,
            ip_conf: None, // This must be set at a later stage
        }
    }
}

lazy_static! {
    static ref TAKEN: Arc<Mutex<bool>> = Arc::new(Mutex::new(false));
}

pub struct EspWifi {
    _netif: Arc<EspNetif>,
    _sys_loop: Arc<EspSysLoop>,
    _nvs: Arc<EspDefaultNvs>,

    sta_netif: *mut esp_netif_t,
    ap_netif: *mut esp_netif_t,

    client_ip_conf: Option<ipv4::ClientConfiguration>,
    router_ip_conf: Option<ipv4::RouterConfiguration>,

    status: RwLock<(Status, bool)>,
}

impl EspWifi {
    pub fn new(netif: Arc<EspNetif>, sys_loop: Arc<EspSysLoop>, nvs: Arc<EspDefaultNvs>) -> Result<EspWifi> {
        let mut taken = TAKEN.lock().unwrap();
        if *taken {
            bail!("Wifi driver is already owned elsewhere");
        }

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
                tx_ba_win: 6,
                rx_ba_win: 6,
                wifi_task_core_id: 0,
                beacon_max_len: 752,
                mgmt_sbuf_num: 32,
                feature_caps: 1, // CONFIG_FEATURE_WPA3_SAE_BIT
                magic: 0x1F2F3F4F,
            };
            esp!(esp_wifi_init(&cfg))?;

            esp!(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Option::Some(EspWifi::event_handler), std::ptr::null_mut()))?;
            esp!(esp_event_handler_register(IP_EVENT, ip_event_t_IP_EVENT_STA_GOT_IP as i32, Option::Some(EspWifi::event_handler), std::ptr::null_mut()))?;
        }

        *taken = true;

        Ok(EspWifi {
            _netif: netif,
            _sys_loop: sys_loop,
            _nvs: nvs,
            sta_netif: ptr::null_mut(),
            ap_netif: ptr::null_mut(),
            client_ip_conf: None,
            router_ip_conf: None,
            status: RwLock::new((Status(ClientStatus::Stopped, ApStatus::Stopped), false)),
        })
    }

    pub unsafe fn get_client_netif(&self) -> *mut esp_netif_t {
        self.sta_netif
    }

    pub unsafe fn get_ap_netif(&self) -> *mut esp_netif_t {
        self.ap_netif
    }

    fn get_client_conf(&self) -> Result<ClientConfiguration> {
        let mut wifi_config = [0 as u8; mem::size_of::<wifi_config_t>()];
        let wifi_config_ref: &mut wifi_config_t = unsafe {mem::transmute(&mut wifi_config)};

        esp!(unsafe {esp_wifi_get_config(esp_interface_t_ESP_IF_WIFI_STA, wifi_config_ref)})?;

        let mut result: ClientConfiguration = unsafe {(&Newtype(wifi_config_ref.sta)).into()};
        result.ip_conf = self.client_ip_conf;

        Ok(result)
    }

    fn set_client_conf(&mut self, conf: &ClientConfiguration) -> Result<()> {
        esp!(unsafe {esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA)})?;

        let mut wifi_config = wifi_config_t {
            sta: Newtype::<wifi_sta_config_t>::from(conf).0
        };

        esp!(unsafe {esp_wifi_set_config(esp_interface_t_ESP_IF_WIFI_STA, &mut wifi_config)})?;

        self.set_client_ip_conf(&conf.ip_conf)?;

        Ok(())
    }

    fn get_ap_conf(&self) -> Result<AccessPointConfiguration> {
        let mut wifi_config = [0 as u8; mem::size_of::<wifi_config_t>()];
        let wifi_config_ref: &mut wifi_config_t = unsafe {mem::transmute(&mut wifi_config)};

        esp!(unsafe {esp_wifi_get_config(esp_interface_t_ESP_IF_WIFI_AP, wifi_config_ref)})?;

        let mut result: AccessPointConfiguration = unsafe {Newtype(wifi_config_ref.ap).into()};
        result.ip_conf = self.router_ip_conf;

        Ok(result)
    }

    fn set_ap_conf(&mut self, conf: &AccessPointConfiguration) -> Result<()> {
        esp!(unsafe {esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP)})?;

        let mut wifi_config = wifi_config_t {
            ap: Newtype::<wifi_ap_config_t>::from(conf).0
        };

        esp!(unsafe{esp_wifi_set_config(esp_interface_t_ESP_IF_WIFI_AP, &mut wifi_config)})?;

        self.set_router_ip_conf(&conf.ip_conf)?;

        Ok(())
    }

    fn set_client_ip_conf(&mut self, conf: &Option<ipv4::ClientConfiguration>) -> Result<()> {
        unsafe {
            EspWifi::clear_ip_conf(&mut self.sta_netif)?;

            if let Some(client_conf) = conf {
                let ip_cfg  = esp_netif_inherent_config_t {
                    flags: match client_conf {
                        ipv4::ClientConfiguration::DHCP => esp_netif_flags_ESP_NETIF_DHCP_CLIENT | esp_netif_flags_ESP_NETIF_FLAG_GARP | esp_netif_flags_ESP_NETIF_FLAG_EVENT_IP_MODIFIED,
                        ipv4::ClientConfiguration::Fixed(_) => esp_netif_flags_ESP_NETIF_FLAG_AUTOUP,
                    },
                    mac: [0; 6],
                    ip_info: match client_conf {
                        ipv4::ClientConfiguration::DHCP => ptr::null_mut(),
                        ipv4::ClientConfiguration::Fixed(ref fixed_conf) => &mut esp_netif_ip_info_t {
                            ip: Newtype::<esp_ip4_addr_t>::from(fixed_conf.ip).0,
                            netmask: esp_ip4_addr_t {
                                addr: u32::MAX << (32 - fixed_conf.subnet.mask),
                            },
                            gw: Newtype::<esp_ip4_addr_t>::from(fixed_conf.subnet.gateway).0,
                        },
                    },
                    get_ip_event: match client_conf {
                        ipv4::ClientConfiguration::DHCP => ip_event_t_IP_EVENT_STA_GOT_IP,
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    lost_ip_event: match client_conf {
                        ipv4::ClientConfiguration::DHCP => ip_event_t_IP_EVENT_STA_LOST_IP,
                        ipv4::ClientConfiguration::Fixed(_) => 0,
                    },
                    if_key: CStr::from_ptr("WIFI_STA_DEF\0".as_ptr() as *const c_types::c_char).as_ptr(),
                    if_desc: CStr::from_ptr("sta".as_ptr() as *const c_types::c_char).as_ptr(),
                    route_prio: 100,
                };

                let cfg: esp_netif_config_t = esp_netif_config_t {
                    base: &ip_cfg,
                    driver: ptr::null(),
                    stack: _g_esp_netif_netstack_default_wifi_sta,
                };

                self.sta_netif = esp_netif_new(&cfg);
                //assert(netif);

                esp!(esp_netif_attach_wifi_station(self.sta_netif))?;
                esp!(esp_wifi_set_default_wifi_sta_handlers())?;
            }
        }

        self.client_ip_conf = conf.clone();

        Ok(())
    }

    fn set_router_ip_conf(&mut self, conf: &Option<ipv4::RouterConfiguration>) -> Result<()> {
        unsafe {
            EspWifi::clear_ip_conf(&mut self.ap_netif)?;

            if let Some(router_conf) = conf {
                let ip_cfg  = esp_netif_inherent_config_t {
                    flags: (if router_conf.dhcp_enabled {esp_netif_flags_ESP_NETIF_DHCP_SERVER} else {0}) | esp_netif_flags_ESP_NETIF_FLAG_AUTOUP,
                    mac: [0; 6],
                    ip_info: &mut esp_netif_ip_info_t {
                        ip: Newtype::<esp_ip4_addr_t>::from(router_conf.subnet.gateway).0,
                        netmask: esp_ip4_addr_t {
                            addr: u32::MAX << (32 - router_conf.subnet.mask),
                        },
                        gw: Newtype::<esp_ip4_addr_t>::from(router_conf.subnet.gateway).0,
                    },
                    get_ip_event: 0,
                    lost_ip_event: 0,
                    if_key: CStr::from_ptr("WIFI_AP_DEF\0".as_ptr() as *const c_types::c_char).as_ptr(),
                    if_desc: CStr::from_ptr("ap".as_ptr() as *const c_types::c_char).as_ptr(),
                    route_prio: 10,
                };

                let cfg: esp_netif_config_t = esp_netif_config_t {
                    base: &ip_cfg,
                    driver: ptr::null(),
                    stack: _g_esp_netif_netstack_default_wifi_sta,
                };

                self.ap_netif = esp_netif_new(&cfg);
                //assert(netif);

                esp!(esp_netif_attach_wifi_ap(self.ap_netif))?;
                esp!(esp_wifi_set_default_wifi_ap_handlers())?;
            }
        }

        self.router_ip_conf = conf.clone();

        Ok(())
    }

    fn wait_status<F: Fn(&Status) -> bool>(&self, waiter: F) -> Status {
        loop {
            let status = self.get_status();

            if waiter(&status) {
                break status
            }

            // TODO: Replace with waiting on a condvar that wakes up when an event is received
            thread::sleep(Duration::from_millis(100));
        }
    }

    fn wait_status_with_timeout<F: Fn(&Status) -> bool>(&self, timeout: Duration, waiter: F) -> Result<(), Status> {
        let start = Instant::now();

        loop {
            let status = self.get_status();

            if waiter(&status) {
                break Ok(())
            }

            if Instant::now() > start + timeout {
                break Err(status)
            }

            // TODO: Replace with waiting on a condvar that wakes up when an event is received
            thread::sleep(Duration::from_millis(100));
        }
    }

    fn start(&mut self, status: Status) -> Result<()> {
        {
            *self.status.write().unwrap() = (status.clone(), status.is_operating());
        }

        if status.is_operating() {
            esp!(unsafe {esp_wifi_start()})?;

            let result = self.wait_status_with_timeout(Duration::from_secs(10), |s| !s.is_transitional());

            if let Err(status) = result {
                bail!("Timeout waiting in transition {:?}", status);
            }
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<()> {
        {
            self.status.write().unwrap().1 = false;
        }

        esp!(unsafe {esp_wifi_disconnect()})?; // TODO
        esp!(unsafe {esp_wifi_stop()})?;

        self.wait_status(|s| match s {
            Status(ClientStatus::Stopped, ApStatus::Stopped) => true,
            _ => false
        });

        Ok(())
    }

    fn clear_all(&mut self) -> Result<()> {
        self.stop()?;

        unsafe {
            EspWifi::clear_ip_conf(&mut self.ap_netif)?;
            EspWifi::clear_ip_conf(&mut self.sta_netif)?;

            esp!(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, Option::Some(EspWifi::event_handler)))?;
            esp!(esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID as i32, Option::Some(EspWifi::event_handler)))?;

            esp!(esp_wifi_deinit())?;
        }

        Ok(())
    }

    unsafe fn clear_ip_conf(netif: &mut *mut esp_netif_t) -> Result<()> {
        if !(*netif).is_null() {
            esp!(esp_wifi_clear_default_wifi_driver_and_handlers(*netif as *mut c_types::c_void))?;
            esp_netif_destroy(*netif);

            *netif = ptr::null_mut();
        }

        Ok(())
    }

    unsafe extern "C" fn event_handler(arg: *mut c_types::c_void, event_base: esp_event_base_t, event_id: c_types::c_int, event_data: *mut c_types::c_void) {
        let slf: &mut EspWifi = mem::transmute(arg);

        if event_base == WIFI_EVENT {
            slf.on_wifi_event(event_id, event_data)
        } else if event_base == IP_EVENT {
            slf.on_ip_event(event_id, event_data)
        } else {
            Ok(())
        }.unwrap();
    }

    #[allow(non_upper_case_globals)]
    unsafe fn on_wifi_event(&mut self, event_id: c_types::c_int, _event_data: *mut c_types::c_void) -> Result<()> {
        let mut status_guard = self.status.write().unwrap();

        status_guard.0 = Status(
            match event_id as u32 {
                wifi_event_t_WIFI_EVENT_STA_START => EspWifi::reconnect_if_operating(status_guard.1)?,
                wifi_event_t_WIFI_EVENT_STA_STOP => ClientStatus::Stopped,
                wifi_event_t_WIFI_EVENT_STA_CONNECTED => ClientStatus::Started(ClientConnectionStatus::Connected(match &self.client_ip_conf.as_ref() {
                    None => ClientIpStatus::Disabled,
                    Some(ipv4::ClientConfiguration::DHCP) => ClientIpStatus::Waiting,
                    Some(ipv4::ClientConfiguration::Fixed(ref status)) => ClientIpStatus::Done(*status),
                })),
                wifi_event_t_WIFI_EVENT_STA_DISCONNECTED => EspWifi::reconnect_if_operating(status_guard.1)?,
                _ => status_guard.0.0.clone(),
            },
            match event_id as u32 {
                wifi_event_t_WIFI_EVENT_AP_START => ApStatus::Started(ApIpStatus::Waiting), // TODO
                wifi_event_t_WIFI_EVENT_AP_STOP => ApStatus::Stopped,
                _ => status_guard.0.1.clone(),
            });

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    unsafe fn on_ip_event(&mut self, event_id: c_types::c_int, event_data: *mut c_types::c_void) -> Result<()> {
        let mut status_guard = self.status.write().unwrap();

        status_guard.0 = Status(
            match event_id as u32 {
                ip_event_t_IP_EVENT_STA_GOT_IP => {
                    let event: *const ip_event_got_ip_t = std::mem::transmute(event_data);

                    ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(ipv4::ClientSettings {
                        ip: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.ip)),
                        subnet: ipv4::Subnet {
                            gateway: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.gw)),
                            mask: (*event).ip_info.netmask.addr.leading_ones() as u8,
                        },
                        dns: None, // TODO
                        secondary_dns: None, // TODO
                    })))
                }
                ip_event_t_IP_EVENT_STA_LOST_IP => EspWifi::reconnect_if_operating(status_guard.1)?,
                _ => status_guard.0.0.clone(),
            },
            status_guard.0.1.clone());

        Ok(())
    }

    unsafe fn reconnect_if_operating(operating: bool) -> Result<ClientStatus> {
        Ok(if operating {
            esp_nofail!(esp_wifi_connect());

            ClientStatus::Started(ClientConnectionStatus::Connecting)
        } else {
            ClientStatus::Started(ClientConnectionStatus::Disconnected)
        })
    }
}

impl Drop for EspWifi {
    fn drop(&mut self) {
        self.clear_all().unwrap();

        *TAKEN.lock().unwrap() = false;
    }
}

impl Wifi for EspWifi {
    fn get_supported_operation_modes(&self) -> Result<collections::HashSet<OperationMode>> {
        Ok(vec! [
                OperationMode::Client,
                OperationMode::AccessPoint,
                OperationMode::Mixed]
            .into_iter()
            .collect())
    }

    fn get_status(&self) -> Status {
        self.status.read().unwrap().0.clone()
    }

    #[allow(non_upper_case_globals)]
    fn scan(&mut self) -> Result<vec::Vec<AccessPointInfo>> {
        let conf = self.get_configuration()?;

        self.stop()?;

        // defer! {
        //     self.set_configuration(&conf);
        // }

        unsafe {
            esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;
            esp!(esp_wifi_start())?;

            // let scan_conf = wifi_scan_config_t {
            //     ssid: ptr::null_mut(),
            //     bssid: ptr::null_mut(),
            //     channel: 0,
            //     show_hidden: true,
            //     scan_type: wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE,
            //     scan_time: wifi_scan_time_t {
            //         active: wifi_active_scan_time_t {
            //             min: 0,
            //             max: 0,
            //         },
            //         passive: 0,
            //     },
            // };
            esp!(esp_wifi_scan_start(ptr::null_mut()/*&scan_conf*/, true))?;
        }

        const MAX_AP: usize = 16;

        let mut ap_info_raw = [0 as u8; std::mem::size_of::<wifi_ap_record_t>() * MAX_AP];
        let mut ap_count: u16 = 0;

        esp!(unsafe {esp_wifi_scan_get_ap_records(&mut ap_count, ap_info_raw.as_mut_ptr() as *mut wifi_ap_record_t)})?;

        // let mut ap_records: Vec<wifi_ap_record_t> = std::vec! [
        //     wifi_ap_record_t {
        //         bssid: [0; 6],
        //         ssid: [0; 33],
        //         primary: 0,
        //         second: 0,
        //         rssi: 0,
        //         authmode: 0,
        //         pairwise_cipher: 0,
        //         group_cipher: 0,
        //         ant: 0,
        //         _bitfield_1: wifi_ap_record_t::new_bitfield_1(0, 0, 0, 0, 0, 0),
        //         country: wifi_country_t {
        //         },
        //     },
        //     ap_num];
        let ap_info: &[wifi_ap_record_t; MAX_AP] = unsafe {mem::transmute(&ap_info_raw)};

        let result = (0..ap_count as usize)
            .map(|i| ap_info[i])
            .map(|a| AccessPointInfo {
                ssid: from_cstr(&a.ssid),
                bssid: a.bssid,
                channel: a.primary,
                secondary_channel: match a.second {
                    wifi_second_chan_t_WIFI_SECOND_CHAN_NONE => SecondaryChannel::None,
                    wifi_second_chan_t_WIFI_SECOND_CHAN_ABOVE => SecondaryChannel::Above,
                    wifi_second_chan_t_WIFI_SECOND_CHAN_BELOW => SecondaryChannel::Below,
                    _ => panic!()
                },
                signal_strength: a.rssi as u8,
                protocols: collections::HashSet::new(), // TODO
                auth_method: AuthMethod::from(Newtype::<wifi_auth_mode_t>(a.authmode)),
            })
            .collect();

        self.set_configuration(&conf)?;

        Ok(result)
    }

    #[allow(non_upper_case_globals)]
    fn get_configuration(&self) -> Result<Configuration> {
        unsafe {
            let mut mode: wifi_mode_t = 0;

            esp!(esp_wifi_get_mode(&mut mode))?;

            Ok(match mode {
                wifi_mode_t_WIFI_MODE_NULL => Configuration::None,
                wifi_mode_t_WIFI_MODE_AP => Configuration::AccessPoint(self.get_ap_conf()?),
                wifi_mode_t_WIFI_MODE_STA => Configuration::Client(self.get_client_conf()?),
                wifi_mode_t_WIFI_MODE_APSTA => Configuration::Mixed(self.get_client_conf()?, self.get_ap_conf()?),
                _ => panic!()
            })
        }
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<()> {
        self.stop()?;

        let status = unsafe {
            match conf {
                Configuration::None => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;
                    Status(ClientStatus::Stopped, ApStatus::Stopped)
                },
                Configuration::AccessPoint(ap_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP))?;
                    self.set_ap_conf(ap_conf)?;
                    Status(ClientStatus::Stopped, ApStatus::Starting)
                },
                Configuration::Client(client_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;
                    self.set_client_conf(client_conf)?;
                    Status(ClientStatus::Starting, ApStatus::Stopped)
                },
                Configuration::Mixed(client_conf, ap_conf) => {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_APSTA))?;
                    self.set_client_conf(client_conf)?;
                    self.set_ap_conf(ap_conf)?;
                    Status(ClientStatus::Starting, ApStatus::Starting)
                },
            }
        };

        self.start(status)
    }
}
