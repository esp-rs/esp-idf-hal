use core::cell::UnsafeCell;
use core::cmp;
use core::ptr;
use core::time::Duration;

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;
use esp_idf_hal::peripheral::PeripheralRef;

use enumset::*;

use embedded_svc::event_bus::{ErrorType, EventBus};
use embedded_svc::ipv4;
use embedded_svc::wifi::*;

use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_hal::peripheral::{Peripheral, PeripheralRef};

use esp_idf_sys::*;

use crate::eventloop::{
    EspSubscription, EspSystemEventLoop, EspTypedEventDeserializer, EspTypedEventSource, System,
};
use crate::netif::*;
use crate::nvs::EspDefaultNvs;
use crate::nvs::EspNvsPartition;
use crate::private::common::*;
use crate::private::cstr::*;
use crate::private::waitable::*;

#[cfg(esp_idf_comp_esp_netif_enabled)]
use crate::netif::*;

#[cfg(all(feature = "nightly", feature = "experimental"))]
pub use asyncify::*;

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

        set_str(&mut result.ssid, conf.ssid.as_ref());
        set_str(&mut result.password, conf.password.as_ref());

        Newtype(result)
    }
}

impl From<Newtype<wifi_sta_config_t>> for ClientConfiguration {
    fn from(conf: Newtype<wifi_sta_config_t>) -> Self {
        Self {
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
            ssid_hidden: u8::from(conf.ssid_hidden),
            max_connection: cmp::max(conf.max_connections, 16) as u8,
            beacon_interval: 100,
            ..Default::default()
        };

        set_str(&mut result.ssid, conf.ssid.as_ref());
        set_str(&mut result.password, conf.password.as_ref());

        Newtype(result)
    }
}

impl From<Newtype<wifi_ap_config_t>> for AccessPointConfiguration {
    fn from(conf: Newtype<wifi_ap_config_t>) -> Self {
        Self {
            ssid: if conf.0.ssid_len == 0 {
                from_cstr(&conf.0.ssid).into()
            } else {
                unsafe {
                    core::str::from_utf8_unchecked(&conf.0.ssid[0..conf.0.ssid_len as usize]).into()
                }
            },
            ssid_hidden: conf.0.ssid_hidden != 0,
            channel: conf.0.channel,
            secondary_channel: None,
            auth_method: AuthMethod::from(Newtype(conf.0.authmode)),
            protocols: EnumSet::<Protocol>::empty(), // TODO
            password: from_cstr(&conf.0.password).into(),
            max_connections: conf.0.max_connection as u16,
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

pub struct WifiDriver<'d, M: WifiModemPeripheral> {
    _modem: PeripheralRef<'d, M>,
    _sysloop: EspSystemEventLoop,
    #[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
    _nvs: Option<EspDefaultNvsPartition>,
}

impl<'d, M: WifiModemPeripheral> WifiDriver<'d, M> {
    #[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
    pub fn new(
        modem: PeripheralRef<'a, M>,
        sysloop: EspSystemEventLoop,
        nvs: Option<EspDefaultNvsPartition>,
    ) -> Result<Self, EspError> {
        crate::into_ref!(modem);

        Self::init(nvs.is_sone())?;

        Ok(Self {
            _modem: modem,
            _sysloop: sysloop,
            _nvs: nvs,
        })
    }

    #[cfg(not(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled)))]
    pub fn new(modem: PeripheralRef<'a, M>, sysloop: EspSystemEventLoop) -> Result<Self, EspError> {
        crate::into_ref!(modem);

        Self::init(false)?;

        Ok(Self {
            _modem: modem,
            _sysloop: sysloop,
        })
    }

    #[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
    fn init(nvs_enabled: bool) -> Result<(), EspError> {
        #[allow(clippy::needless_update)]
        let cfg = wifi_init_config_t {
            #[cfg(esp_idf_version_major = "4")]
            event_handler: Some(esp_event_send_internal),
            osi_funcs: unsafe { &mut g_wifi_osi_funcs },
            wpa_crypto_funcs: unsafe { g_wifi_default_wpa_crypto_funcs },
            static_rx_buf_num: CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM as _,
            dynamic_rx_buf_num: CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM as _,
            tx_buf_type: CONFIG_ESP32_WIFI_TX_BUFFER_TYPE as _,
            static_tx_buf_num: WIFI_STATIC_TX_BUFFER_NUM as _,
            dynamic_tx_buf_num: WIFI_DYNAMIC_TX_BUFFER_NUM as _,
            cache_tx_buf_num: WIFI_CACHE_TX_BUFFER_NUM as _,
            csi_enable: WIFI_CSI_ENABLED as _,
            ampdu_rx_enable: WIFI_AMPDU_RX_ENABLED as _,
            ampdu_tx_enable: WIFI_AMPDU_TX_ENABLED as _,
            amsdu_tx_enable: WIFI_AMSDU_TX_ENABLED as _,
            nvs_enable: if nvs_enabled { 1 } else { 0 },
            nano_enable: WIFI_NANO_FORMAT_ENABLED as _,
            //tx_ba_win: WIFI_DEFAULT_TX_BA_WIN as _,
            rx_ba_win: WIFI_DEFAULT_RX_BA_WIN as _,
            wifi_task_core_id: WIFI_TASK_CORE_ID as _,
            beacon_max_len: WIFI_SOFTAP_BEACON_MAX_LEN as _,
            mgmt_sbuf_num: WIFI_MGMT_SBUF_NUM as _,
            feature_caps: unsafe { g_wifi_feature_caps },
            sta_disconnected_pm: WIFI_STA_DISCONNECTED_PM_ENABLED != 0,
            magic: WIFI_INIT_CONFIG_MAGIC as _,
            ..Default::default()
        };
        esp!(unsafe { esp_wifi_init(&cfg) })?;

        info!("Driver initialized");

        Ok(())
    }

    pub fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        let caps = Capability::Client | Capability::AccessPoint | Capability::Mixed;

        info!("Providing capabilities: {:?}", caps);

        Ok(caps)
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        info!("Start requested");

        esp!(unsafe { esp_wifi_start() })?;

        info!("Starting");

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), EspError> {
        info!("Stop requested");

        let _ = esp!(unsafe { esp_wifi_disconnect() });

        esp!(unsafe { esp_wifi_stop() })?;

        info!("Stopping");

        Ok(())
    }

    pub fn connect(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_wifi_connect() })?;

        Ok(())
    }

    pub fn disconnect(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_wifi_disconnect() })?;

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    pub fn get_configuration(&self) -> Result<Configuration, Self::Error> {
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

    pub fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        info!("Setting configuration: {:?}", conf);

        let _ = self.disconnect();
        let _ = self.stop();

        match conf {
            Configuration::None => {
                unsafe {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL))?;
                }
                info!("Wifi mode NULL set");

                Status(ClientStatus::Stopped, ApStatus::Stopped)
            }
            Configuration::AccessPoint(ap_conf) => {
                unsafe {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP))?;
                }
                info!("Wifi mode AP set");

                self.set_ap_conf(ap_conf)?;
                Status(ClientStatus::Stopped, ApStatus::Starting)
            }
            Configuration::Client(client_conf) => {
                unsafe {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA))?;
                }
                info!("Wifi mode STA set");

                self.set_client_conf(client_conf)?;
                Status(ClientStatus::Starting, ApStatus::Stopped)
            }
            Configuration::Mixed(client_conf, ap_conf) => {
                unsafe {
                    esp!(esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_APSTA))?;
                }
                info!("Wifi mode APSTA set");

                self.set_client_conf(client_conf)?;
                self.set_ap_conf(ap_conf)?;
                Status(ClientStatus::Starting, ApStatus::Starting)
            }
        }

        info!("Configuration set");

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    pub fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        let total_count = self.do_scan()?;

        let mut ap_infos_raw: heapless::Vec<wifi_ap_record_t, N> = heapless::Vec::new();

        let real_count = self.do_get_scan_infos(&mut ap_infos_raw)?;

        let mut result = heapless::Vec::<_, N>::new();
        for ap_info_raw in ap_infos_raw.iter().take(real_count) {
            let ap_info: AccessPointInfo = Newtype(ap_info_raw).into();
            info!("Found access point {:?}", ap_info);

            if result.push(ap_info).is_err() {
                break;
            }
        }

        Ok((result, total_count))
    }

    #[allow(non_upper_case_globals)]
    pub fn scan(&mut self) -> Result<alloc::vec::Vec<AccessPointInfo>, Self::Error> {
        let total_count = self.do_scan()?;

        let mut ap_infos_raw: alloc::vec::Vec<wifi_ap_record_t> =
            alloc::vec::Vec::with_capacity(total_count as usize);

        #[allow(clippy::uninit_vec)]
        // ... because we are filling it in on the next line and only reading the initialized members
        unsafe {
            ap_infos_raw.set_len(total_count as usize)
        };

        let real_count = self.do_get_scan_infos(&mut ap_infos_raw)?;

        let mut result = alloc::vec::Vec::with_capacity(real_count);
        for ap_info_raw in ap_infos_raw.iter().take(real_count) {
            let ap_info: AccessPointInfo = Newtype(ap_info_raw).into();
            info!("Found access point {:?}", ap_info);

            result.push(ap_info);
        }

        Ok(result)
    }

    fn get_sta_conf(&self) -> Result<ClientConfiguration, EspError> {
        let mut wifi_config: wifi_config_t = Default::default();
        esp!(unsafe { esp_wifi_get_config(wifi_interface_t_WIFI_IF_STA, &mut wifi_config) })?;

        let result: ClientConfiguration = unsafe { Newtype(wifi_config.sta).into() };

        info!("Providing STA configuration: {:?}", &result);

        Ok(result)
    }

    fn set_sta_conf(&mut self, conf: &ClientConfiguration) -> Result<(), EspError> {
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

        let result: AccessPointConfiguration = unsafe { Newtype(wifi_config.ap).into() };

        info!("Providing AP configuration: {:?}", &result);

        Ok(result)
    }

    fn set_ap_conf(&mut self, conf: &AccessPointConfiguration) -> Result<(), EspError> {
        info!("Setting AP configuration: {:?}", conf);

        let mut wifi_config = wifi_config_t {
            ap: Newtype::<wifi_ap_config_t>::from(conf).0,
        };

        esp!(unsafe { esp_wifi_set_config(wifi_interface_t_WIFI_IF_AP, &mut wifi_config) })?;

        info!("AP configuration done");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        let _ = self.stop();

        unsafe {
            esp!(esp_wifi_deinit())?;
        }

        info!("Driver deinitialized");

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
}

unsafe impl<'d, M: WifiModemPeripheral> Send for WifiDriver<d, M> {}

impl<'d, M: WifiModemPeripheral> Drop for WifiDriver<'d, M> {
    fn drop(&mut self) {
        self.clear_all()?;

        info!("Dropped");
    }
}

impl<'d, M> Wifi for WifiDriver<'d, M>
where
    M: WifiModemPeripheral,
{
    type Error = EspError;

    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        WifiDriver::get_capabilities(self)
    }

    fn get_configuration(&self) -> Result<Configuration, Self::Error> {
        WifiDriver::get_configuration(self)
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        WifiDriver::set_configuration(self, conf)
    }

    fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        WifiDriver::scan_n(self)
    }

    fn scan(&mut self) -> Result<alloc::vec::Vec<AccessPointInfo>, Self::Error> {
        WifiDriver::scan(self)
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
pub struct EspWifi<'d, M>
where
    M: WifiModemPeripheral,
{
    driver: EthDriver<'d, M>,
    sta_netif: EspNetif,
    ap_netif: EspNetif,
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, M> EspWifi<'d, M>
where
    M: WifiModemPeripheral,
{
    pub fn new(driver: B) -> Result<Self, EspError> {
        Self::wrap(
            driver,
            EspNetif::new(&NetifStack::Sta.default_configuration()),
            EspNetif::new(&NetifStack::Ap.default_configuration()),
        )
    }

    pub fn wrap(driver: B, sta_netif: EspNetif, ap_netif: EspNetif) -> Result<Self, EspError> {
        let glue_handle = unsafe { esp_eth_new_netif_glue(driver.handle()) };

        let this = Self {
            driver,
            sta_netif,
            ap_netif,
        };

        esp!(unsafe { esp_netif_attach_wifi_ap(ap_netif.handle()) })?;
        esp!(unsafe { esp_wifi_set_default_wifi_ap_handlers() })?;

        esp!(unsafe { esp_netif_attach_wifi_station(sta_netif.handle()) })?;
        esp!(unsafe { esp_wifi_set_default_wifi_sta_handlers() })?;

        Ok(this)
    }

    pub fn driver(&self) -> &EthDriver<'d, P> {
        &self.driver
    }

    pub fn driver_mut(&mut self) -> &mut EspNetif {
        &mut self.netif
    }

    pub fn sta_netif(&self) -> &EspNetif {
        &self.sta_netif
    }

    pub fn sta_netif_mut(&mut self) -> &mut EspNetif {
        &mut self.sta_netif
    }

    pub fn ap_netif(&self) -> &EspNetif {
        &self.ap_netif
    }

    pub fn ap_netif_mut(&mut self) -> &mut EspNetif {
        &mut self.ap_netif
    }

    pub fn get_capabilities(&self) -> Result<EnumSet<Capability>, EspError> {
        self.driver().get_capabilities()
    }

    pub fn get_configuration(&self) -> Result<Configuration, EspError> {
        self.driver().get_configuration()
    }

    pub fn set_configuration(&mut self, conf: &Configuration) -> Result<(), EspError> {
        self.driver_mut().set_configuration(conf)
    }

    pub fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), EspError> {
        self.driver_mut().scan_n()
    }

    pub fn scan(&mut self) -> Result<alloc::vec::Vec<AccessPointInfo>, EspError> {
        self.driver_mut().scan()
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, M> Drop for EspWifi<'d, M>
where
    M: WifiModemPeripheral,
{
    fn drop(&mut self) {
        esp!(unsafe {
            esp_wifi_clear_default_wifi_driver_and_handlers(
                self.ap_netif.handle() as *mut c_types::c_void
            )
        })
        .unwrap();

        esp!(unsafe {
            esp_wifi_clear_default_wifi_driver_and_handlers(
                self.sta_netif.handle() as *mut c_types::c_void
            )
        })?
        .unwrap();
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, M> Wifi for EspWifi<'d, M>
where
    M: WifiModemPeripheral,
{
    type Error = EspError;

    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        EspWifi::get_capabilities(self)
    }

    fn get_configuration(&self) -> Result<Configuration, Self::Error> {
        EspWifi::get_configuration(self)
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        EspWifi::set_configuration(self, conf)
    }

    fn scan_n<const N: usize>(
        &mut self,
    ) -> Result<(heapless::Vec<AccessPointInfo, N>, usize), Self::Error> {
        EspWifi::scan_n(self)
    }

    fn scan(&mut self) -> Result<alloc::vec::Vec<AccessPointInfo>, Self::Error> {
        EspWifi::scan(self)
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

pub struct WifiDriverStaStatus<'d, B, M: WifiModemPeripheral>
where
    B: Borrow<WifiDriver<'d, M>>,
{
    driver: B,
    waitable: Arc<Waitable<Status>>,

    _subscription: EspSubscription<System>,
}

impl<'d, B, M: WifiModemPeripheral> WifiDriverStaStatus<'d, B, M>
where
    B: Borrow<WifiDriver<'d, M>>,
{
    pub fn new(driver: B) -> Result<Self, EspError> {
        let waitable: Arc<Waitable<Shared>> = Arc::new(Waitable::new(Status::Unknown));

        let wifi_waitable = waitable.clone();
        let wifi_subscription = sys_loop.subscribe(move |event: &WifiEvent| {
            let mut status = wifi_waitable.state.lock();

            if Self::on_wifi_event(&mut status, event).unwrap() {
                wifi_waitable.cvar.notify_all();
            }
        })?;
    }

    fn on_wifi_event(shared: &mut Shared, event: &WifiEvent) -> Result<bool, EspError> {
        info!("Got wifi event: {:?}", event);

        let status = match event {
            WifiEvent::StaStarted => Some(Status::Started),
            WifiEvent::StaStopped => Some(Status::Stopped),
            WifiEvent::StaConnected => Some(Status::Connected),
            WifiEvent::StaDisconnected => Some(Status::Started),
            _ => None,
        };

        if let Some(status) = status {
            if *current_status != status {
                *current_status = status;

                info!("STA event {:?} handled, set status: {:?}", event, status);

                return Ok(true);
            }
        }

        info!("STA event {:?} skipped", event);

        Ok(false)
    }
}

pub struct WifiDriverApStatus<'d, B, M: WifiModemPeripheral>
where
    B: Borrow<WifiDriver<'d, M>>,
{
    driver: B,
    waitable: Arc<Waitable<Status>>,

    _subscription: EspSubscription<System>,
}

impl<'d, B, M: WifiModemPeripheral> WifiDriverApStatus<'d, B, M>
where
    B: Borrow<WifiDriver<'d, M>>,
{
    pub fn new(driver: B) -> Result<Self, EspError> {
        let waitable: Arc<Waitable<Shared>> = Arc::new(Waitable::new(Status::Unknown));

        let wifi_waitable = waitable.clone();
        let wifi_subscription = sys_loop.subscribe(move |event: &WifiEvent| {
            let mut status = wifi_waitable.state.lock();

            if Self::on_wifi_event(&mut status, event).unwrap() {
                wifi_waitable.cvar.notify_all();
            }
        })?;
    }

    fn on_wifi_event(current_status: &mut Status, event: &WifiEvent) -> Result<bool, EspError> {
        info!("Got wifi event: {:?}", event);

        let status = match event {
            WifiEvent::ApStarted => Some(ApStatus::Started),
            WifiEvent::ApStopped => Some(ApStatus::Stopped),
            _ => None,
        };

        if let Some(status) = status {
            if *current_status != status {
                *current_status = status;

                info!("AP event {:?} handled, set status: {:?}", event, status);

                return Ok(true);
            }
        }

        info!("AP event {:?} skipped", event);

        Ok(false)
    }
}

impl<'a> ErrorType for WifiDriver<'a> {
    type Error = EspError;
}

impl<'b> EventBus<()> for WifiDriver<'b> {
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
            self.sys_loop
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
            self.sys_loop
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

#[cfg(all(feature = "nightly", feature = "experimental"))]
mod asyncify {
    use embedded_svc::utils::asyncify::{event_bus::AsyncEventBus, Asyncify};

    use crate::private::mutex::RawCondvar;

    impl<'a> Asyncify for super::WifiDriver<'a> {
        type AsyncWrapper<S> = AsyncEventBus<(), RawCondvar, S>;
    }
}
