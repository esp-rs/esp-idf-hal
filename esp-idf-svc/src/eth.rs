use core::{convert::TryInto, mem, ptr, time::Duration};

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use enumset::*;

use mutex_trait::Mutex;

use embedded_svc::eth::*;
use embedded_svc::ipv4;
use embedded_svc::mutex::Mutex as ESVCMutex;

use esp_idf_sys::*;

#[cfg(any(
    all(esp32, esp_idf_eth_use_esp32_emac),
    all(
        esp_idf_eth_spi_ethernet_dm9051,
        esp_idf_eth_spi_ethernet_w5500,
        esp_idf_eth_spi_ethernet_ksz8851snl
    )
))]
use esp_idf_hal::gpio;

#[cfg(all(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
use esp_idf_hal::spi;

use crate::netif::*;
use crate::sysloop::*;

use crate::private::common::*;

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
// TODO: #[derive(Debug)]
pub struct Pins<MDC, MDIO> {
    pub rmii_rdx0: gpio::Gpio25<gpio::Output>,
    pub rmii_rdx1: gpio::Gpio26<gpio::Output>,
    pub rmii_crs_dv: gpio::Gpio27<gpio::Output>,
    pub rmii_mdc: MDC,
    pub rmii_txd1: gpio::Gpio22<gpio::Output>,
    pub rmii_tx_en: gpio::Gpio21<gpio::Output>,
    pub rmii_txd0: gpio::Gpio19<gpio::Output>,
    pub rmii_mdio: MDIO,
    pub rmii_ref_clk: gpio::Gpio0<gpio::Output>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
// TODO: #[derive(Debug)]
pub enum Esp32EthHw<MDC, MDIO> {
    IP101(Pins<MDC, MDIO>),
    RTL8201(Pins<MDC, MDIO>),
    LAN87XX(Pins<MDC, MDIO>),
    DP83848(Pins<MDC, MDIO>),
    KSZ8041(Pins<MDC, MDIO>),
    #[cfg(all(esp_idf_version_major = "4", esp_idf_version_minor = "4"))]
    KSZ8081(Pins<MDC, MDIO>),
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<MDC, MDIO> Esp32EthHw<MDC, MDIO>
where
    MDC: gpio::Pin,
    MDIO: gpio::Pin,
{
    fn new_mac(&self) -> *mut esp_eth_mac_t {
        core::ptr::null_mut() //unsafe { esp_eth_mac_new_esp32(&EspEth::eth_mac_default_config()) }
    }

    fn new_phy(&self) -> *mut esp_eth_phy_t {
        let phy_cfg = EspEth::<Pins<MDC, MDIO>>::eth_phy_default_config();

        match self {
            Self::IP101(_) => unsafe { esp_eth_phy_new_ip101(&phy_cfg) },
            Self::RTL8201(_) => unsafe { esp_eth_phy_new_rtl8201(&phy_cfg) },
            Self::LAN87XX(_) => unsafe { esp_eth_phy_new_lan8720(&phy_cfg) },
            Self::DP83848(_) => unsafe { esp_eth_phy_new_dp83848(&phy_cfg) },
            Self::KSZ8041(_) => unsafe { esp_eth_phy_new_ksz8041(&phy_cfg) },
            #[cfg(all(esp_idf_version_major = "4", esp_idf_version_minor = "4"))]
            Self::KSZ8081(_) => unsafe { esp_eth_phy_new_ksz8081(&phy_cfg) },
        }
    }

    fn into_hw(self) -> Pins<MDC, MDIO> {
        match self {
            Self::IP101(pins) => pins,
            Self::RTL8201(pins) => pins,
            Self::LAN87XX(pins) => pins,
            Self::DP83848(pins) => pins,
            Self::KSZ8041(pins) => pins,
            #[cfg(all(esp_idf_version_major = "4", esp_idf_version_minor = "4"))]
            Self::KSZ8081(pins) => pins,
        }
    }
}

#[cfg(all(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
#[derive(Debug)]
pub enum SpiEthHw<S, P> {
    #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
    DM9051(S, P),
    #[cfg(esp_idf_eth_spi_ethernet_w5500)]
    W5500(S, P),
    #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
    KSZ8851SNL(S, P),
}

#[cfg(all(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<S, P> SpiEthHw<S, P>
where
    S: spi::Spi,
    P: gpio::Pin,
{
    fn new_mac(&self) -> *mut esp_eth_mac_t {
        let mac_cfg = unsafe { EspEth::eth_mac_default_config() };

        match self {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            Self::DM9051(_, _) => unsafe { esp_eth_mac_new_dm9051(&mac_cfg) },
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            Self::W5500(_, _) => unsafe { esp_eth_mac_new_w5500(&mac_cfg) },
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            Self::KSZ8851SNL(_, _) => unsafe { esp_eth_mac_new_ksz8851snl(&mac_cfg) },
            _ => unreachable!(),
        }
    }

    fn new_phy(&self) -> *mut esp_eth_phy_t {
        let phy_cfg = EspEth::eth_phy_default_config();

        match self {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            Self::DM9051(_, _) => unsafe { esp_eth_phy_new_dm9051(&phy_cfg) },
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            Self::W5500(_, _) => unsafe { esp_eth_phy_new_w5500(&phy_cfg) },
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            Self::KSZ8851SNL(_, _) => unsafe { esp_eth_phy_new_ksz8851snl(&phy_cfg) },
            _ => unreachable!(),
        }
    }

    fn into_hw(self) -> (S, P) {
        match self {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            Self::DM9051(s, p) => (s, p),
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            Self::W5500(s, p) => (s, p),
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            Self::KSZ8851SNL(s, p) => (s, p),
            _ => unreachable!(),
        }
    }
}

#[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

struct Shared {
    conf: Configuration,

    status: Status,
    operating: bool,
}

impl Default for Shared {
    fn default() -> Self {
        Self {
            conf: Configuration::None,
            status: Status::Stopped,
            operating: false,
        }
    }
}

pub struct EspEth<IO> {
    netif_stack: Arc<EspNetifStack>,
    _sys_loop_stack: Arc<EspSysLoopStack>,

    #[allow(dead_code)]
    io: IO,

    handle: esp_eth_handle_t,
    glue_handle: *mut c_types::c_void,

    netif: Option<EspNetif>,

    shared: Box<EspMutex<Shared>>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<MDC, MDIO> EspEth<Pins<MDC, MDIO>>
where
    MDC: gpio::Pin,
    MDIO: gpio::Pin,
{
    pub fn new(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        hw: Esp32EthHw<MDC, MDIO>,
    ) -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let eth = Self::init(
                        netif_stack,
                        sys_loop_stack,
                        hw.new_mac(),
                        hw.new_phy(),
                        hw.into_hw(),
                    )?;

                    *taken = true;
                    Ok(eth)
                }
            })
        }
    }

    pub fn release(mut self) -> Pins<MDC, MDIO> {
        unsafe {
            TAKEN.lock(|taken| {
                self.clear_all().unwrap();
                *taken = false;
            });
        }

        info!("Released");

        self.io
    }
}

#[cfg(esp_idf_eth_use_openeth)]
impl EspEth<()> {
    pub fn new_openeth(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
    ) -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let mac = esp_eth_mac_new_openeth(&Self::eth_mac_default_config());
                    let phy = esp_eth_phy_new_dp83848(&Self::eth_phy_default_config());

                    let eth = Self::init(netif_stack, sys_loop_stack, mac, phy, ())?;

                    *taken = true;
                    Ok(eth)
                }
            })
        }
    }

    pub fn release(mut self) {
        unsafe {
            TAKEN.lock(|taken| {
                self.clear_all().unwrap();
                *taken = false;
            });
        }

        info!("Released");
    }
}

#[cfg(all(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<S, P> EspEth<(S, P)>
where
    S: spi::Spi,
    P: gpio::Pin,
{
    pub fn new_spi(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        hw: SpiEthHw<S, P>,
    ) -> Result<Self, EspError> {
        Ok(Self::init(
            netif_stack,
            sys_loop_stack,
            hw.new_mac(),
            hw.new_phy(),
            hw.into_hw(),
        )?)
    }

    pub fn release(mut self) -> (S, P) {
        unsafe {
            self.clear_all().unwrap();
        }

        info!("Released");

        self.io
    }
}

impl<IO> EspEth<IO> {
    fn init(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        mac: *mut esp_eth_mac_t,
        phy: *mut esp_eth_phy_t,
        io: IO,
    ) -> Result<Self, EspError> {
        let cfg = Self::eth_default_config(mac, phy);

        let mut handle: esp_eth_handle_t = ptr::null_mut();
        esp!(unsafe { esp_eth_driver_install(&cfg, &mut handle) })?;

        info!("Driver initialized");

        let glue_handle = unsafe { esp_eth_new_netif_glue(handle) };

        let mut shared: Box<EspMutex<Shared>> = Box::new(EspMutex::new(Default::default()));
        let shared_ref: *mut _ = &mut *shared;

        esp!(unsafe {
            esp_event_handler_register(
                ETH_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(Self::event_handler),
                shared_ref as *mut c_types::c_void,
            )
        })?;
        esp!(unsafe {
            esp_event_handler_register(
                IP_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(Self::event_handler),
                shared_ref as *mut c_types::c_void,
            )
        })?;

        info!("Event handlers registered");

        let eth = Self {
            netif_stack,
            _sys_loop_stack: sys_loop_stack,
            io,
            handle,
            glue_handle,
            netif: None,
            shared,
        };

        info!("Initialization complete");

        Ok(eth)
    }

    pub fn with_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        f(self.netif.as_ref())
    }

    pub fn with_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        f(self.netif.as_mut())
    }

    fn set_ip_conf(&mut self, conf: &Configuration) -> Result<(), EspError> {
        Self::netif_unbind(self.netif.as_mut())?;

        let iconf = match conf {
            Configuration::Client(conf) => {
                let mut iconf = InterfaceConfiguration::eth_default_client();
                iconf.ip_configuration = InterfaceIpConfiguration::Client(conf.clone());

                info!("Setting client interface configuration: {:?}", iconf);

                Some(iconf)
            }
            Configuration::Router(conf) => {
                let mut iconf = InterfaceConfiguration::eth_default_router();
                iconf.ip_configuration = InterfaceIpConfiguration::Router(conf.clone());

                info!("Setting router interface configuration: {:?}", iconf);

                Some(iconf)
            }
            _ => None,
        };

        if let Some(iconf) = iconf {
            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach(netif.1, self.glue_handle) })?;

            self.netif = Some(netif);

            info!("IP configuration done");
        } else {
            self.netif = None;

            info!("Skipping IP configuration (not configured)");
        }

        self.shared.with_lock(|shared| shared.conf = conf.clone());

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

            esp!(unsafe { esp_eth_start(self.handle) })?;

            info!("Start requested");

            let result =
                self.wait_status_with_timeout(Duration::from_secs(10), |s| !s.is_transitional());

            if result.is_err() {
                info!("Timeout while waiting for the requested state");

                return Err(EspError::from(ESP_ERR_TIMEOUT as i32).unwrap());
            }

            info!("Started");

            Self::netif_info("ETH", self.netif.as_ref())?;
        } else {
            info!("Status is NOT of operating type, not starting");
        }

        Ok(())
    }

    fn stop(&mut self) -> Result<(), EspError> {
        info!("Stopping");

        self.shared.with_lock(|shared| shared.operating = false);

        esp!(unsafe { esp_eth_stop(self.handle) })?;
        info!("Stop requested");

        self.wait_status(|s| matches!(s, Status::Stopped));

        info!("Stopped");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop()?;

        unsafe {
            Self::netif_unbind(self.netif.as_mut())?;

            esp!(esp_eth_del_netif_glue(self.glue_handle))?;

            esp!(esp_event_handler_unregister(
                ETH_EVENT,
                ESP_EVENT_ANY_ID,
                Option::Some(Self::event_handler)
            ))?;
            esp!(esp_event_handler_unregister(
                IP_EVENT,
                ESP_EVENT_ANY_ID as i32,
                Option::Some(Self::event_handler)
            ))?;

            info!("Event handlers deregistered");

            esp!(esp_eth_driver_uninstall(self.handle))?;

            info!("Driver deinitialized");
        }

        info!("Deinitialization complete");

        Ok(())
    }

    fn netif_unbind(_netif: Option<&mut EspNetif>) -> Result<(), EspError> {
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
            if event_base == ETH_EVENT {
                Self::on_eth_event(shared, event_id, event_data)
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
    unsafe fn on_eth_event(
        shared: &mut Shared,
        event_id: c_types::c_int,
        _event_data: *mut c_types::c_void,
    ) -> Result<(), EspError> {
        info!("Got eth event: {} ", event_id);

        shared.status = match event_id as u32 {
            eth_event_t_ETHERNET_EVENT_START => Status::Starting,
            eth_event_t_ETHERNET_EVENT_STOP => Status::Stopped,
            eth_event_t_ETHERNET_EVENT_CONNECTED => {
                Status::Started(ConnectionStatus::Connected(match shared.conf {
                    Configuration::Client(ipv4::ClientConfiguration::DHCP) => IpStatus::Waiting,
                    Configuration::Client(ipv4::ClientConfiguration::Fixed(ref status)) => {
                        IpStatus::Done(Some(status.clone()))
                    }
                    Configuration::Router(_) => IpStatus::Done(None),
                    _ => IpStatus::Disabled,
                }))
            }
            eth_event_t_ETHERNET_EVENT_DISCONNECTED => {
                Status::Started(ConnectionStatus::Disconnected)
            }
            _ => shared.status.clone(),
        };

        info!("Set status: {:?}", shared.status);

        info!("Eth event {} handled", event_id);

        Ok(())
    }

    #[allow(non_upper_case_globals)]
    unsafe fn on_ip_event(
        shared: &mut Shared,
        event_id: c_types::c_int,
        event_data: *mut c_types::c_void,
    ) -> Result<(), EspError> {
        info!("Got IP event: {}", event_id);

        shared.status = match event_id as u32 {
            ip_event_t_IP_EVENT_ETH_GOT_IP => {
                let event: *const ip_event_got_ip_t = mem::transmute(event_data);

                Status::Started(ConnectionStatus::Connected(IpStatus::Done(Some(
                    ipv4::ClientSettings {
                        ip: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.ip)),
                        subnet: ipv4::Subnet {
                            gateway: ipv4::Ipv4Addr::from(Newtype((*event).ip_info.gw)),
                            mask: Newtype((*event).ip_info.netmask).try_into()?,
                        },
                        dns: None,           // TODO
                        secondary_dns: None, // TODO
                    },
                ))))
            }
            _ => shared.status.clone(),
        };

        info!("Set status: {:?}", shared.status);

        info!("IP event {} handled", event_id);

        Ok(())
    }

    fn eth_default_config(mac: *mut esp_eth_mac_t, phy: *mut esp_eth_phy_t) -> esp_eth_config_t {
        esp_eth_config_t {
            mac,
            phy,
            check_link_period_ms: 2000,
            ..Default::default()
        }
    }

    fn eth_phy_default_config() -> eth_phy_config_t {
        eth_phy_config_t {
            phy_addr: ESP_ETH_PHY_ADDR_AUTO,
            reset_timeout_ms: 100,
            autonego_timeout_ms: 4000,
            reset_gpio_num: 5,
            ..Default::default()
        }
    }

    fn eth_mac_default_config() -> eth_mac_config_t {
        eth_mac_config_t {
            sw_reset_timeout_ms: 100,
            rx_task_stack_size: 2048,
            rx_task_prio: 15,
            smi_mdc_gpio_num: 23,
            smi_mdio_gpio_num: 18,
            flags: 0,
            #[cfg(all(esp_idf_version_major = "4", esp_idf_version_minor = "4"))]
            interface: EMAC_DATA_INTERFACE_RMII,
            #[cfg(all(esp_idf_version_major = "4", esp_idf_version_minor = "4"))]
            clock_config: eth_mac_clock_config_t {
                rmii: rmii {
                    clock_mode: EMAC_CLK_DEFAULT,
                    clock_gpio: EMAC_CLK_IN_GPIO,
                },
            },
            ..Default::default()
        }
    }
}

impl<IO> Eth for EspEth<IO> {
    type Error = EspError;

    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        let caps = Capability::Client | Capability::Router;

        info!("Providing capabilities: {:?}", caps);

        Ok(caps)
    }

    fn get_status(&self) -> Status {
        let status = self.shared.with_lock(|shared| shared.status.clone());

        info!("Providing status: {:?}", status);

        status
    }

    #[allow(non_upper_case_globals)]
    fn get_configuration(&self) -> Result<Configuration, Self::Error> {
        info!("Getting configuration");

        let conf = self.shared.with_lock(|shared| shared.conf.clone());

        info!("Configuration gotten: {:?}", &conf);

        Ok(conf)
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        info!("Setting configuration: {:?}", conf);

        self.stop()?;

        self.set_ip_conf(conf)?;

        let status = if matches!(conf, Configuration::None) {
            Status::Stopped
        } else {
            Status::Starting
        };

        self.start(status)?;

        info!("Configuration set");

        Ok(())
    }
}
