use core::{convert::TryInto, ptr, time::Duration};

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use enumset::*;

#[allow(unused_imports)]
use mutex_trait::Mutex;

use embedded_svc::eth::*;
use embedded_svc::ipv4;
use embedded_svc::mutex::Mutex as ESVCMutex;

use esp_idf_sys::*;

#[cfg(any(
    all(esp32, esp_idf_eth_use_esp32_emac),
    any(
        esp_idf_eth_spi_ethernet_dm9051,
        esp_idf_eth_spi_ethernet_w5500,
        esp_idf_eth_spi_ethernet_ksz8851snl
    )
))]
use esp_idf_hal::gpio;

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
use esp_idf_hal::{spi, units::Hertz};

use crate::netif::*;
use crate::sysloop::*;

use crate::private::common::*;

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
// TODO: #[derive(Debug)]
pub struct Esp32EthHw<MDC, MDIO, RST: gpio::OutputPin = gpio::Gpio10<gpio::Output>> {
    pub rmii_rdx0: gpio::Gpio25<gpio::Output>,
    pub rmii_rdx1: gpio::Gpio26<gpio::Output>,
    pub rmii_crs_dv: gpio::Gpio27<gpio::Output>,
    pub rmii_mdc: MDC,
    pub rmii_txd1: gpio::Gpio22<gpio::Output>,
    pub rmii_tx_en: gpio::Gpio21<gpio::Output>,
    pub rmii_txd0: gpio::Gpio19<gpio::Output>,
    pub rmii_mdio: MDIO,
    pub rmii_ref_clk: gpio::Gpio0<gpio::Output>,
    pub rst: Option<RST>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Esp32EthDriver {
    IP101,
    RTL8201,
    LAN87XX,
    DP83848,
    KSZ8041,
    #[cfg(esp_idf_version = "4.4")]
    KSZ8081,
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
// TODO: #[derive(Debug)]
pub struct SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST = gpio::Gpio10<gpio::Output>>
where
    INT: gpio::InputPin,
    SPI: spi::Spi,
    SCLK: gpio::OutputPin,
    SDO: gpio::OutputPin,
    SDI: gpio::InputPin + gpio::OutputPin,
    CS: gpio::OutputPin,
    RST: gpio::OutputPin,
{
    pub int_pin: INT,
    pub rst_pin: Option<RST>,
    pub spi_pins: spi::Pins<SCLK, SDO, SDI, CS>,
    pub spi: SPI,
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SpiEthDriver {
    #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
    DM9051,
    #[cfg(esp_idf_eth_spi_ethernet_w5500)]
    W5500,
    #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
    KSZ8851SNL,
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

pub struct EspEth<HW> {
    netif_stack: Arc<EspNetifStack>,
    _sys_loop_stack: Arc<EspSysLoopStack>,

    hw: HW,

    handle: esp_eth_handle_t,
    glue_handle: *mut c_types::c_void,

    netif: Option<EspNetif>,

    shared: Box<EspMutex<Shared>>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<MDC, MDIO, RST> EspEth<Esp32EthHw<MDC, MDIO, RST>>
where
    MDC: gpio::OutputPin,
    MDIO: gpio::InputPin + gpio::OutputPin,
    RST: gpio::OutputPin,
{
    pub fn new(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        hw: Esp32EthHw<MDC, MDIO, RST>,
        driver: Esp32EthDriver,
        phy_addr: Option<u32>,
    ) -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let (mac, phy) = Self::initialize(driver, &hw.rst, phy_addr)?;

                    let eth = Self::init(netif_stack, sys_loop_stack, mac, phy, None, hw)?;

                    *taken = true;
                    Ok(eth)
                }
            })
        }
    }

    pub fn release(mut self) -> Result<Esp32EthHw<MDC, MDIO, RST>, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                self.clear_all()?;
                *taken = false;

                Ok(())
            })?;
        }

        info!("Released");

        Ok(self.hw)
    }

    fn initialize(
        driver: Esp32EthDriver,
        reset: &Option<RST>,
        phy_addr: Option<u32>,
    ) -> Result<(*mut esp_eth_mac_t, *mut esp_eth_phy_t), EspError> {
        let mac_cfg = EspEth::<Esp32EthHw<MDC, MDIO>>::eth_mac_default_config();
        let phy_cfg = EspEth::<Esp32EthHw<MDC, MDIO>>::eth_phy_default_config(
            reset.as_ref().map(|_| RST::pin()),
            phy_addr,
        );

        let mac = unsafe { esp_eth_mac_new_esp32(&mac_cfg) };

        let phy = match driver {
            Esp32EthDriver::IP101 => unsafe { esp_eth_phy_new_ip101(&phy_cfg) },
            Esp32EthDriver::RTL8201 => unsafe { esp_eth_phy_new_rtl8201(&phy_cfg) },
            #[cfg(esp_idf_version = "4.4")]
            Esp32EthDriver::LAN87XX => unsafe { esp_eth_phy_new_lan87xx(&phy_cfg) },
            #[cfg(not(esp_idf_version = "4.4"))]
            Esp32EthDriver::LAN87XX => unsafe { esp_eth_phy_new_lan8720(&phy_cfg) },
            Esp32EthDriver::DP83848 => unsafe { esp_eth_phy_new_dp83848(&phy_cfg) },
            Esp32EthDriver::KSZ8041 => unsafe { esp_eth_phy_new_ksz8041(&phy_cfg) },
            #[cfg(esp_idf_version = "4.4")]
            Esp32EthDriver::KSZ8081 => unsafe { esp_eth_phy_new_ksz8081(&phy_cfg) },
        };

        Ok((mac, phy))
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
                    let phy = esp_eth_phy_new_dp83848(&Self::eth_phy_default_config(None, None));

                    let eth = Self::init(netif_stack, sys_loop_stack, mac, phy, None, ())?;

                    *taken = true;
                    Ok(eth)
                }
            })
        }
    }

    pub fn release(mut self) -> Result<(), EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                self.clear_all()?;
                *taken = false;

                Ok(())
            })?;
        }

        info!("Released");

        Ok(self.hw)
    }
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<INT, SPI, SCLK, SDO, SDI, CS, RST>
    EspEth<(
        SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST>,
        spi_device_handle_t,
    )>
where
    INT: gpio::InputPin,
    SPI: spi::Spi,
    SCLK: gpio::OutputPin,
    SDO: gpio::OutputPin,
    SDI: gpio::InputPin + gpio::OutputPin,
    CS: gpio::OutputPin,
    RST: gpio::OutputPin,
{
    pub fn new_spi(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        hw: SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST>,
        driver: SpiEthDriver,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
    ) -> Result<Self, EspError> {
        let (mac, phy, spi_handle) = Self::initialize(driver, baudrate, &hw.rst_pin, phy_addr)?;

        Ok(Self::init(
            netif_stack,
            sys_loop_stack,
            mac,
            phy,
            mac_addr,
            (hw, spi_handle),
        )?)
    }

    pub fn release(mut self) -> Result<SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST>, EspError> {
        self.clear_all()?;
        esp!(unsafe { spi_bus_remove_device(self.hw.1) })?;
        esp!(unsafe { spi_bus_free(SPI::device()) })?;

        info!("Released");

        Ok(self.hw.0)
    }

    fn initialize(
        driver: SpiEthDriver,
        baudrate: Hertz,
        reset_pin: &Option<RST>,
        phy_addr: Option<u32>,
    ) -> Result<(*mut esp_eth_mac_t, *mut esp_eth_phy_t, spi_device_handle_t), EspError> {
        Self::initialize_spi_bus()?;

        let mac_cfg =
            EspEth::<SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST>>::eth_mac_default_config();
        let phy_cfg = EspEth::<SpiEthHw<INT, SPI, SCLK, SDO, SDI, CS, RST>>::eth_phy_default_config(
            reset_pin.as_ref().map(|_| RST::pin()),
            phy_addr,
        );

        let (mac, phy, spi_handle) = match driver {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            SpiEthDriver::DM9051 => {
                let spi_handle = Self::initialize_spi(1, 7, baudrate)?;

                let dm9051_cfg = eth_dm9051_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: INT::pin(),
                };

                let mac = unsafe { esp_eth_mac_new_dm9051(&dm9051_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_dm9051(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            SpiEthDriver::W5500 => {
                let spi_handle = Self::initialize_spi(16, 8, baudrate)?;

                let w5500_cfg = eth_w5500_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: INT::pin(),
                };

                let mac = unsafe { esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_w5500(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            SpiEthDriver::KSZ8851SNL => {
                let spi_handle = Self::initialize_spi(0, 0, baudrate)?;

                let ksz8851snl_cfg = eth_ksz8851snl_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: INT::pin(),
                };

                let mac = unsafe { esp_eth_mac_new_ksz8851snl(&ksz8851snl_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_ksz8851snl(&phy_cfg) };

                (mac, phy, spi_handle)
            }
        };

        Ok((mac, phy, spi_handle))
    }

    fn initialize_spi(
        command_bits: u8,
        address_bits: u8,
        baudrate: Hertz,
    ) -> Result<spi_device_handle_t, EspError> {
        let dev_cfg = spi_device_interface_config_t {
            command_bits,
            address_bits,
            mode: 0,
            clock_speed_hz: baudrate.0 as i32,
            spics_io_num: CS::pin(),
            queue_size: 20,
            ..Default::default()
        };

        let mut spi_handle: spi_device_handle_t = ptr::null_mut();

        esp!(unsafe { spi_bus_add_device(SPI::device(), &dev_cfg, &mut spi_handle) })?;

        Ok(spi_handle)
    }

    fn initialize_spi_bus() -> Result<(), EspError> {
        unsafe { gpio_install_isr_service(0) };

        #[cfg(esp_idf_version = "4.4")]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: SCLK::pin(),

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: SDO::pin(),
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: SDI::pin(),
                //data1_io_num: -1,
            },
            __bindgen_anon_3: spi_bus_config_t__bindgen_ty_3 {
                quadwp_io_num: -1,
                //data2_io_num: -1,
            },
            __bindgen_anon_4: spi_bus_config_t__bindgen_ty_4 {
                quadhd_io_num: -1,
                //data3_io_num: -1,
            },
            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        #[cfg(not(esp_idf_version = "4.4"))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: SCLK::pin(),

            mosi_io_num: SDO::pin(),
            miso_io_num: SDI::pin(),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(SPI::device(), &bus_config, 1) })?; // SPI_DMA_CH_AUTO

        Ok(())
    }
}

impl<HW> EspEth<HW> {
    fn init(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        mac: *mut esp_eth_mac_t,
        phy: *mut esp_eth_phy_t,
        mac_addr: Option<&[u8; 6]>,
        hw: HW,
    ) -> Result<Self, EspError> {
        let cfg = Self::eth_default_config(mac, phy);

        let mut handle: esp_eth_handle_t = ptr::null_mut();
        esp!(unsafe { esp_eth_driver_install(&cfg, &mut handle) })?;

        info!("Driver initialized");

        if let Some(mac_addr) = mac_addr {
            esp!(unsafe {
                esp_eth_ioctl(
                    handle,
                    esp_eth_io_cmd_t_ETH_CMD_S_MAC_ADDR,
                    mac_addr.as_ptr() as *mut _,
                )
            })?;

            info!("Attached MAC address: {:?}", mac_addr);
        }

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
            hw,
            handle,
            glue_handle: glue_handle as *mut _,
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

        let err = unsafe { esp_eth_stop(self.handle) };
        if err != ESP_ERR_INVALID_STATE as i32 {
            esp!(err)?;
        }
        info!("Stop requested");

        self.wait_status(|s| matches!(s, Status::Stopped));

        info!("Stopped");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop()?;

        unsafe {
            Self::netif_unbind(self.netif.as_mut())?;

            esp!(esp_eth_del_netif_glue(self.glue_handle as *mut _))?;

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
            ip_event_t_IP_EVENT_ETH_GOT_IP | ip_event_t_IP_EVENT_STA_GOT_IP => {
                let event = (event_data as *const ip_event_got_ip_t).as_ref().unwrap();

                Status::Started(ConnectionStatus::Connected(IpStatus::Done(Some(
                    ipv4::ClientSettings {
                        ip: ipv4::Ipv4Addr::from(Newtype(event.ip_info.ip)),
                        subnet: ipv4::Subnet {
                            gateway: ipv4::Ipv4Addr::from(Newtype(event.ip_info.gw)),
                            mask: Newtype(event.ip_info.netmask).try_into()?,
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

    fn eth_phy_default_config(reset_pin: Option<i32>, phy_addr: Option<u32>) -> eth_phy_config_t {
        eth_phy_config_t {
            phy_addr: phy_addr.map(|a| a as i32).unwrap_or(ESP_ETH_PHY_ADDR_AUTO),
            reset_timeout_ms: 100,
            autonego_timeout_ms: 4000,
            reset_gpio_num: reset_pin.unwrap_or(-1),
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
            #[cfg(esp_idf_version = "4.4")]
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            #[cfg(esp_idf_version = "4.4")]
            clock_config: eth_mac_clock_config_t {
                rmii: eth_mac_clock_config_t__bindgen_ty_2 {
                    clock_mode: emac_rmii_clock_mode_t_EMAC_CLK_DEFAULT,
                    clock_gpio: emac_rmii_clock_gpio_t_EMAC_CLK_IN_GPIO,
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
