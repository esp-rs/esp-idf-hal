use core::cell::UnsafeCell;
use core::fmt::Debug;
use core::ptr;
use core::time::Duration;

use ::log::*;

use enumset::*;

extern crate alloc;
use alloc::sync::Arc;

use embedded_svc::errors::Errors;
use embedded_svc::eth::*;
use embedded_svc::event_bus::EventBus;
use embedded_svc::ipv4;

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

use esp_idf_sys::*;

use crate::eventloop::{EspSubscription, EspTypedEventDeserializer, EspTypedEventSource, System};
use crate::netif::*;
use crate::private::common::UnsafeCellSendSync;
use crate::private::waitable::*;
use crate::sysloop::*;

#[cfg(feature = "experimental")]
pub use asyncify::*;

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
// TODO: #[derive(Debug)]
pub struct RmiiEthPeripherals<MDC, MDIO, RST: gpio::OutputPin = gpio::Gpio10<gpio::Unknown>> {
    pub rmii_rdx0: gpio::Gpio25<gpio::Unknown>,
    pub rmii_rdx1: gpio::Gpio26<gpio::Unknown>,
    pub rmii_crs_dv: gpio::Gpio27<gpio::Unknown>,
    pub rmii_mdc: MDC,
    pub rmii_txd1: gpio::Gpio22<gpio::Unknown>,
    pub rmii_tx_en: gpio::Gpio21<gpio::Unknown>,
    pub rmii_txd0: gpio::Gpio19<gpio::Unknown>,
    pub rmii_mdio: MDIO,
    pub rmii_ref_clk: gpio::Gpio0<gpio::Unknown>,
    pub rst: Option<RST>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RmiiEthChipset {
    IP101,
    RTL8201,
    LAN87XX,
    DP83848,
    #[cfg(not(esp_idf_version_major = "5"))]
    KSZ8041,
    #[cfg(esp_idf_version = "4.4")]
    KSZ8081,
    #[cfg(esp_idf_version_major = "5")]
    KSZ80XX,
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
// TODO: #[derive(Debug)]
pub struct SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST = gpio::Gpio10<gpio::Unknown>>
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
pub enum SpiEthChipset {
    #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
    DM9051,
    #[cfg(esp_idf_eth_spi_ethernet_w5500)]
    W5500,
    #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
    KSZ8851SNL,
}

#[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
static TAKEN: esp_idf_hal::mutex::Mutex<bool> = esp_idf_hal::mutex::Mutex::new(false);

struct Shared {
    conf: Configuration,

    status: Status,
    operating: bool,

    handle: esp_eth_handle_t,
    netif: Option<EspNetif>,
}

impl Shared {
    fn new(handle: esp_eth_handle_t) -> Self {
        Self {
            conf: Configuration::None,
            status: Status::Stopped,
            operating: false,

            handle,
            netif: None,
        }
    }

    fn is_our_eth_event(&self, event: &EthEvent) -> bool {
        self.handle as *const _ == event.handle()
    }

    fn is_our_ip_event(&self, event: &IpEvent) -> bool {
        self.netif.is_some()
            && self.netif.as_ref().map(|netif| netif.1) == event.handle().map(|handle| handle as _)
    }
}

unsafe impl Send for Shared {}

pub struct EspEth<P> {
    netif_stack: Arc<EspNetifStack>,
    sys_loop_stack: Arc<EspSysLoopStack>,

    peripherals: P,

    glue_handle: *mut c_types::c_void,

    waitable: Arc<Waitable<Shared>>,

    _eth_subscription: EspSubscription<System>,
    _ip_subscription: EspSubscription<System>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<MDC, MDIO, RST> EspEth<RmiiEthPeripherals<MDC, MDIO, RST>>
where
    MDC: gpio::OutputPin,
    MDIO: gpio::InputPin + gpio::OutputPin,
    RST: gpio::OutputPin,
{
    pub fn new_rmii(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        peripherals: RmiiEthPeripherals<MDC, MDIO, RST>,
        chipset: RmiiEthChipset,
        phy_addr: Option<u32>,
    ) -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        let (mac, phy) = Self::initialize(chipset, &peripherals.rst, phy_addr)?;

        let eth = Self::init(netif_stack, sys_loop_stack, mac, phy, None, peripherals)?;

        *taken = true;
        Ok(eth)
    }

    pub fn release(mut self) -> Result<RmiiEthPeripherals<MDC, MDIO, RST>, EspError> {
        {
            let mut taken = TAKEN.lock();

            self.clear_all()?;
            *taken = false;
        }

        info!("Released");

        Ok(self.peripherals)
    }

    fn initialize(
        chipset: RmiiEthChipset,
        reset: &Option<RST>,
        phy_addr: Option<u32>,
    ) -> Result<(*mut esp_eth_mac_t, *mut esp_eth_phy_t), EspError> {
        let mac_cfg = EspEth::<RmiiEthPeripherals<MDC, MDIO>>::eth_mac_default_config();
        let phy_cfg = EspEth::<RmiiEthPeripherals<MDC, MDIO>>::eth_phy_default_config(
            reset.as_ref().map(|p| p.pin()),
            phy_addr,
        );

        let mac = unsafe { esp_eth_mac_new_esp32(&mac_cfg) };

        let phy = match chipset {
            RmiiEthChipset::IP101 => unsafe { esp_eth_phy_new_ip101(&phy_cfg) },
            RmiiEthChipset::RTL8201 => unsafe { esp_eth_phy_new_rtl8201(&phy_cfg) },
            #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
            RmiiEthChipset::LAN87XX => unsafe { esp_eth_phy_new_lan87xx(&phy_cfg) },
            #[cfg(not(any(esp_idf_version = "4.4", esp_idf_version_major = "5")))]
            RmiiEthChipset::LAN87XX => unsafe { esp_eth_phy_new_lan8720(&phy_cfg) },
            RmiiEthChipset::DP83848 => unsafe { esp_eth_phy_new_dp83848(&phy_cfg) },
            #[cfg(not(esp_idf_version_major = "5"))]
            RmiiEthChipset::KSZ8041 => unsafe { esp_eth_phy_new_ksz8041(&phy_cfg) },
            #[cfg(esp_idf_version = "4.4")]
            RmiiEthChipset::KSZ8081 => unsafe { esp_eth_phy_new_ksz8081(&phy_cfg) },
            #[cfg(esp_idf_version_major = "5")]
            RmiiEthChipset::KSZ80XX => unsafe { esp_eth_phy_new_ksz80xx(&phy_cfg) },
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
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        let mac = unsafe { esp_eth_mac_new_openeth(&Self::eth_mac_default_config()) };
        let phy = unsafe { esp_eth_phy_new_dp83848(&Self::eth_phy_default_config(None, None)) };

        let eth = Self::init(netif_stack, sys_loop_stack, mac, phy, None, ())?;

        *taken = true;
        Ok(eth)
    }

    pub fn release(mut self) -> Result<(), EspError> {
        {
            let mut taken = TAKEN.lock();

            self.clear_all()?;
            *taken = false;
        }

        info!("Released");

        Ok(self.peripherals)
    }
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<INT, SPI, SCLK, SDO, SDI, CS, RST>
    EspEth<(
        SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST>,
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
        peripherals: SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST>,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
    ) -> Result<Self, EspError> {
        let (mac, phy, spi_handle) = Self::initialize(
            chipset,
            baudrate,
            &peripherals.spi_pins,
            &peripherals.int_pin,
            &peripherals.rst_pin,
            phy_addr,
        )?;

        Ok(Self::init(
            netif_stack,
            sys_loop_stack,
            mac,
            phy,
            mac_addr,
            (peripherals, spi_handle),
        )?)
    }

    pub fn release(
        mut self,
    ) -> Result<SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST>, EspError> {
        self.clear_all()?;
        esp!(unsafe { spi_bus_remove_device(self.peripherals.1) })?;
        esp!(unsafe { spi_bus_free(SPI::device()) })?;

        info!("Released");

        Ok(self.peripherals.0)
    }

    fn initialize(
        chipset: SpiEthChipset,
        baudrate: Hertz,
        spi_pins: &spi::Pins<SCLK, SDO, SDI, CS>,
        int_pin: &INT,
        reset_pin: &Option<RST>,
        phy_addr: Option<u32>,
    ) -> Result<(*mut esp_eth_mac_t, *mut esp_eth_phy_t, spi_device_handle_t), EspError> {
        Self::initialize_spi_bus(&spi_pins.sclk, &spi_pins.sdo, spi_pins.sdi.as_ref())?;

        let mac_cfg =
            EspEth::<SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST>>::eth_mac_default_config(
            );
        let phy_cfg =
            EspEth::<SpiEthPeripherals<INT, SPI, SCLK, SDO, SDI, CS, RST>>::eth_phy_default_config(
                reset_pin.as_ref().map(|p| p.pin()),
                phy_addr,
            );

        let (mac, phy, spi_handle) = match chipset {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            SpiEthChipset::DM9051 => {
                let spi_handle = Self::initialize_spi(spi_pins.cs.as_ref(), 1, 7, baudrate)?;

                let dm9051_cfg = eth_dm9051_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin.pin(),
                };

                let mac = unsafe { esp_eth_mac_new_dm9051(&dm9051_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_dm9051(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            SpiEthChipset::W5500 => {
                let spi_handle = Self::initialize_spi(spi_pins.cs.as_ref(), 16, 8, baudrate)?;

                let w5500_cfg = eth_w5500_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin.pin(),
                };

                let mac = unsafe { esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_w5500(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            SpiEthChipset::KSZ8851SNL => {
                let spi_handle = Self::initialize_spi(spi_pins.cs.as_ref(), 0, 0, baudrate)?;

                let ksz8851snl_cfg = eth_ksz8851snl_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin.pin(),
                };

                let mac = unsafe { esp_eth_mac_new_ksz8851snl(&ksz8851snl_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_ksz8851snl(&phy_cfg) };

                (mac, phy, spi_handle)
            }
        };

        Ok((mac, phy, spi_handle))
    }

    fn initialize_spi(
        cs_pin: Option<&CS>,
        command_bits: u8,
        address_bits: u8,
        baudrate: Hertz,
    ) -> Result<spi_device_handle_t, EspError> {
        let dev_cfg = spi_device_interface_config_t {
            command_bits,
            address_bits,
            mode: 0,
            clock_speed_hz: baudrate.0 as i32,
            spics_io_num: cs_pin.map(|p| p.pin()).unwrap_or(-1),
            queue_size: 20,
            ..Default::default()
        };

        let mut spi_handle: spi_device_handle_t = ptr::null_mut();

        esp!(unsafe { spi_bus_add_device(SPI::device(), &dev_cfg, &mut spi_handle) })?;

        Ok(spi_handle)
    }

    fn initialize_spi_bus(
        sclk_pin: &SCLK,
        sdo_pin: &SDO,
        sdi_pin: Option<&SDI>,
    ) -> Result<(), EspError> {
        unsafe { gpio_install_isr_service(0) };

        #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk_pin.pin(),

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: sdo_pin.pin(),
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: sdi_pin.map(|p| p.pin()).unwrap_or(-1),
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

        #[cfg(not(any(esp_idf_version = "4.4", esp_idf_version_major = "5")))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk_pin.pin(),

            mosi_io_num: sdo_pin.pin(),
            miso_io_num: sdi_pin.map(|p| p.pin()).unwrap_or(-1),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(SPI::device(), &bus_config, 1) })?; // SPI_DMA_CH_AUTO

        Ok(())
    }
}

impl<P> EspEth<P> {
    fn init(
        netif_stack: Arc<EspNetifStack>,
        sys_loop_stack: Arc<EspSysLoopStack>,
        mac: *mut esp_eth_mac_t,
        phy: *mut esp_eth_phy_t,
        mac_addr: Option<&[u8; 6]>,
        peripherals: P,
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

        let waitable: Arc<Waitable<Shared>> = Arc::new(Waitable::new(Shared::new(handle)));

        let eth_waitable = waitable.clone();
        let eth_subscription =
            sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |event: &EthEvent| {
                    let mut shared = eth_waitable.state.lock();

                    if Self::on_eth_event(&mut shared, event).unwrap() {
                        eth_waitable.cvar.notify_all();
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

        let eth = Self {
            netif_stack,
            sys_loop_stack,
            peripherals,
            glue_handle: glue_handle as *mut _,
            waitable,
            _eth_subscription: eth_subscription,
            _ip_subscription: ip_subscription,
        };

        info!("Initialization complete");

        Ok(eth)
    }

    pub fn with_netif<F, T>(&self, f: F) -> T
    where
        F: FnOnce(Option<&EspNetif>) -> T,
    {
        self.waitable.get(|shared| f(shared.netif.as_ref()))
    }

    pub fn with_netif_mut<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(Option<&mut EspNetif>) -> T,
    {
        self.waitable.get_mut(|shared| f(shared.netif.as_mut()))
    }

    fn set_ip_conf(&mut self, conf: &Configuration) -> Result<(), EspError> {
        {
            let mut shared = self.waitable.state.lock();
            Self::netif_unbind(shared.netif.as_mut())?;
        }

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

        let netif = if let Some(iconf) = iconf {
            let netif = EspNetif::new(self.netif_stack.clone(), &iconf)?;

            esp!(unsafe { esp_netif_attach(netif.1, self.glue_handle) })?;

            info!("IP configuration done");

            Some(netif)
        } else {
            info!("Skipping IP configuration (not configured)");

            None
        };

        {
            let mut shared = self.waitable.state.lock();
            shared.conf = conf.clone();
            shared.netif = netif;
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
            shared.operating = shared.status.is_operating();

            if status.is_operating() {
                info!("Status is of operating type, starting");

                esp!(unsafe { esp_eth_start(shared.handle) })?;

                info!("Start requested");

                Self::netif_info("ETH", shared.netif.as_ref())?;
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

            let err = unsafe { esp_eth_stop(shared.handle) };
            if err != ESP_ERR_INVALID_STATE as i32 {
                esp!(err)?;
            }

            info!("Stop requested");
        }

        if wait {
            self.wait_status(|s| matches!(s, Status::Stopped));
        }

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop(true)?;

        let mut shared = self.waitable.state.lock();

        unsafe {
            Self::netif_unbind(shared.netif.as_mut())?;
            shared.netif = None;

            esp!(esp_eth_del_netif_glue(self.glue_handle as *mut _))?;

            info!("Event handlers deregistered");

            esp!(esp_eth_driver_uninstall(shared.handle))?;

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

    fn on_eth_event(shared: &mut Shared, event: &EthEvent) -> Result<bool, EspError> {
        if shared.is_our_eth_event(event) {
            info!("Got eth event: {:?} ", event);

            shared.status = match event {
                EthEvent::Started(_) => Status::Starting,
                EthEvent::Stopped(_) => Status::Stopped,
                EthEvent::Connected(_) => {
                    Status::Started(ConnectionStatus::Connected(match shared.conf {
                        Configuration::Client(ipv4::ClientConfiguration::DHCP(_)) => {
                            IpStatus::Waiting
                        }
                        Configuration::Client(ipv4::ClientConfiguration::Fixed(ref status)) => {
                            IpStatus::Done(Some(status.clone()))
                        }
                        Configuration::Router(_) => IpStatus::Done(None),
                        _ => IpStatus::Disabled,
                    }))
                }
                EthEvent::Disconnected(_) => Status::Started(ConnectionStatus::Disconnected),
            };

            info!(
                "Eth event {:?} handled, set status: {:?}",
                event, shared.status
            );

            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn on_ip_event(shared: &mut Shared, event: &IpEvent) -> Result<bool, EspError> {
        if shared.is_our_ip_event(event) {
            info!("Got IP event: {:?}", event);

            let status = match event {
                IpEvent::DhcpIpAssigned(assignment) => Some(Status::Started(
                    ConnectionStatus::Connected(IpStatus::Done(Some(assignment.ip_settings))),
                )),
                IpEvent::DhcpIpDeassigned(_) => Some(Status::Started(ConnectionStatus::Connected(
                    IpStatus::Waiting,
                ))),
                _ => None,
            };

            if let Some(status) = status {
                shared.status = status;
                info!(
                    "IP event {:?} handled, set status: {:?}",
                    event, shared.status
                );

                Ok(true)
            } else {
                info!("IP event {:?} skipped", event);

                Ok(false)
            }
        } else {
            Ok(false)
        }
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
            #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            #[cfg(any(esp_idf_version = "4.4", esp_idf_version_major = "5"))]
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

impl<P> Errors for EspEth<P> {
    type Error = EspError;
}

impl<P> Eth for EspEth<P> {
    fn get_capabilities(&self) -> Result<EnumSet<Capability>, Self::Error> {
        let caps = Capability::Client | Capability::Router;

        info!("Providing capabilities: {:?}", caps);

        Ok(caps)
    }

    fn get_status(&self) -> Status {
        let status = self.waitable.get(|shared| shared.status.clone());

        info!("Providing status: {:?}", status);

        status
    }

    #[allow(non_upper_case_globals)]
    fn get_configuration(&self) -> Result<Configuration, Self::Error> {
        info!("Getting configuration");

        let conf = self.waitable.get(|shared| shared.conf.clone());

        info!("Configuration gotten: {:?}", &conf);

        Ok(conf)
    }

    fn set_configuration(&mut self, conf: &Configuration) -> Result<(), Self::Error> {
        info!("Setting configuration: {:?}", conf);

        self.stop(false)?;

        self.set_ip_conf(conf)?;

        let status = if matches!(conf, Configuration::None) {
            Status::Stopped
        } else {
            Status::Starting
        };

        self.start(status, None)?;

        info!("Configuration set");

        Ok(())
    }
}

pub type EthHandle = *const core::ffi::c_void;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum EthEvent {
    Started(EthHandle),
    Stopped(EthHandle),
    Connected(EthHandle),
    Disconnected(EthHandle),
}

impl EthEvent {
    pub fn handle(&self) -> EthHandle {
        match self {
            Self::Started(handle) => *handle,
            Self::Stopped(handle) => *handle,
            Self::Connected(handle) => *handle,
            Self::Disconnected(handle) => *handle,
        }
    }
}

impl EspTypedEventSource for EthEvent {
    fn source() -> *const c_types::c_char {
        unsafe { ETH_EVENT }
    }
}

impl EspTypedEventDeserializer<EthEvent> for EthEvent {
    #[allow(non_upper_case_globals, non_snake_case)]
    fn deserialize<R>(
        data: &crate::eventloop::EspEventFetchData,
        f: &mut impl for<'a> FnMut(&'a EthEvent) -> R,
    ) -> R {
        let eth_handle_ref = unsafe { (data.payload as *const esp_eth_handle_t).as_ref() };

        let event_id = data.event_id as u32;

        let event = if event_id == eth_event_t_ETHERNET_EVENT_START {
            EthEvent::Started(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_STOP {
            EthEvent::Stopped(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_CONNECTED {
            EthEvent::Connected(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_DISCONNECTED {
            EthEvent::Disconnected(*eth_handle_ref.unwrap() as _)
        } else {
            panic!("Unknown event ID: {}", event_id);
        };

        f(&event)
    }
}

impl<P> EventBus<()> for EspEth<P> {
    type Subscription = (EspSubscription<System>, EspSubscription<System>);

    fn subscribe(
        &mut self,
        callback: impl for<'a> FnMut(&'a ()) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        let eth_cb = Arc::new(UnsafeCellSendSync(UnsafeCell::new(callback)));
        let eth_last_status = Arc::new(UnsafeCellSendSync(UnsafeCell::new(self.get_status())));
        let eth_waitable = self.waitable.clone();

        let ip_cb = eth_cb.clone();
        let ip_last_status = eth_last_status.clone();
        let ip_waitable = eth_waitable.clone();

        let subscription1 =
            self.sys_loop_stack
                .get_loop()
                .clone()
                .subscribe(move |event: &EthEvent| {
                    let notify = {
                        let shared = eth_waitable.state.lock();

                        if shared.is_our_eth_event(event) {
                            let last_status_ref =
                                unsafe { eth_last_status.0.get().as_mut().unwrap() };

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
                        let cb_ref = unsafe { eth_cb.0.get().as_mut().unwrap() };

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

                        if shared.is_our_ip_event(event) {
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
mod asyncify {
    use embedded_svc::utils::asyncify::{event_bus::AsyncEventBus, Asyncify};

    impl<P> Asyncify for super::EspEth<P> {
        type AsyncWrapper<S> = AsyncEventBus<(), esp_idf_hal::mutex::Condvar, S>;
    }
}
