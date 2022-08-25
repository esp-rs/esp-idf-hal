use core::borrow::Borrow;
use core::fmt::Debug;
use core::ptr;
use core::time::Duration;

use ::log::*;
use esp_idf_hal::gpio::{InputPin, OutputPin};

extern crate alloc;
use alloc::sync::Arc;

use embedded_svc::eth::*;
use embedded_svc::event_bus::EventBus;

use esp_idf_hal::mac::MAC;
use esp_idf_hal::peripheral::{Peripheral, PeripheralRef};

use crate::handle::RawHandle;
#[cfg(esp_idf_comp_esp_netif_enabled)]
use crate::netif::*;

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

use crate::eventloop::{
    EspEventLoop, EspSubscription, EspSystemEventLoop, EspTypedEventDeserializer,
    EspTypedEventSource, System,
};
#[cfg(esp_idf_comp_esp_netif_enabled)]
use crate::netif::*;
use crate::private::waitable::*;

#[cfg(all(feature = "nightly", feature = "experimental"))]
pub use asyncify::*;

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RmiiEthChipset {
    IP101,
    RTL8201,
    LAN87XX,
    DP83848,
    #[cfg(esp_idf_version_major = "4")]
    KSZ8041,
    #[cfg(esp_idf_version = "4.4")]
    KSZ8081,
    #[cfg(not(esp_idf_version_major = "4"))]
    KSZ80XX,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
pub enum RmiiClockConfig<'d, GPIO0, GPIO16, GPIO17>
where
    GPIO0: Peripheral<P = gpio::Gpio0> + 'd,
    GPIO16: Peripheral<P = gpio::Gpio16> + 'd,
    GPIO17: Peripheral<P = gpio::Gpio17> + 'd,
{
    Input(GPIO0),
    #[cfg(not(esp_idf_version = "4.3"))]
    OutputGpio0(GPIO0),
    /// This according to ESP-IDF is for "testing" only
    #[cfg(not(esp_idf_version = "4.3"))]
    OutputGpio16(GPIO16),
    #[cfg(not(esp_idf_version = "4.3"))]
    OutputInvertedGpio17(GPIO17),
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac, not(esp_idf_version = "4.3")))]
impl<'d, GPIO0, GPIO16, GPIO17> RmiiClockConfig<'d, GPIO0, GPIO16, GPIO17>
where
    GPIO0: Peripheral<P = gpio::Gpio0> + 'd,
    GPIO16: Peripheral<P = gpio::Gpio16> + 'd,
    GPIO17: Peripheral<P = gpio::Gpio17> + 'd,
{
    fn eth_mac_clock_config(&self) -> eth_mac_clock_config_t {
        let rmii = match self {
            Self::Input(_) => eth_mac_clock_config_t__bindgen_ty_2 {
                clock_mode: emac_rmii_clock_mode_t_EMAC_CLK_EXT_IN,
                clock_gpio: emac_rmii_clock_gpio_t_EMAC_CLK_IN_GPIO,
            },
            Self::OutputGpio0(_) => eth_mac_clock_config_t__bindgen_ty_2 {
                clock_mode: emac_rmii_clock_mode_t_EMAC_CLK_OUT,
                clock_gpio: emac_rmii_clock_gpio_t_EMAC_APPL_CLK_OUT_GPIO,
            },
            Self::OutputGpio16(_) => eth_mac_clock_config_t__bindgen_ty_2 {
                clock_mode: emac_rmii_clock_mode_t_EMAC_CLK_OUT,
                clock_gpio: emac_rmii_clock_gpio_t_EMAC_CLK_OUT_GPIO,
            },
            Self::OutputInvertedGpio17(_) => eth_mac_clock_config_t__bindgen_ty_2 {
                clock_mode: emac_rmii_clock_mode_t_EMAC_CLK_OUT,
                clock_gpio: emac_rmii_clock_gpio_t_EMAC_CLK_OUT_180_GPIO,
            },
        };

        eth_mac_clock_config_t { rmii }
    }
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

pub struct EthDriver<'d, P> {
    peripheral: PeripheralRef<'d, P>,
    spi: Option<(spi_device_handle_t, spi_host_device_t)>,
    handle: esp_eth_handle_t,
    _sysloop: EspSystemEventLoop,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<'d> EthDriver<'d, MAC> {
    pub fn new_rmii(
        mac: impl Peripheral<P = MAC> + 'd,
        rmii_rdx0: impl Peripheral<P = gpio::Gpio25> + 'd,
        rmii_rdx1: impl Peripheral<P = gpio::Gpio26> + 'd,
        rmii_crs_dv: impl Peripheral<P = gpio::Gpio27> + 'd,
        rmii_mdc: impl Peripheral<P = impl OutputPin> + 'd,
        rmii_txd1: impl Peripheral<P = gpio::Gpio22> + 'd,
        rmii_tx_en: impl Peripheral<P = gpio::Gpio21> + 'd,
        rmii_txd0: impl Peripheral<P = gpio::Gpio19> + 'd,
        rmii_mdio: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        rmii_ref_clk_config: RmiiClockConfig<
            'd,
            impl Peripheral<P = gpio::Gpio0> + 'd,
            impl Peripheral<P = gpio::Gpio16> + 'd,
            impl Peripheral<P = gpio::Gpio17> + 'd,
        >,
        rst: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        chipset: RmiiEthChipset,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        esp_idf_hal::into_ref!(mac);

        let eth = Self::init(
            mac,
            Self::rmii_mac(&rmii_ref_clk_config),
            Self::rmii_phy(chipset, &rst, phy_addr)?,
            None,
            None,
            sysloop,
        )?;

        Ok(eth)
    }

    fn rmii_phy(
        chipset: RmiiEthChipset,
        reset: &Option<impl Peripheral<P = impl OutputPin> + 'd>,
        phy_addr: Option<u32>,
    ) -> Result<*mut esp_eth_phy_t, EspError> {
        let phy_cfg = Self::eth_phy_default_config(reset.as_ref().map(|p| p.pin()), phy_addr);

        let phy = match chipset {
            RmiiEthChipset::IP101 => unsafe { esp_eth_phy_new_ip101(&phy_cfg) },
            RmiiEthChipset::RTL8201 => unsafe { esp_eth_phy_new_rtl8201(&phy_cfg) },
            #[cfg(not(esp_idf_version = "4.3"))]
            RmiiEthChipset::LAN87XX => unsafe { esp_eth_phy_new_lan87xx(&phy_cfg) },
            #[cfg(esp_idf_version = "4.3")]
            RmiiEthChipset::LAN87XX => unsafe { esp_eth_phy_new_lan8720(&phy_cfg) },
            RmiiEthChipset::DP83848 => unsafe { esp_eth_phy_new_dp83848(&phy_cfg) },
            #[cfg(esp_idf_version_major = "4")]
            RmiiEthChipset::KSZ8041 => unsafe { esp_eth_phy_new_ksz8041(&phy_cfg) },
            #[cfg(esp_idf_version = "4.4")]
            RmiiEthChipset::KSZ8081 => unsafe { esp_eth_phy_new_ksz8081(&phy_cfg) },
            #[cfg(not(esp_idf_version_major = "4"))]
            RmiiEthChipset::KSZ80XX => unsafe { esp_eth_phy_new_ksz80xx(&phy_cfg) },
        };

        Ok(phy)
    }

    fn rmii_mac(
        clk_config: &RmiiClockConfig<
            'd,
            impl Peripheral<P = gpio::Gpio0> + 'd,
            impl Peripheral<P = gpio::Gpio16> + 'd,
            impl Peripheral<P = gpio::Gpio17> + 'd,
        >,
    ) -> *mut esp_eth_mac_t {
        #[cfg(esp_idf_version_major = "4")]
        let mac = {
            let mut config = Self::eth_mac_default_config();

            #[cfg(not(esp_idf_version = "4.3"))]
            {
                config.clock_config = clk_config.eth_mac_clock_config();
            }

            unsafe { esp_eth_mac_new_esp32(&config) }
        };

        #[cfg(not(esp_idf_version_major = "4"))]
        let mac = {
            let mut esp32_config = Self::eth_esp32_emac_default_config();
            esp32_config.clock_config = clk_config.eth_mac_clock_config();

            let config = Self::eth_mac_default_config();

            unsafe { esp_eth_mac_new_esp32(&esp32_config, &config) }
        };

        mac
    }
}

#[cfg(esp_idf_eth_use_openeth)]
impl<'d> EthDriver<'d, MAC> {
    pub fn new_openeth(
        mac: esp_idf_hal::mac::Mac,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        crate::into_ref!(mac);

        let eth = Self::init(
            unsafe { esp_eth_mac_new_openeth(&Self::eth_mac_default_config()) },
            unsafe { esp_eth_phy_new_dp83848(&Self::eth_phy_default_config(None, None)) },
            None,
            (),
            sysloop,
        )?;

        Ok(eth)
    }
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<'d, P: Spi> EthDriver<'d, P> {
    pub fn new_spi(
        spi: impl Peripheral<P = P> + 'd,
        int: impl Peripheral<P = impl gpio::InputPin> + 'd,
        sclk: impl Peripheral<P = impl gpio::OutputPin> + 'd,
        sdo: impl Peripheral<P = impl gpio::OutputPin> + 'd,
        sdi: Option<impl Peripheral<P = impl gpio::InputPin + gpio::OutputPin> + 'd>,
        cs: impl Peripheral<P = impl gpio::OutputPin> + 'd,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        crate::into_ref!(spi);

        let (mac, phy, spi_handle) = Self::init_spi(
            chipset,
            baudrate,
            int.pin(),
            sclk.pin(),
            sdo.pin(),
            sdi.map(|pin| pin.pin()),
            cs.pin(),
            rst.map(|pin| pin.pin()),
            phy_addr,
        )?;

        let eth = Self::init(
            spi,
            mac,
            phy,
            mac_addr,
            Some((spi_handle, P::device())),
            sysloop,
        )?;

        Ok(eth)
    }

    fn init_spi(
        chipset: SpiEthChipset,
        baudrate: Hertz,
        int: i32,
        sclk: i32,
        sdo: i32,
        sdi: Option<i32>,
        cs: i32,
        rst: Option<i32>,
        phy_addr: Option<u32>,
    ) -> Result<(*mut esp_eth_mac_t, *mut esp_eth_phy_t, spi_device_handle_t), EspError> {
        Self::init_spi_bus(sclk, sdo, sdi)?;

        let mac_cfg = EthDriver::eth_mac_default_config();
        let phy_cfg = EthDriver::eth_phy_default_config(rst.map(|pin| pin), phy_addr);

        let (mac, phy, spi_handle) = match chipset {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            SpiEthChipset::DM9051 => {
                let spi_handle = Self::init_spi_device(cs, 1, 7, baudrate)?;

                let dm9051_cfg = eth_dm9051_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin,
                };

                let mac = unsafe { esp_eth_mac_new_dm9051(&dm9051_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_dm9051(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            SpiEthChipset::W5500 => {
                let spi_handle = Self::init_spi_device(cs, 16, 8, baudrate)?;

                let w5500_cfg = eth_w5500_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin,
                };

                let mac = unsafe { esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_w5500(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            SpiEthChipset::KSZ8851SNL => {
                let spi_handle = Self::init_spi_device(cs, 0, 0, baudrate)?;

                let ksz8851snl_cfg = eth_ksz8851snl_config_t {
                    spi_hdl: spi_handle as *mut _,
                    int_gpio_num: int_pin,
                };

                let mac = unsafe { esp_eth_mac_new_ksz8851snl(&ksz8851snl_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_ksz8851snl(&phy_cfg) };

                (mac, phy, spi_handle)
            }
        };

        Ok((mac, phy, spi_handle))
    }

    fn init_spi_device(
        cs_pin: Option<i32>,
        command_bits: u8,
        address_bits: u8,
        baudrate: Hertz,
    ) -> Result<spi_device_handle_t, EspError> {
        let dev_cfg = spi_device_interface_config_t {
            command_bits,
            address_bits,
            mode: 0,
            clock_speed_hz: baudrate.0 as i32,
            spics_io_num: cs_pin.map(|pin| pin).unwrap_or(-1),
            queue_size: 20,
            ..Default::default()
        };

        let mut spi_handle: spi_device_handle_t = ptr::null_mut();

        esp!(unsafe { spi_bus_add_device(P::device(), &dev_cfg, &mut spi_handle) })?;

        Ok(spi_handle)
    }

    fn init_spi_bus(sclk_pin: i32, sdo_pin: i32, sdi_pin: Option<i32>) -> Result<(), EspError> {
        unsafe { gpio_install_isr_service(0) };

        #[cfg(not(esp_idf_version = "4.3"))]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk_pin,

            data4_io_num: -1,
            data5_io_num: -1,
            data6_io_num: -1,
            data7_io_num: -1,
            __bindgen_anon_1: spi_bus_config_t__bindgen_ty_1 {
                mosi_io_num: sdo_pin,
                //data0_io_num: -1,
            },
            __bindgen_anon_2: spi_bus_config_t__bindgen_ty_2 {
                miso_io_num: sdi_pin.map(|pin| pin).unwrap_or(-1),
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

        #[cfg(esp_idf_version = "4.3")]
        let bus_config = spi_bus_config_t {
            flags: SPICOMMON_BUSFLAG_MASTER,
            sclk_io_num: sclk_pin,

            mosi_io_num: sdo_pin,
            miso_io_num: sdi_pin.map(|pin| pin).unwrap_or(-1),
            quadwp_io_num: -1,
            quadhd_io_num: -1,

            //max_transfer_sz: SPI_MAX_TRANSFER_SIZE,
            ..Default::default()
        };

        esp!(unsafe { spi_bus_initialize(P::device(), &bus_config, 1) })?; // SPI_DMA_CH_AUTO

        Ok(())
    }
}

impl<'d, P> EthDriver<'d, P> {
    fn init(
        peripheral: PeripheralRef<'d, P>,
        mac: *mut esp_eth_mac_t,
        phy: *mut esp_eth_phy_t,
        mac_addr: Option<&[u8; 6]>,
        spi: Option<(spi_device_handle_t, spi_host_device_t)>,
        sysloop: EspSystemEventLoop,
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

        let eth = Self {
            peripheral,
            handle,
            spi,
            _sysloop: sysloop,
        };

        info!("Initialization complete");

        Ok(eth)
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_eth_start(self.handle) })?;

        info!("Start requested");

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), EspError> {
        info!("Stopping");

        let err = unsafe { esp_eth_stop(self.handle) };
        if err != ESP_ERR_INVALID_STATE as i32 {
            esp!(err)?;
        }

        info!("Stop requested");

        Ok(())
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        self.stop()?;

        unsafe {
            esp!(esp_eth_driver_uninstall(self.handle))?;
        }

        info!("Driver deinitialized");

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

    #[cfg(esp_idf_version_major = "4")]
    fn eth_mac_default_config(mdc: i32, mdio: i32) -> eth_mac_config_t {
        eth_mac_config_t {
            sw_reset_timeout_ms: 100,
            rx_task_stack_size: 2048,
            rx_task_prio: 15,
            smi_mdc_gpio_num: mdc,
            smi_mdio_gpio_num: mdio,
            flags: 0,
            #[cfg(esp_idf_version = "4.4")]
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            ..Default::default()
        }
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    fn eth_mac_default_config(_mdc: i32, _mdio: i32) -> eth_mac_config_t {
        eth_mac_config_t {
            sw_reset_timeout_ms: 100,
            rx_task_stack_size: 2048,
            rx_task_prio: 15,
            flags: 0,
            ..Default::default()
        }
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    fn eth_esp32_emac_default_config(mdc: i32, mdio: i32) -> eth_esp32_emac_config_t {
        eth_esp32_emac_config_t {
            smi_mdc_gpio_num: mdc,
            smi_mdio_gpio_num: mdio,
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            ..Default::default()
        }
    }
}

impl<'d, P> Eth for EthDriver<'d, P> {
    type Error = EspError;

    fn start(&mut self) -> Result<(), Self::Error> {
        EthDriver::start(self)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        EthDriver::stop(self)
    }

    fn is_up(&self) -> Result<bool, Self::Error> {
        EthDriver::is_up(self)
    }
}

unsafe impl<'d, P> Send for EthDriver<'d, P> {}

impl<'d, P> Drop for EthDriver<'d, P> {
    fn drop(&mut self) {
        self.clear_all().unwrap();

        if let Some((device, bus)) = self.spi {
            esp!(unsafe { spi_bus_remove_device(device) }).unwrap();
            esp!(unsafe { spi_bus_free(bus) }).unwrap();
        }

        info!("Dropped");
    }
}

impl<'d, P> RawHandle for EthDriver<'d, P> {
    type Handle = esp_eth_handle_t;

    fn handle(&self) -> Self::Handle {
        self.handle
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
pub struct EspEth<'d, P> {
    driver: EthDriver<'d, P>,
    netif: EspNetif,
    glue_handle: *mut esp_eth_netif_glue_t,
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, P> EspEth<'d, P> {
    pub fn new(driver: EthDriver<'d, P>) -> Result<Self, EspError> {
        Self::wrap(driver, EspNetif::new(NetifStack::Eth))
    }

    pub fn wrap(driver: EthDriver<'d, P>, netif: EspNetif) -> Result<Self, EspError> {
        let glue_handle = unsafe { esp_eth_new_netif_glue(driver.handle()) };

        let this = Self {
            driver,
            netif,
            glue_handle,
        };

        esp!(unsafe { esp_netif_attach(netif.1, glue_handle) })?;

        Ok(this)
    }

    pub fn driver(&self) -> &EthDriver<'d, P> {
        &self.driver
    }

    pub fn driver_mut(&mut self) -> &mut EspNetif {
        &mut self.netif
    }

    pub fn netif(&self) -> &EspNetif {
        &self.netif
    }

    pub fn netif_mut(&mut self) -> &mut EspNetif {
        &mut self.netif
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, P> Drop for EspEth<'d, P> {
    fn drop(&mut self) {
        unsafe {
            esp!(esp_eth_del_netif_glue(self.glue_handle as *mut _)).unwrap();
        }
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, P> RawHandle for EspEth<'d, P> {
    type Handle = *mut esp_eth_netif_glue_t;

    unsafe fn handle(&self) -> Self::Handle {
        self.glue_handle
    }
}

struct UnsafeCallback(*mut Box<dyn FnMut(&[u8]) + 'static>);

impl UnsafeCallback {
    #[allow(clippy::type_complexity)]
    fn from(boxed: &mut Box<Box<dyn for<'a> FnMut(&[u8]) + 'static>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: &[u8]) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

pub struct EspRawEth<'d, P> {
    driver: EthDriver<'d, P>,
    _callback: Box<Box<dyn FnMut(&[u8]) + 'static>>,
}

impl<'d, P> EspRawEth<'d, P> {
    pub fn new<C>(driver: EthDriver<'d, P>, callback: C) -> Result<Self, EspError>
    where
        C: for<'a> FnMut(&[u8]),
    {
        let callback: Box<dyn FnMut(&[u8]) + 'static> = Box::new(move |data| callback(data));

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        unsafe {
            esp_eth_update_input_path(driver.handle(), Self::handle, unsafe_callback.as_ptr());
        }

        Ok(Self {
            driver,
            _callback: callback,
        })
    }

    pub fn driver(&self) -> &EthDriver<'d, P> {
        &self.driver
    }

    pub fn driver_mut(&mut self) -> &mut EspNetif {
        &mut self.netif
    }

    pub fn send(&mut self, frame: &[u8]) -> Result<(), EspError> {
        esp!(unsafe { esp_eth_transmit(self.driver.handle(), frame.as_ptr(), frame.len()) })?;

        Ok(())
    }

    extern "C" fn handle(
        _handle: esp_eth_handle_t,
        buf: *const u8,
        len: usize,
        event_handler_arg: *const c_types::c_void,
    ) {
        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(core::slice::from_raw_parts(buf, len));
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum EthEvent {
    Started(esp_eth_handle_t),
    Stopped(esp_eth_handle_t),
    Connected(esp_eth_handle_t),
    Disconnected(esp_eth_handle_t),
}

impl EthEvent {
    pub fn is_for(&self, raw_handle: impl RawHandle<Handle = esp_eth_handle_t>) -> bool {
        self.handle() == unsafe { raw_handle.handle() }
    }

    pub fn handle(&self) -> esp_eth_handle_t {
        let handle = match self {
            Self::Started(handle) => *handle,
            Self::Stopped(handle) => *handle,
            Self::Connected(handle) => *handle,
            Self::Disconnected(handle) => *handle,
        };

        handle as esp_eth_handle_t
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

pub struct EthStatus<B, R>
where
    B: Borrow<R>,
    R: RawHandle<Handle = esp_eth_handle_t>,
{
    driver: B,
    waitable: Arc<Waitable<bool>>,
    _subscription: EspSubscription<System>,
}

impl<B, R> EthStatus<B, R>
where
    B: Borrow<R>,
    R: RawHandle<Handle = esp_eth_handle_t>,
{
    pub fn new(driver: B, sysloop: &EspEventLoop<System>) -> Result<Self, EspError> {
        let waitable: Arc<Waitable<Status>> = Arc::new(Waitable::new(Status::Disconnected));

        let eth_waitable = waitable.clone();
        let driver_handle = driver.handle;

        let subscription = sysloop.subscribe(move |event: &EthEvent| {
            let mut status = eth_waitable.state.lock();

            if Self::on_eth_event(driver_handle, &mut status, event).unwrap() {
                eth_waitable.cvar.notify_all();
            }
        })?;

        Ok(Self {
            driver,
            waitable,
            _subscription: subscription,
        })
    }

    pub fn get_status(&self) -> Status {
        return self.waitable.state.lock().clone();
    }

    pub fn wait_status(&self, matcher: impl Fn(&Status) -> bool) {
        info!("About to wait for status");

        self.waitable.wait_while(|status| !matcher(status));

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

    fn on_eth_event(
        driver_handle: esp_eth_handle_t,
        status: &mut Status,
        event: &EthEvent,
    ) -> Result<bool, EspError> {
        if driver_handle == event.handle() as _ {
            info!("Got eth event: {:?} ", event);

            *status = match event {
                EthEvent::Stopped(_) => Status::Stopped,
                EthEvent::Started(_) => Status::Started,
                EthEvent::Connected(_) => Status::Connected,
                EthEvent::Disconnected(_) => Status::Started,
            };

            info!("Eth event {:?} handled, set status: {:?}", event, status);

            Ok(true)
        } else {
            Ok(false)
        }
    }
}

impl<B, R> EventBus<()> for EthStatus<B, R>
where
    B: Borrow<R>,
    R: RawHandle<Handle = esp_eth_handle_t>,
{
    type Subscription = (EspSubscription<System>, EspSubscription<System>);

    fn subscribe(
        &self,
        callback: impl for<'a> FnMut(&'a ()) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        let handle = unsafe { self.driver.handle() };
        let waitable = self.waitable.clone();
        let mut last_up = self.is_up();

        let subscription = self.sys_loop.subscribe(move |event: &EthEvent| {
            let notify = {
                if handle == event.handle() {
                    let guard = waitable.state.lock();

                    if last_up != *guard {
                        last_up = *guard;

                        true
                    } else {
                        false
                    }
                } else {
                    false
                }
            };

            if notify {
                callback(&());
            }
        })?;

        Ok(subscription)
    }
}

#[cfg(all(feature = "nightly", feature = "experimental"))]
mod asyncify {
    use embedded_svc::utils::asyncify::{event_bus::AsyncEventBus, Asyncify};

    impl<'d, P> Asyncify for super::EthDriver<'d, P> {
        type AsyncWrapper<S> = AsyncEventBus<(), crate::private::mutex::RawCondvar, S>;
    }
}
