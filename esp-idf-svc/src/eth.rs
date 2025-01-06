use core::fmt::Debug;
use core::marker::PhantomData;
use core::time::Duration;
use core::{ffi, ops, ptr};

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;

use embedded_svc::eth::*;

use crate::hal::peripheral::Peripheral;

#[cfg(any(
    all(esp32, esp_idf_eth_use_esp32_emac),
    any(
        esp_idf_eth_spi_ethernet_dm9051,
        esp_idf_eth_spi_ethernet_w5500,
        esp_idf_eth_spi_ethernet_ksz8851snl
    )
))]
use crate::hal::gpio;
#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
use crate::hal::{spi, units::Hertz};

use crate::sys::*;

use crate::eventloop::{
    EspEventDeserializer, EspEventLoop, EspEventSource, EspSubscription, EspSystemEventLoop, System,
};
use crate::handle::RawHandle;
#[cfg(esp_idf_comp_esp_netif_enabled)]
use crate::netif::*;
use crate::private::*;

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
pub enum RmiiClockConfig<GPIO0, GPIO16, GPIO17>
where
    GPIO0: Peripheral<P = gpio::Gpio0>,
    GPIO16: Peripheral<P = gpio::Gpio16>,
    GPIO17: Peripheral<P = gpio::Gpio17>,
{
    Input(GPIO0),
    OutputGpio0(GPIO0),
    /// This according to ESP-IDF is for "testing" only    
    OutputGpio16(GPIO16),
    OutputInvertedGpio17(GPIO17),
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<GPIO0, GPIO16, GPIO17> RmiiClockConfig<GPIO0, GPIO16, GPIO17>
where
    GPIO0: Peripheral<P = gpio::Gpio0>,
    GPIO16: Peripheral<P = gpio::Gpio16>,
    GPIO17: Peripheral<P = gpio::Gpio17>,
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

/// Source/mechanism to use for getting notifications/events from the emac.
///
/// # Availability
///
/// Pre version `v5.1.4` of esp-idf, only an interrupt pin could be used as source:
///
/// - v4.4: <https://github.com/espressif/esp-idf/blob/e499576efdb086551abe309a72899302f82077b7/components/esp_eth/include/esp_eth_mac.h#L461-L464>
/// - v5.0: <https://github.com/espressif/esp-idf/blob/bcca689866db3dfda47f77670bf8df2a7ec94721/components/esp_eth/include/esp_eth_mac.h#L513-L517>
/// - v5.1.3: <https://github.com/espressif/esp-idf/blob/e7771c75bd1dbbfb7b3c5381be7e063b197c9734/components/esp_eth/include/esp_eth_mac.h#L612-L617>
/// - V5.2.0: <https://github.com/espressif/esp-idf/blob/11eaf41b37267ad7709c0899c284e3683d2f0b5e/components/esp_eth/include/esp_eth_mac.h#L612-L617>
///
/// Starting with `v5.1.4`, `v5.2.1` and `>= v5.3` the option of `poll_period_ms` became available:
/// - v5.1.4: <https://github.com/espressif/esp-idf/blob/d7b0a45ddbddbac53afb4fc28168f9f9259dbb79/components/esp_eth/include/esp_eth_mac.h#L614-L620>
/// - v5.2.1: <https://github.com/espressif/esp-idf/blob/a322e6bdad4b6675d4597fb2722eea2851ba88cb/components/esp_eth/include/esp_eth_mac.h#L614-L620>
/// - v5.3-dev: <https://github.com/espressif/esp-idf/blob/ea010f84ef878dda07146244e166930738c1c103/components/esp_eth/include/esp_eth_mac.h#L694-L700>
#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SpiEventSource<'d> {
    #[cfg(not(any(
        esp_idf_version_major = "4",
        all(
            esp_idf_version_major = "5",
            any(
                esp_idf_version_minor = "0",
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
            )
        ),
    )))]
    pub(crate) poll_interval_ms: u32,
    pub(crate) interrupt_pin: i32,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(any(
    esp_idf_eth_spi_ethernet_dm9051,
    esp_idf_eth_spi_ethernet_w5500,
    esp_idf_eth_spi_ethernet_ksz8851snl
))]
impl<'d> SpiEventSource<'d> {
    /// Instead of getting informed by an interrupt pin about updates/changes from the emac, the
    /// MCU polls the emac periodically for updates.
    ///
    /// In most cases, [`Self::polling`] should be used as it is more efficient.
    /// But this source makes e.g. sense if the interrupt pin of the emac is not connected to the MCU.
    #[cfg(not(any(
        esp_idf_version_major = "4",
        all(
            esp_idf_version_major = "5",
            any(
                esp_idf_version_minor = "0",
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
            )
        ),
    )))]
    pub fn polling(interval: Duration) -> Result<Self, core::num::TryFromIntError> {
        let poll_interval_ms = interval.as_millis().try_into()?;

        Ok(Self {
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(
                    esp_idf_version_major = "5",
                    any(
                        esp_idf_version_minor = "0",
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                        all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                    )
                ),
            )))]
            poll_interval_ms,
            interrupt_pin: -1,
            _p: PhantomData,
        })
    }

    /// Get status updates/changes from the emac by way of an interrupt pin.
    ///
    /// If the interrupt pin is not connected, see [`Self::polling`] for an alternative.
    pub fn interrupt(pin: impl Peripheral<P = impl gpio::InputPin> + 'd) -> Self {
        crate::hal::into_ref!(pin);

        Self {
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(
                    esp_idf_version_major = "5",
                    any(
                        esp_idf_version_minor = "0",
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                        all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                    )
                ),
            )))]
            poll_interval_ms: 0,
            interrupt_pin: pin.pin(),
            _p: PhantomData,
        }
    }
}

type RawCallback<'a> = Box<dyn FnMut(EthFrame) + Send + 'a>;

struct UnsafeCallback<'a>(*mut RawCallback<'a>);

impl<'a> UnsafeCallback<'a> {
    #[allow(clippy::type_complexity)]
    fn from(boxed: &mut Box<RawCallback<'a>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut ffi::c_void) -> Self {
        Self(ptr.cast())
    }

    fn as_ptr(&self) -> *mut ffi::c_void {
        self.0.cast()
    }

    unsafe fn call(&self, data: EthFrame) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum Status {
    Stopped,
    Started,
    Connected,
    Disconnected,
}

pub struct RmiiEth;

pub struct OpenEth;

pub struct SpiEth<T> {
    _driver: T,
    device: Option<spi_device_handle_t>,
}

impl<T> Drop for SpiEth<T> {
    fn drop(&mut self) {
        if let Some(device) = self.device {
            esp!(unsafe { spi_bus_remove_device(device) }).unwrap();

            ::log::info!("SpiEth dropped");
        }
    }
}

/// This struct provides a safe wrapper over the ESP IDF Ethernet C driver.
///
/// The driver works on Layer 2 (Data Link) in the OSI model, in that it provides
/// facilities for sending and receiving ethernet packets over the built-in
/// RMII interface of `esp32` and/or via a dedicated SPI ethernet peripheral for all
/// other MCUs.
///
/// For most use cases, utilizing `EspEth` - which provides a networking (IP)
/// layer as well - should be preferred. Using `EthDriver` directly is beneficial
/// only when one would like to utilize a custom, non-STD network stack like `smoltcp`.
pub struct EthDriver<'d, T> {
    _flavor: T,
    handle: esp_eth_handle_t,
    status: Arc<mutex::Mutex<Status>>,
    _subscription: EspSubscription<'static, System>,
    callback: Option<Box<RawCallback<'d>>>,
    _p: PhantomData<&'d mut ()>,
}

#[cfg(all(esp32, esp_idf_eth_use_esp32_emac))]
impl<'d> EthDriver<'d, RmiiEth> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        mac: impl Peripheral<P = crate::hal::mac::MAC> + 'd,
        rmii_rdx0: impl Peripheral<P = gpio::Gpio25> + 'd,
        rmii_rdx1: impl Peripheral<P = gpio::Gpio26> + 'd,
        rmii_crs_dv: impl Peripheral<P = gpio::Gpio27> + 'd,
        rmii_mdc: impl Peripheral<P = impl gpio::OutputPin> + 'd,
        rmii_txd1: impl Peripheral<P = gpio::Gpio22> + 'd,
        rmii_tx_en: impl Peripheral<P = gpio::Gpio21> + 'd,
        rmii_txd0: impl Peripheral<P = gpio::Gpio19> + 'd,
        rmii_mdio: impl Peripheral<P = impl gpio::InputPin + gpio::OutputPin> + 'd,
        rmii_ref_clk_config: RmiiClockConfig<
            impl Peripheral<P = gpio::Gpio0> + 'd,
            impl Peripheral<P = gpio::Gpio16> + 'd,
            impl Peripheral<P = gpio::Gpio17> + 'd,
        >,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: RmiiEthChipset,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        Self::new_rmii(
            mac,
            rmii_rdx0,
            rmii_rdx1,
            rmii_crs_dv,
            rmii_mdc,
            rmii_txd1,
            rmii_tx_en,
            rmii_txd0,
            rmii_mdio,
            rmii_ref_clk_config,
            rst,
            chipset,
            phy_addr,
            sysloop,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_rmii(
        _mac: impl Peripheral<P = crate::hal::mac::MAC> + 'd,
        _rmii_rdx0: impl Peripheral<P = gpio::Gpio25> + 'd,
        _rmii_rdx1: impl Peripheral<P = gpio::Gpio26> + 'd,
        _rmii_crs_dv: impl Peripheral<P = gpio::Gpio27> + 'd,
        rmii_mdc: impl Peripheral<P = impl gpio::OutputPin> + 'd,
        _rmii_txd1: impl Peripheral<P = gpio::Gpio22> + 'd,
        _rmii_tx_en: impl Peripheral<P = gpio::Gpio21> + 'd,
        _rmii_txd0: impl Peripheral<P = gpio::Gpio19> + 'd,
        rmii_mdio: impl Peripheral<P = impl gpio::InputPin + gpio::OutputPin> + 'd,
        rmii_ref_clk_config: RmiiClockConfig<
            impl Peripheral<P = gpio::Gpio0> + 'd,
            impl Peripheral<P = gpio::Gpio16> + 'd,
            impl Peripheral<P = gpio::Gpio17> + 'd,
        >,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: RmiiEthChipset,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        crate::hal::into_ref!(rmii_mdc, rmii_mdio);

        let rst = rst.map(|rst| rst.into_ref().pin());

        let eth = Self::init(
            Self::rmii_mac(rmii_mdc.pin(), rmii_mdio.pin(), &rmii_ref_clk_config),
            Self::rmii_phy(chipset, rst, phy_addr)?,
            None,
            RmiiEth {},
            sysloop,
        )?;

        Ok(eth)
    }

    fn rmii_phy(
        chipset: RmiiEthChipset,
        reset: Option<i32>,
        phy_addr: Option<u32>,
    ) -> Result<*mut esp_eth_phy_t, EspError> {
        let phy_cfg = Self::eth_phy_default_config(reset, phy_addr);

        let phy = match chipset {
            RmiiEthChipset::IP101 => unsafe { esp_eth_phy_new_ip101(&phy_cfg) },
            RmiiEthChipset::RTL8201 => unsafe { esp_eth_phy_new_rtl8201(&phy_cfg) },
            RmiiEthChipset::LAN87XX => unsafe { esp_eth_phy_new_lan87xx(&phy_cfg) },
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
        mdc: i32,
        mdio: i32,
        clk_config: &RmiiClockConfig<
            impl Peripheral<P = gpio::Gpio0> + 'd,
            impl Peripheral<P = gpio::Gpio16> + 'd,
            impl Peripheral<P = gpio::Gpio17> + 'd,
        >,
    ) -> *mut esp_eth_mac_t {
        #[cfg(esp_idf_version_major = "4")]
        let mac = {
            let mut config = Self::eth_mac_default_config(mdc, mdio);

            config.clock_config = clk_config.eth_mac_clock_config();

            unsafe { esp_eth_mac_new_esp32(&config) }
        };

        #[cfg(not(esp_idf_version_major = "4"))]
        let mac = {
            let mut esp32_config = Self::eth_esp32_emac_default_config(mdc, mdio);
            esp32_config.clock_config = clk_config.eth_mac_clock_config();

            let config = Self::eth_mac_default_config(mdc, mdio);

            unsafe { esp_eth_mac_new_esp32(&esp32_config, &config) }
        };

        mac
    }

    #[cfg(any(
        esp_idf_version = "5.0",
        esp_idf_version = "5.1",
        esp_idf_version = "5.2"
    ))]
    fn eth_esp32_emac_default_config(mdc: i32, mdio: i32) -> eth_esp32_emac_config_t {
        eth_esp32_emac_config_t {
            smi_mdc_gpio_num: mdc,
            smi_mdio_gpio_num: mdio,
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            ..Default::default()
        }
    }

    #[cfg(esp_idf_version = "5.3")]
    fn eth_esp32_emac_default_config(mdc: i32, mdio: i32) -> eth_esp32_emac_config_t {
        eth_esp32_emac_config_t {
            __bindgen_anon_1: eth_esp32_emac_config_t__bindgen_ty_1 {
                __bindgen_anon_1: eth_esp32_emac_config_t__bindgen_ty_1__bindgen_ty_1 {
                    smi_mdc_gpio_num: mdc,
                    smi_mdio_gpio_num: mdio,
                },
            },
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            ..Default::default()
        }
    }

    #[cfg(not(any(
        esp_idf_version_major = "4",
        esp_idf_version = "5.0",
        esp_idf_version = "5.1",
        esp_idf_version = "5.2",
        esp_idf_version = "5.3"
    )))]
    fn eth_esp32_emac_default_config(mdc: i32, mdio: i32) -> eth_esp32_emac_config_t {
        eth_esp32_emac_config_t {
            __bindgen_anon_1: eth_esp32_emac_config_t__bindgen_ty_1 {
                smi_gpio: emac_esp_smi_gpio_config_t {
                    mdc_num: mdc,
                    mdio_num: mdio,
                },
            },
            interface: eth_data_interface_t_EMAC_DATA_INTERFACE_RMII,
            ..Default::default()
        }
    }
}

#[cfg(esp_idf_eth_use_openeth)]
impl<'d> EthDriver<'d, OpenEth> {
    pub fn new(
        mac: impl Peripheral<P = crate::hal::mac::MAC> + 'd,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        Self::new_openeth(mac, sysloop)
    }

    pub fn new_openeth(
        _mac: impl Peripheral<P = crate::hal::mac::MAC> + 'd,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        let eth = Self::init(
            unsafe { esp_eth_mac_new_openeth(&Self::eth_mac_default_config(0, 0)) },
            unsafe { esp_eth_phy_new_dp83848(&Self::eth_phy_default_config(None, None)) },
            None,
            OpenEth {},
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
impl<'d, T> EthDriver<'d, SpiEth<T>>
where
    T: core::borrow::Borrow<spi::SpiDriver<'d>>,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        driver: T,
        int: impl Peripheral<P = impl gpio::InputPin> + 'd,
        cs: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        Self::new_spi(
            driver, int, cs, rst, chipset, baudrate, mac_addr, phy_addr, sysloop,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_spi(
        driver: T,
        int: impl Peripheral<P = impl gpio::InputPin> + 'd,
        cs: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        Self::new_spi_with_event_source(
            driver,
            SpiEventSource::interrupt(int),
            cs,
            rst,
            chipset,
            baudrate,
            mac_addr,
            phy_addr,
            sysloop,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_spi_with_event_source(
        driver: T,
        event_source: SpiEventSource<'d>,
        cs: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        rst: Option<impl Peripheral<P = impl gpio::OutputPin> + 'd>,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        mac_addr: Option<&[u8; 6]>,
        phy_addr: Option<u32>,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        let (mac, phy, device) = Self::init_spi(
            driver.borrow().host(),
            chipset,
            baudrate,
            event_source.interrupt_pin,
            #[cfg(not(any(
                esp_idf_version_major = "4",
                all(
                    esp_idf_version_major = "5",
                    any(
                        esp_idf_version_minor = "0",
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                        all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                        all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                    )
                ),
            )))]
            event_source.poll_interval_ms,
            cs.map(|pin| pin.into_ref().pin()),
            rst.map(|pin| pin.into_ref().pin()),
            phy_addr,
        )?;

        let eth = Self::init(
            mac,
            phy,
            mac_addr,
            SpiEth {
                _driver: driver,
                device,
            },
            sysloop,
        )?;

        Ok(eth)
    }

    #[allow(clippy::unnecessary_literal_unwrap, clippy::too_many_arguments)]
    fn init_spi(
        host: spi_host_device_t,
        chipset: SpiEthChipset,
        baudrate: Hertz,
        int_gpio_num: i32,
        #[cfg(not(any(
            esp_idf_version_major = "4",
            all(
                esp_idf_version_major = "5",
                any(
                    esp_idf_version_minor = "0",
                    all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                    all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                    all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                    all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                    all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                )
            ),
        )))]
        poll_period_ms: u32,
        cs: Option<i32>,
        rst: Option<i32>,
        phy_addr: Option<u32>,
    ) -> Result<
        (
            *mut esp_eth_mac_t,
            *mut esp_eth_phy_t,
            Option<spi_device_handle_t>,
        ),
        EspError,
    > {
        crate::hal::gpio::enable_isr_service()?;

        let mac_cfg = Self::eth_mac_default_config(0, 0);
        let phy_cfg = Self::eth_phy_default_config(rst, phy_addr);

        let (mac, phy, spi_handle) = match chipset {
            #[cfg(esp_idf_eth_spi_ethernet_dm9051)]
            SpiEthChipset::DM9051 => {
                let spi_devcfg = Self::get_spi_conf(cs, 1, 7, baudrate);

                #[cfg(esp_idf_version_major = "4")]
                let spi_handle = Some(Self::init_spi_device(host, &spi_devcfg)?);

                #[cfg(not(esp_idf_version_major = "4"))]
                let spi_handle = None;

                #[cfg(esp_idf_version_major = "4")]
                let dm9051_cfg = eth_dm9051_config_t {
                    spi_hdl: spi_handle.unwrap() as *mut _,
                    int_gpio_num,
                };

                #[cfg(not(esp_idf_version_major = "4"))]
                #[allow(clippy::needless_update)]
                let dm9051_cfg = eth_dm9051_config_t {
                    spi_host_id: host,
                    spi_devcfg: &spi_devcfg as *const _ as *mut _,
                    int_gpio_num,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(
                                esp_idf_version_minor = "0",
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                                all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                            )
                        ),
                    )))]
                    poll_period_ms,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(esp_idf_version_minor = "0", esp_idf_version_minor = "1")
                        ),
                    )))]
                    custom_spi_driver: eth_spi_custom_driver_config_t::default(),
                    ..Default::default()
                };

                let mac = unsafe { esp_eth_mac_new_dm9051(&dm9051_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_dm9051(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_w5500)]
            SpiEthChipset::W5500 => {
                let spi_devcfg = Self::get_spi_conf(cs, 16, 8, baudrate);

                #[cfg(esp_idf_version_major = "4")]
                let spi_handle = Some(Self::init_spi_device(host, &spi_devcfg)?);

                #[cfg(not(esp_idf_version_major = "4"))]
                let spi_handle = None;

                #[cfg(esp_idf_version_major = "4")]
                let w5500_cfg = eth_w5500_config_t {
                    spi_hdl: spi_handle.unwrap() as *mut _,
                    int_gpio_num,
                };

                #[cfg(not(esp_idf_version_major = "4"))]
                #[allow(clippy::needless_update)]
                let w5500_cfg = eth_w5500_config_t {
                    spi_host_id: host,
                    spi_devcfg: &spi_devcfg as *const _ as *mut _,
                    int_gpio_num,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(
                                esp_idf_version_minor = "0",
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                                all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                            )
                        ),
                    )))]
                    poll_period_ms,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(esp_idf_version_minor = "0", esp_idf_version_minor = "1")
                        ),
                    )))]
                    custom_spi_driver: eth_spi_custom_driver_config_t::default(),
                    ..Default::default()
                };

                let mac = unsafe { esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_w5500(&phy_cfg) };

                (mac, phy, spi_handle)
            }
            #[cfg(esp_idf_eth_spi_ethernet_ksz8851snl)]
            SpiEthChipset::KSZ8851SNL => {
                let spi_devcfg = Self::get_spi_conf(cs, 0, 0, baudrate);

                #[cfg(esp_idf_version_major = "4")]
                let spi_handle = Some(Self::init_spi_device(host, &spi_devcfg)?);

                #[cfg(not(esp_idf_version_major = "4"))]
                let spi_handle = None;

                #[cfg(esp_idf_version_major = "4")]
                let ksz8851snl_cfg = eth_ksz8851snl_config_t {
                    spi_hdl: spi_handle.unwrap() as *mut _,
                    int_gpio_num,
                };

                #[cfg(not(esp_idf_version_major = "4"))]
                #[allow(clippy::needless_update)]
                let ksz8851snl_cfg = eth_ksz8851snl_config_t {
                    spi_host_id: host,
                    spi_devcfg: &spi_devcfg as *const _ as *mut _,
                    int_gpio_num,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(
                                esp_idf_version_minor = "0",
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "1"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "2"),
                                all(esp_idf_version_minor = "1", esp_idf_version_patch = "3"),
                                all(esp_idf_version_minor = "2", esp_idf_version_patch = "0"),
                            )
                        ),
                    )))]
                    poll_period_ms,
                    #[cfg(not(any(
                        esp_idf_version_major = "4",
                        all(
                            esp_idf_version_major = "5",
                            any(esp_idf_version_minor = "0", esp_idf_version_minor = "1")
                        ),
                    )))]
                    custom_spi_driver: eth_spi_custom_driver_config_t::default(),
                    ..Default::default()
                };

                let mac = unsafe { esp_eth_mac_new_ksz8851snl(&ksz8851snl_cfg, &mac_cfg) };
                let phy = unsafe { esp_eth_phy_new_ksz8851snl(&phy_cfg) };

                (mac, phy, spi_handle)
            }
        };

        Ok((mac, phy, spi_handle))
    }

    fn get_spi_conf(
        cs: Option<i32>,
        command_bits: u8,
        address_bits: u8,
        baudrate: Hertz,
    ) -> spi_device_interface_config_t {
        spi_device_interface_config_t {
            command_bits,
            address_bits,
            mode: 0,
            clock_speed_hz: baudrate.0 as i32,
            spics_io_num: cs.unwrap_or(-1),
            queue_size: 20,
            ..Default::default()
        }
    }

    #[cfg(esp_idf_version_major = "4")]
    fn init_spi_device(
        host: spi_host_device_t,
        conf: &spi_device_interface_config_t,
    ) -> Result<spi_device_handle_t, EspError> {
        let mut spi_handle: spi_device_handle_t = ptr::null_mut();

        esp!(unsafe { spi_bus_add_device(host, conf, &mut spi_handle) })?;

        Ok(spi_handle)
    }
}

impl<'d, T> EthDriver<'d, T> {
    fn init(
        mac: *mut esp_eth_mac_t,
        phy: *mut esp_eth_phy_t,
        mac_addr: Option<&[u8; 6]>,
        flavor: T,
        sysloop: EspSystemEventLoop,
    ) -> Result<Self, EspError> {
        let cfg = Self::eth_default_config(mac, phy);

        let mut handle: esp_eth_handle_t = ptr::null_mut();
        esp!(unsafe { esp_eth_driver_install(&cfg, &mut handle) })?;

        ::log::info!("Driver initialized");

        if let Some(mac_addr) = mac_addr {
            esp!(unsafe {
                esp_eth_ioctl(
                    handle,
                    esp_eth_io_cmd_t_ETH_CMD_S_MAC_ADDR,
                    mac_addr.as_ptr() as *mut _,
                )
            })?;

            ::log::info!("Attached MAC address: {:?}", mac_addr);
        }

        let (waitable, subscription) = Self::subscribe(handle, &sysloop)?;

        let eth = Self {
            handle,
            _flavor: flavor,
            status: waitable,
            _subscription: subscription,
            callback: None,
            _p: PhantomData,
        };

        ::log::info!("Initialization complete");

        Ok(eth)
    }

    fn subscribe(
        handle: esp_eth_handle_t,
        sysloop: &EspEventLoop<System>,
    ) -> Result<(Arc<mutex::Mutex<Status>>, EspSubscription<'static, System>), EspError> {
        let status = Arc::new(mutex::Mutex::new(Status::Stopped));
        let s_status = status.clone();

        let handle = handle as usize;

        let subscription = sysloop.subscribe::<EthEvent, _>(move |event| {
            if event.is_for_handle(handle as _) {
                let mut guard = s_status.lock();

                match event {
                    EthEvent::Started(_) => *guard = Status::Started,
                    EthEvent::Stopped(_) => *guard = Status::Stopped,
                    EthEvent::Connected(_) => *guard = Status::Connected,
                    EthEvent::Disconnected(_) => *guard = Status::Disconnected,
                }
            }
        })?;

        Ok((status, subscription))
    }

    pub fn is_started(&self) -> Result<bool, EspError> {
        let guard = self.status.lock();

        Ok(*guard == Status::Started
            || *guard == Status::Connected
            || *guard == Status::Disconnected)
    }

    pub fn is_connected(&self) -> Result<bool, EspError> {
        let guard = self.status.lock();

        Ok(*guard == Status::Connected)
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        esp!(unsafe { esp_eth_start(self.handle) })?;

        ::log::info!("Start requested");

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), EspError> {
        ::log::info!("Stopping");

        let err = unsafe { esp_eth_stop(self.handle) };
        if err != ESP_ERR_INVALID_STATE {
            esp!(err)?;
        }

        ::log::info!("Stop requested");

        Ok(())
    }

    pub fn set_rx_callback<F>(&mut self, callback: F) -> Result<(), EspError>
    where
        F: FnMut(EthFrame) + Send + 'static,
    {
        self.internal_set_rx_callback(callback)
    }

    /// # Safety
    ///
    /// This method - in contrast to method `set_rx_callback` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    ///   scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent to a hidden ESP IDF thread.
    /// This means that if the service is forgotten, Rust is free to e.g. unwind the stack
    /// and the closure now owned by this other thread will end up with references to variables that no longer exist.
    ///
    /// The destructor of the service takes care - prior to the service being dropped and e.g.
    /// the stack being unwind - to remove the closure from the hidden thread and destroy it.
    /// Unfortunately, when the service is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    pub unsafe fn set_nonstatic_rx_callback<F>(&mut self, callback: F) -> Result<(), EspError>
    where
        F: FnMut(EthFrame) + Send + 'd,
    {
        self.internal_set_rx_callback(callback)
    }

    fn internal_set_rx_callback<F>(&mut self, callback: F) -> Result<(), EspError>
    where
        F: FnMut(EthFrame) + Send + 'd,
    {
        let _ = self.stop();

        let mut callback: Box<RawCallback> = Box::new(Box::new(callback));

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        esp!(unsafe {
            esp_eth_update_input_path(self.handle(), Some(Self::handle), unsafe_callback.as_ptr())
        })?;

        self.callback = Some(callback);

        Ok(())
    }

    pub fn send(&mut self, frame: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_eth_transmit(self.handle(), frame.as_ptr() as *mut _, frame.len() as _)
        })?;

        Ok(())
    }

    unsafe extern "C" fn handle(
        _handle: esp_eth_handle_t,
        buf: *mut u8,
        len: u32,
        event_handler_arg: *mut ffi::c_void,
    ) -> esp_err_t {
        UnsafeCallback::from_ptr(event_handler_arg as *mut _).call(EthFrame::new(buf, len));

        ESP_OK
    }

    fn clear_all(&mut self) -> Result<(), EspError> {
        let _ = self.stop(); // Driver might be stopped already

        unsafe {
            esp!(esp_eth_driver_uninstall(self.handle))?;
        }

        ::log::info!("Driver deinitialized");

        Ok(())
    }

    /// Enables or disables promiscuous mode for the [`EthDriver`].
    ///
    /// When promiscuous mode is enabled, the driver captures all Ethernet frames
    /// on the network, regardless of their destination MAC address. This is useful for
    /// debugging or monitoring purposes.
    pub fn set_promiscuous(&mut self, state: bool) -> Result<(), EspError> {
        esp!(unsafe {
            esp_eth_ioctl(
                self.handle(),
                esp_eth_io_cmd_t_ETH_CMD_S_PROMISCUOUS,
                &raw const state as *mut _,
            )
        })?;

        if state {
            ::log::info!("Driver set in promiscuous mode");
        } else {
            ::log::info!("Driver set in non-promiscuous mode");
        }

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

    #[allow(clippy::needless_update)]
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
            rx_task_stack_size: 4096,
            rx_task_prio: 15,
            flags: 0,
        }
    }
}

impl<T> Eth for EthDriver<'_, T> {
    type Error = EspError;

    fn start(&mut self) -> Result<(), Self::Error> {
        EthDriver::start(self)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        EthDriver::stop(self)
    }

    fn is_started(&self) -> Result<bool, Self::Error> {
        EthDriver::is_started(self)
    }

    fn is_connected(&self) -> Result<bool, Self::Error> {
        EthDriver::is_connected(self)
    }
}

unsafe impl<T> Send for EthDriver<'_, T> {}

impl<T> Drop for EthDriver<'_, T> {
    fn drop(&mut self) {
        self.clear_all().unwrap();

        ::log::info!("EthDriver dropped");
    }
}

impl<T> RawHandle for EthDriver<'_, T> {
    type Handle = esp_eth_handle_t;

    fn handle(&self) -> Self::Handle {
        self.handle
    }
}

pub struct EthFrame {
    buf: *mut u8,
    len: u32,
}

unsafe impl Send for EthFrame {}

impl EthFrame {
    const unsafe fn new(buf: *mut u8, len: u32) -> Self {
        Self { buf, len }
    }

    pub const fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.buf, self.len as _) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.buf, self.len as _) }
    }
}

impl ops::Deref for EthFrame {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.buf, self.len as _) }
    }
}

impl ops::DerefMut for EthFrame {
    fn deref_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.buf, self.len as _) }
    }
}

impl Drop for EthFrame {
    fn drop(&mut self) {
        unsafe { free(self.buf.cast()) };
    }
}

/// `EspEth` wraps an `EthDriver` Data Link layer instance, and binds the OSI
/// Layer 3 (network) facilities of ESP IDF to it.
///
/// In other words, it connects the ESP IDF ethernet Netif interface to the
/// ethernet driver. This allows users to utilize the Rust STD APIs for working with
/// TCP and UDP sockets.
///
/// This struct should be the default option for an ethernet driver in all use cases
/// but the niche one where bypassing the ESP IDF Netif and lwIP stacks is
/// desirable. E.g., using `smoltcp` or other custom IP stacks on top of the
/// ESP IDF ethernet peripheral.
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub struct EspEth<'d, T> {
    glue_handle: *mut esp_eth_netif_glue_t,
    netif: EspNetif,
    driver: EthDriver<'d, T>,
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<'d, T> EspEth<'d, T> {
    pub fn wrap(driver: EthDriver<'d, T>) -> Result<Self, EspError> {
        Self::wrap_all(driver, EspNetif::new(NetifStack::Eth)?)
    }

    pub fn wrap_all(driver: EthDriver<'d, T>, netif: EspNetif) -> Result<Self, EspError> {
        let mut this = Self {
            driver,
            netif,
            glue_handle: core::ptr::null_mut(),
        };

        this.attach_netif()?;

        Ok(this)
    }

    pub fn swap_netif(&mut self, netif: EspNetif) -> Result<EspNetif, EspError> {
        self.detach_netif()?;

        let old_netif = core::mem::replace(&mut self.netif, netif);

        self.attach_netif()?;

        Ok(old_netif)
    }

    pub fn driver(&self) -> &EthDriver<'d, T> {
        &self.driver
    }

    pub fn driver_mut(&mut self) -> &mut EthDriver<'d, T> {
        &mut self.driver
    }

    pub fn netif(&self) -> &EspNetif {
        &self.netif
    }

    pub fn netif_mut(&mut self) -> &mut EspNetif {
        &mut self.netif
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        self.driver_mut().start()
    }

    pub fn stop(&mut self) -> Result<(), EspError> {
        self.driver_mut().stop()
    }

    pub fn is_started(&self) -> Result<bool, EspError> {
        self.driver().is_started()
    }

    pub fn is_connected(&self) -> Result<bool, EspError> {
        self.driver().is_connected()
    }

    pub fn is_up(&self) -> Result<bool, EspError> {
        Ok(self.is_connected()? && self.netif().is_up()?)
    }

    fn attach_netif(&mut self) -> Result<(), EspError> {
        let _ = self.driver.stop();

        let glue_handle = unsafe { esp_eth_new_netif_glue(self.driver.handle()) };

        esp!(unsafe { esp_netif_attach(self.netif.handle(), glue_handle as *mut _) })?;

        self.glue_handle = glue_handle;

        Ok(())
    }

    fn detach_netif(&mut self) -> Result<(), EspError> {
        let _ = self.driver.stop();

        esp!(unsafe { esp_eth_del_netif_glue(self.glue_handle as *mut _) })?;

        self.glue_handle = core::ptr::null_mut();

        Ok(())
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> Drop for EspEth<'_, T> {
    fn drop(&mut self) {
        self.detach_netif().unwrap();

        ::log::info!("EspEth dropped");
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
unsafe impl<T> Send for EspEth<'_, T> {}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> RawHandle for EspEth<'_, T> {
    type Handle = *mut esp_eth_netif_glue_t;

    fn handle(&self) -> Self::Handle {
        self.glue_handle
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> Eth for EspEth<'_, T> {
    type Error = EspError;

    fn start(&mut self) -> Result<(), Self::Error> {
        EspEth::start(self)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        EspEth::stop(self)
    }

    fn is_started(&self) -> Result<bool, Self::Error> {
        EspEth::is_started(self)
    }

    fn is_connected(&self) -> Result<bool, Self::Error> {
        EspEth::is_connected(self)
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> NetifStatus for EspEth<'_, T> {
    fn is_up(&self) -> Result<bool, EspError> {
        EspEth::is_up(self)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum EthEvent {
    Started(esp_eth_handle_t),
    Stopped(esp_eth_handle_t),
    Connected(esp_eth_handle_t),
    Disconnected(esp_eth_handle_t),
}

unsafe impl Send for EthEvent {}

impl EthEvent {
    pub fn is_for(&self, raw_handle: impl RawHandle<Handle = esp_eth_handle_t>) -> bool {
        self.is_for_handle(raw_handle.handle())
    }

    pub fn is_for_handle(&self, handle: esp_eth_handle_t) -> bool {
        self.handle() == handle
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

unsafe impl EspEventSource for EthEvent {
    fn source() -> Option<&'static ffi::CStr> {
        Some(unsafe { ffi::CStr::from_ptr(ETH_EVENT) })
    }
}

impl EspEventDeserializer for EthEvent {
    type Data<'a> = Self;

    #[allow(non_upper_case_globals, non_snake_case)]
    fn deserialize(data: &crate::eventloop::EspEvent) -> Self {
        let eth_handle_ref =
            unsafe { (data.payload.unwrap() as *const _ as *const esp_eth_handle_t).as_ref() };

        let event_id = data.event_id as u32;

        if event_id == eth_event_t_ETHERNET_EVENT_START {
            EthEvent::Started(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_STOP {
            EthEvent::Stopped(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_CONNECTED {
            EthEvent::Connected(*eth_handle_ref.unwrap() as _)
        } else if event_id == eth_event_t_ETHERNET_EVENT_DISCONNECTED {
            EthEvent::Disconnected(*eth_handle_ref.unwrap() as _)
        } else {
            panic!("Unknown event ID: {}", event_id);
        }
    }
}

const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);

pub struct BlockingEth<T> {
    eth: T,
    event_loop: crate::eventloop::EspSystemEventLoop,
}

impl<T> BlockingEth<T>
where
    T: Eth<Error = EspError>,
{
    pub fn wrap(eth: T, event_loop: EspSystemEventLoop) -> Result<Self, EspError> {
        Ok(Self { eth, event_loop })
    }

    pub fn eth(&self) -> &T {
        &self.eth
    }

    pub fn eth_mut(&mut self) -> &mut T {
        &mut self.eth
    }

    pub fn is_started(&self) -> Result<bool, EspError> {
        self.eth.is_started()
    }

    pub fn start(&mut self) -> Result<(), EspError> {
        self.eth.start()?;
        self.eth_wait_while(|| self.eth.is_started().map(|s| !s), None)
    }

    pub fn stop(&mut self) -> Result<(), EspError> {
        self.eth.stop()?;
        self.eth_wait_while(|| self.eth.is_started(), None)
    }

    pub fn is_connected(&self) -> Result<bool, EspError> {
        self.eth.is_connected()
    }

    pub fn wait_connected(&self) -> Result<(), EspError> {
        self.eth_wait_while(
            || self.eth.is_connected().map(|s| !s),
            Some(CONNECT_TIMEOUT),
        )
    }

    pub fn eth_wait_while<F: Fn() -> Result<bool, EspError>>(
        &self,
        matcher: F,
        timeout: Option<Duration>,
    ) -> Result<(), EspError> {
        let wait = crate::eventloop::Wait::new::<EthEvent>(&self.event_loop)?;

        wait.wait_while(matcher, timeout)
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> BlockingEth<T>
where
    T: NetifStatus,
{
    pub fn is_up(&self) -> Result<bool, EspError> {
        self.eth.is_up()
    }

    pub fn wait_netif_up(&self) -> Result<(), EspError> {
        self.ip_wait_while(|| self.eth.is_up().map(|s| !s), Some(CONNECT_TIMEOUT))
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

impl<T> Eth for BlockingEth<T>
where
    T: Eth<Error = EspError>,
{
    type Error = EspError;

    fn is_started(&self) -> Result<bool, Self::Error> {
        BlockingEth::is_started(self)
    }

    fn is_connected(&self) -> Result<bool, Self::Error> {
        BlockingEth::is_connected(self)
    }

    fn start(&mut self) -> Result<(), Self::Error> {
        BlockingEth::start(self)
    }

    fn stop(&mut self) -> Result<(), Self::Error> {
        BlockingEth::stop(self)
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> NetifStatus for BlockingEth<T>
where
    T: NetifStatus,
{
    fn is_up(&self) -> Result<bool, EspError> {
        BlockingEth::is_up(self)
    }
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub struct AsyncEth<T> {
    pub(crate) eth: T,
    pub(crate) event_loop: crate::eventloop::EspSystemEventLoop,
    pub(crate) timer_service: crate::timer::EspTaskTimerService,
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
impl<T> AsyncEth<T>
where
    T: Eth<Error = EspError>,
{
    pub fn wrap(
        eth: T,
        event_loop: EspSystemEventLoop,
        timer_service: crate::timer::EspTaskTimerService,
    ) -> Result<Self, EspError> {
        Ok(Self {
            eth,
            event_loop,
            timer_service,
        })
    }

    pub fn eth(&self) -> &T {
        &self.eth
    }

    pub fn eth_mut(&mut self) -> &mut T {
        &mut self.eth
    }

    pub fn is_started(&self) -> Result<bool, EspError> {
        self.eth.is_started()
    }

    pub fn is_connected(&self) -> Result<bool, EspError> {
        self.eth.is_connected()
    }

    pub async fn start(&mut self) -> Result<(), EspError> {
        self.eth.start()?;
        self.eth_wait_while(|this| this.eth.is_started().map(|s| !s), None)
            .await
    }

    pub async fn stop(&mut self) -> Result<(), EspError> {
        self.eth.stop()?;
        self.eth_wait_while(|this| this.eth.is_started(), None)
            .await
    }

    pub async fn wait_connected(&mut self) -> Result<(), EspError> {
        self.eth_wait_while(
            |this| this.eth.is_connected().map(|s| !s),
            Some(CONNECT_TIMEOUT),
        )
        .await
    }

    pub async fn eth_wait_while<F: FnMut(&mut Self) -> Result<bool, EspError>>(
        &mut self,
        mut matcher: F,
        timeout: Option<Duration>,
    ) -> Result<(), EspError> {
        let mut wait =
            crate::eventloop::AsyncWait::<EthEvent, _>::new(&self.event_loop, &self.timer_service)?;

        wait.wait_while(|| matcher(self), timeout).await
    }
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> AsyncEth<T>
where
    T: NetifStatus,
{
    pub fn is_up(&self) -> Result<bool, EspError> {
        self.eth.is_up()
    }

    pub async fn wait_netif_up(&mut self) -> Result<(), EspError> {
        self.ip_wait_while(|this| this.eth.is_up().map(|s| !s), Some(CONNECT_TIMEOUT))
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

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
impl<T> embedded_svc::eth::asynch::Eth for AsyncEth<T>
where
    T: Eth<Error = EspError>,
{
    type Error = T::Error;

    async fn start(&mut self) -> Result<(), Self::Error> {
        AsyncEth::start(self).await
    }

    async fn stop(&mut self) -> Result<(), Self::Error> {
        AsyncEth::stop(self).await
    }

    async fn is_started(&self) -> Result<bool, Self::Error> {
        AsyncEth::is_started(self)
    }

    async fn is_connected(&self) -> Result<bool, Self::Error> {
        AsyncEth::is_connected(self)
    }
}

#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> crate::netif::asynch::NetifStatus for EspEth<'_, T> {
    async fn is_up(&self) -> Result<bool, EspError> {
        EspEth::is_up(self)
    }
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
#[cfg(esp_idf_comp_esp_netif_enabled)]
impl<T> crate::netif::asynch::NetifStatus for AsyncEth<T>
where
    T: NetifStatus,
{
    async fn is_up(&self) -> Result<bool, EspError> {
        AsyncEth::is_up(self)
    }
}
