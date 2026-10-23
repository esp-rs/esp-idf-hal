//! This example demonstrates how to configure an RMII based Ethernet adapter
//!
//! To use it, you need an RMII-capable Espressif MCU, like the original ESP32 chip, the ESP32-P4
//! or the ESP32-S31. The ESP32-P4 pins are those of the ESP32-P4-Function-EV-Board; the ESP32-S31
//! ones are an example only.
//!
//! Note: On ESP-IDF v6.0+, specific RMII PHY drivers were moved out of the main tree.
//! This example uses `RmiiEthChipset::Generic` which is always available. To use a
//! specific PHY, add the corresponding component to your `Cargo.toml`, e.g.:
//! ```toml
//! [[package.metadata.esp-idf-sys.extra_components]]
//! remote_component = { name = "espressif/lan87xx", version = "1.*" }
//! ```
//! Other available PHY components: `espressif/ip101`, `espressif/dp83848`,
//! `espressif/rtl8201`, `espressif/ksz80xx`.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

#[cfg(any(esp32, esp32p4, esp32s31))]
use esp_idf_svc::{
    eth::{BlockingEth, EspEth, EthDriver},
    eventloop::EspSystemEventLoop,
    hal::peripherals::Peripherals,
    log::EspLogger,
};
#[cfg(any(esp32, esp32p4, esp32s31))]
use log::info;

#[cfg(any(esp32, esp32p4, esp32s31))]
fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let sys_loop = EspSystemEventLoop::take()?;

    // Make sure to configure ethernet in sdkconfig and adjust the parameters below for your hardware
    #[cfg(esp32)]
    let eth_driver = EthDriver::new_rmii(
        peripherals.mac,
        pins.gpio25,
        pins.gpio26,
        pins.gpio27,
        pins.gpio23,
        pins.gpio22,
        pins.gpio21,
        pins.gpio19,
        pins.gpio18,
        esp_idf_svc::eth::RmiiClockConfig::OutputInvertedGpio17(pins.gpio17),
        Some(pins.gpio5),
        // Replace with a specific PHY variant (e.g. LAN87XX, IP101) if you have it available.
        // On ESP-IDF v6.0+ these require external components (see note at top of file).
        // Generic is available since ESP-IDF v5.4; use LAN87XX for older versions.
        #[cfg(esp_idf_version_at_least_5_4_0)]
        esp_idf_svc::eth::RmiiEthChipset::Generic,
        #[cfg(not(esp_idf_version_at_least_5_4_0))]
        esp_idf_svc::eth::RmiiEthChipset::LAN87XX,
        Some(0),
        &esp_idf_svc::eth::EthConfiguration::new(),
        sys_loop.clone(),
    )?;

    #[cfg(esp32p4)]
    let eth_driver = EthDriver::new_rmii(
        peripherals.mac,
        pins.gpio29,
        pins.gpio30,
        pins.gpio28,
        pins.gpio31,
        pins.gpio35,
        pins.gpio49,
        pins.gpio34,
        pins.gpio52,
        // The PHY outputs the reference clock
        esp_idf_svc::eth::RmiiClockConfig::Input(pins.gpio50.degrade_input()),
        Some(pins.gpio51),
        // The board has an IP101; see the note at the top of the file for using its driver
        #[cfg(esp_idf_version_at_least_5_4_0)]
        esp_idf_svc::eth::RmiiEthChipset::Generic,
        #[cfg(not(esp_idf_version_at_least_5_4_0))]
        esp_idf_svc::eth::RmiiEthChipset::IP101,
        Some(1),
        &esp_idf_svc::eth::EthConfiguration::new(),
        sys_loop.clone(),
    )?;

    // Adjust these pins and the PHY to your board: the RMII ones can only go to the
    // pins the EMAC can be routed to (see the ESP32-S31 datasheet)
    #[cfg(esp32s31)]
    let eth_driver = EthDriver::new_rmii(
        peripherals.mac,
        pins.gpio19,
        pins.gpio18,
        pins.gpio15,
        pins.gpio5,
        pins.gpio9,
        pins.gpio12,
        pins.gpio8,
        pins.gpio6,
        esp_idf_svc::eth::RmiiClockConfig::Input(pins.gpio13.degrade_input()),
        Some(pins.gpio7),
        esp_idf_svc::eth::RmiiEthChipset::Generic,
        Some(1),
        &esp_idf_svc::eth::EthConfiguration::new(),
        sys_loop.clone(),
    )?;
    let eth = EspEth::wrap(eth_driver)?;

    info!("Eth created");

    let mut eth = BlockingEth::wrap(eth, sys_loop.clone())?;

    info!("Starting eth...");

    eth.start()?;

    info!("Waiting for DHCP lease...");

    eth.wait_netif_up()?;

    let ip_info = eth.eth().netif().get_ip_info()?;

    info!("Eth DHCP info: {ip_info:?}");

    Ok(())
}

#[cfg(not(any(esp32, esp32p4, esp32s31)))]
fn main() {
    use esp_idf_svc::{self as _};

    panic!("This example is configured for esp32, esp32p4 and esp32s31, please adjust pins to your module");
}
