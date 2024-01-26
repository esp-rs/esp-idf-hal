//! This example demonstrates how to configure an SPI based Ethernet adapter
//!
//! To be able to use this example, you need to set the following in your sdkconfig.default file:
//! CONFIG_ETH_SPI_ETHERNET_DM9051=y
//! CONFIG_ETH_SPI_ETHERNET_W5500=y
//! CONFIG_ETH_SPI_ETHERNET_KSZ8851SNL=y
//! You only pick one of the three, depending on which chip you have on your board.
//! Also adjust the EspEth::wrap call below to match your board's chip.

fn main() {
    #[cfg(esp32)]
    example::main().unwrap();

    #[cfg(not(esp32))]
    panic!("This example is configured for esp32, please adjust pins to your module");
}

#[cfg(esp32)]
pub mod example {
    use esp_idf_svc::eth;
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::spi;
    use esp_idf_svc::hal::{prelude::Peripherals, units::FromValueType};
    use esp_idf_svc::log::EspLogger;
    use esp_idf_svc::sys::EspError;
    use esp_idf_svc::timer::EspTaskTimerService;
    use esp_idf_svc::{ipv4, ping};

    use log::{info, warn};

    pub fn main() -> anyhow::Result<()> {
        esp_idf_svc::sys::link_patches();
        EspLogger::initialize_default();

        let peripherals = Peripherals::take()?;
        let pins = peripherals.pins;
        let sysloop = EspSystemEventLoop::take()?;
        let timer_service = EspTaskTimerService::new()?;

        let mut eth = eth::EspEth::wrap(eth::EthDriver::new_spi(
            spi::SpiDriver::new(
                peripherals.spi2,
                pins.gpio13,
                pins.gpio12,
                Some(pins.gpio26),
                &spi::SpiDriverConfig::new().dma(spi::Dma::Auto(4096)),
            )?,
            pins.gpio27,
            Some(pins.gpio14),
            Some(pins.gpio25),
            // Replace with DM9051 or KSZ8851SNL if you have some of these variants
            eth::SpiEthChipset::W5500,
            20_u32.MHz().into(),
            Some(&[0x02, 0x00, 0x00, 0x12, 0x34, 0x56]),
            None,
            sysloop.clone(),
        )?)?;

        // Wait for the Eth peripheral and network layer 3 to come up - in an async way because we can
        let ip_info = esp_idf_svc::hal::task::block_on(async {
            let mut eth = eth::AsyncEth::wrap(&mut eth, sysloop.clone(), timer_service)?;

            info!("Starting eth...");

            eth.start().await?;

            info!("Waiting for DHCP lease...");

            eth.wait_netif_up().await?;

            let ip_info = eth.eth().netif().get_ip_info()?;

            info!("Eth DHCP info: {:?}", ip_info);

            Result::<_, EspError>::Ok(ip_info)
        })?;

        ping(ip_info.subnet.gateway)?;

        Ok(())
    }

    fn ping(ip: ipv4::Ipv4Addr) -> Result<(), EspError> {
        info!("About to do some pings for {:?}", ip);

        let ping_summary = ping::EspPing::default().ping(ip, &Default::default())?;
        if ping_summary.transmitted != ping_summary.received {
            warn!("Pinging IP {} resulted in timeouts", ip);
        }

        info!("Pinging done");

        Ok(())
    }
}
