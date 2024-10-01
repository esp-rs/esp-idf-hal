//! Example of a Thread Border Router.
//!
//! This example only works on MCUs that do have Wifi capabilities, as follows:
//! - On MCUs with both native `Thread` as well as `Wifi` capabilities, the example will run in co-exist mode, on a single MCU;
//! - On MCUs with only `Wifi` capabilities, the example will run in UART mode, so you need to flash the `thread_rcp` example
//!   on a separate MCU which does have native `Thread` capabilities, and connect the two via UART.
//!
//! NOTE NOTE NOTE:
//! To build, you need to put the following in your `sdkconfig.defaults`:
//! ```text
//! CONFIG_OPENTHREAD_ENABLED=y
//!
//! # Thread Border Router
//! CONFIG_OPENTHREAD_BORDER_ROUTER=y
//!
//! # These are also necessary for the Joiner feature
//! CONFIG_MBEDTLS_CMAC_C=y
//! CONFIG_MBEDTLS_SSL_PROTO_DTLS=y
//! CONFIG_MBEDTLS_KEY_EXCHANGE_ECJPAKE=y
//! CONFIG_MBEDTLS_ECJPAKE_C=y
//!
//! # Border Router again, lwIP
//! CONFIG_LWIP_IPV6_NUM_ADDRESSES=12
//! CONFIG_LWIP_NETIF_STATUS_CALLBACK=y
//! CONFIG_LWIP_IPV6_FORWARD=y
//! CONFIG_LWIP_MULTICAST_PING=y
//! CONFIG_LWIP_NETIF_STATUS_CALLBACK=y
//! CONFIG_LWIP_HOOK_IP6_ROUTE_DEFAULT=y
//! CONFIG_LWIP_HOOK_ND6_GET_GW_DEFAULT=y
//! CONFIG_LWIP_HOOK_IP6_INPUT_CUSTOM=y
//! CONFIG_LWIP_HOOK_IP6_SELECT_SRC_ADDR_CUSTOM=y
//! CONFIG_LWIP_IPV6_AUTOCONFIG=y
//! CONFIG_LWIP_TCPIP_TASK_STACK_SIZE=4096
//!
//! # Border Router again, mDNS
//! CONFIG_MDNS_MULTIPLE_INSTANCE=y
//! ```
//!
//! And also the following in your `Cargo.toml`:
//! ```toml
//! [[package.metadata.esp-idf-sys.extra_components]]
//! remote_component = { name = "espressif/mdns", version = "1.2" }
//! ```

#![allow(unexpected_cfgs)]

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    #[cfg(any(esp32h2, esp32h4))]
    {
        log::error!("This example only works on MCUs which do have Wifi support.");
    }

    #[cfg(not(any(esp32h2, esp32h4)))]
    {
        #[cfg(i_have_done_all_configs_from_the_top_comment)]
        // Remove this `cfg` when you have done all of the above for the example to compile
        example::main()?;

        // Remove this whole code block when you have done all of the above for the example to compile
        #[cfg(not(i_have_done_all_configs_from_the_top_comment))]
        {
            log::error!("Please follow the instructions in the source code.");
        }
    }

    Ok(())
}

#[cfg(i_have_done_all_configs_from_the_top_comment)] // Remove this `cfg` when you have done all of the above for the example to compile
#[cfg(not(any(esp32h2, esp32h4)))]
mod example {
    use core::convert::TryInto;

    use std::sync::Arc;

    use log::info;

    use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};

    use esp_idf_svc::eventloop::EspSystemSubscription;
    use esp_idf_svc::hal::prelude::Peripherals;
    use esp_idf_svc::io::vfs::MountedEventfs;
    use esp_idf_svc::thread::{EspThread, ThreadEvent};
    use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
    use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};

    const SSID: &str = env!("WIFI_SSID");
    const PASSWORD: &str = env!("WIFI_PASS");

    pub fn main() -> anyhow::Result<()> {
        let peripherals = Peripherals::take()?;
        let sys_loop = EspSystemEventLoop::take()?;
        let nvs = EspDefaultNvsPartition::take()?;

        #[cfg(esp32c6)]
        let (wifi_modem, _thread_modem) = { peripherals.modem.split() };

        #[cfg(not(esp32c6))]
        let (wifi_modem, _thread_modem) = { (peripherals.modem, ()) };

        let mounted_event_fs = Arc::new(MountedEventfs::mount(6)?);

        let mut wifi = BlockingWifi::wrap(
            EspWifi::new(wifi_modem, sys_loop.clone(), Some(nvs.clone()))?,
            sys_loop.clone(),
        )?;

        connect_wifi(&mut wifi)?;

        let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

        info!("Wifi DHCP info: {:?}", ip_info);

        info!("Initializing Thread Border Router...");

        let _subscription = log_thread_sysloop(sys_loop.clone())?;

        // On the C6, run the Thread Border Router in co-exist mode
        #[cfg(esp32c6)]
        let mut thread = EspThread::new_br(
            _thread_modem,
            sys_loop,
            nvs,
            mounted_event_fs,
            wifi.wifi().sta_netif(),
        )?;

        // On all other chips, run the Thread Border Router in UART mode
        #[cfg(not(esp32c6))]
        let mut thread = EspThread::new_br_uart(
            peripherals.uart1,
            peripherals.pins.gpio2,
            peripherals.pins.gpio3,
            &esp_idf_svc::thread::config::uart_default_cfg(),
            sys_loop,
            nvs,
            mounted_event_fs,
            wifi.wifi().sta_netif(),
        )?;

        thread.init()?;

        #[cfg(esp32c6)]
        thread.init_coex()?;

        thread.set_tod_from_cfg()?;

        info!("Thread Border Router initialized, now running...");

        thread.run()?;

        Ok(())
    }

    fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
        let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
            ssid: SSID.try_into().unwrap(),
            bssid: None,
            auth_method: AuthMethod::WPA2Personal,
            password: PASSWORD.try_into().unwrap(),
            channel: None,
            ..Default::default()
        });

        wifi.set_configuration(&wifi_configuration)?;

        wifi.start()?;
        info!("Wifi started");

        wifi.connect()?;
        info!("Wifi connected");

        wifi.wait_netif_up()?;
        info!("Wifi netif up");

        Ok(())
    }

    fn log_thread_sysloop(
        sys_loop: EspSystemEventLoop,
    ) -> Result<EspSystemSubscription<'static>, anyhow::Error> {
        let subscription = sys_loop.subscribe::<ThreadEvent, _>(|event| {
            info!("Got: {:?}", event);
        })?;

        Ok(subscription)
    }
}
