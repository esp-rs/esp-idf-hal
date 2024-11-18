//! This crate contains wrappers which are mostly implementations of the
//! abstractions defined in the [embedded-svc](../embedded_svc/index.html)
//! project. It has features such as wifi, networking, httpd, logging.
//!
//! ## Features
//!
//! This crate specifies a few cargo features, including:
//!
//! - `std`: Enable the use of std. Enabled by default.
//! - `experimental`: Enable the use of experimental features.
//! - `embassy-time-driver`: Implement an embassy time driver.
#![no_std]
#![allow(async_fn_in_trait)]
#![allow(unknown_lints)]
#![allow(renamed_and_removed_lints)]
#![allow(unexpected_cfgs)]
#![warn(clippy::large_futures)]

#[cfg(feature = "std")]
#[allow(unused_imports)]
#[macro_use]
extern crate std;

#[cfg(feature = "alloc")]
#[allow(unused_imports)]
#[macro_use]
extern crate alloc;

#[cfg(not(esp32s2))]
#[cfg(all(
    esp_idf_bt_enabled,
    esp_idf_bt_bluedroid_enabled,
    feature = "alloc",
    feature = "experimental"
))]
pub mod bt;
#[cfg(all(
    not(esp32h2),
    feature = "alloc",
    esp_idf_comp_esp_wifi_enabled,
    esp_idf_comp_esp_event_enabled,
))]
pub mod espnow;
#[cfg(all(
    feature = "alloc",
    esp_idf_comp_esp_eth_enabled,
    esp_idf_comp_esp_event_enabled,
))]
#[cfg(any(
    all(esp32, esp_idf_eth_use_esp32_emac),
    any(
        esp_idf_eth_spi_ethernet_dm9051,
        esp_idf_eth_spi_ethernet_w5500,
        esp_idf_eth_spi_ethernet_ksz8851snl
    ),
    esp_idf_eth_use_openeth
))]
pub mod eth;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_event_enabled))]
pub mod eventloop;
#[cfg(feature = "experimental")]
pub mod fs;
pub mod hal;
pub mod handle;
#[cfg(feature = "alloc")]
pub mod http;
pub mod io;
pub mod ipv4;
#[cfg(feature = "alloc")]
pub mod log;
#[cfg(all(
    feature = "alloc",
    any(esp_idf_comp_mdns_enabled, esp_idf_comp_espressif__mdns_enabled)
))]
pub mod mdns;
#[cfg(all(
    feature = "alloc",
    esp_idf_comp_mqtt_enabled,
    esp_idf_comp_esp_event_enabled
))]
pub mod mqtt;
#[cfg(esp_idf_lwip_ipv4_napt)]
pub mod napt;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_netif_enabled))]
pub mod netif;
#[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
pub mod nvs;
#[cfg(all(
    esp_idf_comp_app_update_enabled,
    any(esp_idf_comp_spi_flash_enabled, esp_idf_comp_esp_partition_enabled)
))]
pub mod ota;
#[cfg(all(
    feature = "experimental",
    any(esp_idf_comp_spi_flash_enabled, esp_idf_comp_esp_partition_enabled)
))]
pub mod partition;
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub mod ping;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_netif_enabled))]
pub mod sntp;
pub mod sys;
pub mod systime;
#[cfg(all(
    feature = "alloc",
    feature = "experimental",
    esp_idf_comp_openthread_enabled,
    esp_idf_openthread_enabled,
    esp_idf_comp_esp_event_enabled,
    esp_idf_comp_nvs_flash_enabled,
    esp_idf_comp_vfs_enabled,
))]
pub mod thread;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub mod timer;
pub mod tls;
#[cfg(all(
    not(esp32h2),
    feature = "alloc",
    esp_idf_comp_esp_wifi_enabled,
    esp_idf_comp_esp_event_enabled,
))]
pub mod wifi;
pub mod ws;

mod private;
