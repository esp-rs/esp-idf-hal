#![cfg_attr(not(feature = "std"), no_std)]
#![feature(cfg_version)]
#![cfg_attr(
    all(feature = "nightly", not(version("1.65"))),
    feature(generic_associated_types)
)]
#![cfg_attr(feature = "nightly", feature(type_alias_impl_trait))]
#![cfg_attr(not(version("1.66")), feature(const_btree_new))]

#[cfg(any(feature = "alloc"))]
#[allow(unused_imports)]
#[macro_use]
extern crate alloc;

pub mod errors;
#[cfg(all(
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
pub mod handle;
#[cfg(all(feature = "experimental", feature = "alloc"))]
pub mod http;
#[cfg(all(feature = "std", esp_idf_comp_esp_http_server_enabled))]
pub mod httpd;
pub mod log;
#[cfg(feature = "alloc")]
pub mod mdns;
#[cfg(all(
    feature = "alloc",
    esp_idf_comp_mqtt_enabled,
    esp_idf_comp_esp_event_enabled
))]
pub mod mqtt;
#[cfg(esp_idf_lwip_ipv4_napt)]
pub mod napt;
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub mod netif;
#[cfg(all(feature = "experimental", feature = "alloc"))]
pub mod notify;
#[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
pub mod nvs;
#[cfg(all(
    feature = "experimental",
    esp_idf_comp_app_update_enabled,
    esp_idf_comp_spi_flash_enabled
))]
pub mod ota;
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub mod ping;
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub mod sntp;
pub mod systime;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub mod timer;
#[cfg(all(
    feature = "alloc",
    esp_idf_comp_esp_wifi_enabled,
    esp_idf_comp_esp_event_enabled,
))]
pub mod wifi;
pub mod ws;

mod private;
