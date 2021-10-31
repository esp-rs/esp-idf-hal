#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(feature = "experimental", feature(generic_associated_types))] // for http, http::client, http::server, ota
#![feature(const_btree_new)]

#[cfg(any(feature = "alloc"))]
#[macro_use]
extern crate alloc;

#[cfg(feature = "alloc")]
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
#[cfg(all(feature = "experimental", feature = "alloc"))]
pub mod http;
#[cfg(all(feature = "std", esp_idf_comp_esp_http_server_enabled))]
// TODO: Lower requirements to "alloc"
pub mod httpd;
#[cfg(feature = "alloc")]
// TODO: Ideally should not need "alloc" (also for performance reasons)
pub mod log;
#[cfg(esp_idf_config_lwip_ipv4_napt)]
pub mod napt;
#[cfg(feature = "alloc")]
pub mod netif;
#[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
// TODO: Expose a subset which does not require "alloc"
pub mod nvs;
#[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
// TODO: Expose a subset which does not require "alloc"
pub mod nvs_storage;
#[cfg(all(
    feature = "experimental",
    feature = "alloc",
    esp_idf_comp_app_update_enabled,
    esp_idf_comp_spi_flash_enabled
))]
pub mod ota;
pub mod ping;
pub mod sysloop;
#[cfg(feature = "alloc")] // TODO: Expose a subset which does not require "alloc"
pub mod wifi;

mod private;
