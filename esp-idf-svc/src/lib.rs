//! This crate contains wrappers which are mostly implementations of the
//! abstractions defined in the [embedded-svc](../embedded_svc/index.html)
//! project. It has features such as wifi, networking, httpd, logging.
//!
//! ## Features
//!
//! This crates specifies a few cargo features, including:
//!
//! - `std`: Enable the use of std. Enabled by default.
//! - `experimental`: Enable the use of experimental features.
//! - `embassy-time-driver`
//! - `embassy-time-isr-queue`
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(cfg_version)]
#![cfg_attr(feature = "nightly", feature(type_alias_impl_trait))]
#![cfg_attr(
    all(feature = "nightly", version("1.70")),
    feature(impl_trait_in_assoc_type)
)]
#![allow(clippy::unused_unit)] // enumset

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
pub mod errors;
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
pub mod hal;
pub mod handle;
#[cfg(feature = "alloc")]
pub mod http;
#[cfg(all(feature = "std", esp_idf_comp_esp_http_server_enabled))]
pub mod httpd;
#[cfg(feature = "alloc")]
pub mod log;
#[cfg(all(feature = "alloc", esp_idf_comp_mdns_enabled))]
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
#[cfg(feature = "alloc")]
pub mod notify;
#[cfg(all(feature = "alloc", esp_idf_comp_nvs_flash_enabled))]
pub mod nvs;
#[cfg(all(esp_idf_comp_app_update_enabled, esp_idf_comp_spi_flash_enabled))]
pub mod ota;
#[cfg(esp_idf_comp_esp_netif_enabled)]
pub mod ping;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_netif_enabled))]
pub mod sntp;
pub mod sys;
pub mod systime;
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
