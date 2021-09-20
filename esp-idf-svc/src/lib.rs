#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(feature = "experimental", feature(generic_associated_types))] // for http, http::client, http::server, ota
#![feature(const_btree_new)]

#[cfg(any(feature = "alloc"))]
#[macro_use]
extern crate alloc;

#[cfg(all(feature = "experimental", feature = "alloc"))]
pub mod http;
#[cfg(any(feature = "std"))] // TODO: Lower requirements to "alloc"
pub mod httpd;
#[cfg(any(feature = "alloc"))]
// TODO: Ideally should not need "alloc" (also for performance reasons)
pub mod log;
#[cfg(esp_idf_config_lwip_ipv4_napt)]
pub mod napt;
pub mod netif;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs_storage;
#[cfg(all(feature = "experimental", feature = "alloc"))]
pub mod ota;
pub mod ping;
pub mod sysloop;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod wifi;

#[cfg(any(feature = "binstart", feature = "libstart"))]
pub mod start;

mod private;
