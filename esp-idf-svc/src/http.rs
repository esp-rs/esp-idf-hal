//! Experimental HTTP server and client
//!
//! Note: This module requires the `experimental` cargo feature to be enabled.
#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_client_enabled))]
pub mod client;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_server_enabled))]
pub mod server;
