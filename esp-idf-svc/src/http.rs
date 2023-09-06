//! HTTP server and client

pub use embedded_svc::http::{headers, status, Method};
pub use embedded_svc::utils::http::{cookies, HeaderSetError, Headers};

#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_client_enabled))]
pub mod client;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_server_enabled))]
pub mod server;
