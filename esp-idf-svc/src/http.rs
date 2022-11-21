#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_client_enabled))]
pub mod client;
#[cfg(all(feature = "alloc", esp_idf_comp_esp_http_server_enabled))]
pub mod server;
