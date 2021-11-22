#[cfg(esp_idf_comp_esp_http_client_enabled)]
pub mod client;
#[cfg(all(esp_idf_comp_esp_http_server_enabled, feature = "std"))]
pub mod server;
