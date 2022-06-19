#[cfg(esp_idf_comp_esp_http_client_enabled)]
pub mod client;
#[cfg(esp_idf_comp_esp_http_server_enabled)]
pub mod server;
