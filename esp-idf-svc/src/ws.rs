//! WebSocket protocol

#[cfg(all(feature = "alloc", esp_idf_version_major = "4", esp_idf_ws_transport))]
pub mod client;
