//! WebSocket protocol

#[cfg(all(
    feature = "alloc",
    feature = "experimental",
    esp_idf_version_major = "4"
))]
pub mod client;
