//! WebSocket protocol
//!
//! Websockets support is disabled by default. To enable it, add this to your
//! `sdkconfig.defaults` file:
//!
//! ```
//! CONFIG_HTTPD_WS_SUPPORT=y
//! ```
//!

pub use embedded_svc::ws::{Final, Fragmented, FrameType};

#[cfg(all(
    feature = "alloc",
    esp_idf_comp_tcp_transport_enabled,
    esp_idf_comp_esp_tls_enabled,
    any(
        all(esp_idf_version_major = "4", esp_idf_ws_transport),
        esp_idf_comp_espressif__esp_websocket_client_enabled
    )
))]
pub mod client;
