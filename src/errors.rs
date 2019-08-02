use esp_idf_sys::{
    esp_err_t, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, ESP_ERR_INVALID_MAC,
    ESP_ERR_INVALID_RESPONSE, ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_STATE, ESP_ERR_INVALID_VERSION,
    ESP_ERR_NOT_FOUND, ESP_ERR_NOT_SUPPORTED, ESP_ERR_NO_MEM, ESP_ERR_TIMEOUT, ESP_OK,
};

pub struct EspError(pub esp_err_t);

impl EspError {
    pub fn into_result(self) -> Result<()> {
        Result::from(self)
    }
}

// TODO: implement Error
#[derive(Clone, Copy, Debug)]
pub enum Error {
    /// Out of memory
    NoMem,
    /// Invalid argument
    InvalidArg,
    /// Invalid state
    InvalidState,
    /// Invalid size
    InvalidSize,
    /// Requested resource not found
    NotFound,
    /// Operation or feature not supported
    NotSupported,
    /// Operation timed out
    Timeout,
    /// Received response was invalid
    InvalidResponse,
    /// CRC or checksum was invalid
    InvalidCrc,
    /// Version was invalid
    InvalidVersion,
    /// MAC address was invalid
    InvalidMac,

    Other(esp_err_t),
}

pub type Result<T, E = Error> = core::result::Result<T, E>;

impl From<EspError> for Result<()> {
    fn from(value: EspError) -> Self {
        use Error::*;

        Err(match value.0 as u32 {
            ESP_OK => return Ok(()),
            ESP_ERR_NO_MEM => NoMem,
            ESP_ERR_INVALID_ARG => InvalidArg,
            ESP_ERR_INVALID_STATE => InvalidState,
            ESP_ERR_INVALID_SIZE => InvalidSize,
            ESP_ERR_NOT_FOUND => NotFound,
            ESP_ERR_NOT_SUPPORTED => NotSupported,
            ESP_ERR_TIMEOUT => Timeout,
            ESP_ERR_INVALID_RESPONSE => InvalidResponse,
            ESP_ERR_INVALID_CRC => InvalidCrc,
            ESP_ERR_INVALID_VERSION => InvalidVersion,
            ESP_ERR_INVALID_MAC => InvalidMac,
            _ => Other(value.0),
        })
    }
}
