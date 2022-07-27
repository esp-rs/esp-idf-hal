//! ESP32 chip variants support.

use anyhow::{bail, Result};
use strum::{Display, EnumString};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Display, EnumString)]
#[repr(u32)]
pub enum Chip {
    /// Xtensa LX7 based dual core
    #[strum(serialize = "esp32")]
    ESP32 = 0,
    /// Xtensa LX7 based single core
    #[strum(serialize = "esp32s2")]
    ESP32S2,
    /// Xtensa LX7 based single core
    #[strum(serialize = "esp32s3")]
    ESP32S3,
    /// RISC-V based single core
    #[strum(serialize = "esp32c3")]
    ESP32C3,
}

impl Chip {
    pub fn detect(rust_target_triple: &str) -> Result<Chip> {
        if rust_target_triple.starts_with("xtensa-esp") {
            if rust_target_triple.contains("esp32s3") {
                return Ok(Chip::ESP32S3);
            } else if rust_target_triple.contains("esp32s2") {
                return Ok(Chip::ESP32S2);
            } else {
                return Ok(Chip::ESP32);
            }
        } else if rust_target_triple.starts_with("riscv32imc-esp") {
            return Ok(Chip::ESP32C3);
        }
        bail!("Unsupported target '{}'", rust_target_triple)
    }

    /// The name of the gcc toolchain (to compile the `esp-idf`) for `idf_tools.py`.
    pub fn gcc_toolchain(self) -> &'static str {
        match self {
            Self::ESP32 => "xtensa-esp32-elf",
            Self::ESP32S2 => "xtensa-esp32s2-elf",
            Self::ESP32S3 => "xtensa-esp32s3-elf",
            Self::ESP32C3 => "riscv32-esp-elf",
        }
    }

    /// The name of the gcc toolchain for the ultra low-power co-processor for
    /// `idf_tools.py`.
    pub fn ulp_gcc_toolchain(self) -> Option<&'static str> {
        match self {
            Self::ESP32 => Some("esp32ulp-elf"),
            Self::ESP32S2 | Self::ESP32S3 => Some("esp32s2ulp-elf"),
            _ => None,
        }
    }

    pub fn cmake_toolchain_file(self) -> String {
        format!("toolchain-{}.cmake", self)
    }
}
