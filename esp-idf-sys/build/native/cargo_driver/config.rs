use std::path::PathBuf;
use std::str::FromStr;

use anyhow::{anyhow, Result};
use cargo_metadata::{Metadata, Package};
use embuild::espidf::parse_esp_idf_git_ref;
use embuild::{cmake, git};
use serde::Deserialize;
use strum::IntoEnumIterator;

use crate::config::utils::set_when_none;
use crate::config::InnerMetadata;

pub const ESP_IDF_VERSION_VAR: &str = "ESP_IDF_VERSION";
pub const ESP_IDF_REPOSITORY_VAR: &str = "ESP_IDF_REPOSITORY";

pub const DEFAULT_ESP_IDF_VERSION: &str = "v4.4.1";
pub const DEFAULT_CMAKE_GENERATOR: cmake::Generator = {
    // No Ninja builds for linux=aarch64 from Espressif yet
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    {
        cmake::Generator::UnixMakefiles
    }

    #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
    {
        cmake::Generator::Ninja
    }
};

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct NativeConfig {
    /// The version of the esp-idf to use.
    #[serde(deserialize_with = "parse::git_ref")]
    pub esp_idf_version: Option<git::Ref>,
    /// The URL to the git repository of the `esp-idf`.
    pub esp_idf_repository: Option<String>,
    /// The cmake generator to use when cmake builds the esp-idf.
    #[serde(deserialize_with = "parse::cmake_generator")]
    esp_idf_cmake_generator: Option<cmake::Generator>,

    /// The path to the esp-idf repository.
    idf_path: Option<PathBuf>,
}

impl NativeConfig {
    /// Get the user-specified esp-idf version or [`DEFAULT_ESP_IDF_VERSION`] if unset.
    pub fn esp_idf_version(&self) -> git::Ref {
        self.esp_idf_version
            .clone()
            .unwrap_or_else(|| parse_esp_idf_git_ref(DEFAULT_ESP_IDF_VERSION))
    }

    /// Get the user-specified cmake generator or [`DEFAULT_CMAKE_GENERATOR`] if unset.
    pub fn esp_idf_cmake_generator(&self) -> cmake::Generator {
        self.esp_idf_cmake_generator
            .unwrap_or(DEFAULT_CMAKE_GENERATOR)
    }

    pub fn with_cargo_metadata(&mut self, root: &Package, metadata: &Metadata) -> Result<()> {
        let InnerMetadata {
            v:
                NativeConfig {
                    esp_idf_version,
                    esp_idf_repository,
                    esp_idf_cmake_generator,
                    idf_path,
                },
        } = InnerMetadata::deserialize(dbg!(&root.metadata))?;

        set_when_none(&mut self.esp_idf_version, esp_idf_version);
        set_when_none(&mut self.esp_idf_repository, esp_idf_repository);
        set_when_none(&mut self.esp_idf_cmake_generator, esp_idf_cmake_generator);
        set_when_none(&mut self.idf_path, idf_path);

        Ok(())
    }
}

/// Parse a cmake generator, either `default` or one of [`cmake::Generator`].
fn parse_cmake_generator(generator: &str) -> Result<cmake::Generator> {
    let generator = generator.trim().to_lowercase();
    match generator.as_str() {
        "default" => Ok(DEFAULT_CMAKE_GENERATOR),
        other => cmake::Generator::from_str(other).map_err(|_| {
            anyhow!(
                "Invalid CMake generator. Should be either `default`, or one of [{}]",
                cmake::Generator::iter()
                    .map(|e| e.into())
                    .collect::<Vec<&'static str>>()
                    .join(", ")
            )
        }),
    }
}

mod parse {
    use embuild::{cmake, git};
    use serde::{Deserialize, Deserializer};

    pub fn cmake_generator<'d, D: Deserializer<'d>>(
        de: D,
    ) -> Result<Option<cmake::Generator>, D::Error> {
        let gen = Option::<String>::deserialize(de)?;
        match gen.as_deref().map(str::trim) {
            Some(val) if !val.is_empty() => super::parse_cmake_generator(val)
                .map(Some)
                .map_err(serde::de::Error::custom),
            _ => Ok(None),
        }
    }

    pub fn git_ref<'d, D: Deserializer<'d>>(de: D) -> Result<Option<git::Ref>, D::Error> {
        Ok(Option::<String>::deserialize(de)?
            .map(|val| embuild::espidf::parse_esp_idf_git_ref(val.trim())))
    }
}
