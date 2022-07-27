use std::path::PathBuf;

use anyhow::Result;
use embuild::cargo;
use serde::Deserialize;

use crate::common::{workspace_dir, InstallDir, InstallDirLocation};

pub const ESP_IDF_TOOLS_INSTALL_DIR_VAR: &str = "ESP_IDF_TOOLS_INSTALL_DIR";

pub const DEFAULT_TOOLS_INSTALL_DIR: InstallDirLocation = InstallDirLocation::Workspace;
pub const ESP_IDF_GLOB_VAR_PREFIX: &str = "ESP_IDF_GLOB";

pub const DEFAULT_SDKCONFIG_FILE: &str = "sdkconfig";
pub const DEFAULT_SDKCONFIG_DEFAULTS_FILE: &str = "sdkconfig.defaults";

#[derive(Debug, Deserialize, Default, Clone)]
#[serde(default)]
pub struct BuildConfig {
    /// The install location for the esp-idf and its toolchain.
    esp_idf_tools_install_dir: Option<InstallDir>,

    /// A path to the sdkconfig used by the esp-idf.
    esp_idf_sdkconfig: Option<PathBuf>,

    /// One or more paths to sdkconfig.defaults files used by the esp-idf.
    #[serde(deserialize_with = "parse::sdkconfig_defaults")]
    esp_idf_sdkconfig_defaults: Option<Vec<PathBuf>>,

    /// The MCU (esp32, esp32s2, esp32s3, esp32c3, ...) to compile for if unset will be
    /// detected from the cargo target triple.
    pub mcu: Option<String>,

    #[cfg(feature = "native")]
    /// Additional configurations for the native builder.
    #[serde(skip)]
    pub native: crate::native::cargo_driver::config::NativeConfig,
}

impl BuildConfig {
    /// Parse the build configuration from the environment variables.
    ///
    /// Note: The environment variables to deserialize must be valid rust [`String`]s
    /// (can only contain utf-8).
    pub fn try_from_env() -> Result<BuildConfig> {
        for var in utils::serde_introspect::<BuildConfig>() {
            cargo::track_env_var(var.to_uppercase());
        }

        let cfg: BuildConfig = envy::from_env()?;

        #[cfg(feature = "native")]
        let cfg = {
            use crate::native::cargo_driver::config::NativeConfig;
            for var in utils::serde_introspect::<NativeConfig>() {
                cargo::track_env_var(var.to_uppercase());
            }

            BuildConfig {
                native: envy::from_env()?,
                ..cfg
            }
        };

        Ok(cfg)
    }

    /// Print the build configuration to stderr.
    pub fn print(&self) {
        eprintln!("Build configuration: {:#?}", self);
    }

    /// Get the [`InstallDir`] and whether it was **not** specified by the user.
    pub fn esp_idf_tools_install_dir(&self) -> Result<(InstallDir, bool)> {
        match self.esp_idf_tools_install_dir.clone() {
            Some(val) => Ok((val, false)),
            None => InstallDir::try_from(None).map(|v| (v, true)),
        }
    }

    /// Get the user-specified path to the esp-idf sdkconfig or [`DEFAULT_SDKCONFIG_FILE`]
    /// if unset.
    pub fn esp_idf_sdkconfig(&self) -> PathBuf {
        self.esp_idf_sdkconfig
            .clone()
            .unwrap_or_else(|| DEFAULT_SDKCONFIG_FILE.into())
    }

    /// Get a list of user-specified paths to sdkconfig.defaults files or
    /// [`DEFAULT_SDKCONFIG_DEFAULTS_FILE`] if unset.
    pub fn esp_idf_sdkconfig_defaults(&self) -> Vec<PathBuf> {
        self.esp_idf_sdkconfig_defaults
            .clone()
            .unwrap_or_else(|| vec![DEFAULT_SDKCONFIG_DEFAULTS_FILE.into()])
    }

    pub fn from_cargo_metadata() -> Result<BuildConfig> {
        let metadata = cargo_metadata::MetadataCommand::new()
            .current_dir(workspace_dir()?)
            .other_options(vec!["--frozen".into(), "--offline".into()])
            .exec()?;

        todo!()
    }
}

mod parse {
    use std::path::PathBuf;

    use serde::{Deserialize, Deserializer};

    pub fn sdkconfig_defaults<'d, D: Deserializer<'d>>(
        de: D,
    ) -> Result<Option<Vec<PathBuf>>, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum StringOrVec {
            Str(String),
            Vec(Vec<PathBuf>),
        }

        Option::<StringOrVec>::deserialize(de).map(|val| match val {
            Some(StringOrVec::Str(s)) => Some(
                s.split(';')
                    .filter(|s| !s.is_empty())
                    .map(PathBuf::from)
                    .collect(),
            ),
            Some(StringOrVec::Vec(v)) => Some(v),
            None => None,
        })
    }
}

pub mod utils {
    use serde::de::{self, Deserialize, Deserializer, Visitor};

    /// Gets the serialization names for structs.
    ///
    /// Taken from <https://github.com/vityafx/serde-aux/blob/c6f8482f51da7f187ecea62931c8f38edcf355c9/src/serde_introspection.rs>.
    ///
    /// Note: Doesn't work with `#[serde(flatten)]` attributes.
    pub fn serde_introspect<'de, T>() -> &'static [&'static str]
    where
        T: Deserialize<'de>,
    {
        struct StructFieldsDeserializer<'a> {
            fields: &'a mut Option<&'static [&'static str]>,
        }

        impl<'de, 'a> Deserializer<'de> for StructFieldsDeserializer<'a> {
            type Error = serde::de::value::Error;

            fn deserialize_any<V>(self, _visitor: V) -> Result<V::Value, Self::Error>
            where
                V: Visitor<'de>,
            {
                Err(de::Error::custom("I'm just here for the fields"))
            }

            fn deserialize_struct<V>(
                self,
                _name: &'static str,
                fields: &'static [&'static str],
                _visitor: V,
            ) -> Result<V::Value, Self::Error>
            where
                V: Visitor<'de>,
            {
                *self.fields = Some(fields); // get the names of the deserialized fields
                Err(de::Error::custom("I'm just here for the fields"))
            }

            serde::forward_to_deserialize_any! {
                bool i8 i16 i32 i64 u8 u16 u32 u64 f32 f64 char str string bytes
                byte_buf option unit unit_struct newtype_struct seq tuple
                tuple_struct map enum identifier ignored_any
            }
        }

        let mut serialized_names = None;
        let _ = T::deserialize(StructFieldsDeserializer {
            fields: &mut serialized_names,
        });
        serialized_names.unwrap_or_default()
    }
}
