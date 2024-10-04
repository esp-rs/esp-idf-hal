use std::path::PathBuf;

use anyhow::{anyhow, bail, Context, Result};
use serde::{Deserialize, Deserializer};

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
    #[serde(deserialize_with = "parse::list")]
    esp_idf_sdkconfig_defaults: Option<Vec<PathBuf>>,

    /// The MCU (esp32, esp32s2, esp32s3, esp32c3, ...) to compile for if unset will be
    /// detected from the cargo target triple.
    pub mcu: Option<String>,

    #[cfg(any(feature = "native", not(feature = "pio")))]
    /// Additional configurations for the native builder.
    #[serde(skip)]
    pub native: crate::native::cargo_driver::config::NativeConfig,

    /// The name of the root crate currently compiling for, in the event that the
    /// workspace does not have a root crate.
    pub esp_idf_sys_root_crate: Option<String>,
}

impl BuildConfig {
    /// Parse the build configuration from the environment variables.
    ///
    /// Note: The environment variables to deserialize must be valid rust [`String`]s
    /// (can only contain utf-8).
    pub fn try_from_env() -> Result<BuildConfig> {
        let cfg: BuildConfig = utils::parse_from_env(&[])?;

        #[cfg(any(feature = "native", not(feature = "pio")))]
        let cfg = {
            use crate::native::cargo_driver::config::NativeConfig;
            BuildConfig {
                native: NativeConfig::try_from_env()?,
                ..cfg
            }
        };

        Ok(cfg)
    }

    /// Print the build configuration to stderr.
    pub fn print(&self) {
        eprintln!("Build configuration: {self:#?}");
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

    /// Get the configuration from the `package.metadata.esp-idf-sys` object of the root
    /// crate's manifest, and update all options that are [`None`].
    ///
    /// This has the effect that currently set values (coming from
    /// [`BuildConfig::try_from_env`]) take precedence over config options coming from
    /// cargo metadata, meaning environment variables take precedence over cargo metadata.
    ///
    /// This will execute `cargo metadata` in the [`workspace_dir`] and use the manifest's
    /// metadata of the [root crate], or if `cargo metadata` doesn't give a root crate,
    /// the crate given by the `ESP_IDF_SYS_ROOT_CRATE` environment variable
    /// ([`BuildConfig::esp_idf_sys_root_crate`]).
    ///
    /// [root crate]: https://doc.rust-lang.org/cargo/reference/workspaces.html#root-package
    pub fn with_cargo_metadata(&mut self) -> Result<()> {
        // workaround for https://github.com/esp-rs/esp-idf-sys/issues/260
        let current_target = std::env::var("TARGET")?;
        let filter_string = format!("--filter-platform={}", current_target);

        let metadata = cargo_metadata::MetadataCommand::new()
            .current_dir(workspace_dir()?)
            .other_options(vec!["--locked".into(), filter_string])
            .exec()?;

        let root_package = match (metadata.root_package(), &self.esp_idf_sys_root_crate) {
            (_, Some(pkg_name)) => {
                metadata.workspace_packages()
                    .into_iter().find(|p| &p.name == pkg_name)
                    .ok_or_else(|| anyhow!("the crate given by `ESP_IDF_SYS_ROOT_CRATE` does not exist in this workspace"))? 
            },
            (Some(pkg), _) => pkg,
            (None, None) => bail!("could not identify the root crate and `ESP_IDF_SYS_ROOT_CRATE` not specified")
        };

        // Deserialize the options from the `esp-idf-sys` object.
        let EspIdfSys {
            v:
                BuildConfig {
                    esp_idf_tools_install_dir,
                    esp_idf_sdkconfig,
                    esp_idf_sdkconfig_defaults,
                    mcu,
                    #[cfg(any(feature = "native", not(feature = "pio")))]
                        native: _,
                    esp_idf_sys_root_crate: _,
                },
        } = EspIdfSys::deserialize(&root_package.metadata)?;

        // Update all options that are currently [`None`].
        utils::set_when_none(&mut self.esp_idf_sdkconfig, esp_idf_sdkconfig);
        utils::set_when_none(
            &mut self.esp_idf_sdkconfig_defaults,
            esp_idf_sdkconfig_defaults,
        );
        utils::set_when_none(
            &mut self.esp_idf_tools_install_dir,
            esp_idf_tools_install_dir,
        );
        utils::set_when_none(&mut self.mcu, mcu);

        #[cfg(any(feature = "native", not(feature = "pio")))]
        self.native.with_cargo_metadata(root_package, &metadata)?;

        Ok(())
    }
}

/// A container to defer to the `esp-idf-sys` table of the metadata.
#[derive(Deserialize, Default)]
pub struct EspIdfSys<T: Default> {
    #[serde(default, rename = "esp-idf-sys")]
    pub v: T,
}

impl<'a, T> EspIdfSys<T>
where
    T: Default,
    T: Deserialize<'a>,
{
    /// Deserialize an `esp-idf-sys` field.
    pub fn deserialize<D>(de: D) -> Result<Self>
    where
        D: Deserializer<'a>,
        D::Error: Send + Sync + std::error::Error + 'static,
    {
        let result = Option::<Self>::deserialize(de)
            .with_context(|| anyhow!("could not read build config from manifest metadata"))?;
        Ok(result.unwrap_or_default())
    }
}

pub mod parse {
    use serde::{Deserialize, Deserializer};

    use super::utils::ValueOrVec;

    /// Parse a string into a `;`-separated list of `T`s or parse a list of `T`s directly.
    pub fn list<'d, T, D>(de: D) -> Result<Option<Vec<T>>, D::Error>
    where
        D: Deserializer<'d>,
        T: for<'s> From<&'s str> + Deserialize<'d>,
    {
        Option::<ValueOrVec<String, T>>::deserialize(de).map(|val| match val {
            Some(ValueOrVec::Val(s)) => Some(
                s.split(';')
                    .filter(|s| !s.is_empty())
                    .map(Into::into)
                    .collect(),
            ),
            Some(ValueOrVec::Vec(v)) => Some(v),
            None => None,
        })
    }
}

pub mod utils {
    use embuild::cargo;
    use serde::de::{self, Deserializer, Visitor};
    use serde::Deserialize;

    /// A helper enum for deserializing a single value or a list of values.
    #[derive(Deserialize, Debug)]
    #[serde(untagged)]
    pub enum ValueOrVec<V, E = V> {
        Val(V),
        Vec(Vec<E>),
    }

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

        impl<'de> Deserializer<'de> for StructFieldsDeserializer<'_> {
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

    /// Set the [`Option`] `val` to `new` if it is [`None`].
    pub fn set_when_none<T>(val: &mut Option<T>, new: Option<T>) {
        if val.is_none() {
            *val = new;
        }
    }

    /// Parse the value from the environment variables and exclude all fields of `T` that
    /// are in `exclude_list`.
    pub fn parse_from_env<T>(exclude_list: &[&str]) -> envy::Result<T>
    where
        T: for<'d> Deserialize<'d>,
    {
        let var_filter = |k: &str| !exclude_list.contains(&k);

        for var in serde_introspect::<T>().iter().filter(|s| var_filter(s)) {
            cargo::track_env_var(var.to_uppercase());
        }

        let vars = std::env::vars().filter(|(key, _)| var_filter(&key.to_lowercase()));
        envy::from_iter(vars)
    }
}
