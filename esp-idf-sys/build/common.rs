use std::collections::HashSet;
use std::fmt::Display;
use std::iter;
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::{env, error, fs};

use anyhow::{anyhow, bail, Result};
use embuild::cargo::{self, IntoWarning};
use embuild::utils::{OsStrExt, PathExt};
use embuild::{bindgen, build, kconfig};

pub const ESP_IDF_TOOLS_INSTALL_DIR_VAR: &str = "ESP_IDF_TOOLS_INSTALL_DIR";
pub const ESP_IDF_GLOB_VAR_PREFIX: &str = "ESP_IDF_GLOB";
pub const ESP_IDF_SDKCONFIG_DEFAULTS_VAR: &str = "ESP_IDF_SDKCONFIG_DEFAULTS";
pub const ESP_IDF_SDKCONFIG_VAR: &str = "ESP_IDF_SDKCONFIG";
pub const MCU_VAR: &str = "MCU";

pub const SDKCONFIG_FILE: &str = "sdkconfig";
pub const SDKCONFIG_DEFAULTS_FILE: &str = "sdkconfig.defaults";

pub const V_4_3_2_PATCHES: &[&str] = &[
    "patches/missing_riscv_atomics_fix.diff",
    "patches/missing_xtensa_atomics_fix.diff",
    "patches/pthread_destructor_fix.diff",
];

const TOOLS_WORKSPACE_INSTALL_DIR: &str = ".embuild";

pub struct EspIdfBuildOutput {
    pub cincl_args: build::CInclArgs,
    pub link_args: Option<build::LinkArgs>,
    pub kconfig_args: Box<dyn Iterator<Item = (String, kconfig::Value)>>,
    pub components: EspIdfComponents,
    pub bindgen: bindgen::Factory,
    pub env_path: Option<String>,
    pub esp_idf: PathBuf,
}

pub struct EspIdfComponents(Vec<String>);

impl EspIdfComponents {
    pub fn new(components: Vec<String>) -> Self {
        Self(components)
    }

    pub fn from_esp_idf(esp_idf: &Path) -> Result<Self> {
        Self::from_dirs(&[esp_idf.join("components")])
    }

    pub fn from_dirs(dirs: impl IntoIterator<Item = impl AsRef<Path>>) -> Result<Self> {
        let components = dirs
            .into_iter()
            .filter_map(|dir| Self::scan(dir.as_ref()).ok())
            .flatten()
            .collect::<Vec<_>>();

        Ok(Self::new(components))
    }

    #[allow(dead_code)]
    pub fn from<I, S>(enabled: I) -> Self
    where
        I: Iterator<Item = S>,
        S: Into<String>,
    {
        // NOTE: The components which are always enabled by ESP-IDF's CMake build (for ESP-IDF V4.4) are as follows:
        // cxx;
        // newlib;
        // freertos;
        // esp_hw_support;
        // heap;
        // log;
        // lwip;
        // soc;
        // hal;
        // esp_rom;
        // esp_common;
        // esp_system;
        // esp32; <- Depends on the selected MCU
        //
        // Note also, that for now you always have to explicitly include the `pthread` component,
        // or else `esp-idf-hal` will currently fail to compile (due to its mutex and condvar being implemented on top of pthread)
        //
        // `pthread` is also mandatory when compiling with Rust STD enabled, or else you'll get linker errors
        Self::new(
            enabled
                .map(Into::into)
                // For some reason, the "driver" component is not returned
                // by the ESP-IDF CMake build, yet it is always enabled
                .chain(iter::once("driver".to_owned()))
                .collect::<HashSet<_>>()
                .into_iter()
                .collect::<Vec<_>>(),
        )
    }

    fn scan(path: &Path) -> Result<Box<dyn Iterator<Item = String>>> {
        let comp_name = Self::get_comp_name(path);
        let components: Box<dyn Iterator<Item = String>> = if let Some(comp_name) = comp_name {
            if path.join("CMakeLists.txt").exists() {
                Box::new(iter::once(comp_name.to_owned()))
            } else {
                Box::new(
                    path.read_dir()?
                        .filter_map(|entry| entry.ok())
                        .filter_map(|entry| Self::scan(&entry.path()).ok())
                        .flatten(),
                )
            }
        } else {
            Box::new(iter::empty())
        };

        Ok(components)
    }

    fn get_comp_name(path: &Path) -> Option<&str> {
        if path.is_dir() {
            path.file_name()
                .and_then(|file_name| file_name.to_str())
                .and_then(|c| if c.starts_with('.') { None } else { Some(c) })
        } else {
            None
        }
    }

    #[allow(clippy::needless_lifetimes)]
    pub fn clang_args<'a>(&'a self) -> impl Iterator<Item = String> + 'a {
        self.0.iter().map(|c| {
            format!(
                "-DESP_IDF_COMP_{}_ENABLED",
                c.to_uppercase().replace('-', '_')
            )
        })
    }

    #[allow(clippy::needless_lifetimes)]
    pub fn cfg_args<'a>(&'a self) -> impl Iterator<Item = String> + 'a {
        self.0.iter().map(|c| {
            format!(
                "esp_idf_comp_{}_enabled",
                c.to_lowercase().replace('-', '_')
            )
        })
    }
}

pub struct EspIdfVersion {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
}

impl EspIdfVersion {
    pub fn parse(bindings_file: impl AsRef<Path>) -> Result<Self> {
        let bindings_content = fs::read_to_string(bindings_file.as_ref())?;

        Ok(Self {
            major: Self::grab_const(&bindings_content, "ESP_IDF_VERSION_MAJOR", "u32")?,
            minor: Self::grab_const(&bindings_content, "ESP_IDF_VERSION_MINOR", "u32")?,
            patch: Self::grab_const(bindings_content, "ESP_IDF_VERSION_PATCH", "u32")?,
        })
    }

    pub fn cfg_args(&self) -> impl Iterator<Item = String> {
        iter::once(format!(
            "esp_idf_version_full=\"{}.{}.{}\"",
            self.major, self.minor, self.patch
        ))
        .chain(iter::once(format!(
            "esp_idf_version=\"{}.{}\"",
            self.major, self.minor
        )))
        .chain(iter::once(format!(
            "esp_idf_version_major=\"{}\"",
            self.major
        )))
        .chain(iter::once(format!(
            "esp_idf_version_minor=\"{}\"",
            self.minor
        )))
        .chain(iter::once(format!(
            "esp_idf_version_patch=\"{}\"",
            self.patch
        )))
    }

    fn grab_const<T>(
        text: impl AsRef<str>,
        const_name: impl AsRef<str>,
        const_type: impl AsRef<str>,
    ) -> Result<T>
    where
        T: FromStr,
        T::Err: error::Error + Send + Sync + 'static,
    {
        // Future: Consider using bindgen::callbacks::ParseCallbacks for grabbing macro-based constants. Should be more reliable compared to grepping

        let const_name = const_name.as_ref();

        let value = regex::Regex::new(&format!(
            r"\s+const\s+{}\s*:\s*{}\s*=\s*(\S+)\s*;",
            const_name,
            const_type.as_ref()
        ))?
        .captures(text.as_ref())
        .ok_or_else(|| anyhow!("Failed to capture constant {}", const_name))?
        .get(1)
        .ok_or_else(|| anyhow!("Failed to capture the value of constant {}", const_name))?
        .as_str()
        .parse::<T>()?;

        Ok(value)
    }
}

pub fn build_profile() -> String {
    std::env::var("PROFILE").expect("No cargo `PROFILE` environment variable")
}

/// List all appropriate sdkconfig files.
///
/// Returns an iterator of paths with the following patterns and ordering if they exist
/// and are files:
/// 1. `<path>.<profile>.<chip>`
/// 2. `<path>.<chip>`
/// 3. `<path>.<profile>`
/// 4. `<path>`
pub fn list_specific_sdkconfigs(
    path: PathBuf,
    profile: &str,
    chip: &str,
) -> impl DoubleEndedIterator<Item = PathBuf> {
    path.file_name()
        .and_then(|filename| filename.try_to_str().into_warning())
        .map(|filename| {
            let profile_specific = format!("{}.{}", filename, profile);
            let chip_specific = format!("{}.{}", filename, chip);
            let profile_chip_specific = format!("{}.{}", &profile_specific, chip);

            [
                profile_chip_specific,
                chip_specific,
                profile_specific,
                filename.to_owned(),
            ]
        })
        .into_iter()
        .flatten()
        .filter_map(move |s| {
            let path = path.with_file_name(s);
            if path.is_file() {
                Some(path)
            } else {
                None
            }
        })
}

#[derive(Clone, Debug)]
pub enum InstallDir {
    Global,
    Workspace(PathBuf),
    Out(PathBuf),
    Custom(PathBuf),
    FromEnv,
}

impl InstallDir {
    /// Get the install directory from the [`ESP_IDF_TOOLS_INSTALL_DIR_VAR`] env variable.
    ///
    /// If this env variable is unset or empty uses `default_install_dir` instead.
    /// On success returns `(install_dir as InstallDir, is_default as bool)`.
    pub fn from_env_or(
        default_install_dir: &str,
        builder_name: &str,
    ) -> Result<(InstallDir, bool)> {
        let location = env::var_os(ESP_IDF_TOOLS_INSTALL_DIR_VAR);
        let (location, is_default) = match &location {
            None => (default_install_dir, true),
            Some(val) => {
                let val = val.try_to_str()?.trim();
                if val.is_empty() {
                    (default_install_dir, true)
                } else {
                    (val, false)
                }
            }
        };
        let install_dir = match location.to_lowercase().as_str() {
            "global" => Self::Global,
            "workspace" => Self::Workspace(
                workspace_dir()?
                    .join(TOOLS_WORKSPACE_INSTALL_DIR)
                    .join(builder_name),
            ),
            "out" => Self::Out(cargo::out_dir().join(builder_name)),
            "fromenv" => Self::FromEnv,
            _ => Self::Custom({
                if let Some(suffix) = location.strip_prefix("custom:") {
                    Path::new(suffix).abspath_relative_to(&workspace_dir()?)
                } else {
                    bail!(
                        "Invalid installation directory format. \
                         Should be one of `global`, `workspace`, `out`, `fromenv` or `custom:<dir>`."
                    );
                }
            }),
        };
        Ok((install_dir, is_default))
    }

    pub fn is_from_env(&self) -> bool {
        matches!(self, Self::FromEnv)
    }

    pub fn path(&self) -> Option<&Path> {
        match self {
            Self::Global | Self::FromEnv => None,
            Self::Workspace(ref path) => Some(path.as_ref()),
            Self::Out(ref path) => Some(path.as_ref()),
            Self::Custom(ref path) => Some(path.as_ref()),
        }
    }
}

impl Display for InstallDir {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Global => write!(f, "global"),
            Self::Workspace(ref path) => write!(f, "workspace ({})", path.display()),
            Self::Out(ref path) => write!(f, "out ({})", path.display()),
            Self::Custom(ref path) => write!(f, "custom ({})", path.display()),
            Self::FromEnv => write!(f, "fromenv"),
        }
    }
}

pub fn workspace_dir() -> Result<PathBuf> {
    cargo::workspace_dir().ok_or_else(|| anyhow!("Cannot fetch crate's workspace dir"))
}
