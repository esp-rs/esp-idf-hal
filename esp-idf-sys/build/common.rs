use std::collections::HashSet;
use std::fmt::Display;
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::{error, fs, iter};

use anyhow::{anyhow, bail, Result};
use embuild::cargo::{self, IntoWarning};
use embuild::utils::{OsStrExt, PathExt};
use embuild::{bindgen, build, kconfig};
use strum::{Display, EnumDiscriminants, EnumString};

use crate::config::BuildConfig;

#[allow(dead_code)]
pub const NO_PATCHES: &[&str] = &[];

#[allow(dead_code)]
pub const V_5_0_PATCHES: &[&str] = &["patches/esp_app_format_weak_v5.0.diff"];

#[allow(dead_code)]
pub const V_4_4_PATCHES: &[&str] = &["patches/esp_app_format_weak_v4.4.diff"];

const TOOLS_WORKSPACE_INSTALL_DIR: &str = ".embuild";

pub struct EspIdfBuildOutput {
    pub cincl_args: build::CInclArgs,
    pub link_args: Option<build::LinkArgs>,
    pub kconfig_args: Box<dyn Iterator<Item = (String, kconfig::Value)>>,
    pub components: EspIdfComponents,
    pub bindgen: bindgen::Factory,
    pub env_path: Option<String>,
    pub esp_idf: PathBuf,
    #[allow(dead_code)]
    pub config: BuildConfig,
}

pub struct EspIdfComponents(Vec<String>);

impl EspIdfComponents {
    pub fn new(components: Vec<String>) -> Self {
        Self(components)
    }

    #[allow(dead_code)]
    pub fn from_esp_idf(esp_idf: &Path) -> Result<Self> {
        Self::from_dirs([esp_idf.join("components")])
    }

    #[allow(dead_code)]
    pub fn from_dirs(dirs: impl IntoIterator<Item = impl AsRef<Path>>) -> Result<Self> {
        let components = dirs
            .into_iter()
            .filter_map(|dir| Self::scan(dir.as_ref()).ok())
            .flatten()
            .collect::<Vec<_>>();

        Ok(Self::new(components))
    }

    #[allow(dead_code)]
    pub fn from(enabled: impl IntoIterator<Item = impl Into<String>>) -> Self {
        // NOTE: The components which are always enabled by ESP-IDF's CMake build (for ESP-IDF V4.4) are as follows:
        // cxx; newlib; freertos; esp_hw_support; heap; log; lwip; soc; hal; esp_rom; esp_common; esp_system;
        // esp32; <- Depends on the selected MCU
        //
        // Note also, that for now you always have to explicitly include the `pthread` component,
        // or else `esp-idf-hal` will currently fail to compile (due to its mutex and condvar being implemented on top of pthread)
        //
        // `pthread` is also mandatory when compiling with Rust STD enabled, or else you'll get linker errors
        Self::new(
            enabled
                .into_iter()
                .map(Into::into)
                // deduplicate the components
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
                c.to_uppercase().replace('-', "_")
            )
        })
    }

    #[allow(clippy::needless_lifetimes)]
    pub fn cfg_args<'a>(&'a self) -> impl Iterator<Item = String> + 'a {
        self.0.iter().map(|c| {
            format!(
                "esp_idf_comp_{}_enabled",
                c.to_lowercase().replace('-', "_")
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
            let profile_specific = format!("{filename}.{profile}");
            let chip_specific = format!("{filename}.{chip}");
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

#[derive(Clone, Debug, EnumDiscriminants)]
#[strum_discriminants(name(InstallDirLocation), derive(Display, EnumString))]
#[strum_discriminants(strum(serialize_all = "lowercase"))]
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
    pub fn try_from(location: Option<&str>) -> Result<Self> {
        let (location, path) = match &location {
            None => (crate::config::DEFAULT_TOOLS_INSTALL_DIR, None),
            Some(val) => {
                let (loc, path) = val
                    .split_once(':')
                    .map(|(l, r)| (l, Some(r)))
                    .unwrap_or((val, None));
                (InstallDirLocation::from_str(&loc.to_lowercase())?, path)
            }
        };
        let install_dir = match location {
            InstallDirLocation::Global => Self::Global,
            InstallDirLocation::Workspace => Self::Workspace(
                workspace_dir()?
                    .join(TOOLS_WORKSPACE_INSTALL_DIR)
                    .join(crate::build_driver::TOOLS_DIR),
            ),
            InstallDirLocation::Out => {
                Self::Out(cargo::out_dir().join(crate::build_driver::TOOLS_DIR))
            }
            InstallDirLocation::FromEnv => Self::FromEnv,
            _ => Self::Custom({
                if let Some(path) = path {
                    Path::new(path).abspath_relative_to(workspace_dir()?)
                } else {
                    bail!(
                        "Invalid installation directory format. \
                         Should be one of `global`, `workspace`, `out`, `fromenv` or `custom:<dir>`."
                    );
                }
            }),
        };
        Ok(install_dir)
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

impl<'d> serde::Deserialize<'d> for InstallDir {
    fn deserialize<D>(de: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'d>,
    {
        InstallDir::try_from(Some(&String::deserialize(de)?)).map_err(serde::de::Error::custom)
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

pub fn manifest_dir() -> Result<PathBuf> {
    std::env::var_os("CARGO_MANIFEST_DIR")
        .ok_or_else(|| {
            anyhow!(
                "Environment variable `CARGO_MANIFEST_DIR` unavailable: not in cargo build script"
            )
        })
        .map(PathBuf::from)
}

pub fn sanitize_c_env_vars() -> Result<()> {
    const POLICY_VAR: &str = "ESP_IDF_C_ENV_VARS_ISSUES";
    const C_ENV_VARS: &[&str] = &[
        "CC",
        "CXX",
        "CFLAGS",
        "CCFLAGS",
        "CXXFLAGS",
        "CPPFLAGS",
        "LDFLAGS",
        "GCC_EXEC_PREFIX",
        "COMPILER_PATH",
        "C_INCLUDE_PATH",
        "CPLUS_INCLUDE_PATH",
    ];

    let set_vars = C_ENV_VARS
        .iter()
        .filter_map(|var| std::env::var(var).ok().map(|value| (var, value)))
        .collect::<Vec<_>>();

    if !set_vars.is_empty() {
        let message = set_vars
            .iter()
            .map(|(key, value)| format!("{key}={value}"))
            .collect::<Vec<_>>()
            .join(";");

        let policy = std::env::var(POLICY_VAR)
            .ok()
            .unwrap_or("warnremove".into())
            .to_ascii_lowercase();

        match policy.as_str() {
            "warn" | "err" => {
                let message =
                    format!("Detected env vars that might affect the ESP IDF C build: `{message}`");

                if policy == "warn" {
                    cargo::print_warning(format!("(esp-idf-sys) {message}"));
                } else {
                    bail!(message);
                }
            }
            "warnremove" | "remove" => {
                for (var, _) in set_vars {
                    std::env::remove_var(var);
                }

                if policy == "warnremove" {
                    let message = format!(
                        "Removed env vars that might affect the ESP IDF C build: `{message}`"
                    );

                    cargo::print_warning(format!("(esp-idf-sys) {message}"));
                }
            }
            "ignore" => (),
            _ => bail!("Unsupported value for {POLICY_VAR} env var: {policy}"),
        }
    }

    Ok(())
}

pub fn sanitize_project_path() -> Result<()> {
    const POLICY_VAR: &str = "ESP_IDF_PATH_ISSUES";

    let out_dir = std::fs::canonicalize(cargo::out_dir())?
        .to_string_lossy()
        .into_owned();

    let report = |message: String| {
        let policy = std::env::var(POLICY_VAR).ok().unwrap_or("err".into());

        match policy.as_str() {
            "warn" => cargo::print_warning(format!("(esp-idf-sys) {message}")),
            "err" => bail!(message),
            "ignore" => (),
            _ => bail!("Unsupported value for {POLICY_VAR} env var: {policy}"),
        }

        Ok(())
    };

    #[cfg(windows)]
    {
        if out_dir.len() > 88 {
            report(format!("Too long output directory: `{out_dir}`. Shorten your project path down to no more than 10 characters (or use WSL2 and its native Linux filesystem). Note that tricks like Windows `subst` do NOT work!"))?;
        }
    }

    #[cfg(not(windows))]
    {
        if out_dir.contains(' ') {
            report(format!("Output directory contains spaces: `{out_dir}`."))?;
        }
    }

    Ok(())
}

pub fn setup_clang_env(path: Option<&Path>) -> Result<()> {
    if let Some(path) = path {
        // Path was provided, use that instead of relying on the `espup` symlink
        std::env::set_var("LIBCLANG_PATH", path);
    } else {
        const POLICY_VAR: &str = "ESP_IDF_ESPUP_CLANG_SYMLINK";
        let policy = std::env::var(POLICY_VAR)
            .ok()
            .unwrap_or("try".into())
            .to_lowercase();

        if policy != "ignore" {
            #[allow(deprecated)]
            let espup_clang_path =
                std::env::home_dir().map(|home| home.join(".espup").join("esp-clang"));

            let err_msg = if let Some(espup_clang_path) = espup_clang_path {
                if let Ok(real_path) = std::fs::read_link(&espup_clang_path) {
                    if real_path.is_dir() {
                        std::env::set_var("LIBCLANG_PATH", real_path.as_os_str());
                        None
                    } else {
                        Some(format!(
                            "Symlink {} points to a file",
                            espup_clang_path.display()
                        ))
                    }
                } else {
                    Some(format!(
                        "Symlink {} does not exist or points to a non-existing location",
                        espup_clang_path.display()
                    ))
                }
            } else {
                Some("Cannot locate user home directory".into())
            };

            if let Some(err_msg) = err_msg {
                if policy == "warn" {
                    cargo::print_warning(format!("(esp-idf-sys) {err_msg}"));
                } else if policy == "err" {
                    bail!(err_msg);
                }
            }
        }
    }

    Ok(())
}
