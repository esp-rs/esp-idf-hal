#[cfg(not(any(feature = "pio", feature = "native")))]
compile_error!("One of the features `pio` or `native` must be selected.");

use anyhow::*;
use regex::{self};
use std::{
    env, error, fs,
    iter::once,
    path::{Path, PathBuf},
    str::FromStr,
};

use embuild::{bindgen, build, cargo, kconfig, path_buf, utils::OsStrExt};

// Note that the feature `native` must come before `pio`. These features are really
// mutually exclusive but that would require that all dependencies specify the same
// feature so instead we prefer the `native` feature over `pio` so that if one package
// specifies it, this overrides the `pio` feature for all other dependencies too.
// See https://doc.rust-lang.org/cargo/reference/features.html#mutually-exclusive-features.
#[cfg(any(feature = "pio", feature = "native"))]
#[cfg_attr(feature = "native", path = "build_native.rs")]
#[cfg_attr(all(feature = "pio", not(feature = "native")), path = "build_pio.rs")]
mod build_impl;

pub(crate) const STABLE_PATCHES: &[&str] = &[
    "patches/missing_xtensa_atomics_fix.diff",
    "patches/pthread_destructor_fix.diff",
    "patches/ping_setsockopt_fix.diff",
];

#[allow(unused)]
pub(crate) const MASTER_PATCHES: &[&str] = &["patches/master_missing_xtensa_atomics_fix.diff"];

pub(crate) struct EspIdfBuildOutput<I>
where
    I: Iterator<Item = (String, kconfig::Value)>,
{
    pub(crate) cincl_args: build::CInclArgs,
    pub(crate) link_args: Option<build::LinkArgs>,
    pub(crate) kconfig_args: I,
    pub(crate) bindgen: bindgen::Factory,
}

struct EspIdfVersion {
    major: u32,
    minor: u32,
    patch: u32,
}

impl EspIdfVersion {
    fn parse(bindings_file: impl AsRef<Path>) -> Result<Self> {
        let bindings_content = fs::read_to_string(bindings_file.as_ref())?;

        Ok(Self {
            major: Self::grab_const(&bindings_content, "ESP_IDF_VERSION_MAJOR", "u32")?,
            minor: Self::grab_const(&bindings_content, "ESP_IDF_VERSION_MINOR", "u32")?,
            patch: Self::grab_const(bindings_content, "ESP_IDF_VERSION_PATCH", "u32")?,
        })
    }

    fn get_cfg(&self) -> impl Iterator<Item = String> {
        once(format!(
            "esp_idf_full_version=\"{}.{}.{}\"",
            self.major, self.minor, self.patch
        ))
        .chain(once(format!(
            "esp_idf_version=\"{}.{}\"",
            self.major, self.minor
        )))
        .chain(once(format!("esp_idf_major_version=\"{}\"", self.major)))
        .chain(once(format!("esp_idf_minor_version=\"{}\"", self.minor)))
        .chain(once(format!("esp_idf_patch_version=\"{}\"", self.patch)))
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

fn main() -> anyhow::Result<()> {
    let build_output = build_impl::main()?;

    // We need to restrict the kconfig parameters which are turned into rustc cfg items
    // because otherwise we would be hitting rustc command line restrictions on Windows
    //
    // For now, we take all tristate parameters which are set to true, as well as a few
    // selected string ones, as per below
    //
    // This might change in future
    let kconfig_str_allow = regex::Regex::new(r"IDF_TARGET")?;

    let cfg_args = build::CfgArgs {
        args: build_output
            .kconfig_args
            .filter(|(key, value)| {
                matches!(value, kconfig::Value::Tristate(kconfig::Tristate::True))
                    || kconfig_str_allow.is_match(key)
            })
            .filter_map(|(key, value)| value.to_rustc_cfg("esp_idf", key))
            .collect(),
    };

    let mcu = cfg_args.get("esp_idf_idf_target").ok_or_else(|| {
        anyhow!(
            "Failed to get IDF_TARGET from kconfig. cfgs:\n{:?}",
            cfg_args.args
        )
    })?;

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);

    let header_file = path_buf![
        manifest_dir,
        "src",
        "include",
        if mcu == "esp8266" {
            "esp-8266-rtos-sdk"
        } else {
            "esp-idf"
        },
        "bindings.h"
    ];

    cargo::track_file(&header_file);

    let bindings_file = bindgen::run(
        build_output
            .bindgen
            .builder()?
            .ctypes_prefix("c_types")
            .header(header_file.try_to_str()?)
            .blacklist_function("strtold")
            .blacklist_function("_strtold_r")
            .clang_args(vec![
                "-target",
                if mcu == "esp32c3" {
                    // Necessary to pass explicitly, because of https://github.com/rust-lang/rust-bindgen/issues/1555
                    "riscv32"
                } else {
                    // We don't really have a similar issue with Xtensa, but we pass it explicitly as well just in case
                    "xtensa"
                },
            ]),
    )?;

    let cfg_args = build::CfgArgs {
        args: cfg_args
            .args
            .into_iter()
            .chain(EspIdfVersion::parse(bindings_file)?.get_cfg())
            .chain(once(mcu))
            .collect(),
    };

    cfg_args.propagate();
    cfg_args.output();

    // In case other SYS crates need to have access to the ESP-IDF C headers
    build_output.cincl_args.propagate();

    if let Some(link_args) = build_output.link_args {
        link_args.propagate();
    }

    Ok(())
}
