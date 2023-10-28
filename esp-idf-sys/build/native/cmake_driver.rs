use std::path::PathBuf;
use std::{env, iter};

use anyhow::{anyhow, Context, Result};
use embuild::bindgen;

use crate::common::{sanitize_project_path, setup_clang_env, EspIdfBuildOutput, EspIdfComponents};

pub const CARGO_CMAKE_BUILD_ACTIVE_VAR: &str = "CARGO_CMAKE_BUILD_ACTIVE";
pub const CARGO_CMAKE_BUILD_INCLUDES_VAR: &str = "CARGO_CMAKE_BUILD_INCLUDES";
const CARGO_CMAKE_BUILD_LINK_LIBRARIES_VAR: &str = "CARGO_CMAKE_BUILD_LINK_LIBRARIES";
const CARGO_CMAKE_BUILD_COMPILER_VAR: &str = "CARGO_CMAKE_BUILD_COMPILER";
const CARGO_CMAKE_BUILD_SDKCONFIG_VAR: &str = "CARGO_CMAKE_BUILD_SDKCONFIG";
const CARGO_CMAKE_BUILD_ESP_IDF_VAR: &str = "CARGO_CMAKE_BUILD_ESP_IDF";

pub fn build() -> Result<EspIdfBuildOutput> {
    sanitize_project_path()?;
    setup_clang_env()?;

    let components = EspIdfComponents::from(
        env::var(CARGO_CMAKE_BUILD_LINK_LIBRARIES_VAR)?
            .split(';')
            .filter_map(|c| {
                // All ESP-IDF components are prefixed with `__idf_`
                // Check this comment for more info:
                // https://github.com/esp-rs/esp-idf-sys/pull/17#discussion_r723133416
                c.strip_prefix("__idf_")
            })
            // For some reason, the "driver" component is not returned
            // by the ESP-IDF CMake build, yet it is always enabled
            .chain(iter::once("driver")),
    );

    let sdkconfig = PathBuf::from(env::var(CARGO_CMAKE_BUILD_SDKCONFIG_VAR)?);

    let build_output = EspIdfBuildOutput {
        cincl_args: embuild::build::CInclArgs {
            args: env::var(CARGO_CMAKE_BUILD_INCLUDES_VAR)?,
        },
        link_args: None,
        kconfig_args: Box::new(
            embuild::kconfig::try_from_config_file(sdkconfig.clone())
                .with_context(|| anyhow!("Failed to read '{:?}'", sdkconfig))?
                .map(|(key, value)| {
                    (
                        key.strip_prefix("CONFIG_")
                            .map(str::to_string)
                            .unwrap_or(key),
                        value,
                    )
                }),
        ),
        components,
        bindgen: bindgen::Factory::new()
            .with_linker(env::var(CARGO_CMAKE_BUILD_COMPILER_VAR)?)
            .with_clang_args(
                env::var(CARGO_CMAKE_BUILD_INCLUDES_VAR)?
                    .split(';')
                    .map(|dir| format!("-I{dir}"))
                    .collect::<Vec<_>>(),
            ),
        env_path: None,
        esp_idf: PathBuf::from(env::var(CARGO_CMAKE_BUILD_ESP_IDF_VAR)?),
        config: Default::default(),
    };

    Ok(build_output)
}
