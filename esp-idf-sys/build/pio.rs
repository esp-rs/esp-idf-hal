//! Install tools and build the `esp-idf` using `platformio`.

use std::path::PathBuf;
use std::{env, fs};

use anyhow::*;
use embuild::cargo::IntoWarning;
use embuild::pio::project;
use embuild::utils::PathExt;
use embuild::{bindgen, build, cargo, kconfig, path_buf, pio};

use super::common::*;
use crate::config::{BuildConfig, ESP_IDF_GLOB_VAR_PREFIX, ESP_IDF_TOOLS_INSTALL_DIR_VAR};

/// The name of the tools sub-directory.
pub const TOOLS_DIR: &str = "platformio";

const ESP_IDF_PIO_CONF_VAR_PREFIX: &str = "ESP_IDF_PIO_CONF";

pub fn build() -> Result<EspIdfBuildOutput> {
    sanitize_project_path()?;
    sanitize_c_env_vars()?;
    setup_clang_env(None)?;

    let (pio_scons_vars, link_args, config) =
        if let Some(pio_scons_vars) = project::SconsVariables::from_piofirst() {
            println!("cargo:info=PIO->Cargo build detected: generating bindings only");

            (pio_scons_vars, None, Default::default())
        } else {
            let config = BuildConfig::try_from_env().map(|mut config| {
                config.with_cargo_metadata().into_warning();
                config
            })?;
            config.print();

            let out_dir = cargo::out_dir();
            let workspace_dir = workspace_dir()?;
            let profile = build_profile();

            let (install_dir, allow_from_env) = config.esp_idf_tools_install_dir()?;
            // Pio must come from the environment if `esp_idf_tools_install_dir` == `fromenv`.
            let require_from_env = install_dir.is_from_env();
            let maybe_from_env = require_from_env || allow_from_env;

            let install = |install_dir: &InstallDir| -> Result<pio::Pio> {
                let install_dir = install_dir.path().map(ToOwned::to_owned);

                if let Some(install_dir) = &install_dir {
                    // Workaround an issue in embuild until it is fixed in the next version
                    fs::create_dir_all(install_dir)?;
                }

                pio::Pio::install(install_dir, pio::LogLevel::Standard, false)
            };

            let pio = match (pio::Pio::try_from_env(), maybe_from_env) {
                (Some(pio), true) => {
                    eprintln!(
                        "Using platformio from environment at '{}'",
                        pio.platformio_exe.display()
                    );

                    pio
                }
                (Some(_), false) => {
                    cargo::print_warning(format_args!(
                        "Ignoring platformio in environment: {ESP_IDF_TOOLS_INSTALL_DIR_VAR} != {}",
                        InstallDir::FromEnv
                    ));
                    install(&install_dir)?
                }
                (None, true) if require_from_env => {
                    bail!(
                        "platformio not found in environment ($PATH) \
                       but required by {ESP_IDF_TOOLS_INSTALL_DIR_VAR} == {install_dir}"
                    );
                }
                (None, _) => install(&install_dir)?,
            };

            let resolution = pio::Resolver::new(pio.clone())
                .params(pio::ResolutionParams {
                    platform: Some("espressif32".into()),
                    frameworks: vec!["espidf".into()],
                    mcu: config.mcu.clone().map(|mcu| mcu.to_uppercase()), // MCU always uppercase in PlatformIO
                    target: Some(env::var("TARGET")?),
                    ..Default::default()
                })
                .resolve(true)?;

            let mut builder = project::Builder::new(out_dir.join("esp-idf"));

            // Resolve `ESP_IDF_SDKCONFIG` and `ESP_IDF_SDKCONFIG_DEFAULTS` to an absolute path
            // relative to the workspace directory if not empty.
            let sdkconfig = {
                let path = config
                    .esp_idf_sdkconfig()
                    .abspath_relative_to(&workspace_dir);
                let cfg = list_specific_sdkconfigs(path, &profile, &resolution.mcu).next();

                cfg.map(|path| {
                    cargo::track_file(&path);

                    (path, format!("sdkconfig.{profile}").into())
                })
            };

            let sdkconfig_defaults = config
                .esp_idf_sdkconfig_defaults()
                .into_iter()
                .flat_map(|v| {
                    list_specific_sdkconfigs(
                        v.abspath_relative_to(&workspace_dir),
                        &profile,
                        &resolution.mcu,
                    )
                    // We need to reverse the order here so that the more specific
                    // defaults come last.
                    .rev()
                })
                .map(|path| {
                    cargo::track_file(&path);
                    let file_name = PathBuf::from(path.file_name().unwrap());
                    (path, file_name)
                });

            builder
                .enable_scons_dump()
                .enable_c_entry_points()
                .options(build::env_options_iter(ESP_IDF_PIO_CONF_VAR_PREFIX)?)
                .files(build::tracked_env_globs_iter(ESP_IDF_GLOB_VAR_PREFIX)?)
                .files(sdkconfig.into_iter())
                .files(sdkconfig_defaults);

            let project_path = builder.generate(&resolution)?;

            pio.build(&project_path, profile == "release")?;

            let pio_scons_vars = project::SconsVariables::from_dump(&project_path)?;

            let link_args = build::LinkArgsBuilder::try_from(&pio_scons_vars)?.build()?;

            (pio_scons_vars, Some(link_args), config)
        };

    let sdkconfig = path_buf![
        &pio_scons_vars.project_dir,
        if pio_scons_vars.release_build {
            "sdkconfig.release"
        } else {
            "sdkconfig.debug"
        }
    ];

    let esp_idf = PathBuf::from(&pio_scons_vars.pio_framework_dir);

    let build_output = EspIdfBuildOutput {
        cincl_args: build::CInclArgs::try_from(&pio_scons_vars)?,
        env_path: Some(pio_scons_vars.path.clone()),
        link_args,
        bindgen: bindgen::Factory::from_scons_vars(&pio_scons_vars)?,
        components: EspIdfComponents::from_esp_idf(&esp_idf)?,
        kconfig_args: Box::new(
            kconfig::try_from_config_file(sdkconfig.clone())
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
        esp_idf,
        config,
    };

    Ok(build_output)
}
