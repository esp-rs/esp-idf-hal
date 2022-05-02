//! Install tools and build the `esp-idf` using `platformio`.

use std::convert::TryFrom;
use std::path::{Path, PathBuf};
use std::{env, fs};

use anyhow::*;
use embuild::pio::project;
use embuild::utils::{OsStrExt, PathExt};
use embuild::{bindgen, build, cargo, kconfig, path_buf, pio};

use super::common::*;

const ESP_IDF_PIO_CONF_VAR_PREFIX: &str = "ESP_IDF_PIO_CONF";

pub fn build() -> Result<EspIdfBuildOutput> {
    let (pio_scons_vars, link_args) = if let Some(pio_scons_vars) =
        project::SconsVariables::from_piofirst()
    {
        println!("cargo:info=PIO->Cargo build detected: generating bindings only");

        (pio_scons_vars, None)
    } else {
        cargo::track_env_var(ESP_IDF_TOOLS_INSTALL_DIR_VAR);
        cargo::track_env_var(ESP_IDF_SDKCONFIG_VAR);
        cargo::track_env_var(ESP_IDF_SDKCONFIG_DEFAULTS_VAR);
        cargo::track_env_var(MCU_VAR);

        let out_dir = cargo::out_dir();
        let workspace_dir = workspace_dir()?;
        let profile = build_profile();

        // Get the install dir from the $ESP_IDF_TOOLS_INSTALL_DIR, if unset use
        // "workspace" and allow platformio from the environment.
        let (install_dir, allow_from_env) = InstallDir::from_env_or("workspace", "platformio")?;
        // Pio must come from the environment if $ESP_IDF_TOOLS_INSTALL_DIR == "fromenv".
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
                    "Ignoring platformio in environment: ${ESP_IDF_TOOLS_INSTALL_DIR_VAR} != {}",
                    InstallDir::FromEnv
                ));
                install(&install_dir)?
            }
            (None, true) if require_from_env => {
                bail!(
                    "platformio not found in environment ($PATH) \
                       but required by ${ESP_IDF_TOOLS_INSTALL_DIR_VAR} == {install_dir}"
                );
            }
            (None, _) => install(&install_dir)?,
        };

        let resolution = pio::Resolver::new(pio.clone())
            .params(pio::ResolutionParams {
                platform: Some("espressif32".into()),
                frameworks: vec!["espidf".into()],
                mcu: env::var(MCU_VAR).ok(),
                target: Some(env::var("TARGET")?),
                ..Default::default()
            })
            .resolve(true)?;

        let mut builder = project::Builder::new(out_dir.join("esp-idf"));

        // Resolve `ESP_IDF_SDKCONFIG` and `ESP_IDF_SDKCONFIG_DEFAULTS` to an absolute path
        // relative to the workspace directory if not empty.
        let sdkconfig = {
            let file = env::var_os(ESP_IDF_SDKCONFIG_VAR).unwrap_or_else(|| SDKCONFIG_FILE.into());
            let path = Path::new(&file).abspath_relative_to(&workspace_dir);
            let cfg = list_specific_sdkconfigs(path, &profile, &resolution.mcu).next();

            cfg.map(|path| {
                cargo::track_file(&path);

                (path, format!("sdkconfig.{}", profile).into())
            })
        };

        let sdkconfig_defaults_var = env::var_os(ESP_IDF_SDKCONFIG_DEFAULTS_VAR)
            .unwrap_or_else(|| SDKCONFIG_DEFAULTS_FILE.into());
        let sdkconfig_defaults = sdkconfig_defaults_var
            .try_to_str()?
            .split(';')
            .filter_map(|v| {
                if !v.is_empty() {
                    let path = Path::new(v).abspath_relative_to(&workspace_dir);
                    Some(
                        list_specific_sdkconfigs(path, &profile, &resolution.mcu)
                            // We need to reverse the order here so that the more
                            // specific defaults come last.
                            .rev(),
                    )
                } else {
                    None
                }
            })
            .flatten()
            .map(|path| {
                cargo::track_file(&path);
                let file_name = PathBuf::from(path.file_name().unwrap());
                (path, file_name)
            });

        builder
            .enable_scons_dump()
            .enable_c_entry_points()
            .options(build::env_options_iter(ESP_IDF_PIO_CONF_VAR_PREFIX)?)
            .files(build::tracked_globs_iter(path_buf!["."], &["patches/**"])?)
            .files(build::tracked_env_globs_iter(ESP_IDF_GLOB_VAR_PREFIX)?)
            .files(sdkconfig.into_iter())
            .files(sdkconfig_defaults);

        let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);
        for patch in V_4_3_2_PATCHES {
            // TODO: fix patches not applying
            builder.platform_package_patch(manifest_dir.join(patch), path_buf!["framework-espidf"]);
        }

        let project_path = builder.generate(&resolution)?;

        pio.build(&project_path, profile == "release")?;

        let pio_scons_vars = project::SconsVariables::from_dump(&project_path)?;

        let link_args = build::LinkArgsBuilder::try_from(&pio_scons_vars)?.build()?;

        (pio_scons_vars, Some(link_args))
    };

    let sdkconfig = path_buf![
        &pio_scons_vars.project_dir,
        if pio_scons_vars.release_build {
            "sdkconfig.release"
        } else {
            "sdkconfig.debug"
        }
    ];

    let esp_idf = PathBuf::from(pio_scons_vars.pio_framework_dir);

    let build_output = EspIdfBuildOutput {
        cincl_args: build::CInclArgs::try_from(&pio_scons_vars)?,
        env_path: link_args.as_ref().map(|_| pio_scons_vars.path.clone()),
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
    };

    Ok(build_output)
}
