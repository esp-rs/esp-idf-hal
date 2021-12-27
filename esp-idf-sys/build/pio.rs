//! Install tools and build the `esp-idf` using `platformio`.

use std::convert::TryFrom;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

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

        let install_dir = get_install_dir("platformio")?;

        if let Some(install_dir) = install_dir.as_ref() {
            // Workaround an issue in embuild until it is fixed in the next version
            fs::create_dir_all(install_dir)?;
        }

        let pio = pio::Pio::install(install_dir, pio::LogLevel::Standard, false)?;

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
        let sdkconfig = env::var_os(ESP_IDF_SDKCONFIG_VAR)
            .filter(|v| !v.is_empty())
            .map(|v| {
                let path = Path::new(&v).abspath_relative_to(&workspace_dir);
                let path = get_sdkconfig_profile(&path, &profile, &resolution.mcu).unwrap_or(path);
                cargo::track_file(&path);

                (path, format!("sdkconfig.{}", profile).into())
            });

        let sdkconfig_defaults_var =
            env::var_os(ESP_IDF_SDKCONFIG_DEFAULTS_VAR).unwrap_or_default();
        let sdkconfig_defaults = sdkconfig_defaults_var
            .try_to_str()?
            .split(';')
            .map(|v| v.trim())
            .filter(|v| !v.is_empty())
            .map(|v| {
                let path = Path::new(v).abspath_relative_to(&workspace_dir);
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
        for patch in STABLE_PATCHES {
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

    let build_output = EspIdfBuildOutput {
        cincl_args: build::CInclArgs::try_from(&pio_scons_vars)?,
        link_args,
        bindgen: bindgen::Factory::from_scons_vars(&pio_scons_vars)?
            .with_clang_args(EspIdfComponents::new().clang_args().collect::<Vec<_>>()),
        components: EspIdfComponents::new(),
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
    };

    Ok(build_output)
}
