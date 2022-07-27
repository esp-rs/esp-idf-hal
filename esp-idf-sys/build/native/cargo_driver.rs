use std::convert::TryFrom;
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::{env, fs};

use anyhow::{anyhow, bail, Context, Error, Result};
use embuild::cargo::IntoWarning;
use embuild::cmake::file_api::codemodel::Language;
use embuild::cmake::file_api::ObjKind;
use embuild::espidf::{EspIdfOrigin, EspIdfRemote, FromEnvError};
use embuild::fs::copy_file_if_different;
use embuild::utils::{OsStrExt, PathExt};
use embuild::{bindgen, build, cargo, cmake, espidf, git, kconfig, path_buf};
use strum::IntoEnumIterator;

use self::chip::Chip;
use crate::common::{
    self, list_specific_sdkconfigs, workspace_dir, EspIdfBuildOutput, EspIdfComponents, InstallDir,
    ESP_IDF_GLOB_VAR_PREFIX, ESP_IDF_SDKCONFIG_DEFAULTS_VAR, ESP_IDF_SDKCONFIG_VAR,
    ESP_IDF_TOOLS_INSTALL_DIR_VAR, MCU_VAR, SDKCONFIG_DEFAULTS_FILE, SDKCONFIG_FILE,
    V_4_3_2_PATCHES,
};

pub mod chip;

const ESP_IDF_VERSION_VAR: &str = "ESP_IDF_VERSION";
const ESP_IDF_REPOSITORY_VAR: &str = "ESP_IDF_REPOSITORY";
pub const ESP_IDF_CMAKE_GENERATOR_VAR: &str = "ESP_IDF_CMAKE_GENERATOR";

const DEFAULT_ESP_IDF_VERSION: &str = "v4.4.1";

pub fn build() -> Result<EspIdfBuildOutput> {
    let out_dir = cargo::out_dir();
    let target = env::var("TARGET")?;
    let workspace_dir = workspace_dir()?;
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);

    let chip = if let Some(mcu) = env::var_os(MCU_VAR) {
        Chip::from_str(&mcu.to_string_lossy())?
    } else {
        Chip::detect(&target)?
    };
    let chip_name = chip.to_string();
    let profile = common::build_profile();

    cargo::track_env_var(espidf::IDF_PATH_VAR);
    cargo::track_env_var(ESP_IDF_TOOLS_INSTALL_DIR_VAR);
    cargo::track_env_var(ESP_IDF_VERSION_VAR);
    cargo::track_env_var(ESP_IDF_REPOSITORY_VAR);
    cargo::track_env_var(ESP_IDF_SDKCONFIG_DEFAULTS_VAR);
    cargo::track_env_var(ESP_IDF_SDKCONFIG_VAR);
    cargo::track_env_var(MCU_VAR);

    let cmake_generator = get_cmake_generator()?;

    let make_tools = move |repo: &git::Repository,
                           version: &Result<espidf::EspIdfVersion>|
          -> Result<Vec<espidf::Tools>> {
        eprintln!(
            "Using esp-idf {} at '{}'",
            espidf::EspIdfVersion::format(version),
            repo.worktree().display()
        );

        let mut tools = vec![];
        let mut subtools = vec![chip.gcc_toolchain()];
        //
        // Use custom cmake for esp-idf<4.4, because we need at least cmake-3.20
        match version.as_ref().map(|v| (v.major, v.minor, v.patch)) {
            Ok((major, minor, _)) if major >= 4 && minor >= 4 => subtools.push("cmake"),
            _ => {
                tools.push(espidf::Tools::cmake()?);
            }
        }

        if cmake_generator == cmake::Generator::Ninja {
            subtools.push("ninja")
        }
        if !cfg!(target_os = "linux") || !cfg!(target_arch = "aarch64") {
            subtools.extend(chip.ulp_gcc_toolchain());
        }
        tools.push(espidf::Tools::new(subtools));

        Ok(tools)
    };

    // Get the install dir from the $ESP_IDF_TOOLS_INSTALL_DIR, if unset use
    // "workspace" and allow esp-idf from the environment.
    let (install_dir, allow_from_env) = InstallDir::from_env_or("workspace", "espressif")?;
    // EspIdf must come from the environment if $ESP_IDF_TOOLS_INSTALL_DIR == "fromenv".
    let require_from_env = install_dir.is_from_env();
    let maybe_from_env = require_from_env || allow_from_env;

    let install = |esp_idf_origin: EspIdfOrigin| -> Result<espidf::EspIdf> {
        let (custom_url, custom_version) = esp_idf_remote_parts()?;
        match &esp_idf_origin {
            EspIdfOrigin::Custom(repo) => {
                eprintln!(
                    "Using custom user-supplied esp-idf repository at '{}' (detected from env variable `{}`)",
                    repo.worktree().display(),
                    espidf::IDF_PATH_VAR
                );
                if let Some(custom_url) = custom_url {
                    cargo::print_warning(format_args!(
                        "Ignoring configuration setting `{ESP_IDF_REPOSITORY_VAR}=\"{custom_url}\"`: \
                         custom esp-idf repository detected via ${}",
                        espidf::IDF_PATH_VAR
                    ));
                }
                if let Some(custom_version) = custom_version {
                    cargo::print_warning(format_args!(
                        "Ignoring configuration setting `{ESP_IDF_VERSION_VAR}` ({custom_version}): \
                         custom esp-idf repository detected via ${}",
                        espidf::IDF_PATH_VAR
                    ));
                }
            }
            EspIdfOrigin::Managed(remote) => {
                eprintln!("Using managed esp-idf repository: {remote:?}");
            }
        };

        espidf::Installer::new(esp_idf_origin)
            .install_dir(install_dir.path().map(Into::into))
            .with_tools(make_tools)
            .install()
            .context("Could not install esp-idf")
    };

    let idf = match (espidf::EspIdf::try_from_env(), maybe_from_env) {
        (Ok(idf), true) => {
            eprintln!(
                "Using activated esp-idf {} environment at '{}'",
                espidf::EspIdfVersion::format(&idf.version),
                idf.repository.worktree().display()
            );

            idf
        },
        (Ok(idf), false) => {
                cargo::print_warning(format_args!(
                    "Ignoring activated esp-idf environment: ${ESP_IDF_TOOLS_INSTALL_DIR_VAR} != {}", InstallDir::FromEnv
                ));
                install(EspIdfOrigin::Custom(idf.repository))?
        },
        (Err(FromEnvError::NotActivated { source: err, .. }), true) |
        (Err(FromEnvError::NoRepo(err)), true) if require_from_env => {
            return Err(err.context(
                format!("activated esp-idf environment not found but required by ${ESP_IDF_TOOLS_INSTALL_DIR_VAR} == {install_dir}")
            ))
        }
        (Err(FromEnvError::NoRepo(_)), _) => {
            let (repo_url, git_ref) = esp_idf_remote_parts()?;
            let git_ref = git_ref.unwrap_or_else(|| espidf::parse_esp_idf_git_ref(DEFAULT_ESP_IDF_VERSION));

            install(EspIdfOrigin::Managed(EspIdfRemote {
                git_ref,
                repo_url
            }))?
        },
        (Err(FromEnvError::NotActivated { esp_idf_repo, .. }), _) => {
            install(EspIdfOrigin::Custom(esp_idf_repo))?
        }
    };

    // Apply patches, only if the patches were not previously applied and if the esp-idf repo is managed.
    if idf.is_managed_espidf {
        let patch_set = match idf.version.map(|v| (v.major, v.minor, v.patch)) {
            // master branch
            _ if idf.repository.get_default_branch()? == idf.repository.get_branch_name()? => &[],
            Ok((4, 4, _)) => &[],
            Ok((4, 3, patch)) if patch > 2 => &[],
            Ok((4, 3, patch)) if patch == 2 => V_4_3_2_PATCHES,
            Ok((major, minor, patch)) => {
                cargo::print_warning(format_args!(
                    "esp-idf version ({major}.{minor}.{patch}) not officially supported by `esp-idf-sys`. \
                     Supported versions are 'master', 'release/v4.4', 'release/v4.3', 'v4.4(.X)', 'v4.3.3', 'v4.3.2'.",
                ));
                &[]
            }
            Err(err) => {
                err.context("could not determine patch-set for esp-idf repository")
                    .into_warning();
                &[]
            }
        };
        if !patch_set.is_empty() {
            idf.repository
                .apply_once(patch_set.iter().map(|p| manifest_dir.join(p)))?;
        }
    }

    env::set_var("PATH", &idf.exported_path);

    // Remove the sdkconfig file generated by the esp-idf so that potential changes
    // in the user provided sdkconfig and sdkconfig.defaults don't get ignored.
    // Note: I'm really not sure why we have to do this.
    let _ = fs::remove_file(path_buf![&out_dir, "sdkconfig"]);

    // Create cmake project.
    copy_file_if_different(
        manifest_dir.join(path_buf!("resources", "cmake_project", "CMakeLists.txt")),
        &out_dir,
    )?;
    copy_file_if_different(
        manifest_dir.join(path_buf!("resources", "cmake_project", "main.c")),
        &out_dir,
    )?;

    // Copy additional globbed files specified by user env variables
    for file in build::tracked_env_globs_iter(ESP_IDF_GLOB_VAR_PREFIX)? {
        let dest_path = out_dir.join(file.1);
        fs::create_dir_all(dest_path.parent().unwrap())?;
        // TODO: Maybe warn if this overwrites a critical file (e.g. CMakeLists.txt).
        // It could be useful for the user to explicitly overwrite our files.
        copy_file_if_different(&file.0, &out_dir)?;
    }

    // The `kconfig.cmake` script looks at this variable if it should compile `mconf` on windows.
    // But this variable is also present when using git-bash which doesn't have gcc.
    env::remove_var("MSYSTEM");

    // Resolve `ESP_IDF_SDKCONFIG` and `ESP_IDF_SDKCONFIG_DEFAULTS` to an absolute path
    // relative to the workspace directory if not empty.
    let sdkconfig = {
        let file = env::var_os(ESP_IDF_SDKCONFIG_VAR).unwrap_or_else(|| SDKCONFIG_FILE.into());
        let path = Path::new(&file).abspath_relative_to(&workspace_dir);
        let cfg = list_specific_sdkconfigs(path, &profile, &chip_name).next();
        if let Some(ref file) = cfg {
            cargo::track_file(file);
        }
        cfg
    };

    let sdkconfig_defaults = {
        let gen_defaults_path = out_dir.join("gen-sdkconfig.defaults");
        fs::write(&gen_defaults_path, generate_sdkconfig_defaults()?)?;

        let mut result = vec![gen_defaults_path];
        result.extend(
            env::var_os(ESP_IDF_SDKCONFIG_DEFAULTS_VAR)
                .unwrap_or_else(|| SDKCONFIG_DEFAULTS_FILE.into())
                .try_to_str()?
                .split(';')
                .filter_map(|v| {
                    if !v.is_empty() {
                        let path = Path::new(v).abspath_relative_to(&workspace_dir);
                        Some(
                            list_specific_sdkconfigs(path, &profile, &chip_name)
                                // We need to reverse the order here so that the more
                                // specific defaults come last.
                                .rev()
                                .inspect(|p| cargo::track_file(p)),
                        )
                    } else {
                        None
                    }
                })
                .flatten(),
        );
        result
    };

    let defaults_files = sdkconfig_defaults
        .iter()
        // Use the `sdkconfig` as a defaults file to prevent it from being changed by the
        // build. It must be the last defaults file so that its options have precendence
        // over any actual defaults from files before it.
        .chain(sdkconfig.as_ref())
        .try_fold(OsString::new(), |mut accu, p| -> Result<OsString> {
            if !accu.is_empty() {
                accu.push(";");
            }
            // Windows uses `\` as directory separators which cmake can't deal with, so we
            // convert all back-slashes to forward-slashes here. This would be tedious to
            // do with an `OsString` so we have to convert it to `str` first.
            if cfg!(windows) {
                accu.push(p.try_to_str()?.replace('\\', "/"));
            } else {
                accu.push(p);
            }
            Ok(accu)
        })?;

    let cmake_toolchain_file = path_buf![
        &idf.repository.worktree(),
        "tools",
        "cmake",
        chip.cmake_toolchain_file()
    ];

    // Get the asm, C and C++ flags from the toolchain file, these would otherwise get
    // overwritten because `cmake::Config` also sets these (see
    // https://github.com/espressif/esp-idf/issues/7507).
    let (asm_flags, c_flags, cxx_flags) = {
        let extractor_script = cmake::script_variables_extractor(&cmake_toolchain_file)?;

        let output = embuild::cmd!(
            cmake::cmake(),
            "-P",
            extractor_script.as_ref().as_os_str();
            env=("IDF_PATH", &idf.repository.worktree().as_os_str()))
        .stdout()?;

        let mut vars = cmake::process_script_variables_extractor_output(output)?;
        (
            vars.remove("CMAKE_ASM_FLAGS").unwrap_or_default(),
            vars.remove("CMAKE_C_FLAGS").unwrap_or_default(),
            vars.remove("CMAKE_CXX_FLAGS").unwrap_or_default(),
        )
    };

    // `cmake::Config` automatically uses `<out_dir>/build` and there is no way to query
    // what build directory it sets, so we hard-code it.
    let cmake_build_dir = out_dir.join("build");

    let query = cmake::Query::new(
        &cmake_build_dir,
        "cargo",
        &[ObjKind::Codemodel, ObjKind::Toolchains],
    )?;

    let mut cmake_config = cmake::Config::new(&out_dir);
    cmake_config
        .generator(cmake_generator.name())
        .out_dir(&out_dir)
        .no_build_target(true)
        .define("CMAKE_TOOLCHAIN_FILE", &cmake_toolchain_file)
        .define("CMAKE_BUILD_TYPE", "")
        .always_configure(true)
        .pic(false)
        .asmflag(asm_flags)
        .cflag(c_flags)
        .cxxflag(cxx_flags)
        .env("IDF_PATH", &idf.repository.worktree())
        .env("PATH", &idf.exported_path)
        .env("SDKCONFIG_DEFAULTS", defaults_files)
        .env("IDF_TARGET", &chip_name);

    if let Some(install_dir) = install_dir.path() {
        cmake_config.env("IDF_TOOLS_PATH", install_dir);
    }

    // Build the esp-idf.
    cmake_config.build();

    let replies = query.get_replies()?;
    let target = replies
        .get_codemodel()?
        .into_first_conf()
        .get_target("libespidf.elf")
        .unwrap_or_else(|| {
            bail!("Could not read build information from cmake: Target 'libespidf.elf' not found")
        })?;

    let compiler = replies
        .get_toolchains()
        .and_then(|mut t| {
            t.take(Language::C)
                .ok_or_else(|| Error::msg("No C toolchain"))
        })
        .and_then(|t| {
            t.compiler
                .path
                .ok_or_else(|| Error::msg("No compiler path set"))
        })
        .context("Could not determine the compiler from cmake")?;

    let build_info = espidf::EspIdfBuildInfo {
        esp_idf_dir: idf.repository.worktree().to_owned(),
        exported_path_var: idf.exported_path.try_to_str()?.to_owned(),
        venv_python: idf.venv_python,
        build_dir: cmake_build_dir.clone(),
        project_dir: out_dir.clone(),
        compiler: compiler.clone(),
        mcu: chip_name,
        sdkconfig,
        sdkconfig_defaults: Some(sdkconfig_defaults),
    };

    // Save information about the esp-idf build to the out dir so that it can be
    // easily retrieved by tools that need it.
    build_info.save_json(out_dir.join(espidf::BUILD_INFO_FILENAME))?;

    let sdkconfig_json = path_buf![&cmake_build_dir, "config", "sdkconfig.json"];
    let build_output = EspIdfBuildOutput {
        cincl_args: build::CInclArgs::try_from(&target.compile_groups[0])?,
        link_args: Some(
            build::LinkArgsBuilder::try_from(&target.link.unwrap())?
                .linker(&compiler)
                .working_directory(&cmake_build_dir)
                .force_ldproxy(true)
                .build()?,
        ),
        bindgen: bindgen::Factory::from_cmake(&target.compile_groups[0])?.with_linker(&compiler),
        components: EspIdfComponents::from_esp_idf(&build_info.esp_idf_dir)?,
        kconfig_args: Box::new(
            kconfig::try_from_json_file(sdkconfig_json.clone())
                .with_context(|| anyhow!("Failed to read '{:?}'", sdkconfig_json))?,
        ),
        env_path: Some(idf.exported_path.try_to_str()?.to_owned()),
        esp_idf: build_info.esp_idf_dir,
    };

    Ok(build_output)
}

fn esp_idf_remote_parts() -> Result<(Option<String>, Option<git::Ref>)> {
    let version_ref = match env::var(ESP_IDF_VERSION_VAR) {
        Err(env::VarError::NotPresent) => None,
        v => Some(v?.trim().to_owned()),
    }
    .filter(|s| !s.is_empty())
    .map(|s| espidf::parse_esp_idf_git_ref(&s));

    let repo_url = match env::var(ESP_IDF_REPOSITORY_VAR) {
        Err(env::VarError::NotPresent) => None,
        git_url => Some(git_url?),
    };

    Ok((repo_url, version_ref))
}

// Generate `sdkconfig.defaults` content based on the crate manifest (`Cargo.toml`).
//
// This is currently only used to forward the optimization options to the esp-idf.
fn generate_sdkconfig_defaults() -> Result<String> {
    const OPT_VARS: [&str; 4] = [
        "CONFIG_COMPILER_OPTIMIZATION_NONE",
        "CONFIG_COMPILER_OPTIMIZATION_DEFAULT",
        "CONFIG_COMPILER_OPTIMIZATION_PERF",
        "CONFIG_COMPILER_OPTIMIZATION_SIZE",
    ];

    let opt_level = env::var("OPT_LEVEL")?;
    let debug = env::var("DEBUG")?;
    let opt_index = match (opt_level.as_str(), debug.as_str()) {
        ("s" | "z", _) => 3,               // -Os
        ("1", _) | (_, "2" | "true") => 1, // -Og
        ("0", _) => 0,                     // -O0
        ("2" | "3", _) => 2,               // -O2
        _ => unreachable!("Invalid DEBUG or OPT_LEVEL"),
    };

    Ok(OPT_VARS
        .iter()
        .enumerate()
        .map(|(i, s)| format!("{}={}\n", s, if i == opt_index { 'y' } else { 'n' }))
        .collect::<String>())
}

/// Get the cmake generator depending on `ESP_IDF_CMAKE_GENERATOR` or the host
/// architecture.
fn get_cmake_generator() -> Result<cmake::Generator> {
    let generator = match env::var(ESP_IDF_CMAKE_GENERATOR_VAR) {
        Err(env::VarError::NotPresent) => None,
        var => Some(var?.trim().to_lowercase()),
    };

    let generator = match generator.as_deref() {
        None | Some("default") => {
            // No Ninja builds for linux=aarch64 from Espressif yet
            #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
            {
                cmake::Generator::UnixMakefiles
            }

            #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
            {
                cmake::Generator::Ninja
            }
        }
        Some(other) => cmake::Generator::from_str(other).map_err(|_| {
            anyhow!(
                "Invalid CMake generator. Should be either `default`, or one of [{}]",
                cmake::Generator::iter()
                    .map(|e| e.into())
                    .collect::<Vec<&'static str>>()
                    .join(", ")
            )
        })?,
    };

    Ok(generator)
}
