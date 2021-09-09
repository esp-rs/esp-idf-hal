//! Install tools and build the `esp-idf` using native tooling.

use std::convert::TryFrom;
use std::env;
use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::str::FromStr;

use anyhow::*;
use embuild::cargo::IntoWarning;
use embuild::cmake::codemodel::Language;
use embuild::cmake::ObjKind;
use embuild::fs::copy_file_if_different;
use embuild::git::{CloneOptions, Repository};
use embuild::python::{check_python_at_least, PYTHON};
use embuild::utils::{OsStrExt, PathExt};
use embuild::{bindgen, build, cargo, cmake, cmd, cmd_output, git, kconfig, path_buf};
use strum::{Display, EnumString};

const SDK_DIR_VAR: &str = "SDK_DIR";
const ESP_IDF_VERSION_VAR: &str = "ESP_IDF_VERSION";
const ESP_IDF_REPOSITORY_VAR: &str = "ESP_IDF_REPOSITORY";
const ESP_IDF_SDKCONFIG_DEFAULTS_VAR: &str = "ESP_IDF_SDKCONFIG_DEFAULTS";
const ESP_IDF_SDKCONFIG_VAR: &str = "ESP_IDF_SDKCONFIG";
const ESP_IDF_EXTRA_TOOLS_VAR: &str = "ESP_IDF_EXTRA_TOOLS";
const MCU_VAR: &str = "MCU";

const DEFAULT_SDK_DIR: &str = ".espressif";
const DEFAULT_ESP_IDF_REPOSITORY: &str = "https://github.com/espressif/esp-idf.git";
const DEFAULT_ESP_IDF_VERSION: &str = "v4.3";

const STABLE_PATCHES: &[&str] = &[
    "patches/missing_xtensa_atomics_fix.diff",
    "patches/pthread_destructor_fix.diff",
    "patches/ping_setsockopt_fix.diff",
];
const MASTER_PATCHES: &[&str] = &[
    "patches/master_missing_xtensa_atomics_fix.diff",
    "patches/ping_setsockopt_fix.diff",
];

fn esp_idf_version() -> git::Ref {
    let version = env::var(ESP_IDF_VERSION_VAR).unwrap_or(DEFAULT_ESP_IDF_VERSION.to_owned());
    let version = version.trim();
    assert!(
        !version.is_empty(),
        "${} (='{}') must contain a valid version",
        ESP_IDF_VERSION_VAR,
        version
    );

    match version.split_once(':') {
        Some(("commit", c)) => git::Ref::Commit(c.to_owned()),
        Some(("tag", t)) => git::Ref::Tag(t.to_owned()),
        Some(("branch", b)) => git::Ref::Branch(b.to_owned()),
        _ => match version.chars().next() {
            Some(c) if c.is_ascii_digit() => git::Ref::Tag("v".to_owned() + version),
            Some('v') if version.len() > 1 && version.chars().nth(1).unwrap().is_ascii_digit() => {
                git::Ref::Tag(version.to_owned())
            }
            Some(_) => git::Ref::Branch(version.to_owned()),
            _ => unreachable!(),
        },
    }
}

pub fn main() -> Result<()> {
    let out_dir = path_buf![env::var("OUT_DIR")?];
    let target = env::var("TARGET")?;
    let workspace_dir = out_dir.pop_times(6);
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);

    let chip = if let Some(mcu) = env::var_os(MCU_VAR) {
        Chip::from_str(&mcu.to_string_lossy())?
    } else {
        Chip::detect(&target)?
    };

    cargo::track_env_var(SDK_DIR_VAR);
    cargo::track_env_var(ESP_IDF_VERSION_VAR);
    cargo::track_env_var(ESP_IDF_REPOSITORY_VAR);
    cargo::track_env_var(ESP_IDF_SDKCONFIG_DEFAULTS_VAR);
    cargo::track_env_var(ESP_IDF_SDKCONFIG_VAR);
    cargo::track_env_var(ESP_IDF_EXTRA_TOOLS_VAR);
    cargo::track_env_var(MCU_VAR);

    let sdk_dir = path_buf![env::var(SDK_DIR_VAR).unwrap_or(DEFAULT_SDK_DIR.to_owned())]
        .abspath_relative_to(&workspace_dir);

    // Clone the esp-idf.
    let esp_idf_dir = sdk_dir.join("esp-idf");
    let esp_idf_version = esp_idf_version();
    let esp_idf_repo =
        env::var(ESP_IDF_REPOSITORY_VAR).unwrap_or(DEFAULT_ESP_IDF_REPOSITORY.to_owned());
    let mut esp_idf = Repository::new(&esp_idf_dir);

    esp_idf.clone_ext(
        &esp_idf_repo,
        CloneOptions::new()
            .force_ref(esp_idf_version.clone())
            .depth(1),
    )?;

    // Apply patches, only if the patches were not previously applied.
    let patch_set = match esp_idf_version {
        git::Ref::Branch(b) if esp_idf.get_default_branch()?.as_ref() == Some(&b) => {
            &MASTER_PATCHES[..]
        }
        git::Ref::Tag(t) if t == DEFAULT_ESP_IDF_VERSION => &STABLE_PATCHES[..],
        _ => {
            cargo::print_warning(format_args!(
                "`esp-idf` version ({:?}) not officially supported by `esp-idf-sys`. \
                 Supported versions are 'master', '{}'.",
                &esp_idf_version, DEFAULT_ESP_IDF_VERSION
            ));
            &[]
        }
    };
    if !patch_set.is_empty() {
        esp_idf.apply_once(patch_set.iter().map(|p| manifest_dir.join(p)))?;
    }

    // This is a workaround for msys or even git bash.
    // When using them `idf_tools.py` prints unix paths (ex. `/c/user/` instead of
    // `C:\user\`), so we correct this with an invocation of `cygpath` which converts the
    // path to the windows representation.
    let cygpath_works = cfg!(windows) && cmd_output!("cygpath", "--version").is_ok();
    let to_win_path = if cygpath_works {
        |p: String| cmd_output!("cygpath", "-w", p).unwrap().to_string()
    } else {
        |p: String| p
    };
    let path_var_sep = if cygpath_works || cfg!(not(windows)) {
        ':'
    } else {
        ';'
    };

    // Create python virtualenv or use a previously installed one.
    check_python_at_least(3, 7)?;
    let idf_tools_py = path_buf![&esp_idf_dir, "tools", "idf_tools.py"];

    let get_python_env_dir = || -> Result<String> {
        Ok(cmd_output!(PYTHON, &idf_tools_py, "--idf-path", &esp_idf_dir, "--quiet", "export", "--format=key-value";
                       ignore_exitcode, env=("IDF_TOOLS_PATH", &sdk_dir))
                            .lines()
                            .find(|s| s.trim_start().starts_with("IDF_PYTHON_ENV_PATH="))
                            .ok_or(anyhow!("`idf_tools.py export` result contains no `IDF_PYTHON_ENV_PATH` item"))?
                            .trim()
                            .strip_prefix("IDF_PYTHON_ENV_PATH=").unwrap()
                                  .to_string())
    };

    let python_env_dir = get_python_env_dir().map(&to_win_path);
    let python_env_dir: PathBuf = if python_env_dir.is_err()
        || !Path::new(&python_env_dir.as_ref().unwrap()).exists()
    {
        cmd!(PYTHON, &idf_tools_py, "--idf-path", &esp_idf_dir, "--quiet", "--non-interactive", "install-python-env";
             env=("IDF_TOOLS_PATH", &sdk_dir))?;
        to_win_path(get_python_env_dir()?)
    } else {
        python_env_dir.unwrap()
    }.into();

    // TODO: better way to get the virtualenv python executable
    let python = embuild::which::which_in(
        "python",
        #[cfg(windows)]
        Some(&python_env_dir.join("Scripts")),
        #[cfg(not(windows))]
        Some(&python_env_dir.join("bin")),
        env::current_dir()?,
    )?;

    // Install tools.
    let mut tools = vec!["ninja", chip.gcc_toolchain()];
    tools.extend(chip.ulp_gcc_toolchain().iter());
    cmd!(python, &idf_tools_py, "--idf-path", &esp_idf_dir, "install"; env=("IDF_TOOLS_PATH", &sdk_dir), args=(tools))?;

    // Intall extra tools if requested, but don't fail compilation if this errors
    if let Some(extra_tools) = env::var_os(ESP_IDF_EXTRA_TOOLS_VAR) {
        cmd!(
            python, &idf_tools_py, "--idf-path", &esp_idf_dir, "install";
            args=(extra_tools.to_string_lossy().split(';').filter(|s| !s.is_empty()).map(str::trim)),
            env=("IDF_TOOLS_PATH", &sdk_dir)
        )
        .into_warning();
    }

    // Get the paths to the tools.
    let mut bin_paths: Vec<_> = cmd_output!(python, &idf_tools_py, "--idf-path", &esp_idf_dir, "--quiet", "export", "--format=key-value"; 
                                            ignore_exitcode, env=("IDF_TOOLS_PATH", &sdk_dir))
                            .lines()
                            .find(|s| s.trim_start().starts_with("PATH="))
                            .expect("`idf_tools.py export` result contains no `PATH` item").trim()
                            .strip_prefix("PATH=").unwrap()
                            .split(path_var_sep)
                            .map(|s| s.to_owned())
                            .collect();
    bin_paths.pop();
    let bin_paths: Vec<_> = bin_paths
        .into_iter()
        .map(|s| PathBuf::from(to_win_path(s)))
        .chain(env::split_paths(&env::var("PATH")?))
        .collect();
    let paths = env::join_paths(bin_paths.iter())?;

    // Create cmake project.
    copy_file_if_different(
        manifest_dir.join(path_buf!("resources", "cmake_project", "CMakeLists.txt")),
        &out_dir,
    )?;
    copy_file_if_different(
        manifest_dir.join(path_buf!("resources", "cmake_project", "main.c")),
        &out_dir,
    )?;

    // The `kconfig.cmake` script looks at this variable if it should compile `mconf` on windows.
    // But this variable is also present when using git-bash which doesn't have gcc.
    env::remove_var("MSYSTEM");

    // Resolve `ESP_IDF_SDKCONFIG` and `ESP_IDF_SDKCONFIG_DEFAULTS` to an absolute path
    // relative to the workspace directory if not empty.
    let sdkconfig = env::var_os(ESP_IDF_SDKCONFIG_VAR)
        .filter(|v| !v.is_empty())
        .map(|v| {
            let path = Path::new(&v).abspath_relative_to(&workspace_dir);
            cargo::track_file(&path);
            path.into_os_string()
        })
        .unwrap_or_else(|| OsString::new());

    let sdkconfig_defaults = env::var_os(ESP_IDF_SDKCONFIG_DEFAULTS_VAR)
        .filter(|v| !v.is_empty())
        .map(|v| -> Result<OsString> {
            let mut result = OsString::new();
            for s in v
                .try_to_str()?
                .split(';')
                .filter(|v| !v.is_empty())
                .map(|v| Path::new(v).abspath_relative_to(&workspace_dir))
            {
                cargo::track_file(&s);
                if !result.is_empty() {
                    result.push(";");
                }
                result.push(s);
            }

            Ok(result)
        })
        .unwrap_or_else(|| Ok(OsString::new()))?;

    let cmake_toolchain_file =
        path_buf![&esp_idf_dir, "tools", "cmake", chip.cmake_toolchain_file()];

    // Get the asm, C and C++ flags from the toolchain file, these would otherwise get
    // overwritten because `cmake::Config` also sets these (see
    // https://github.com/espressif/esp-idf/issues/7507).
    let (asm_flags, c_flags, cxx_flags) = {
        let mut vars = cmake::get_script_variables(&cmake_toolchain_file)?;
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

    // Build the esp-idf.
    cmake::Config::new(&out_dir)
        .generator("Ninja")
        .out_dir(&out_dir)
        .no_build_target(true)
        .define("CMAKE_TOOLCHAIN_FILE", &cmake_toolchain_file)
        .always_configure(true)
        .pic(false)
        .asmflag(asm_flags)
        .cflag(c_flags)
        .cxxflag(cxx_flags)
        .env("IDF_PATH", &esp_idf_dir)
        .env("PATH", &paths)
        .env("SDKCONFIG", sdkconfig)
        .env("SDKCONFIG_DEFAULTS", sdkconfig_defaults)
        .env("IDF_TARGET", &chip.to_string())
        .build();

    let replies = query.get_replies()?;
    let target = replies
        .get_codemodel()?
        .into_first_conf()
        .get_target("libespidf.elf")
        .unwrap_or_else(|| {
            bail!("Could not read build information from cmake: Target 'libespidf.elf' not found",)
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

    let header_file = path_buf![&manifest_dir, "src", "include", "esp-idf", "bindings.h"];

    bindgen::run(
        bindgen::Factory::from_cmake(&target.compile_groups[0])?
            .with_linker(&compiler)
            .builder()?
            .ctypes_prefix("c_types")
            .header(header_file.try_to_str()?)
            .blacklist_function("strtold")
            .blacklist_function("_strtold_r")
            .clang_args(["-target", chip.clang_target()]),
    )?;

    // Output the exact ESP32 MCU, so that we and crates depending directly on us can branch using e.g. #[cfg(esp32xxx)]
    cargo::set_rustc_cfg(chip, "");
    cargo::set_metadata("MCU", chip);

    build::LinkArgsBuilder::try_from(&target.link.unwrap())?
        .linker(&compiler)
        .working_directory(&cmake_build_dir)
        .force_ldproxy(true)
        .build()?
        .propagate();

    // In case other SYS crates need to have access to the ESP-IDF C headers
    build::CInclArgs::try_from(&target.compile_groups[0])?.propagate();

    let sdkconfig_json = path_buf![&cmake_build_dir, "config", "sdkconfig.json"];
    let cfgs = kconfig::CfgArgs::try_from_json(&sdkconfig_json)
        .with_context(|| anyhow!("Failed to read '{:?}'", sdkconfig_json))?;
    cfgs.propagate("ESP_IDF");
    cfgs.output("ESP_IDF");

    Ok(())
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Display, EnumString)]
#[repr(u32)]
pub enum Chip {
    /// Xtensa LX7 based dual core
    #[strum(serialize = "esp32")]
    ESP32 = 0,
    /// Xtensa LX7 based single core
    #[strum(serialize = "esp32s2")]
    ESP32S2,
    /// Xtensa LX7 based single core
    #[strum(serialize = "esp32s3")]
    ESP32S3,
    /// RISC-V based single core
    #[strum(serialize = "esp32c3")]
    ESP32C3,
}

impl Chip {
    pub fn detect(rust_target_triple: &str) -> Result<Chip> {
        if rust_target_triple.starts_with("xtensa-esp") {
            if rust_target_triple.contains("esp32s3") {
                return Ok(Chip::ESP32S3);
            } else if rust_target_triple.contains("esp32s2") {
                return Ok(Chip::ESP32S2);
            } else {
                return Ok(Chip::ESP32);
            }
        } else if rust_target_triple.starts_with("riscv32imc-esp") {
            return Ok(Chip::ESP32C3);
        }
        bail!("Unsupported target '{}'", rust_target_triple)
    }

    /// The name of the gcc toolchain (to compile the `esp-idf`) for `idf_tools.py`.
    pub fn gcc_toolchain(self) -> &'static str {
        match self {
            Self::ESP32 => "xtensa-esp32-elf",
            Self::ESP32S2 => "xtensa-esp32s2-elf",
            Self::ESP32S3 => "xtensa-esp32s3-elf",
            Self::ESP32C3 => "riscv32-esp-elf",
        }
    }

    /// The name of the gcc toolchain for the ultra low-power co-processor for
    /// `idf_tools.py`.
    pub fn ulp_gcc_toolchain(self) -> Option<&'static str> {
        match self {
            Self::ESP32 => Some("esp32ulp-elf"),
            Self::ESP32S2 => Some("esp32s2ulp-elf"),
            _ => None,
        }
    }

    pub fn cmake_toolchain_file(self) -> String {
        format!("toolchain-{}.cmake", self)
    }

    pub fn clang_target(self) -> &'static str {
        match self {
            Self::ESP32 | Self::ESP32S2 | Self::ESP32S3 => "xtensa",
            Self::ESP32C3 => "riscv32",
        }
    }
}
