use std::{env, ffi::OsString, fs::{self, read_to_string}, io::{BufReader, BufRead, Write}, path::{Path, PathBuf}, process::{self, Command, Stdio}};

use anyhow::*;

use pio::bindgen::*;

fn main() -> Result<()> {
    let (runner, idf_target) = if let Some(runner) = Runner::from_pio() {
        // With cargo-pio
        (runner, get_target()?.1)
    } else {
        // In case the crate is being built without cargo-pio
        println!("cargo:warning=Building without cargo-pio");

        let (idf_target, linker) = get_target()?;

        (Runner {
            should_generate: env::var("ESP_IDF_SYS_REGENERATE").is_ok(),
            clang_args: get_sdk_includes(idf_target)?,
            linker: Some(linker.to_owned()),
            mcu: Some(idf_target.to_owned()),
        },
        idf_target)
    };

    runner.run(
        &[format!("src/include/{}/bindings.h", if idf_target == "esp8266" {"esp-8266-rtos-sdk"} else {"esp-idf"})],
        Language::C)
}

fn get_target() -> Result<(&'static str, &'static str)> {
    Ok(match env::var("TARGET")?.as_ref() {
        "xtensa-esp32-none-elf" => ("esp32", "xtensa-esp32-elf-ld"),
        "xtensa-esp32s2-none-elf" => ("esp32s2", "xtensa-esp32s2-elf-ld"),
        "xtensa-esp8266-none-elf" => ("esp8266", "xtensa-esp8266-elf-ld"),
        target => {
            println!("cargo:error=Generating ESP IDF bindings for target '{}' is not supported", target);
            bail!("Generating ESP IDF bindings for target '{}' is not supported", target)
        }
    })
}

//
// Support for getting ESP-IDF include dirs when cargo-pio is NOT used for building
//

const ESP_IDF_GIT_URL: &'static str = "https://github.com/espressif/esp-idf.git";
const ESP_IDF_VERSION: &'static str = "v4.2";

const ESP8266_RTOS_SDK_GIT_URL: &'static str = "https://github.com/espressif/ESP8266_RTOS_SDK.git";
const ESP8266_RTOS_VERSION: &'static str = "v3.3";

fn get_sdk_includes(idf_target: impl AsRef<str>) -> Result<Vec<String>> {
    let sdk_path = env::var("IDF_PATH")
        .map(|var| PathBuf::from(var))
        .or_else(|_| get_sdk_sources(PathBuf::from(env::var("OUT_DIR")?).join("espressif"), idf_target.as_ref() == "esp8266"))?;

    let component_includes =
        globwalk::GlobWalkerBuilder::from_patterns(
        &sdk_path,
        &["components/*/include"],
        )
        .build()?
        .filter_map(Result::ok)
        .map(|d| d.into_path());

    let component_additional_includes = globwalk::GlobWalkerBuilder::from_patterns(
        &sdk_path,
        &["components/*/component.mk"],
        )
        .build()?
        .filter_map(Result::ok)
        .flat_map(|makefile| {
            let path = makefile.into_path();
            let component_path = path.parent().unwrap();

            let mut contents = read_to_string(&path).expect("failed reading component.mk").replace("$(info ", "$(warn ");

            // Define these variables since they affect `COMPONENT_ADD_INCLUDEDIRS`.
            contents.insert_str(0, r"
                CONFIG_SYSVIEW_ENABLE :=
                CONFIG_AWS_IOT_SDK :=
                CONFIG_BT_ENABLED :=
                CONFIG_BLUEDROID_ENABLED :=
            ");
            contents.push_str("\n$(info ${COMPONENT_ADD_INCLUDEDIRS})");

            let mut child = Command::new("make")
                .current_dir(&component_path)
                .arg("-f")
                .arg("-")
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::null())
                .env("IDF_TARGET", OsString::from(idf_target.as_ref()))
                .env("SOC_NAME", OsString::from(idf_target.as_ref()))
                .env("COMPONENT_PATH", &component_path)
                .spawn()
                .expect("make failed");

            let mut stdin = child.stdin.take().unwrap();
            let stdout = child.stdout.take().unwrap();

            writeln!(stdin, "{}", contents).unwrap();

            BufReader::new(stdout).lines()
                .filter_map(Result::ok)
                .map(|s| s.trim_end().to_string())
                .filter(|s| !s.is_empty())
                .flat_map(|s| s.split(' ').map(str::to_string).collect::<Vec<_>>().into_iter())
                .map(move |s| path.parent().unwrap().join(s))
                .filter(|s| s.is_dir())
        });

    let mut includes = component_includes.chain(component_additional_includes)
        .map(|include| format!("-I{}", include.display()))
        .collect::<Vec<_>>();

    includes.sort();
    includes.dedup();

    // So that the default "sdkconfig.h" file is used
    includes.push("-Isrc".into());

    Ok(includes)
}

fn get_sdk_sources(location: impl AsRef<Path>, esp8266: bool) -> Result<PathBuf> {
    let git_path = location.as_ref().join(if esp8266 {"ESP8266_RTOS_SDK"} else {"esp-idf"});

    if !git_path.exists() {
        fs::create_dir_all(git_path.parent().unwrap())?;

        let sdk_version = env::var_os("ESP_IDF_SYS_VERSION")
            .map(|os_str| os_str.to_owned())
            .unwrap_or((if esp8266 {ESP8266_RTOS_VERSION} else {ESP_IDF_VERSION}).into());

        process::Command::new("git")
            .arg("clone")
            .arg("-b")
            .arg(sdk_version)
            .arg("--recursive")
            .arg(if esp8266 {ESP8266_RTOS_SDK_GIT_URL} else {ESP_IDF_GIT_URL})
            .arg(&git_path)
            .status()?;
    }

    Ok(git_path)
}
