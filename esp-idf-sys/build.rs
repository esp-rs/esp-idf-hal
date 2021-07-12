use std::{env, path::PathBuf};

use anyhow::*;

use pio;
use pio::bindgen;
use pio::cargofirst;

fn main() -> Result<()> {
    let pio_scons_vars = if let Some(pio_scons_vars) = pio::SconsVariables::from_piofirst() {
        println!("cargo:info=PIO->Cargo build detected: generating bindings only");

        pio_scons_vars
    } else {
        let pio = pio::Pio::install_default()?;

        let resolution = pio::Resolver::new(pio.clone())
            .params(pio::ResolutionParams {
                platform: Some("espressif32".into()),
                frameworks: vec!["espidf".into()],
                target: Some(env::var("TARGET")?),
                ..Default::default()
            })
            .resolve(true)?;

        let project_path = PathBuf::from(env::var("OUT_DIR")?).join("esp-idf");

        #[cfg(feature = "espidf_master")]
        let platform_packages = ["framework-espidf@https://github.com/ivmarkov/esp-idf.git#master"];

        #[cfg(feature = "espidf_master")]
        let patches: [(&std::path::Path, &std::path::Path); 0] = [];

        #[cfg(not(feature = "espidf_master"))]
        let platform_packages: [&str; 0] = [];

        #[cfg(not(feature = "espidf_master"))]
        let patches = [
            (
                PathBuf::from("patches").join("pthread_destructor_fix.diff"),
                PathBuf::from("framework-espidf"),
            ),
            (
                PathBuf::from("patches").join("missing_xtensa_atomics_fix.diff"),
                PathBuf::from("framework-espidf"),
            ),
        ];

        let pio_scons_vars = cargofirst::build_framework(
            &pio,
            &project_path,
            env::var("PROFILE")? == "release",
            &resolution,
            &platform_packages,
            &patches,
            Some("ESP_IDF_SYS_PIO_CONF_"),
            Some("ESP_IDF_SYS_GLOB_"),
        )?;

        // pio_scons_vars.output_cargo_link_args(project_path, true, true)?; // No longer works due to this issue: https://github.com/rust-lang/cargo/issues/9641
        pio_scons_vars.propagate_cargo_link_args(project_path, true, true)?;

        pio_scons_vars
    };

    // In case other SYS crates need to have access to the ESP-IDF C headers
    // pio_scons_vars.output_cargo_c_include_args()?; // No longer works due to this issue: https://github.com/rust-lang/cargo/issues/9641
    pio_scons_vars.propagate_cargo_c_include_args()?;

    let mcu = pio_scons_vars.mcu.as_str();

    // Output the exact ESP32 MCU, so that we and crates depending directly on us can branch using e.g. #[cfg(esp32xxx)]
    println!("cargo:rustc-cfg={}", mcu);
    println!("cargo:MCU={}", mcu);

    bindgen::Runner::from_scons_vars(&pio_scons_vars)?.run(
        &[PathBuf::from("src")
            .join("include")
            .join(if mcu == "esp8266" {
                "esp-8266-rtos-sdk"
            } else {
                "esp-idf"
            })
            .join("bindings.h")
            .as_os_str()
            .to_str()
            .unwrap()],
        "c_types",
        if mcu == "esp32c3" {
            Some("riscv32")
        } else {
            None
        },
        bindgen::Language::C,
    )
}
