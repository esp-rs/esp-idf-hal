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

        let pio_scons_vars = cargofirst::build_framework(
            &pio,
            &project_path,
            env::var("PROFILE")? == "release",
            &resolution,
            &[
                // For now, until the pthread spawning issues with V4.3 are fixed
                "framework-espidf@https://github.com/ivmarkov/esp-idf.git#release/v4.2",
            ],
            &[(
                &PathBuf::from("patches").join("pthread_destructor_fix.diff"),
                &PathBuf::from("framework-espidf"),
            )],
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

    bindgen::Runner::from_scons_vars(&pio_scons_vars)?.run(
        &[PathBuf::from("src")
            .join("include")
            .join(if pio_scons_vars.mcu == "esp8266" {
                "esp-8266-rtos-sdk"
            } else {
                "esp-idf"
            })
            .join("bindings.h")
            .as_os_str()
            .to_str()
            .unwrap()],
        "c_types",
        if pio_scons_vars.mcu == "esp32c3" {
            Some("riscv32")
        } else {
            None
        },
        bindgen::Language::C,
    )
}
