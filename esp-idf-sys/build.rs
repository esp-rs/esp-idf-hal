use std::{env, path::PathBuf};

use anyhow::*;

use pio;
use pio::bindgen;
use pio::cargofirst;

fn main() -> Result<()> {
    let idf_target = get_target()?;

    let pio_scons_vars = if let Some(pio_scons_vars) = pio::SconsVariables::from_piofirst() {
        println!("cargo:info=PIO->Cargo build detected: generating bindings only");

        pio_scons_vars
    } else {
        let pio = pio::Pio::install_default()?;

        let resolution = pio::Resolver::new(pio.clone())
            .params(pio::ResolutionParams {
                platform: Some("espressif32".into()),
                frameworks: vec!["espidf".into()],
                mcu: Some(idf_target.to_uppercase()),
                target: Some(env::var("TARGET")?),
                ..Default::default()
            })
            .resolve()?;

        let project_path = PathBuf::from(env::var("OUT_DIR")?).join("esp-idf");

        let pio_scons_vars = cargofirst::build_framework(
            &pio,
            &project_path,
            env::var("PROFILE")? == "release",
            &resolution,
            &[(&PathBuf::from("patches").join("pthread_destructor_fix.diff"), &PathBuf::from("framework-espidf"))],
            Some("ESP_IDF_SYS_PIO_CONF_"),
            Some("ESP_IDF_SYS_GLOB_"),
        )?;

        pio_scons_vars.output_cargo_link_args(project_path, true, true)?;

        pio_scons_vars
    };

    // In case other SYS crates need to have access to the ESP-IDF C headers
    pio_scons_vars.output_cargo_c_include_paths()?;

    bindgen::Runner::from_scons_vars(&pio_scons_vars)?
        .run(
            &[format!("src/include/{}/bindings.h", if idf_target == "esp8266" {"esp-8266-rtos-sdk"} else {"esp-idf"})],
            bindgen::Language::C)
}

fn get_target() -> Result<&'static str> {
    Ok(match env::var("TARGET")?.as_ref() {
        "xtensa-esp32-none-elf" => "esp32",
        "xtensa-esp32s2-none-elf" => "esp32s2",
        //"xtensa-esp8266-none-elf" => "esp8266",
        target => {
            println!("cargo:error=Generating ESP IDF bindings for target '{}' is not supported", target);
            bail!("Generating ESP IDF bindings for target '{}' is not supported", target)
        }
    })
}
