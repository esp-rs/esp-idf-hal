fn main() -> anyhow::Result<()> {
    pio::kconfig::CfgArgs::output_propagated("ESP_IDF")?;

    println!(
        "cargo:rustc-cfg={}",
        std::env::var("DEP_ESP_IDF_MCU").unwrap()
    );

    Ok(())
}
