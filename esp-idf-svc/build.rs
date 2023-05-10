fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")?;

    // Will not be available when built with a CMake-first or a PIO-first build
    // We need to output these only when building the examples' binaries anyway
    if let Ok(args) = embuild::build::LinkArgs::try_from_env("ESP_IDF") {
        args.output();
    }

    Ok(())
}
