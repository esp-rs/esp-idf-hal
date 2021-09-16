fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")
}
