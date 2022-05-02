#[cfg(not(feature = "riscv-ulp-hal"))]
fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")?;

    // Will not be available when built a CMake-first or a PIO-first build
    // We need to output these only when building the examples' binaries anyway
    embuild::build::LinkArgs::try_from_env("ESP_IDF")
        .ok()
        .map(|args| args.output());

    Ok(())
}

#[cfg(feature = "riscv-ulp-hal")]
fn main() {
    println!("cargo:rustc-cfg=esp32s2");

    let riscv_ulp_dir = std::env::current_dir().unwrap().join("riscv-ulp");
    println!("cargo:rustc-link-search={}", riscv_ulp_dir.display());

    println!(
        "cargo:rerun-if-changed={}",
        riscv_ulp_dir.join("libriscv_ulp_start.a").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        riscv_ulp_dir.join("riscv_ulp_link_base.x").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        riscv_ulp_dir.join("riscv_ulp_link_default.x").display()
    );
}
