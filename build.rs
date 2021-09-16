#[cfg(not(feature = "ulp"))]
fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")
}

#[cfg(feature = "ulp")]
fn main() -> anyhow::Result<()> {
    println!("cargo:rustc-cfg=esp32s2");

    let ulp_dir = std::env::current_dir().unwrap().join("ulp");

    println!("cargo:rustc-link-search={}", ulp_dir.display());

    println!("cargo:rustc-link-lib=static=ulp_start");

    println!("cargo:rerun-if-changed=build.rs");
    println!(
        "cargo:rerun-if-changed={}",
        ulp_dir.join("libulp_start.a").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        ulp_dir.join("ulp_link.x").display()
    );

    Ok(())
}
