#[cfg(not(feature = "ulp"))]
fn main() -> anyhow::Result<()> {
    embuild::build::CfgArgs::output_propagated("ESP_IDF")
}

#[cfg(feature = "ulp")]
fn main() {
    println!("cargo:rustc-cfg=esp32s2");

    let ulp_dir = std::env::current_dir().unwrap().join("ulp");
    println!("cargo:rustc-link-search={}", ulp_dir.display());

    println!(
        "cargo:rerun-if-changed={}",
        ulp_dir.join("libulp_start.a").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        ulp_dir.join("ulp_link_base.x").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        ulp_dir.join("ulp_link_default.x").display()
    );
}
