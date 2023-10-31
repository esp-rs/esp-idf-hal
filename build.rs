#[cfg(not(feature = "riscv-ulp-hal"))]
fn main() {
    embuild::espidf::sysenv::relay();
    embuild::espidf::sysenv::output(); // Only necessary for building the examples
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
