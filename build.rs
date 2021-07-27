fn main() {
    #[cfg(not(feature = "ulp"))]
    let mcu = std::env::var("DEP_ESP_IDF_MCU").unwrap();

    #[cfg(feature = "ulp")]
    let mcu = "esp32s2";

    println!("cargo:rustc-cfg={}", mcu);

    #[cfg(feature = "ulp")]
    {
        let ulp_dir = std::env::current_dir().unwrap().join("ulp");

        println!("cargo:rustc-link-search={}", ulp_dir.display());

        println!("cargo:rustc-link-lib=static=ulp_start");

        println!("cargo:rerun-if-changed=build.rs");
        println!("cargo:rerun-if-changed={}", ulp_dir.join("libulp_start.a").display());
        println!("cargo:rerun-if-changed={}", ulp_dir.join("ulp_link.x").display());
    }
}
