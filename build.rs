fn main() {
    #[cfg(not(feature = "ulp"))]
    let mcu = std::env::var("DEP_ESP_IDF_MCU").unwrap();

    #[cfg(feature = "ulp")]
    let mcu = "esp32s2";

    println!("cargo:rustc-cfg={}", mcu);
}
