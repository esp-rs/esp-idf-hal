fn main() {
    println!(
        "cargo:rustc-cfg={}",
        std::env::var("DEP_ESP_IDF_MCU").unwrap()
    );
}
