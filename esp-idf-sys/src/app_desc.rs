pub const fn str_to_cstr_array<const C: usize>(s: &str) -> [i8; C] {
    let mut ret = [0i8; C];
    let mut i = 0;

    while i < C {
        if i < s.len() {
            ret[i] = s.as_bytes()[i] as i8;
        } else {
            break;
        }

        i += 1;
    }

    ret
}

#[macro_export]
macro_rules! esp_app_desc {
    () => {
        use esp_idf_sys::*;
        use $crate::build_time::build_time_utc;
        use $crate::const_format::formatcp;

        #[no_mangle]
        #[used]
        #[link_section = ".rodata_desc"]
        #[allow(non_upper_case_globals)]
        pub static esp_app_desc: esp_app_desc_t = esp_app_desc_t {
            magic_word: ESP_APP_DESC_MAGIC_WORD,
            secure_version: 0,
            reserv1: [0; 2],
            version: str_to_cstr_array(env!("CARGO_PKG_VERSION")),
            project_name: str_to_cstr_array(env!("CARGO_PKG_NAME")),
            time: str_to_cstr_array(build_time_utc!("%Y-%m-%d")),
            date: str_to_cstr_array(build_time_utc!("%H:%M:%S")),
            idf_ver: str_to_cstr_array(formatcp!(
                "{}.{}.{}",
                ESP_IDF_VERSION_MAJOR,
                ESP_IDF_VERSION_MINOR,
                ESP_IDF_VERSION_PATCH
            )),
            app_elf_sha256: [0; 32],
            reserv2: [0; 20],
        };
    };
}
