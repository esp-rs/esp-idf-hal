#[macro_export]
macro_rules! esp_app_desc {
    () => {
        #[no_mangle]
        #[used]
        #[link_section = ".rodata.desc"]
        #[allow(non_upper_case_globals)]
        pub static esp_app_desc: $crate::esp_app_desc_t = {
            const fn str_to_cstr_array<const C: usize>(s: &str) -> [$crate::c_types::c_char; C] {
                let mut ret: [$crate::c_types::c_char; C] = [0; C];

                let mut i = 0;
                while i < C {
                    if i < s.len() {
                        ret[i] = s.as_bytes()[i] as _;
                    } else {
                        break;
                    }

                    i += 1;
                }

                ret
            }

            $crate::esp_app_desc_t {
                magic_word: $crate::ESP_APP_DESC_MAGIC_WORD,
                secure_version: 0,
                reserv1: [0; 2],
                version: str_to_cstr_array(env!("CARGO_PKG_VERSION")),
                project_name: str_to_cstr_array(env!("CARGO_PKG_NAME")),
                time: str_to_cstr_array($crate::build_time::build_time_utc!("%Y-%m-%d")),
                date: str_to_cstr_array($crate::build_time::build_time_utc!("%H:%M:%S")),
                idf_ver: str_to_cstr_array($crate::const_format::formatcp!(
                    "{}.{}.{}",
                    $crate::ESP_IDF_VERSION_MAJOR,
                    $crate::ESP_IDF_VERSION_MINOR,
                    $crate::ESP_IDF_VERSION_PATCH
                )),
                app_elf_sha256: [0; 32],
                reserv2: [0; 20],
            }
        };
    };
}
