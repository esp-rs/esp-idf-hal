
use crate::esp_app_desc_t;
use crate::ESP_APP_DESC_MAGIC_WORD;

#[no_mangle]
#[used]
#[link_section = ".rodata_desc"]
#[allow(non_upper_case_globals)]
pub static esp_app_desc: esp_app_desc_t = esp_app_desc_t {
    magic_word: ESP_APP_DESC_MAGIC_WORD,
    secure_version: 0,
    reserv1: [0; 2],
    version: [0; 32],
    project_name: [0; 32],
    time: [0; 16],
    date: [0; 16],
    idf_ver: [0; 32],
    app_elf_sha256: [0; 32],
    reserv2: [0; 20],
};

