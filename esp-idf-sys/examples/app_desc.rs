//! This example just calls the `esp_idf_sys::esp_app_desc! {}` macro
//! sp as to setup the `esp_app_desc` structure with default information.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

esp_idf_sys::esp_app_desc! {}

fn main() {
    esp_idf_sys::link_patches();
}
