# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.32] - 2022-12-09

* Remove the custom `c_types` module in favor of `core::ffi`
* Switch to `embuild` 0.31 and `bindgen` 0.63. Since 0.61, `bindgen` has the `--size_t-is-usize` flag is enabled by default. This removes a lot of unnecessary casting from `usize` to `u32` and makes the `esp-idf-sys` bindings more egonomic
