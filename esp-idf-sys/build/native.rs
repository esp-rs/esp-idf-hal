//! Install tools and build the `esp-idf` using native tooling.
#![cfg_attr(feature = "pio", allow(unused))]

use super::common::EspIdfBuildOutput;
use anyhow::Result;
use std::env;

/// The name of the tools sub-directory.
pub const TOOLS_DIR: &str = "espressif";

pub fn build() -> Result<EspIdfBuildOutput> {
    if env::var_os(cmake_driver::CARGO_CMAKE_BUILD_ACTIVE_VAR).is_some()
        || env::var_os(cmake_driver::CARGO_CMAKE_BUILD_INCLUDES_VAR).is_some()
    {
        cmake_driver::build()
    } else {
        cargo_driver::build()
    }
}

/// The root rust crate is a component in an esp-idf project, the esp-idf is built
/// externally by cmake.
///
/// We only need to generate the esp-idf bindings and don't need to compile the esp-idf.
///
/// See
/// [esp-idf-template/README-cmake](https://github.com/esp-rs/esp-idf-template/blob/master/README-cmake.md)
/// for how to setup such a project.
pub mod cmake_driver;

/// The root rust crate is the entry-point, the esp-idf should be compiled by the build script.
///
/// This module does the heavy-lifting of compiling the esp-idf and generating the
/// bindings to it. This process is heavily configurable (see <https://github.com/esp-rs/esp-idf-sys#configuration>).
///
/// See
/// [esp-idf-template/README](https://github.com/esp-rs/esp-idf-template/blob/master/README.md)
/// for how to setup such a project.
pub mod cargo_driver;
