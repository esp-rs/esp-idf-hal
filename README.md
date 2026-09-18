# Rust on ESP-IDF

[![Matrix](https://img.shields.io/matrix/esp-rs:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#esp-rs:matrix.org)

The Rust crates that build on top of Espressif's [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/) framework, developed together in this repository:

| Crate | Description | CI |
|-------|-------------|----|
| [`esp-idf-sys`](esp-idf-sys) [![crates.io](https://img.shields.io/crates/v/esp-idf-sys.svg)](https://crates.io/crates/esp-idf-sys) | Raw Rust bindings for ESP-IDF, plus the `cargo`-driven build of ESP-IDF itself | [![CI](https://github.com/esp-rs/esp-idf/actions/workflows/ci-sys.yml/badge.svg)](https://github.com/esp-rs/esp-idf/actions/workflows/ci-sys.yml) |
| [`esp-idf-hal`](esp-idf-hal) [![crates.io](https://img.shields.io/crates/v/esp-idf-hal.svg)](https://crates.io/crates/esp-idf-hal) | Safe Rust wrappers for the ESP-IDF drivers, implementing the [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits | [![CI](https://github.com/esp-rs/esp-idf/actions/workflows/ci-hal.yml/badge.svg)](https://github.com/esp-rs/esp-idf/actions/workflows/ci-hal.yml) |
| [`esp-idf-svc`](esp-idf-svc) [![crates.io](https://img.shields.io/crates/v/esp-idf-svc.svg)](https://crates.io/crates/esp-idf-svc) | Safe Rust wrappers for the ESP-IDF services, implementing the [`embedded-svc`](https://github.com/esp-rs/embedded-svc) traits | [![CI](https://github.com/esp-rs/esp-idf/actions/workflows/ci-svc.yml/badge.svg)](https://github.com/esp-rs/esp-idf/actions/workflows/ci-svc.yml) |

Each crate is self-contained in its directory, with its own `README.md`, `CHANGELOG.md`, examples and `.cargo/config.toml`. Build and run the examples from within the crate directory, exactly as before:

```sh
cd esp-idf-svc
cargo build --example wifi
```

The crates depend on each other through their crates.io releases, so there is no Cargo workspace at the repository root.

## Getting started

- The [Rust on ESP Book](https://esp-rs.github.io/book/)
- The [esp-idf-template](https://github.com/esp-rs/esp-idf-template) project template
- The `README.md` of each crate for its features and ESP-IDF version support
- The [esp-rs Matrix channel](https://matrix.to/#/#esp-rs:matrix.org) for questions

## Repository layout and history

This repository continues the former [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal) repository, whose history is unchanged: for `esp-idf-hal` files, `git log --follow` walks past the commit that moved them into the `esp-idf-hal/` directory.

The histories of the former [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys) and [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc) repositories were merged in with their paths rewritten under `esp-idf-sys/` and `esp-idf-svc/`, so plain `git log` works for their files all the way back.

Release tags are prefixed with the crate name: `esp-idf-sys-v0.38.1`, `esp-idf-hal-v0.47.0`, `esp-idf-svc-v0.53.0`. The pre-monorepo `esp-idf-hal` tags keep their unprefixed `vX.Y.Z` names as well.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
