# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.43] - 2022-11-01

Release 0.43 is a backwards-incompatible release where almost all services were touched in one way or another.

### Major Changes

The main themes of the 0.43 release are:
* Public API
* Separate the notions of using a "nightly" compiler (which is a precondition for all async support) from "experimental" features, which might or might not be async-related
* Expose access to the wrapped ESP IDF services / drivers
* Wifi & Eth: separate layer 2 (ethernet) from layer 3 (IP)
* Http client & server: implement the new traits from `embedded-svc`
* Merge the `nvs_storage` module into `nvs`
* Support for the `embassy-time` crate by providing alarm and timer queue implementations

### Major changes elaboration

### Public API

In addition to implementing the `embedded-svc` traits where possible, all services now have public API. While the public API loosely follows the APIs from `embedded-svc`, it deviates where appropriate so that the native underlying ESP IDF service is better utilized.

These public APIs mean that the user is no longer required to depend on the `embedded-svc` crate so as to consume the `esp-idf-svc` services.
Consuming the services via the `embedded-svc` traits is now only necessary when the user is targetting cross-platform portability of their application code.

### Expose access to the wrapped ESP IDF services / drivers

TBD

### Wifi & Eth: separate layer 2 (ethernet) from layer 3 (IP)

TBD

### Http client & server: implement the new traits from `embedded-svc`

TBD

### Merge the `nvs_storage` module into `nvs`

TBD

### Support for the `embassy-time` crate by providing alarm and timer queue implementations

TBD
