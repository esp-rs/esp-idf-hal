#![cfg_attr(not(feature = "std"), no_std)]
#![feature(const_btree_new)]

#[cfg(any(feature = "alloc"))]
#[macro_use]
extern crate alloc;

#[cfg(any(feature = "std"))] // TODO: Lower requirements to "alloc"
pub mod httpd;
#[cfg(any(feature = "alloc"))]
// TODO: Ideally should not need "alloc" (also for performance reasons)
pub mod log;
pub mod netif;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs_storage;
pub mod ping;
pub mod sysloop;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod wifi;

#[cfg(any(feature = "binstart", feature = "libstart"))]
pub mod start;

mod private;
