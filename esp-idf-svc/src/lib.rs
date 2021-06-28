#![cfg_attr(not(feature = "std"), no_std)]
#![feature(const_btree_new)]

#[cfg(any(feature = "alloc"))]
#[macro_use]
extern crate alloc;

pub mod sysloop;
pub mod netif;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod nvs_storage;
#[cfg(any(feature = "alloc"))] // TODO: Expose a subset which does not require "alloc"
pub mod wifi;
pub mod ping;
#[cfg(any(feature = "std"))] // TODO: Lower requirements to "alloc"
pub mod httpd;
#[cfg(any(feature = "alloc"))] // TODO: Ideally should not need "alloc" (also for performance reasons)
pub mod log;

#[cfg(any(feature = "binstart", feature = "libstart"))]
pub mod start;

mod private;
