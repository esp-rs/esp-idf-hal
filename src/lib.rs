#![cfg_attr(not(feature = "std"), no_std)]

pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod serial;
pub mod units;

#[cfg_attr(not(feature = "std"), panic_handler)]
#[allow(dead_code)]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe {
        esp_idf_sys::abort();
        core::hint::unreachable_unchecked();
    }
}
