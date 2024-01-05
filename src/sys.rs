#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
pub use esp_idf_sys::*;

#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
pub use crate::riscv_ulp_hal::sys::*;
