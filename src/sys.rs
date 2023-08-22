#[cfg(not(feature = "riscv-ulp-hal"))]
pub use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
pub use crate::riscv_ulp_hal::sys::*;
