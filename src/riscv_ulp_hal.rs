#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
mod pac;
#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
mod reg;

#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
#[macro_use]
pub mod sys;
#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
pub mod delay;
#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
pub mod start;
