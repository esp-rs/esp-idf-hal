/// A mini "esp-idf-ulp-sys" module exposing stuff on top of which the ULP HAL support is implemented
/// (currently, only GPIO) + some utilities for the riscv ULP processor
pub use self::cpu::*;
pub use self::gpio::*;

pub mod cpu;
#[allow(clippy::missing_safety_doc)]
pub mod gpio;

pub type EspError = core::convert::Infallible;

#[macro_export]
macro_rules! esp_result {
    ($err:expr, $value:expr) => {{
        $err;

        Ok($value)
    }};
}

#[macro_export]
macro_rules! esp {
    ($err:expr) => {{
        $err;

        core::result::Result::<(), EspError>::Ok(())
    }};
}
