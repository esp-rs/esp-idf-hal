//! The prelude.
//!
//! To use the esp_idf_hal  effectively, a lot of traits and types need to be imported.
//! Instead of importing them one by one manually, the prelude contains the most
//! commonly used imports that are used around application runtime management.
//!
//! This can be imported as use `esp_idf_hal::prelude::*`.

pub use crate::peripherals::*;
pub use crate::units::*;

pub use embedded_hal::digital::v2::InputPin as _;
pub use embedded_hal::digital::v2::OutputPin as _;
pub use embedded_hal::digital::v2::StatefulOutputPin as _;
pub use embedded_hal::digital::v2::ToggleableOutputPin as _;
pub use embedded_hal::prelude::*;
pub use embedded_hal::timer::{Cancel, CountDown, Periodic};
