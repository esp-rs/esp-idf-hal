//! The prelude.
//!
//! To use the esp_idf_hal  effectively, a lot of traits and types need to be imported.
//! Instead of importing them one by one manually, the prelude contains the most
//! commonly used imports that are used around application runtime management.
//!
//! This can be imported as `use esp_idf_hal::prelude::*`.

pub use embedded_hal::{
    adc::nb::{Channel as _, OneShot as _},
    can::{Error as _, Frame as _},
    delay::blocking::DelayUs as _,
    digital::blocking::{
        InputPin as _, OutputPin as _, StatefulOutputPin as _, ToggleableOutputPin as _,
    },
    i2c::{
        blocking::{Read as _, Write as _, WriteRead as _},
        Error as _,
    },
    serial::{
        nb::{Read as _, Write as _},
        Error as _,
    },
    spi::{
        blocking::{
            Read as _, Transactional as _, Transfer as _, TransferInplace as _, Write as _,
            WriteIter as _,
        },
        Error as _,
    },
};
pub use embedded_hal_0_2::prelude::*;

pub use crate::peripherals::*;
pub use crate::units::*;
