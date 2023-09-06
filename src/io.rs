//! Error types

use core::fmt::{self, Display, Formatter};

use embedded_io::{Error, ErrorKind};

use crate::sys::EspError;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct EspIOError(pub EspError);

impl Error for EspIOError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl From<EspError> for EspIOError {
    fn from(e: EspError) -> Self {
        EspIOError(e)
    }
}

impl Display for EspIOError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EspIOError {}
