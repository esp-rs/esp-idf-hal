//! Units of measurement implementation for times and frequencies.
//!
//! It provides type safety, easy conversion and limited arithmetic support.
//!
//! # Usage
//!
//! ```
//! let frequency_mhz_1 = MegaHertz(10),
//! let frequency_mhz_2 = 10.MHz(),
//! let frequency_khz_1: KiloHertz = frequency_mhz_1.into(),
//! let frequency_khz_2 = KiloHertz::from(frequency_mhz_2),
//! let frequency_khz_3 = frequency_khz_1 + 10.MHz().into(),
//! let frequency_hz_1 = 1.Hz() + frequency_khz_3.into(),
//! ```

use core::convert::TryFrom;
use core::convert::TryInto;
use core::fmt;

pub type ValueType = u32;
pub type LargeValueType = u64;

pub trait Quantity: Sized {}
pub trait Time: Quantity + Into<NanoSeconds> {}
pub trait Frequency: Quantity + Into<Hertz> {}
pub trait Count: Quantity + Into<Ticks> {}

pub trait TimeU64: Quantity + Into<NanoSecondsU64> {}
pub trait FrequencyU64: Quantity + Into<HertzU64> {}
pub trait CountU64: Quantity + Into<TicksU64> {}

/// defines and implements extension traits for quantities with units
macro_rules! define {
    ($primitive:ident, $trait:ident, $( ($type: ident, $quantity: ident, $unit: ident,
        $print_unit: literal), )+) => {
        $(
            #[derive(Eq, PartialEq, Ord, PartialOrd, Clone, Copy, Hash, Default)]
            pub struct $quantity(pub $primitive);

            impl Quantity for $quantity {}
            impl $type for $quantity {}
        )*

        pub trait $trait {
            $(
                #[allow(non_snake_case)]
                fn $unit(self) -> $quantity;
            )*
        }

        impl $trait for $primitive {
            $(
                fn $unit(self) -> $quantity {
                    $quantity(self)
                }
            )*
        }

        $(
            impl From<$quantity> for $primitive {
                fn from(x: $quantity) -> Self {
                    x.0
                }
            }

            impl From<$primitive> for $quantity {
                fn from(x: $primitive) -> $quantity {
                    $quantity(x)
                }
            }

            impl fmt::Debug for $quantity {
                fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                    write!(f, "{}{}", self.0, $print_unit)
                }
            }

            impl fmt::Display for $quantity {
                fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                    write!(f, "{}{}", self.0, $print_unit)
                }
            }

            impl core::ops::Div<$primitive> for $quantity {
                type Output = Self;
                fn div(self, rhs: $primitive) -> Self::Output {
                    $quantity(self.0/rhs)
                }
            }

            impl core::ops::Mul<$primitive> for $quantity {
                type Output = Self;
                fn mul(self, rhs: $primitive) -> Self::Output {
                    $quantity(self.0*rhs)
                }
            }

            impl core::ops::Mul<$quantity> for $primitive {
                type Output = $quantity;
                fn mul(self, rhs: $quantity) -> Self::Output {
                    $quantity(self*rhs.0)
                }
            }

            impl core::ops::Div<$quantity> for $quantity {
                type Output = $primitive;
                fn div(self, rhs: Self) -> Self::Output {
                    self.0/rhs.0
                }
            }

            impl core::ops::Add<$quantity> for $quantity {
                type Output = Self;
                fn add(self, rhs: Self) -> Self::Output {
                    Self(self.0+rhs.0)
                }
            }

            impl core::ops::Sub<$quantity> for $quantity {
                type Output = Self;
                fn sub(self, rhs: Self) -> Self::Output {
                    Self(self.0-rhs.0)
                }
            }
        )*
    };
}

/// Define ValueType and LargeValueType quantities and conversion from ValueType to LargeValueType
macro_rules! define_large {
    ($( ($type: ident, $quantity: ident, $unit:ident, $type_large: ident,
            $quantity_large: ident, $unit_large:ident, $print_unit: literal) ),+) => {

        define!(
            ValueType,
            FromValueType,
            $(($type, $quantity, $unit, $print_unit),)*
        );

        define!(
            LargeValueType,
            FromLargeValueType,
            $(($type_large, $quantity_large, $unit_large, $print_unit),)*
        );

        $(
        impl From<$quantity> for $quantity_large {
            fn from(x: $quantity) -> Self {
                Self(LargeValueType::from(x.0))
            }
        }
        impl TryFrom<$quantity_large> for $quantity {
            type Error=core::num::TryFromIntError;
            fn try_from(x: $quantity_large) -> Result<$quantity, Self::Error> {
                Ok(Self(ValueType::try_from(x.0)?))
            }
        }
        )*

    };
}

/// defines From trait for pair or quantities with scaling
macro_rules! convert {
    ($( ($from: ty, $from_large: ty, $into: ty, $into_large: ty, $factor: expr) ),+) => {
        $(
        impl From<$from> for $into {
            fn from(x: $from) -> Self {
                Self(x.0 * $factor)
            }
        }
        impl From<$from> for $into_large {
            fn from(x: $from) -> Self {
                Self(LargeValueType::from(x.0) * $factor)
            }
        }
        impl From<$from_large> for $into_large {
            fn from(x: $from_large) -> Self {
                Self(x.0 * $factor)
            }
        }
        )*
    };
}

/// defines multiply trait for frequency and time
macro_rules! multiply {
    ($( ($time: ty, $time_large: ty, $freq: ty, $freq_large: ty,
        $factor: expr, $divider: expr) ),+) => {
        $(
        impl core::ops::Mul<$freq> for $time {
            type Output = Ticks;
            fn mul(self, rhs: $freq) -> Self::Output {
                TicksU64::from(LargeValueType::from(self.0) * LargeValueType::from(rhs.0)
                    * $factor / $divider).try_into().unwrap()
            }
        }

        impl core::ops::Mul<$time> for $freq {
            type Output = Ticks;
            fn mul(self, rhs: $time) -> Self::Output {
                TicksU64::from(LargeValueType::from(self.0) * LargeValueType::from(rhs.0)
                    * $factor / $divider).try_into().unwrap()
            }
        }

        impl core::ops::Mul<$freq_large> for $time_large {
            type Output = TicksU64;
            fn mul(self, rhs: $freq_large) -> Self::Output {
                (self.0 * rhs.0 * $factor / $divider).into()
            }
        }

        impl core::ops::Mul<$time_large> for $freq_large {
            type Output = TicksU64;
            fn mul(self, rhs: $time_large) -> Self::Output {
                (self.0 * rhs.0 * $factor / $divider).into()
            }
        }

        impl core::ops::Mul<$freq> for $time_large {
            type Output = TicksU64;
            fn mul(self, rhs: $freq) -> Self::Output {
                (self.0 * LargeValueType::from(rhs.0) * $factor / $divider).into()
            }
        }

        impl core::ops::Mul<$time> for $freq_large {
            type Output = TicksU64;
            fn mul(self, rhs: $time) -> Self::Output {
                (self.0 * LargeValueType::from(rhs.0) * $factor / $divider).into()
            }
        }

        impl core::ops::Mul<$freq_large> for $time {
            type Output = TicksU64;
            fn mul(self, rhs: $freq_large) -> Self::Output {
                (LargeValueType::from(self.0) * rhs.0 * $factor / $divider).into()
            }
        }

        impl core::ops::Mul<$time_large> for $freq {
            type Output = TicksU64;
            fn mul(self, rhs: $time_large) -> Self::Output {
                (LargeValueType::from(self.0) * rhs.0 * $factor / $divider).into()
            }
        }
        )*
    };
}

macro_rules! divide {
    ($( ($freq: ty, $freq_large: ty, $time: ty, $time_large: ty, $factor: expr) ),+) => {
        $(
        impl core::ops::Div<$freq> for Ticks {
            type Output = $time;
            fn div(self, rhs: $freq) -> Self::Output {
                ValueType::try_from(
                    (LargeValueType::from(self.0) * $factor / LargeValueType::from(rhs.0))
                ).unwrap().into()
            }
        }

        impl core::ops::Div<$freq> for TicksU64 {
            type Output = $time_large;
            fn div(self, rhs: $freq) -> Self::Output {
                (self.0 * $factor / LargeValueType::from(rhs.0) ).into()
            }
        }

        impl core::ops::Div<$freq_large> for TicksU64 {
            type Output = $time_large;
            fn div(self, rhs: $freq_large) -> Self::Output {
                (self.0 * $factor / rhs.0 ).into()
            }
        }

        impl core::ops::Div<$freq_large> for Ticks {
            type Output = $time_large;
            fn div(self, rhs: $freq_large) -> Self::Output {
                (LargeValueType::from(self.0) * $factor / rhs.0).into()
            }
        }
        )*
    };
}

#[rustfmt::skip::macros(define_large)]
define_large!(
    (Frequency, Hertz,        Hz,    FrequencyU64, HertzU64,        Hz_large,    "Hz"  ),
    (Frequency, KiloHertz,    kHz,   FrequencyU64, KiloHertzU64,    kHz_large,   "kHz" ),
    (Frequency, MegaHertz,    MHz,   FrequencyU64, MegaHertzU64,    MHz_large,   "MHz" ),
    (Time,      NanoSeconds,  ns,    TimeU64,      NanoSecondsU64,  ns_large,    "ns"  ),
    (Time,      MicroSeconds, us,    TimeU64,      MicroSecondsU64, us_large,    "us"  ),
    (Time,      MilliSeconds, ms,    TimeU64,      MilliSecondsU64, ms_large,    "ms"  ),
    (Time,      Seconds,      s,     TimeU64,      SecondsU64,      s_large,     "s"   ),
    (Count,     Ticks,        ticks, CountU64,     TicksU64,        ticks_large, ""    )
);

#[rustfmt::skip::macros(convert)]
convert!(
    (KiloHertz,    KiloHertzU64,    Hertz,        HertzU64,        1_000         ),
    (MegaHertz,    MegaHertzU64,    Hertz,        HertzU64,        1_000_000     ),
    (MegaHertz,    MegaHertzU64,    KiloHertz,    KiloHertzU64,    1_000         ),
    (Seconds,      SecondsU64,      MilliSeconds, MilliSecondsU64, 1_000         ),
    (Seconds,      SecondsU64,      MicroSeconds, MicroSecondsU64, 1_000_000     ),
    (Seconds,      SecondsU64,      NanoSeconds,  NanoSecondsU64,  1_000_000_000 ),
    (MilliSeconds, MilliSecondsU64, MicroSeconds, MicroSecondsU64, 1_000         ),
    (MilliSeconds, MilliSecondsU64, NanoSeconds,  NanoSecondsU64,  1_000_000     ),
    (MicroSeconds, MicroSecondsU64, NanoSeconds,  NanoSecondsU64,  1_000         )
);

#[rustfmt::skip::macros(multiply)]
multiply!(
    (Seconds,      SecondsU64,      Hertz,     HertzU64,     1,         1             ),
    (Seconds,      SecondsU64,      KiloHertz, KiloHertzU64, 1_000,     1             ),
    (Seconds,      SecondsU64,      MegaHertz, MegaHertzU64, 1_000_000, 1             ),
    (MilliSeconds, MilliSecondsU64, Hertz,     HertzU64,     1,         1_000         ),
    (MilliSeconds, MilliSecondsU64, KiloHertz, KiloHertzU64, 1,         1             ),
    (MilliSeconds, MilliSecondsU64, MegaHertz, MegaHertzU64, 1_000,     1             ),
    (MicroSeconds, MicroSecondsU64, Hertz,     HertzU64,     1,         1_000_000     ),
    (MicroSeconds, MicroSecondsU64, KiloHertz, KiloHertzU64, 1,         1_000         ),
    (MicroSeconds, MicroSecondsU64, MegaHertz, MegaHertzU64, 1,         1             ),
    (NanoSeconds,  NanoSecondsU64,  Hertz,     HertzU64,     1,         1_000_000_000 ),
    (NanoSeconds,  NanoSecondsU64,  KiloHertz, KiloHertzU64, 1,         1_000_000     ),
    (NanoSeconds,  NanoSecondsU64,  MegaHertz, MegaHertzU64, 1,         1_000         )
);

#[rustfmt::skip::macros(divide)]
divide!(
    (Hertz,     HertzU64,     NanoSeconds, NanoSecondsU64, 1_000_000_000 ),
    (KiloHertz, KiloHertzU64, NanoSeconds, NanoSecondsU64, 1_000_000     ),
    (MegaHertz, MegaHertzU64, NanoSeconds, NanoSecondsU64, 1_000         )
);
