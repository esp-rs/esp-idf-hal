//! Cyclic Redundancy Check
//!
//! These are safe abstractions to the CRC functions in the ESP32 ROM.
//! Some chips may not include all of these functions so they will be compiled
//! into the program binary in those cases.
//!
//! # Parameters
//!
//! The ROM provides the following polynomials for each CRC width:
//!
//! | CRC Width | Polynomial  |
//! | --------- | ----------- |
//! | CRC-8     | 0x07        |
//! | CRC-16    | 0x1021      |
//! | CRC-32    | 0x04c11db7  |
//!
//! The "big-endian" `*_be()` functions are left-shifting algorithms to be used
//! when input and output reflection are **not** needed. If input and output
//! reflection **are** needed, the right-shifting "little-endian" `*_le()`
//! functions should be used.
//!
//! These functions are designed to compute a CRC over a single buffer or as an
//! ongoing calculation over multiple buffers. To do this, the initial value
//! passed in and the final value returned are one's complemented.
//!
//! ```
//! // CRC-32/MPEG-2
//! const CRC_INITIAL = 0xffffffff; // "init" or "xorin" of all ones
//! let mut crc = crc32_be(!CRC_INITIAL, &data0); // start
//! crc = crc32_be(crc, &data1);
//! crc = !crc32_be(crc, &data2); // finish
//! ```
//!
//! # Examples
//!
//! A catalogue of these parameters can be found at
//! <https://reveng.sourceforge.io/crc-catalogue/all.htm>
//!
//! CRC-32/ISO-HDLC poly=0x04c11db7 init=0xffffffff refin=true refout=true xorout=0xffffffff
//!
//! ```
//! let crc = crc32_le(!0xffffffff, &data);
//! ```
//!
//! CRC-32/BZIP2 poly=0x04c11db7 init=0xffffffff refin=false refout=false xorout=0xffffffff
//!
//! ```
//! let crc = crc32_be(!0xffffffff, &data);
//! ```
//!
//! CRC-32/MPEG-2 poly=0x04c11db7 init=0xffffffff refin=false refout=false xorout=0x00000000
//!
//! ```
//! let crc = !crc32_be(!0xffffffff, &data);
//! ```
//!
//! CRC-32/CKSUM poly=0x04c11db7 init=0x00000000 refin=false refout=false xorout=0xffffffff
//!
//! ```
//! let crc = crc32_be(!0, &data);
//! ```
//!
//! CRC-16/KERMIT poly=0x1021 init=0x0000 refin=true refout=true xorout=0x0000
//!
//! ```
//! let crc = !crc16_le(!0, &data);
//! ```

use esp_idf_sys::*;

// SAFETY: These functions are all implemented as table lookups. No locking is
// needed to access them, they are all referentially transparent, and the size
// and alignment of `usize` and `u32` are identical on all ESP32 chips.

/// Right-shifting CRC-32 with polynomial 0x04c11db7
#[inline(always)]
pub fn crc32_le(crc: u32, buf: &[u8]) -> u32 {
    unsafe { esp_rom_crc32_le(crc, buf.as_ptr(), buf.len() as u32) }
}

/// Left-shifting CRC-32 with polynomial 0x04c11db7
#[inline(always)]
pub fn crc32_be(crc: u32, buf: &[u8]) -> u32 {
    unsafe { esp_rom_crc32_be(crc, buf.as_ptr(), buf.len() as u32) }
}

/// Right-shifting CRC-16 with polynomial 0x1021
#[inline(always)]
pub fn crc16_le(crc: u16, buf: &[u8]) -> u16 {
    unsafe { esp_rom_crc16_le(crc, buf.as_ptr(), buf.len() as u32) }
}

/// Left-shifting CRC-16 with polynomial 0x1021
#[inline(always)]
pub fn crc16_be(crc: u16, buf: &[u8]) -> u16 {
    unsafe { esp_rom_crc16_be(crc, buf.as_ptr(), buf.len() as u32) }
}

/// Right-shifting CRC-8 with polynomial 0x07
#[inline(always)]
pub fn crc8_le(crc: u8, buf: &[u8]) -> u8 {
    unsafe { esp_rom_crc8_le(crc, buf.as_ptr(), buf.len() as u32) }
}

/// Left-shifting CRC-8 with polynomial 0x07
#[inline(always)]
pub fn crc8_be(crc: u8, buf: &[u8]) -> u8 {
    unsafe { esp_rom_crc8_be(crc, buf.as_ptr(), buf.len() as u32) }
}
