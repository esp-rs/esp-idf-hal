use core::cmp::min;

use esp_idf_sys::c_types;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum X509<'a> {
    Der(&'a [u8]),
    Pem(&'a [u8]),
}

impl<'a> X509<'a> {
    const SCAN_LAST_CHARS: usize = 16;

    pub(crate) fn as_raw_ptr(&self) -> *const c_types::c_char {
        match self {
            Self::Der(data) => data.as_ptr() as _,
            Self::Pem(data) => {
                Self::check_pem(data);

                data.as_ptr() as _
            }
        }
    }

    pub(crate) fn as_raw_len(&self) -> u32 {
        match self {
            Self::Der(data) => data.len() as _,
            Self::Pem(data) => {
                Self::check_pem(data);

                0
            }
        }
    }

    fn check_pem(data: &[u8]) {
        if data
            .iter()
            .rev()
            .take(min(Self::SCAN_LAST_CHARS, data.len()))
            .find(|c| **c == 0)
            .is_none()
        {
            panic!("PEM certificates should end with a NIL (`\\0`) ASCII character. No NIL found in the last {} bytes", Self::SCAN_LAST_CHARS);
        }
    }
}
