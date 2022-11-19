use core::cmp::min;

use esp_idf_sys::c_types;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum X509<'a> {
    Der(&'a [u8]),
    Pem(&'a [u8]),
}

impl<'a> X509<'a> {
    const SCAN_LAST_BYTES: usize = 16;

    pub fn data(&self) -> &[u8] {
        match self {
            Self::Der(data) => data,
            Self::Pem(data) => data,
        }
    }

    pub(crate) fn as_esp_idf_raw_ptr(&self) -> *const c_types::c_char {
        self.check();
        self.data().as_ptr() as _
    }

    pub(crate) fn as_esp_idf_raw_len(&self) -> u32 {
        match self {
            Self::Der(data) => data.len() as _,
            Self::Pem(_) => 0,
        }
    }

    fn check(&self) {
        if matches!(self, Self::Pem(_)) {
            let data = self.data();

            if !data
                .iter()
                .rev()
                .take(min(Self::SCAN_LAST_BYTES, data.len()))
                .any(|c| *c == 0)
            {
                panic!("PEM certificates should end with a NIL (`\\0`) ASCII character. No NIL found in the last {} bytes", Self::SCAN_LAST_BYTES);
            }
        }
    }
}
