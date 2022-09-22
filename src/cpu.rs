#[cfg(any(esp32, esp32s3))]
use core::arch::asm;

use esp_idf_sys::*;

/// Returns the number of cores supported by the esp32* chip
pub const CORES: u32 = SOC_CPU_CORES_NUM;

#[repr(C)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Core {
    Core0 = 0, // PRO on dual-core systems, the one and only CPU on single-core systems
    #[cfg(any(esp32, esp32s3))]
    Core1 = 1, // APP on dual-core systems
}

impl Core {
    #[inline(always)]
    #[link_section = ".iram1.cpu_core"]
    pub fn is_active(&self) -> bool {
        *self == core()
    }
}

impl From<Core> for i32 {
    fn from(core: Core) -> Self {
        core as _
    }
}

impl From<i32> for Core {
    fn from(core: i32) -> Self {
        match core {
            0 => Core::Core0,
            #[cfg(any(esp32, esp32s3))]
            1 => Core::Core1,
            _ => panic!(),
        }
    }
}

/// Returns the currently active core ID
/// On single-core systems, like esp32s2 and esp32c3 this function always returns 0
///
/// On dual-core systems like esp32 and esp32s3 this function returns:
/// 0 - when the active core is the PRO CPU
/// 1 - when the active core is the APP CPU
#[inline(always)]
#[link_section = ".iram1.cpu_core"]
pub fn core() -> Core {
    #[cfg(any(esp32c3, esp32s2))]
    let core = 0;

    #[allow(unused_assignments)]
    #[cfg(any(esp32, esp32s3))]
    let mut core = 0;

    #[cfg(any(esp32, esp32s3))]
    unsafe {
        asm!("rsr.prid {0}", "extui {0},{0},13,1", out(reg) core);
    }

    match core {
        0 => Core::Core0,
        #[cfg(any(esp32, esp32s3))]
        1 => Core::Core1,
        other => panic!("Unknown core: {}", other),
    }
}
