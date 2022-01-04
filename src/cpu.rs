use esp_idf_sys::*;

/// Returns the number of cores supported by the esp32* chip
pub const CORES: u32 = SOC_CPU_CORES_NUM;

/// Returns the currently active core ID
/// On single-core systems, like esp32s2 and esp32c3 this function always returns 0
///
/// On dual-core systems like esp32 and esp32s3 this function returns:
/// 0 - when the active core is the PRO CPU
/// 1 - when the active core is the APP CPU
#[inline(always)]
#[link_section = ".rwtext"]
pub fn core() -> u32 {
    #[cfg(any(esp32c3, esp32s2))]
    let core = 0;

    #[allow(unused_assignments)]
    #[cfg(any(esp32, esp32s3))]
    let mut core = 0;

    #[cfg(any(esp32, esp32s3))]
    #[allow(deprecated)]
    unsafe {
        llvm_asm!("rsr.prid $0\nextui $0,$0,13,1" : "=r"(core) : : : "volatile");
    }

    core
}
