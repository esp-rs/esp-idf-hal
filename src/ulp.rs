use core::ffi::c_void;
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::time::Duration;

use esp_idf_sys::*;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum SleepTimer {
    First = 0,
    Second = 1,
    Third = 2,
    Fourth = 3,
    Fifth = 4,
    #[cfg(esp32s2)]
    Sixth = 5,
}

impl Default for SleepTimer {
    fn default() -> Self {
        Self::First
    }
}

#[cfg(any(
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        not(esp_idf_esp32s3_ulp_coproc_riscv)
    )
))]
#[derive(Copy, Clone, Debug)]
pub struct Word {
    pc: u16,
    value: u16,
}

#[cfg(any(
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        not(esp_idf_esp32s3_ulp_coproc_riscv)
    )
))]
impl Word {
    pub fn pc(&self) -> u16 {
        self.pc
    }

    pub fn value(&self) -> u16 {
        self.value
    }

    pub fn is_modified(&self) -> bool {
        self.pc != 0
    }
}

pub struct ULP(PhantomData<*const ()>);

unsafe impl Send for ULP {}

impl ULP {
    /// # Safety
    ///
    /// Care should be taken not to instantiate the ULP instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Self(PhantomData)
    }
}

#[cfg(any(
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
impl ULP {
    const RTC_SLOW_MEM: u32 = 0x50000000_u32;

    pub const MEM_START: *mut c_void = Self::RTC_SLOW_MEM as _;

    #[cfg(esp32)]
    pub const MEM_SIZE: usize = CONFIG_ESP32_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(esp32s2)]
    pub const MEM_SIZE: usize = CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(esp32s3)]
    pub const MEM_SIZE: usize = CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(esp32)]
    const TIMER_REG: *mut u32 = RTC_CNTL_STATE0_REG as _;

    #[cfg(any(esp32s2, esp32s3))]
    const TIMER_REG: *mut u32 = RTC_CNTL_ULP_CP_TIMER_REG;

    #[cfg(any(esp32, esp32s2, esp32s3))]
    const TIMER_EN_BIT: u32 = RTC_CNTL_ULP_CP_SLP_TIMER_EN_V << RTC_CNTL_ULP_CP_SLP_TIMER_EN_S;

    pub fn stop(&mut self) -> Result<(), EspError> {
        unsafe {
            // disable ULP timer
            ptr::write_volatile(
                Self::TIMER_REG,
                ptr::read_volatile(Self::TIMER_REG) & !Self::TIMER_EN_BIT,
            );

            // wait for at least 1 RTC_SLOW_CLK cycle
            esp_rom_delay_us(10);
        }

        Ok(())
    }

    pub fn is_started(&self) -> Result<bool, EspError> {
        unsafe {
            let enabled = (ptr::read_volatile(Self::TIMER_REG) & Self::TIMER_EN_BIT) != 0;

            Ok(enabled)
        }
    }

    pub fn set_sleep_period_default(&mut self, duration: Duration) -> Result<(), EspError> {
        self.set_sleep_period(Default::default(), duration)
    }

    pub fn set_sleep_period(
        &mut self,
        timer: SleepTimer,
        duration: Duration,
    ) -> Result<(), EspError> {
        esp!(unsafe { ulp_set_wakeup_period(timer as size_t, duration.as_micros() as u32) })?;

        Ok(())
    }

    fn check_boundaries<T>(ptr: *const T) -> Result<(), EspError> {
        let ptr = ptr as *const u8;
        let mem_start = Self::MEM_START as *const u8;

        unsafe {
            if ptr < mem_start
                || ptr.offset(mem::size_of::<T>() as _) > mem_start.offset(Self::MEM_SIZE as _)
            {
                esp!(ESP_ERR_INVALID_SIZE)?;
            }
        }

        Ok(())
    }
}

#[cfg(any(
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        not(esp_idf_esp32s3_ulp_coproc_riscv)
    )
))]
impl ULP {
    pub unsafe fn load_at_ulp_address(
        &mut self,
        address: *mut c_void,
        program: &[u8],
    ) -> Result<(), EspError> {
        let address: usize = std::mem::transmute(address);
        if address % mem::size_of::<u32>() != 0 {
            esp!(ESP_ERR_INVALID_ARG)?;
        }

        if program.len() % mem::size_of::<u32>() != 0 {
            esp!(ESP_ERR_INVALID_SIZE)?;
        }

        esp!(ulp_load_binary(
            (address / mem::size_of::<u32>()) as u32,
            program.as_ptr(),
            (program.len() / mem::size_of::<u32>()) as u32
        ))?;

        Ok(())
    }

    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), EspError> {
        self.load_at_ulp_address(Self::MEM_START, program)
    }

    pub unsafe fn start(&mut self, address: *mut c_void) -> Result<(), EspError> {
        esp!(ulp_run(address as u32))?;

        Ok(())
    }

    pub unsafe fn read_word(&self, ptr: *const u32) -> Result<Word, EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        let value = ptr::read_volatile(ptr);

        Ok(Word {
            pc: ((value >> 16) & 0xffff_u32) as u16,
            value: (value & 0xffff_u32) as u16,
        })
    }

    pub unsafe fn write_word(&self, ptr: *mut u32, value: u16) -> Result<(), EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        ptr::write_volatile(ptr as *mut u32, value as u32);

        Ok(())
    }

    pub unsafe fn swap_word(&mut self, ptr: *mut u32, value: u16) -> Result<Word, EspError> {
        let old_value = self.read_word(ptr)?;

        self.write_word(ptr, value)?;

        Ok(old_value)
    }

    fn check_alignment<T>(ptr: *const T) -> Result<(), EspError> {
        let ptr_usize: usize = unsafe { std::mem::transmute(ptr) };

        if ptr_usize % mem::size_of::<T>() != 0 {
            esp!(ESP_ERR_INVALID_SIZE)?;
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        esp_idf_esp32s2_ulp_coproc_riscv
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        esp_idf_esp32s3_ulp_coproc_riscv
    )
))]
impl ULP {
    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), EspError> {
        esp!(ulp_riscv_load_binary(program.as_ptr(), program.len() as _))?;

        Ok(())
    }

    pub unsafe fn start(&mut self) -> Result<(), EspError> {
        esp!(ulp_riscv_run())?;

        Ok(())
    }

    pub unsafe fn read_var<T>(&self, src: *const T) -> Result<T, EspError> {
        Self::check_boundaries(src)?;

        Ok(ptr::read_volatile(src))
    }

    pub unsafe fn write_var(&self, dst: *mut T, value: T) -> Result<(), EspError> {
        Self::check_boundaries(dst)?;

        ptr::write_volatile(dst, value);

        Ok(())
    }

    pub unsafe fn swap_var(&mut self, ptr: *mut T, value: T) -> Result<T, EspError> {
        let old_value = self.read_var(ptr)?;

        self.write_var(ptr, value)?;

        Ok(old_value)
    }
}
