use core::marker::PhantomData;

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
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
    all(esp_idf_version_major = "4", esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp_idf_version_major = "4",
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp_idf_version_major = "4",
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
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
    all(esp_idf_version_major = "4", esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp_idf_version_major = "4",
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp_idf_version_major = "4",
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
    all(not(esp_idf_version_major = "4"), esp_idf_ulp_coproc_enabled),
    all(esp_idf_version_major = "4", esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp_idf_version_major = "4",
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled
    ),
    all(
        esp_idf_version_major = "4",
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled
    )
))]
impl ULP {
    const RTC_SLOW_MEM: u32 = 0x5000_0000_u32;

    pub const MEM_START_ULP: *mut core::ffi::c_void = 0_u32 as _;

    pub const MEM_START: *mut core::ffi::c_void = Self::RTC_SLOW_MEM as _;

    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub const MEM_SIZE: usize = esp_idf_sys::CONFIG_ESP32_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(all(esp32s2, esp_idf_version_major = "4"))]
    pub const MEM_SIZE: usize = esp_idf_sys::CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(all(esp32s3, esp_idf_version_major = "4"))]
    pub const MEM_SIZE: usize = esp_idf_sys::CONFIG_ESP32S3_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(not(esp_idf_version_major = "4"))]
    pub const MEM_SIZE: usize = esp_idf_sys::CONFIG_ULP_COPROC_RESERVE_MEM as _;

    #[cfg(esp32)]
    const TIMER_REG: *mut u32 = esp_idf_sys::RTC_CNTL_STATE0_REG as _;

    #[cfg(any(esp32s2, esp32s3))]
    const TIMER_REG: *mut u32 = esp_idf_sys::RTC_CNTL_ULP_CP_TIMER_REG as _;

    #[cfg(any(esp32, esp32s2, esp32s3))]
    const TIMER_EN_BIT: u32 =
        esp_idf_sys::RTC_CNTL_ULP_CP_SLP_TIMER_EN_V << esp_idf_sys::RTC_CNTL_ULP_CP_SLP_TIMER_EN_S;

    pub fn stop(&mut self) -> Result<(), esp_idf_sys::EspError> {
        unsafe {
            // disable ULP timer
            core::ptr::write_volatile(
                Self::TIMER_REG,
                core::ptr::read_volatile(Self::TIMER_REG) & !Self::TIMER_EN_BIT,
            );

            // wait for at least 1 RTC_SLOW_CLK cycle
            esp_idf_sys::esp_rom_delay_us(10);
        }

        Ok(())
    }

    pub fn is_started(&self) -> Result<bool, esp_idf_sys::EspError> {
        unsafe {
            let enabled = (core::ptr::read_volatile(Self::TIMER_REG) & Self::TIMER_EN_BIT) != 0;

            Ok(enabled)
        }
    }

    pub fn set_sleep_period_default(
        &mut self,
        duration: core::time::Duration,
    ) -> Result<(), esp_idf_sys::EspError> {
        self.set_sleep_period(Default::default(), duration)
    }

    pub fn set_sleep_period(
        &mut self,
        timer: SleepTimer,
        duration: core::time::Duration,
    ) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(unsafe {
            esp_idf_sys::ulp_set_wakeup_period(
                timer as esp_idf_sys::size_t,
                duration.as_micros() as u32,
            )
        })?;

        Ok(())
    }

    fn check_boundaries<T>(ptr: *const T) -> Result<(), esp_idf_sys::EspError> {
        let ptr = ptr as *const u8;
        let mem_start = Self::MEM_START as *const u8;

        unsafe {
            if ptr < mem_start
                || ptr.offset(core::mem::size_of::<T>() as _)
                    > mem_start.offset(Self::MEM_SIZE as _)
            {
                esp_idf_sys::esp!(esp_idf_sys::ESP_ERR_INVALID_SIZE)?;
            }
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
    all(esp_idf_version_major = "4", esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(
        esp_idf_version_major = "4",
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        not(esp_idf_esp32s2_ulp_coproc_riscv)
    ),
    all(
        esp_idf_version_major = "4",
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        not(esp_idf_esp32s3_ulp_coproc_riscv)
    )
))]
impl ULP {
    pub unsafe fn load_at_ulp_address(
        &mut self,
        address: *mut core::ffi::c_void,
        program: &[u8],
    ) -> Result<(), esp_idf_sys::EspError> {
        let address: usize = core::mem::transmute(address);
        if address % core::mem::size_of::<u32>() != 0 {
            esp_idf_sys::esp!(esp_idf_sys::ESP_ERR_INVALID_ARG)?;
        }

        if program.len() % core::mem::size_of::<u32>() != 0 {
            esp_idf_sys::esp!(esp_idf_sys::ESP_ERR_INVALID_SIZE)?;
        }

        esp_idf_sys::esp!(esp_idf_sys::ulp_load_binary(
            (address / core::mem::size_of::<u32>()) as u32,
            program.as_ptr(),
            (program.len() / core::mem::size_of::<u32>()) as u32
        ))?;

        Ok(())
    }

    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), esp_idf_sys::EspError> {
        self.load_at_ulp_address(Self::MEM_START_ULP, program)
    }

    pub unsafe fn start(&mut self, address: *const u32) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_run(address as u32))?;

        Ok(())
    }

    pub unsafe fn read_word(&self, ptr: *const u32) -> Result<Word, esp_idf_sys::EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        let value = core::ptr::read_volatile(ptr);

        Ok(Word {
            pc: ((value >> 16) & 0xffff_u32) as u16,
            value: (value & 0xffff_u32) as u16,
        })
    }

    pub unsafe fn write_word(
        &self,
        ptr: *mut u32,
        value: u16,
    ) -> Result<(), esp_idf_sys::EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        core::ptr::write_volatile(ptr as *mut u32, value as u32);

        Ok(())
    }

    pub unsafe fn swap_word(
        &mut self,
        ptr: *mut u32,
        value: u16,
    ) -> Result<Word, esp_idf_sys::EspError> {
        let old_value = self.read_word(ptr)?;

        self.write_word(ptr, value)?;

        Ok(old_value)
    }

    fn check_alignment<T>(ptr: *const T) -> Result<(), esp_idf_sys::EspError> {
        let ptr_usize: usize = unsafe { core::mem::transmute(ptr) };

        if ptr_usize % core::mem::size_of::<T>() != 0 {
            esp_idf_sys::esp!(esp_idf_sys::ESP_ERR_INVALID_SIZE)?;
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        not(esp_idf_ulp_coproc_type_fsm)
    ),
    all(
        esp_idf_version_major = "4",
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        esp_idf_esp32s2_ulp_coproc_riscv
    ),
    all(
        esp_idf_version_major = "4",
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        esp_idf_esp32s3_ulp_coproc_riscv
    )
))]
impl ULP {
    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_riscv_load_binary(
            program.as_ptr(),
            program.len() as _
        ))?;

        Ok(())
    }

    pub unsafe fn start(&mut self) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_riscv_run())?;

        Ok(())
    }

    pub unsafe fn read_var<T>(&self, src: *const T) -> Result<T, esp_idf_sys::EspError> {
        Self::check_boundaries(src)?;

        Ok(core::ptr::read_volatile(src))
    }

    pub unsafe fn write_var<T>(&self, dst: *mut T, value: T) -> Result<(), esp_idf_sys::EspError> {
        Self::check_boundaries(dst)?;

        core::ptr::write_volatile(dst, value);

        Ok(())
    }

    pub unsafe fn swap_var<T>(
        &mut self,
        ptr: *mut T,
        value: T,
    ) -> Result<T, esp_idf_sys::EspError> {
        let old_value = self.read_var(ptr)?;

        self.write_var(ptr, value)?;

        Ok(old_value)
    }
}
