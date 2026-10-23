#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
pub enum SleepTimer {
    #[default]
    First = 0,
    Second = 1,
    Third = 2,
    Fourth = 3,
    Fifth = 4,
    #[cfg(esp32s2)]
    Sixth = 5,
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
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
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
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

/// The events which wake the LP core up
#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
// The discriminants are the bit indexes of the `ULP_LP_CORE_WAKEUP_SOURCE_*` flags
// in `ulp_lp_core.h`, which bindgen cannot evaluate, as they are defined with `BIT()`
#[derive(Debug, enumset::EnumSetType)]
#[repr(u32)]
pub enum LpCoreWakeupSource {
    /// The HP core, once when the LP core is started, and then on each `UlpDriver::trigger` call
    HpCpu = 0,
    /// A number of RX pulses on the LP UART
    LpUart = 1,
    /// An LP IO interrupt
    LpIo = 2,
    /// An ETM event
    Etm = 3,
    /// The LP timer, every `LpCoreConfig::sleep_period`
    LpTimer = 4,
    /// The LP voice activity detection
    #[cfg(esp_idf_soc_lp_vad_supported)]
    LpVad = 5,
}

#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
impl LpCoreWakeupSource {
    const fn flag(&self) -> u32 {
        1 << *self as u32
    }
}

/// How `UlpDriver::start` runs the LP core
#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
#[derive(Debug, Clone)]
pub struct LpCoreConfig {
    /// The events which wake the LP core up; must not be empty
    pub wakeup_sources: enumset::EnumSet<LpCoreWakeupSource>,
    /// The LP timer period, used when `wakeup_sources` contains `LpCoreWakeupSource::LpTimer`.
    /// With a zero period the LP timer wakes the LP core up once, unless the LP program
    /// re-arms it with `ulp_lp_core_lp_timer_set_wakeup_time()`
    pub sleep_period: core::time::Duration,
    /// Boot straight into the program in LP RAM, skipping the LP ROM. Boots faster, but
    /// skips the LP ROM setup, e.g. of the LP UART
    #[cfg(esp_idf_esp_rom_has_lp_rom)]
    pub skip_lp_rom_boot: bool,
}

#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
impl LpCoreConfig {
    pub const fn new() -> Self {
        Self {
            wakeup_sources: enumset::enum_set!(LpCoreWakeupSource::HpCpu),
            sleep_period: core::time::Duration::ZERO,
            #[cfg(esp_idf_esp_rom_has_lp_rom)]
            skip_lp_rom_boot: false,
        }
    }

    #[must_use]
    pub fn wakeup_sources(mut self, wakeup_sources: enumset::EnumSet<LpCoreWakeupSource>) -> Self {
        self.wakeup_sources = wakeup_sources;
        self
    }

    #[must_use]
    pub fn sleep_period(mut self, sleep_period: core::time::Duration) -> Self {
        self.sleep_period = sleep_period;
        self
    }

    #[cfg(esp_idf_esp_rom_has_lp_rom)]
    #[must_use]
    pub fn skip_lp_rom_boot(mut self, skip_lp_rom_boot: bool) -> Self {
        self.skip_lp_rom_boot = skip_lp_rom_boot;
        self
    }
}

#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
impl Default for LpCoreConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_ulp_coproc_enabled),
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
pub struct UlpDriver<'d>(core::marker::PhantomData<&'d mut ()>);

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_ulp_coproc_enabled),
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
unsafe impl<'d> Send for UlpDriver<'d> {}

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_ulp_coproc_enabled),
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
impl<'d> UlpDriver<'d> {
    pub fn new(_ulp: ULP<'d>) -> Result<Self, esp_idf_sys::EspError> {
        Ok(Self(core::marker::PhantomData))
    }

    fn check_boundaries<T>(ptr: *const T) -> Result<(), esp_idf_sys::EspError> {
        let ptr = ptr as *const u8;
        let mem_start = ULP::mem_start() as *const u8;

        if ptr < mem_start
            || ptr.wrapping_add(core::mem::size_of::<T>()) > mem_start.wrapping_add(ULP::MEM_SIZE)
        {
            return Err(esp_idf_sys::EspError::from_infallible::<
                { esp_idf_sys::ESP_ERR_INVALID_SIZE },
            >());
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        any(esp_idf_ulp_coproc_type_fsm, esp_idf_ulp_coproc_type_riscv)
    ),
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
impl<'d> UlpDriver<'d> {
    pub fn stop(&mut self) -> Result<(), esp_idf_sys::EspError> {
        unsafe {
            // disable ULP timer
            core::ptr::write_volatile(
                ULP::TIMER_REG,
                core::ptr::read_volatile(ULP::TIMER_REG) & !ULP::TIMER_EN_BIT,
            );

            // wait for at least 1 RTC_SLOW_CLK cycle
            esp_idf_sys::esp_rom_delay_us(10);
        }

        Ok(())
    }

    pub fn is_started(&self) -> Result<bool, esp_idf_sys::EspError> {
        unsafe {
            let enabled = (core::ptr::read_volatile(ULP::TIMER_REG) & ULP::TIMER_EN_BIT) != 0;

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
            esp_idf_sys::ulp_set_wakeup_period(timer as usize, duration.as_micros() as u32)
        })?;

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_fsm
    ),
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
impl<'d> UlpDriver<'d> {
    /// # Safety
    ///
    /// `program` must be a binary built for the ULP. Loading it overwrites the memory of the ULP
    /// at `address`, so the ULP must not be running.
    pub unsafe fn load_at_ulp_address(
        &mut self,
        address: *mut core::ffi::c_void,
        program: &[u8],
    ) -> Result<(), esp_idf_sys::EspError> {
        let address = address as usize;
        if address % core::mem::size_of::<u32>() != 0 {
            return Err(esp_idf_sys::EspError::from_infallible::<
                { esp_idf_sys::ESP_ERR_INVALID_ARG },
            >());
        }

        if program.len() % core::mem::size_of::<u32>() != 0 {
            return Err(esp_idf_sys::EspError::from_infallible::<
                { esp_idf_sys::ESP_ERR_INVALID_SIZE },
            >());
        }

        esp_idf_sys::esp!(esp_idf_sys::ulp_load_binary(
            (address / core::mem::size_of::<u32>()) as u32,
            program.as_ptr(),
            program.len() / core::mem::size_of::<u32>()
        ))?;

        Ok(())
    }

    /// # Safety
    ///
    /// `program` must be a binary built for the ULP. Loading it overwrites the memory reserved
    /// for the ULP, so the ULP must not be running.
    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), esp_idf_sys::EspError> {
        self.load_at_ulp_address(ULP::MEM_START_ULP, program)
    }

    /// # Safety
    ///
    /// `address` must be the entry point of a program loaded with `load` or `load_at_ulp_address`.
    pub unsafe fn start(&mut self, address: *const u32) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_run(address as u32))?;

        Ok(())
    }

    /// # Safety
    ///
    /// `ptr` must point to a word of the program loaded in the memory reserved for the ULP.
    pub unsafe fn read_word(&self, ptr: *const u32) -> Result<Word, esp_idf_sys::EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        let value = core::ptr::read_volatile(ptr);

        Ok(Word {
            pc: ((value >> 16) & 0xffff_u32) as u16,
            value: (value & 0xffff_u32) as u16,
        })
    }

    /// # Safety
    ///
    /// `ptr` must point to a word of the program loaded in the memory reserved for the ULP.
    pub unsafe fn write_word(
        &self,
        ptr: *mut u32,
        value: u16,
    ) -> Result<(), esp_idf_sys::EspError> {
        Self::check_boundaries(ptr)?;
        Self::check_alignment(ptr)?;

        core::ptr::write_volatile(ptr, value as u32);

        Ok(())
    }

    /// # Safety
    ///
    /// `ptr` must point to a word of the program loaded in the memory reserved for the ULP.
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
        let ptr_usize = ptr as usize;

        if ptr_usize % core::mem::size_of::<T>() != 0 {
            return Err(esp_idf_sys::EspError::from_infallible::<
                { esp_idf_sys::ESP_ERR_INVALID_SIZE },
            >());
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        esp_idf_ulp_coproc_type_riscv
    ),
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        esp_idf_esp32s2_ulp_coproc_riscv
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        esp_idf_esp32s3_ulp_coproc_riscv
    ),
))]
impl<'d> UlpDriver<'d> {
    /// # Safety
    ///
    /// `program` must be a binary built for the ULP. Loading it overwrites the memory reserved
    /// for the ULP, so the ULP must not be running.
    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_riscv_load_binary(
            program.as_ptr(),
            program.len() as _
        ))?;

        Ok(())
    }

    /// # Safety
    ///
    /// A program must be loaded with `load` first.
    pub unsafe fn start(&mut self) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_riscv_run())?;

        Ok(())
    }
}

#[cfg(all(
    not(esp_idf_version_major = "4"),
    esp_idf_ulp_coproc_enabled,
    esp_idf_ulp_coproc_type_lp_core
))]
impl<'d> UlpDriver<'d> {
    /// Stops the LP core, then loads `program` at the start of the memory reserved for it
    ///
    /// # Safety
    ///
    /// `program` must be a binary built for the LP core.
    pub unsafe fn load(&mut self, program: &[u8]) -> Result<(), esp_idf_sys::EspError> {
        esp_idf_sys::esp!(esp_idf_sys::ulp_lp_core_load_binary(
            program.as_ptr(),
            program.len() as _
        ))?;

        Ok(())
    }

    /// # Safety
    ///
    /// A program must be loaded with `load` first.
    pub unsafe fn start(&mut self, config: &LpCoreConfig) -> Result<(), esp_idf_sys::EspError> {
        let mut cfg = esp_idf_sys::ulp_lp_core_cfg_t {
            wakeup_source: config
                .wakeup_sources
                .iter()
                .fold(0, |flags, source| flags | source.flag()),
            lp_timer_sleep_duration_us: config.sleep_period.as_micros() as u32,
            #[cfg(esp_idf_esp_rom_has_lp_rom)]
            skip_lp_rom_boot: config.skip_lp_rom_boot,
        };

        esp_idf_sys::esp!(esp_idf_sys::ulp_lp_core_run(&mut cfg))?;

        Ok(())
    }

    /// Disables all wake-up sources and puts the LP core to sleep
    pub fn stop(&mut self) -> Result<(), esp_idf_sys::EspError> {
        unsafe {
            esp_idf_sys::ulp_lp_core_stop();
        }

        Ok(())
    }

    /// Wakes the LP core up, if it was started with `LpCoreWakeupSource::HpCpu`
    #[cfg(esp_idf_version_at_least_5_3_0)]
    pub fn trigger(&mut self) -> Result<(), esp_idf_sys::EspError> {
        #[cfg(not(esp_idf_version_at_least_6_0_0))]
        unsafe {
            esp_idf_sys::ulp_lp_core_sw_intr_trigger();
        }

        #[cfg(esp_idf_version_at_least_6_0_0)]
        unsafe {
            esp_idf_sys::ulp_lp_core_sw_intr_to_lp_trigger();
        }

        Ok(())
    }
}

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        esp_idf_ulp_coproc_enabled,
        any(esp_idf_ulp_coproc_type_riscv, esp_idf_ulp_coproc_type_lp_core)
    ),
    all(
        esp32s2,
        esp_idf_esp32s2_ulp_coproc_enabled,
        esp_idf_esp32s2_ulp_coproc_riscv
    ),
    all(
        esp32s3,
        esp_idf_esp32s3_ulp_coproc_enabled,
        esp_idf_esp32s3_ulp_coproc_riscv
    ),
))]
impl<'d> UlpDriver<'d> {
    /// # Safety
    ///
    /// `src` must point to a `T` in the memory reserved for the ULP, such as a variable
    /// of the loaded program.
    pub unsafe fn read_var<T>(&self, src: *const T) -> Result<T, esp_idf_sys::EspError> {
        Self::check_boundaries(src)?;

        Ok(core::ptr::read_volatile(src))
    }

    /// # Safety
    ///
    /// `dst` must point to a `T` in the memory reserved for the ULP, such as a variable
    /// of the loaded program.
    pub unsafe fn write_var<T>(&self, dst: *mut T, value: T) -> Result<(), esp_idf_sys::EspError> {
        Self::check_boundaries(dst)?;

        core::ptr::write_volatile(dst, value);

        Ok(())
    }

    /// # Safety
    ///
    /// `ptr` must point to a `T` in the memory reserved for the ULP, such as a variable
    /// of the loaded program.
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

crate::impl_peripheral!(ULP);

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_ulp_coproc_enabled),
    all(esp32, esp_idf_esp32_ulp_coproc_enabled),
    all(esp32s2, esp_idf_esp32s2_ulp_coproc_enabled),
    all(esp32s3, esp_idf_esp32s3_ulp_coproc_enabled)
))]
impl ULP<'_> {
    #[cfg(not(esp_idf_ulp_coproc_type_lp_core))]
    const RTC_SLOW_MEM: u32 = 0x5000_0000_u32;

    #[cfg(all(esp_idf_ulp_coproc_type_lp_core, not(esp_idf_esp_rom_has_lp_rom)))]
    const RTC_SLOW_MEM: u32 = esp_idf_sys::SOC_RTC_DRAM_LOW;

    pub const MEM_START_ULP: *mut core::ffi::c_void = 0_u32 as _;

    /// The start of the memory reserved for the ULP, as seen by the HP core.
    /// On chips with an LP ROM the start is only known at link time, so use `ULP::mem_start()`
    #[cfg(not(esp_idf_esp_rom_has_lp_rom))]
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

    /// The start of the memory reserved for the ULP, as seen by the HP core
    pub fn mem_start() -> *mut core::ffi::c_void {
        #[cfg(not(esp_idf_esp_rom_has_lp_rom))]
        let mem_start = Self::MEM_START;

        // The LP ROM keeps its stack and data in LP RAM, so ESP-IDF places the memory
        // reserved for the LP core after its own reserved LP RAM, at a linker-defined address
        #[cfg(esp_idf_esp_rom_has_lp_rom)]
        let mem_start = {
            extern "C" {
                static _rtc_ulp_memory_start: u32;
            }

            core::ptr::addr_of!(_rtc_ulp_memory_start) as *mut _
        };

        mem_start
    }
}
