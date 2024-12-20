use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::gpio::{self, InputPin, OutputPin};
use crate::peripheral::Peripheral;
use crate::sys::*;

/// Indicates that card detect line is not used
const SDMMC_SLOT_NO_CD: i32 = -1;
/// Indicates that write protect line is not used
const SDMMC_SLOT_NO_WP: i32 = -1;
/// Bit indicating that internal pullups should be enabled
const SDMMC_INTERNAL_PULLUPS_ENABLE_FLAG: u32 = 1;

static USED_SLOTS: AtomicU8 = AtomicU8::new(0);
static USED_SLOTS_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

pub type SdMmcHostConfiguration = config::Configuration;

pub mod config {
    /// Configuration for the SD-MMC Host driver
    #[non_exhaustive]
    pub struct Configuration {
        /// Enable internal pullups on the data lines.
        ///
        /// Pullups (either internal or external) MUST be enabled for the data lines
        /// so as the driver to operate correctly.
        ///
        /// Espressif recommends using external pullups (10k each) - yet - for
        /// demo/debugging purposes internal pullups should be fine
        ///
        /// Set this to `false` only when external pullups are used.
        /// When using external pullups note that those should be set even on the pins
        /// which are not actually used (i.e. on pins d1, d2 and d3 for slot-1 of ESP32
        /// 1-bit mode)
        pub enable_internal_pullups: bool,
    }

    impl Configuration {
        /// Create a new configuration with default values
        pub const fn new() -> Self {
            Self {
                enable_internal_pullups: true,
            }
        }
    }

    impl Default for Configuration {
        fn default() -> Self {
            Self::new()
        }
    }
}

/// SDMMC host slot peripheral
pub trait SdMmc {
    fn slot() -> u8;
}

/// SD-MMC Host driver (per slot) for SD Cards supporting the MMC protocol.
pub struct SdMmcHostDriver<'d> {
    slot: u8,
    width: u8,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> SdMmcHostDriver<'d> {
    /// Create a new driver for the provided slot peripheral with data line width 1.
    #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
    #[allow(clippy::too_many_arguments)]
    pub fn new_1bit<S: SdMmc>(
        slot: impl Peripheral<P = S> + 'd,
        cmd: impl Peripheral<P = impl OutputPin> + 'd,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        d0: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            1,
            config.enable_internal_pullups,
            slot,
            cmd,
            clk,
            d0,
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            cd,
            wp,
        )
    }

    /// Create a new driver for the provided slot peripheral with data line width 4.
    #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
    #[allow(clippy::too_many_arguments)]
    pub fn new_4bits<S: SdMmc>(
        slot: impl Peripheral<P = S> + 'd,
        cmd: impl Peripheral<P = impl OutputPin> + 'd,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        d0: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d1: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d2: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d3: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            4,
            config.enable_internal_pullups,
            slot,
            cmd,
            clk,
            d0,
            Some(d1),
            Some(d2),
            Some(d3),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            gpio::AnyIOPin::none(),
            cd,
            wp,
        )
    }

    /// Create a new driver for the provided slot peripheral with data line width 8.
    #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
    #[allow(clippy::too_many_arguments)]
    pub fn new_8bits<S: SdMmc>(
        slot: impl Peripheral<P = S> + 'd,
        cmd: impl Peripheral<P = impl OutputPin> + 'd,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        d0: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d1: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d2: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d3: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d4: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d5: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d6: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d7: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            8,
            config.enable_internal_pullups,
            slot,
            cmd,
            clk,
            d0,
            Some(d1),
            Some(d2),
            Some(d3),
            Some(d4),
            Some(d5),
            Some(d6),
            Some(d7),
            cd,
            wp,
        )
    }

    /// Create a new driver for slot 0 of the SD-MMC peripheral with data line width 1.
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot0_1bit(
        slot0: impl Peripheral<P = SDMMC0> + 'd,
        cmd: impl Peripheral<P = gpio::Gpio11> + 'd,
        clk: impl Peripheral<P = gpio::Gpio6> + 'd,
        d0: impl Peripheral<P = gpio::Gpio7> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            1,
            config.enable_internal_pullups,
            slot0,
            cmd,
            clk,
            d0,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            cd,
            wp,
        )
    }

    /// Create a new driver for slot 0 of the SD-MMC peripheral with data line width 4.
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot0_4bits(
        slot0: impl Peripheral<P = SDMMC0> + 'd,
        cmd: impl Peripheral<P = gpio::Gpio11> + 'd,
        clk: impl Peripheral<P = gpio::Gpio6> + 'd,
        d0: impl Peripheral<P = gpio::Gpio7> + 'd,
        d1: impl Peripheral<P = gpio::Gpio8> + 'd,
        d2: impl Peripheral<P = gpio::Gpio9> + 'd,
        d3: impl Peripheral<P = gpio::Gpio10> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            4,
            config.enable_internal_pullups,
            slot0,
            cmd,
            clk,
            d0,
            Some(d1),
            Some(d2),
            Some(d3),
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            cd,
            wp,
        )
    }

    /// Create a new driver for slot 0 of the SD-MMC peripheral with data line width 8.
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot0_8bits(
        slot0: impl Peripheral<P = SDMMC0> + 'd,
        cmd: impl Peripheral<P = gpio::Gpio11> + 'd,
        clk: impl Peripheral<P = gpio::Gpio6> + 'd,
        d0: impl Peripheral<P = gpio::Gpio7> + 'd,
        d1: impl Peripheral<P = gpio::Gpio8> + 'd,
        d2: impl Peripheral<P = gpio::Gpio9> + 'd,
        d3: impl Peripheral<P = gpio::Gpio10> + 'd,
        d4: impl Peripheral<P = gpio::Gpio16> + 'd,
        d5: impl Peripheral<P = gpio::Gpio17> + 'd,
        d6: impl Peripheral<P = gpio::Gpio5> + 'd,
        d7: impl Peripheral<P = gpio::Gpio18> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            8,
            config.enable_internal_pullups,
            slot0,
            cmd,
            clk,
            d0,
            Some(d1),
            Some(d2),
            Some(d3),
            Some(d4),
            Some(d5),
            Some(d6),
            Some(d7),
            cd,
            wp,
        )
    }

    /// Create a new driver for slot 1 of the SD-MMC peripheral with data line width 1.
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot1_1bit(
        slot1: impl Peripheral<P = SDMMC1> + 'd,
        cmd: impl Peripheral<P = gpio::Gpio15> + 'd,
        clk: impl Peripheral<P = gpio::Gpio14> + 'd,
        d0: impl Peripheral<P = gpio::Gpio2> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            1,
            config.enable_internal_pullups,
            slot1,
            cmd,
            clk,
            d0,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            cd,
            wp,
        )
    }

    /// Create a new driver for slot 1 of the SD-MMC peripheral with data line width 4.
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot1_4bits(
        slot1: impl Peripheral<P = SDMMC1> + 'd,
        cmd: impl Peripheral<P = gpio::Gpio15> + 'd,
        clk: impl Peripheral<P = gpio::Gpio14> + 'd,
        d0: impl Peripheral<P = gpio::Gpio2> + 'd,
        d1: impl Peripheral<P = gpio::Gpio4> + 'd,
        d2: impl Peripheral<P = gpio::Gpio12> + 'd,
        d3: impl Peripheral<P = gpio::Gpio13> + 'd,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
        config: &config::Configuration,
    ) -> Result<Self, EspError> {
        Self::new_internal(
            4,
            config.enable_internal_pullups,
            slot1,
            cmd,
            clk,
            d0,
            Some(d1),
            Some(d2),
            Some(d3),
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            cd,
            wp,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new_internal<S: SdMmc>(
        width: u8,
        internal_pullups: bool,
        _slot: impl Peripheral<P = S> + 'd,
        _cmd: impl Peripheral<P = impl OutputPin> + 'd,
        _clk: impl Peripheral<P = impl OutputPin> + 'd,
        _d0: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        _d1: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d2: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d3: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d4: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d5: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d6: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        _d7: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        cd: Option<impl Peripheral<P = impl InputPin> + 'd>,
        wp: Option<impl Peripheral<P = impl InputPin> + 'd>,
    ) -> Result<Self, EspError> {
        let slot_config = sdmmc_slot_config_t {
            width: width as _,
            flags: if internal_pullups {
                SDMMC_INTERNAL_PULLUPS_ENABLE_FLAG
            } else {
                0
            },
            __bindgen_anon_1: sdmmc_slot_config_t__bindgen_ty_1 {
                cd: cd
                    .map(|cd| cd.into_ref().deref().pin())
                    .unwrap_or(SDMMC_SLOT_NO_CD),
            },
            __bindgen_anon_2: sdmmc_slot_config_t__bindgen_ty_2 {
                wp: wp
                    .map(|wp| wp.into_ref().deref().pin())
                    .unwrap_or(SDMMC_SLOT_NO_WP),
            },
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            clk: _clk.into_ref().deref().pin(),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            cmd: _cmd.into_ref().deref().pin(),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d0: _d0.into_ref().deref().pin(),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d1: _d1.map(|d1| d1.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d2: _d2.map(|d2| d2.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d3: _d3.map(|d3| d3.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d4: _d4.map(|d4| d4.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d5: _d5.map(|d5| d5.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d6: _d6.map(|d6| d6.into_ref().deref().pin()).unwrap_or(-1),
            #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
            d7: _d7.map(|d7| d7.into_ref().deref().pin()).unwrap_or(-1),
        };

        {
            let _cs = USED_SLOTS_CS.enter();

            if USED_SLOTS.load(Ordering::SeqCst) == 0 {
                esp!(unsafe { sdmmc_host_init() })?;

                USED_SLOTS.fetch_add(1, Ordering::SeqCst);
            }
        }

        esp!(unsafe { sdmmc_host_init_slot(S::slot() as _, &slot_config) })?;

        Ok(Self {
            slot: S::slot(),
            width,
            _p: PhantomData,
        })
    }

    pub(crate) fn slot(&self) -> u8 {
        self.slot
    }

    pub(crate) fn width(&self) -> u8 {
        self.width
    }
}

impl<'d> Drop for SdMmcHostDriver<'d> {
    fn drop(&mut self) {
        let _cs = USED_SLOTS_CS.enter();

        if USED_SLOTS.fetch_sub(1, Ordering::SeqCst) == 1 {
            esp!(unsafe { sdmmc_host_deinit() }).unwrap();
        }
    }
}

macro_rules! impl_slot {
    ($instance:ident: $slot:expr) => {
        crate::impl_peripheral!($instance);

        impl SdMmc for $instance {
            fn slot() -> u8 {
                $slot
            }
        }
    };
}

impl_slot!(SDMMC0: 0);
impl_slot!(SDMMC1: 1);
