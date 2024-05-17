use core::{marker::PhantomData, ops::Deref};

use esp_idf_hal::{
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
};

#[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
use esp_idf_hal::gpio;

use crate::sys::*;

pub struct SlotConfiguration<'d> {
    slot: Slot,
    configuration: sdmmc_slot_config_t,
    _p: PhantomData<&'d mut ()>,
}

/// Indicates that card detect line is not used
const SDMMC_SLOT_NO_CD: i32 = -1;
/// Indicates that write protect line is not used
const SDMMC_SLOT_NO_WP: i32 = -1;

#[derive(Clone, Copy, Debug)]
pub(crate) enum Slot {
    #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
    NotUsed = 0,
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    _0 = 0,
    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    _1 = 1,
}

impl<'d> SlotConfiguration<'d> {
    #[cfg(esp_idf_soc_sdmmc_use_gpio_matrix)]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        cmd: impl Peripheral<P = impl OutputPin> + 'd,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        d0: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        d1: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d2: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d3: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d4: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d5: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d6: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        d7: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        card_detect: Option<impl Peripheral<P = impl InputPin> + 'd>,
        write_protect: Option<impl Peripheral<P = impl InputPin> + 'd>,
    ) -> Self {
        Self::new_internal(
            Slot::NotUsed,
            cmd,
            clk,
            d0,
            d1,
            d2,
            d3,
            d4,
            d5,
            d6,
            d7,
            card_detect,
            write_protect,
        )
    }

    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot_0(
        cmd: impl Peripheral<P = gpio::Gpio11> + 'd,
        clk: impl Peripheral<P = gpio::Gpio6> + 'd,
        d0: impl Peripheral<P = gpio::Gpio7> + 'd,
        d1: Option<impl Peripheral<P = gpio::Gpio8> + 'd>,
        d2: Option<impl Peripheral<P = gpio::Gpio9> + 'd>,
        d3: Option<impl Peripheral<P = gpio::Gpio10> + 'd>,
        d4: Option<impl Peripheral<P = gpio::Gpio16> + 'd>,
        d5: Option<impl Peripheral<P = gpio::Gpio17> + 'd>,
        d6: Option<impl Peripheral<P = gpio::Gpio15> + 'd>,
        d7: Option<impl Peripheral<P = gpio::Gpio18> + 'd>,
        card_detect: Option<impl Peripheral<P = impl InputPin> + 'd>,
        write_protect: Option<impl Peripheral<P = impl InputPin> + 'd>,
    ) -> Self {
        Self::new_internal(
            Slot::_0,
            cmd,
            clk,
            d0,
            d1,
            d2,
            d3,
            d4,
            d5,
            d6,
            d7,
            card_detect,
            write_protect,
        )
    }

    #[cfg(not(esp_idf_soc_sdmmc_use_gpio_matrix))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_slot_1(
        cmd: impl Peripheral<P = gpio::Gpio15> + 'd,
        clk: impl Peripheral<P = gpio::Gpio14> + 'd,
        d0: impl Peripheral<P = gpio::Gpio2> + 'd,
        d1: Option<impl Peripheral<P = gpio::Gpio4> + 'd>,
        d2: Option<impl Peripheral<P = gpio::Gpio12> + 'd>,
        d3: Option<impl Peripheral<P = gpio::Gpio13> + 'd>,
        card_detect: Option<impl Peripheral<P = gpio::Gpio34> + 'd>,
        write_protect: Option<impl Peripheral<P = gpio::Gpio35> + 'd>,
    ) -> Self {
        Self::new_internal(
            Slot::_1,
            cmd,
            clk,
            d0,
            d1,
            d2,
            d3,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            Option::<gpio::AnyIOPin>::None,
            card_detect,
            write_protect,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn new_internal(
        _slot: Slot,
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
        _card_detect: Option<impl Peripheral<P = impl InputPin> + 'd>,
        _write_protect: Option<impl Peripheral<P = impl InputPin> + 'd>,
    ) -> Self {
        Self {
            slot: _slot,
            configuration: sdmmc_slot_config_t {
                width: Self::get_slot_width(
                    _d1.as_ref(),
                    _d2.as_ref(),
                    _d3.as_ref(),
                    _d4.as_ref(),
                    _d5.as_ref(),
                    _d6.as_ref(),
                    _d7.as_ref(),
                ),
                flags: 0,
                __bindgen_anon_1: sdmmc_slot_config_t__bindgen_ty_1 {
                    cd: _card_detect
                        .map(|cd| cd.into_ref().deref().pin())
                        .unwrap_or(SDMMC_SLOT_NO_CD),
                },
                __bindgen_anon_2: sdmmc_slot_config_t__bindgen_ty_2 {
                    wp: _write_protect
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
            },
            _p: PhantomData,
        }
    }

    fn get_slot_width<A, B, C, D, E, F, G>(
        _d1: Option<A>,
        _d2: Option<B>,
        _d3: Option<C>,
        _d4: Option<D>,
        _d5: Option<E>,
        _d6: Option<F>,
        _d7: Option<G>,
    ) -> u8 {
        if _d3.is_some() {
            if !(_d2.is_some() && _d1.is_some()) {
                panic!("D2 and D1 must be provided if D3 is provided");
            }

            if _d7.is_some() {
                if !(_d6.is_some() && _d5.is_some() && _d4.is_some()) {
                    panic!("D6, D5, and D4 must be provided if D7 is provided");
                }

                8
            } else {
                4
            }
        } else {
            1
        }
    }

    pub(crate) fn get_inner(&self) -> &sdmmc_slot_config_t {
        &self.configuration
    }

    pub(crate) fn get_slot(&self) -> Slot {
        self.slot
    }
}
