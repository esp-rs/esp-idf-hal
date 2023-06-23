//! Time-division multiplexing (TDM) support for I2S.
use super::*;
use crate::{gpio::*, peripheral::*};
use core::{marker::PhantomData, ptr::null_mut};
use esp_idf_sys::*;

#[cfg(all(not(esp_idf_version_major = "4"), feature = "alloc"))]
extern crate alloc;

#[cfg(all(not(esp_idf_version_major = "4"), feature = "alloc"))]
use alloc::boxed::Box;

pub(super) mod config {
    #[allow(unused)]
    use crate::{gpio::*, i2s::config::*, peripheral::*};
    use esp_idf_sys::*;

    /// Automatic total number of slots, equivalent to the maximum active slot number.
    pub const TDM_AUTO_SLOT_NUM: u32 = 0;

    /// Automatic word-select signal width, equivalent to half the width of a frame.
    pub const TDM_AUTO_WS_WIDTH: u32 = 0;

    /// The time-division multiplexing (TDM) mode configuration for the I2S peripheral.
    pub struct TdmConfig {
        /// The base channel configuration.
        pub(super) channel_cfg: Config,

        /// TDM mode channel clock configuration.
        clk_cfg: TdmClkConfig,

        /// TDM mode channel slot configuration.
        slot_cfg: TdmSlotConfig,

        /// TDM mode channel data configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        gpio_cfg: TdmGpioConfig,
    }

    impl TdmConfig {
        /// Create a new TDM mode channel configuration from the given base configuration, clock configuration, slot
        /// configuration, and GPIO configuration.
        #[inline(always)]
        pub fn new(
            channel_cfg: Config,
            clk_cfg: TdmClkConfig,
            slot_cfg: TdmSlotConfig,
            #[cfg(not(esp_idf_version_major = "4"))] gpio_cfg: TdmGpioConfig,
        ) -> Self {
            Self {
                channel_cfg,
                clk_cfg,
                slot_cfg,
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk<'d>(
            &self,
            bclk: PeripheralRef<'d, impl InputPin + OutputPin>,
            din: Option<PeripheralRef<'d, impl InputPin>>,
            dout: Option<PeripheralRef<'d, impl OutputPin>>,
            mclk: Option<PeripheralRef<'d, impl InputPin + OutputPin>>,
            ws: PeripheralRef<'d, impl InputPin + OutputPin>,
        ) -> i2s_tdm_config_t {
            i2s_tdm_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(bclk, din, dout, mclk, ws),
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_driver_config_t` representation.
        ///
        /// # Note
        /// The mode field is not fully set by this function. Only the controller/target field is set. Before using,
        /// the following bits must be considered: `I2S_MODE_TX`, `I2S_MODE_RX`. `I2S_MODE_DAC_BUILT_IN`, and
        /// `I2S_MODE_ADC_BUILT_IN`, and `I2S_MODE_PDM` should not be used here.
        #[cfg(esp_idf_version_major = "4")]
        pub(crate) fn as_sdk(&self) -> i2s_driver_config_t {
            let chan_fmt = match self.slot_cfg.slot_mode {
                SlotMode::Stereo => i2s_channel_fmt_t_I2S_CHANNEL_FMT_MULTIPLE,
                SlotMode::Mono => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_LEFT, // Use channel 0
            };

            i2s_driver_config_t {
                mode: self.channel_cfg.role.as_sdk(),
                sample_rate: self.clk_cfg.sample_rate_hz,
                bits_per_sample: self.slot_cfg.data_bit_width.as_sdk(),
                channel_format: chan_fmt,
                communication_format: self.slot_cfg.comm_fmt.as_sdk(),
                intr_alloc_flags: 1 << 1, // ESP_INTR_FLAG_LEVEL1
                dma_buf_count: self.channel_cfg.dma_desc as i32,
                dma_buf_len: self.channel_cfg.frames as i32,
                #[cfg(any(esp32, esp32s2))]
                use_apll: matches!(self.clk_cfg.clk_src, ClockSource::Apll),
                #[cfg(not(any(esp32, esp32s2)))]
                use_apll: false,
                tx_desc_auto_clear: self.channel_cfg.auto_clear,
                fixed_mclk: 0,
                mclk_multiple: self.clk_cfg.mclk_multiple.as_sdk(),
                bits_per_chan: self.slot_cfg.slot_bit_width.as_sdk(),
                chan_mask: self.slot_cfg.slot_mask.as_sdk(),
                total_chan: self.slot_cfg.slot_mask.0.count_ones(),
                left_align: self.slot_cfg.left_align,
                big_edin: self.slot_cfg.big_endian,
                bit_order_msb: !self.slot_cfg.bit_order_lsb,
                skip_msk: self.slot_cfg.skip_mask,
            }
        }
    }

    /// TDM mode channel clock configuration.
    #[derive(Clone)]
    pub struct TdmClkConfig {
        /// I2S sample rate.
        sample_rate_hz: u32,

        /// Clock source.
        clk_src: ClockSource,

        /// The multiple of MCLK to the sample rate.
        mclk_multiple: MclkMultiple,

        /// The division from MCLK to BCLK. This is used only in I2S target (slave) mode. This should not be smaller
        /// than TDM_BCLK_DIV_MIN (8). Increase this field if the target device is not able to transmit data in time.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        bclk_div: u32,
    }

    /// The minimum division from MCLK to BCLK.
    pub const TDM_BCLK_DIV_MIN: u32 = 8;

    impl TdmClkConfig {
        /// Create a TDM clock configuration with the specified rate (in Hz), clock source, and MCLK multiple of
        /// the sample rate.
        #[cfg(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0")
        ))]
        #[inline(always)]
        pub fn new(sample_rate_hz: u32, clk_src: ClockSource, mclk_multiple: MclkMultiple) -> Self {
            Self {
                sample_rate_hz,
                clk_src,
                mclk_multiple,
            }
        }

        /// Create a TDM clock configuration with the specified rate (in Hz), clock source, and MCLK multiple of
        /// the sample rate.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        #[inline(always)]
        pub fn new(sample_rate_hz: u32, clk_src: ClockSource, mclk_multiple: MclkMultiple) -> Self {
            Self {
                sample_rate_hz,
                clk_src,
                mclk_multiple,
                bclk_div: TDM_BCLK_DIV_MIN,
            }
        }

        /// Create a TDM clock configuration with the specified rate in Hz. This will set the clock source to
        /// PLL_F160M and the MCLK multiple to 256 times the sample rate.
        ///
        /// # Note
        /// Set the mclk_multiple to [MclkMultiple::M384] when using 24-bit data width. Otherwise, the sample rate
        /// might be imprecise since the BCLK division is not an integer.
        #[cfg(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0")
        ))]
        #[inline(always)]
        pub fn from_sample_rate_hz(rate: u32) -> Self {
            Self {
                sample_rate_hz: rate,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
            }
        }

        /// Create a TDM clock configuration with the specified rate in Hz. This will set the clock source to
        /// PLL_F160M, MCLK multiple to 256 times the sample rate, and MCLK to BCLK division to 8.
        ///
        /// # Note
        /// Set the mclk_multiple to [MclkMultiple::M384] when using 24-bit data width. Otherwise, the sample rate
        /// might be imprecise since the BCLK division is not an integer.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        #[inline(always)]
        pub fn from_sample_rate_hz(rate: u32) -> Self {
            Self {
                sample_rate_hz: rate,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
                bclk_div: TDM_BCLK_DIV_MIN,
            }
        }

        /// Set the clock source on this TDM clock configuration.
        #[inline(always)]
        pub fn clk_src(mut self, clk_src: ClockSource) -> Self {
            self.clk_src = clk_src;
            self
        }

        /// Set the MCLK multiple on this TDM clock configuration.
        #[inline(always)]
        pub fn mclk_multiple(mut self, mclk_multiple: MclkMultiple) -> Self {
            self.mclk_multiple = mclk_multiple;
            self
        }

        /// Set the MCLK to BCLK division on this TDM clock configuration.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        #[inline(always)]
        pub fn bclk_div(mut self, bclk_div: u32) -> Self {
            self.bclk_div = bclk_div;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_clk_config_t` representation.
        #[cfg(all(esp_idf_version_major = "5", esp_idf_version_minor = "0"))]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_tdm_clk_config_t {
            i2s_tdm_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_clk_config_t` representation.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_tdm_clk_config_t {
            i2s_tdm_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
                bclk_div: self.bclk_div,
            }
        }
    }

    #[cfg(esp_idf_version_major = "4")]
    pub type TdmCommFormat = crate::i2s::std::config::StdCommFormat;

    /// TDM mode GPIO (general purpose input/output) configuration.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[derive(Default)]
    pub struct TdmGpioConfig {
        /// Invert the BCLK signal.
        bclk_invert: bool,

        /// Invert the MCLK signal.
        mclk_invert: bool,

        /// Invert the WS signal.
        ws_invert: bool,
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    impl TdmGpioConfig {
        /// Create a new TDM mode GPIO configuration with the specified inversion flags for BCLK, MCLK, and WS.
        pub fn new(bclk_invert: bool, mclk_invert: bool, ws_invert: bool) -> Self {
            Self {
                bclk_invert,
                mclk_invert,
                ws_invert,
            }
        }

        /// Set the BCLK inversion flag on this TDM GPIO configuration.
        #[inline(always)]
        pub fn bclk_invert(mut self, bclk_invert: bool) -> Self {
            self.bclk_invert = bclk_invert;
            self
        }

        /// Set the MCLK inversion flag on this TDM GPIO configuration.
        #[inline(always)]
        pub fn mclk_invert(mut self, mclk_invert: bool) -> Self {
            self.mclk_invert = mclk_invert;
            self
        }

        /// Set the WS inversion flag on this TDM GPIO configuration.
        #[inline(always)]
        pub fn ws_invert(mut self, ws_invert: bool) -> Self {
            self.ws_invert = ws_invert;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_gpio_config_t` representation.
        pub(crate) fn as_sdk<'d>(
            &self,
            bclk: PeripheralRef<'d, impl InputPin + OutputPin>,
            din: Option<PeripheralRef<'d, impl InputPin>>,
            dout: Option<PeripheralRef<'d, impl OutputPin>>,
            mclk: Option<PeripheralRef<'d, impl InputPin + OutputPin>>,
            ws: PeripheralRef<'d, impl InputPin + OutputPin>,
        ) -> i2s_tdm_gpio_config_t {
            let invert_flags = i2s_tdm_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_tdm_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.mclk_invert as u32,
                    self.bclk_invert as u32,
                    self.ws_invert as u32,
                ),
                ..Default::default()
            };

            i2s_tdm_gpio_config_t {
                bclk: bclk.pin(),
                din: if let Some(din) = din { din.pin() } else { -1 },
                dout: if let Some(dout) = dout {
                    dout.pin()
                } else {
                    -1
                },
                mclk: if let Some(mclk) = mclk {
                    mclk.pin()
                } else {
                    -1
                },
                ws: ws.pin(),
                invert_flags,
            }
        }
    }

    /// TDM mode channel slot configuration.
    ///
    /// To create a slot configuration, use [TdmSlotConfig::philips_slot_default], [TdmSlotConfig::pcm_slot_default], or
    /// [TdmSlotConfig::msb_slot_default], then customize it as needed.
    pub struct TdmSlotConfig {
        /// I2S sample data bit width (valid data bits per sample).
        data_bit_width: DataBitWidth,

        /// I2S slot bit width (total bits per slot).
        slot_bit_width: SlotBitWidth,

        /// Mono or stereo mode operation.
        slot_mode: SlotMode,

        /// Are we using the left, right, or both data slots?
        slot_mask: TdmSlotMask,

        /// The word select (WS) signal width, in terms of the bit clock (BCK) periods.
        #[cfg(not(esp_idf_version_major = "4"))]
        ws_width: u32,

        /// The word select signal polarity; true enables the light lever first.
        #[cfg(not(esp_idf_version_major = "4"))]
        ws_polarity: bool,

        /// Set to enable the additional bit-shift needed in Philips mode.
        #[cfg(not(esp_idf_version_major = "4"))]
        bit_shift: bool,

        #[cfg(esp_idf_version_major = "4")]
        comm_fmt: TdmCommFormat,

        /// Enable left-alignment.
        left_align: bool,

        /// Enable big-endian.
        big_endian: bool,

        /// Enable LSB-first.
        bit_order_lsb: bool,

        /// Set to enable the skip mask. When enabled, only the data of the enabled channels will be sent. Otherwise,
        /// all data stored in the DMA transmit buffer will be sent.
        skip_mask: bool,

        /// The total number of slots. If this is smaller than the highest activated channel number, it will be set
        /// to that number automatically.
        total_slots: u32,
    }

    impl TdmSlotConfig {
        /// Update the data bit width on this TDM slot configuration.
        #[inline(always)]
        pub fn data_bit_width(mut self, data_bit_width: DataBitWidth) -> Self {
            self.data_bit_width = data_bit_width;
            self
        }

        /// Update the slot bit width on this TDM slot configuration.
        #[inline(always)]
        pub fn slot_bit_width(mut self, slot_bit_width: SlotBitWidth) -> Self {
            self.slot_bit_width = slot_bit_width;
            self
        }

        /// Update the slot mode and mask on this TDM slot configuration.
        #[inline(always)]
        pub fn slot_mode_mask(mut self, slot_mode: SlotMode, slot_mask: TdmSlotMask) -> Self {
            self.slot_mode = slot_mode;
            self.slot_mask = slot_mask;
            self
        }

        /// Update the word select signal width on this TDM slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn ws_width(mut self, ws_width: u32) -> Self {
            self.ws_width = ws_width;
            self
        }

        /// Update the word select signal polarity on this TDM slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn ws_polarity(mut self, ws_polarity: bool) -> Self {
            self.ws_polarity = ws_polarity;
            self
        }

        /// Update the bit shift flag on this TDM slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn bit_shift(mut self, bit_shift: bool) -> Self {
            self.bit_shift = bit_shift;
            self
        }

        /// Update the communication format on this TDM slot configuration.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub fn comm_fmt(mut self, comm_fmt: TdmCommFormat) -> Self {
            self.comm_fmt = comm_fmt;
            self
        }

        /// Update the left-alignment flag on this TDM slot configuration.
        #[inline(always)]
        pub fn left_align(mut self, left_align: bool) -> Self {
            self.left_align = left_align;
            self
        }

        /// Update the big-endian flag on this TDM slot configuration.
        #[inline(always)]
        pub fn big_endian(mut self, big_endian: bool) -> Self {
            self.big_endian = big_endian;
            self
        }

        /// Update the LSB-first flag on this TDM slot configuration.
        #[inline(always)]
        pub fn bit_order_lsb(mut self, bit_order_lsb: bool) -> Self {
            self.bit_order_lsb = bit_order_lsb;
            self
        }

        /// Update the skip mask flag on this TDM slot configuration.
        #[inline(always)]
        pub fn skip_mask(mut self, skip_mask: bool) -> Self {
            self.skip_mask = skip_mask;
            self
        }

        /// Update the total number of slots on this TDM slot configuration.
        #[inline(always)]
        pub fn total_slots(mut self, total_slots: u32) -> Self {
            self.total_slots = total_slots;
            self
        }

        /// Configure in Philips format with the active slots enabled by the specified mask.
        #[inline(always)]
        pub fn philips_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mode: SlotMode,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: TDM_AUTO_WS_WIDTH,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: false,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: true,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: TdmCommFormat::Philips,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
                skip_mask: false,
                total_slots: TDM_AUTO_SLOT_NUM,
            }
        }

        /// Configure in MSB format with the active slots enabled by the specified mask.
        #[inline(always)]
        pub fn msb_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mode: SlotMode,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: TDM_AUTO_WS_WIDTH,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: false,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: false,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: TdmCommFormat::Msb,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
                skip_mask: false,
                total_slots: TDM_AUTO_SLOT_NUM,
            }
        }

        /// Configure in PCM (short) format with the active slots enabled by the specified mask.
        pub fn pcm_short_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mode: SlotMode,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: 1,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: true,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: false,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: TdmCommFormat::PcmShort,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
                skip_mask: false,
                total_slots: TDM_AUTO_SLOT_NUM,
            }
        }

        /// Configure in PCM (long) format with the active slots enabled by the specified mask.
        pub fn pcm_long_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mode: SlotMode,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: bits_per_sample.into(),
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: true,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: false,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: TdmCommFormat::PcmLong,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
                skip_mask: false,
                total_slots: TDM_AUTO_SLOT_NUM,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_slot_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_tdm_slot_config_t {
            i2s_tdm_slot_config_t {
                data_bit_width: self.data_bit_width.as_sdk(),
                slot_bit_width: self.slot_bit_width.as_sdk(),
                slot_mode: self.slot_mode.as_sdk(),
                slot_mask: self.slot_mask.as_sdk(),
                ws_width: self.ws_width,
                ws_pol: self.ws_polarity,
                bit_shift: self.bit_shift,
                left_align: self.left_align,
                big_endian: self.big_endian,
                bit_order_lsb: self.bit_order_lsb,
                skip_mask: self.skip_mask,
                total_slot: self.total_slots,
            }
        }
    }

    // The derived Clone appears to have problems with some of the cfg attributes in rust-analyzer.
    impl Clone for TdmSlotConfig {
        fn clone(&self) -> Self {
            Self {
                data_bit_width: self.data_bit_width,
                slot_bit_width: self.slot_bit_width,
                slot_mode: self.slot_mode,
                slot_mask: self.slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: self.ws_width,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: self.ws_polarity,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: self.bit_shift,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: self.comm_fmt,
                left_align: self.left_align,
                big_endian: self.big_endian,
                bit_order_lsb: self.bit_order_lsb,
                skip_mask: self.skip_mask,
                total_slots: self.total_slots,
            }
        }
    }

    /// Mask of TDM slots to enable.
    #[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
    pub struct TdmSlotMask(u16);

    pub const SLOT0: TdmSlotMask = TdmSlotMask(1 << 0);
    pub const SLOT1: TdmSlotMask = TdmSlotMask(1 << 1);
    pub const SLOT2: TdmSlotMask = TdmSlotMask(1 << 2);
    pub const SLOT3: TdmSlotMask = TdmSlotMask(1 << 3);
    pub const SLOT4: TdmSlotMask = TdmSlotMask(1 << 4);
    pub const SLOT5: TdmSlotMask = TdmSlotMask(1 << 5);
    pub const SLOT6: TdmSlotMask = TdmSlotMask(1 << 6);
    pub const SLOT7: TdmSlotMask = TdmSlotMask(1 << 7);
    pub const SLOT8: TdmSlotMask = TdmSlotMask(1 << 8);
    pub const SLOT9: TdmSlotMask = TdmSlotMask(1 << 9);
    pub const SLOT10: TdmSlotMask = TdmSlotMask(1 << 10);
    pub const SLOT11: TdmSlotMask = TdmSlotMask(1 << 11);
    pub const SLOT12: TdmSlotMask = TdmSlotMask(1 << 12);
    pub const SLOT13: TdmSlotMask = TdmSlotMask(1 << 13);
    pub const SLOT14: TdmSlotMask = TdmSlotMask(1 << 14);
    pub const SLOT15: TdmSlotMask = TdmSlotMask(1 << 15);

    impl core::ops::BitAnd for TdmSlotMask {
        type Output = Self;

        fn bitand(self, rhs: Self) -> Self::Output {
            Self(self.0 & rhs.0)
        }
    }

    impl core::ops::BitAndAssign for TdmSlotMask {
        fn bitand_assign(&mut self, rhs: Self) {
            self.0 &= rhs.0;
        }
    }

    impl core::ops::BitOr for TdmSlotMask {
        type Output = Self;

        fn bitor(self, rhs: Self) -> Self::Output {
            Self(self.0 | rhs.0)
        }
    }

    impl core::ops::BitOrAssign for TdmSlotMask {
        fn bitor_assign(&mut self, rhs: Self) {
            self.0 |= rhs.0;
        }
    }

    impl core::ops::Not for TdmSlotMask {
        type Output = Self;

        fn not(self) -> Self::Output {
            Self(!self.0)
        }
    }

    impl TdmSlotMask {
        /// Converts this mask to an ESP-IDF SDK `i2s_tdm_slot_mask_t` value.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_tdm_slot_mask_t {
            self.0 as i2s_tdm_slot_mask_t
        }

        /// Converts this mask to an ESP-IDF SDK `i2s_channel_t` value.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_channel_t {
            ((self.0 as u32) << 16) as i2s_channel_t
        }
    }
}

/// The I2S TDM mode driver.
pub struct I2sTdmDriver<'d, Dir> {
    /// The Rx channel, possibly None.
    #[cfg(not(esp_idf_version_major = "4"))]
    rx: Option<I2sChannel>,

    /// The Tx channel, possibly None.
    #[cfg(not(esp_idf_version_major = "4"))]
    tx: Option<I2sChannel>,

    /// The I2S peripheral number. Either 0 or 1 (ESP32 and ESP32S3 only).
    i2s: u8,

    /// Driver lifetime -- mimics the lifetime of the peripheral.
    _p: PhantomData<&'d ()>,

    /// Directionality -- mimics the directionality of the peripheral.
    _dir: PhantomData<Dir>,
}

unsafe impl<'d, Dir> Send for I2sTdmDriver<'d, Dir> {}
unsafe impl<'d, Dir> Sync for I2sTdmDriver<'d, Dir> {}

impl<'d, Dir> I2sPort for I2sTdmDriver<'d, Dir> {
    /// Returns the I2S port number of this driver.
    fn port(&self) -> i2s_port_t {
        self.i2s as u32
    }
}

impl<'d, Dir: I2sRxSupported + I2sTxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new TDM mode driver for the given I2S peripheral with both the receive and transmit channels open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_bidir<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let chan_cfg = config.channel_cfg.as_sdk(port);

        let mut rx_chan_handle: i2s_chan_handle_t = null_mut();
        let mut tx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe {
            esp!(i2s_new_channel(
                &chan_cfg,
                &mut tx_chan_handle,
                &mut rx_chan_handle
            ))?
        };

        if tx_chan_handle.is_null() {
            panic!("Expected non-null tx channel handle");
        }

        if rx_chan_handle.is_null() {
            panic!("Expected non-null rx channel handle");
        }

        // Allocate the internal channel structs.
        let rx = I2sChannel::new(port as u8, rx_chan_handle);
        let tx = I2sChannel::new(port as u8, tx_chan_handle);

        // At this point, returning early will drop the rx and tx channels, closing them properly.

        // Create the channel configuration.
        let tdm_config = config.as_sdk(
            bclk.into_ref(),
            din.map(|d_in| d_in.into_ref()),
            dout.map(|d_out| d_out.into_ref()),
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: rx/tx.chan_handle are valid, non-null i2s_chan_handle_t,
        // and &tdm_config is a valid pointer to an i2s_tdm_config_t.
        unsafe {
            // Open the RX channel.
            esp!(i2s_channel_init_tdm_mode(rx.chan_handle, &tdm_config))?;

            // Open the TX channel.
            esp!(i2s_channel_init_tdm_mode(tx.chan_handle, &tdm_config))?;
        }

        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            tx: Some(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    /// Create a new TDM mode driver for the given I2S peripheral with both the receive and transmit
    /// channels open.
    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_bdir<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let mut driver_cfg = config.as_sdk();
        driver_cfg.mode |= i2s_mode_t_I2S_MODE_RX | i2s_mode_t_I2S_MODE_TX;

        // Safety: &driver_cfg is a valid pointer to an i2s_driver_config_t.
        unsafe {
            esp!(i2s_driver_install(port, &driver_cfg, 0, null_mut()))?;
        }

        // Set the pin configuration.
        let pin_cfg = i2s_pin_config_t {
            bck_io_num: bclk.into_ref().pin(),
            data_in_num: din.map(|din| din.into_ref().pin()).unwrap_or(-1),
            data_out_num: dout.map(|dout| dout.into_ref().pin()).unwrap_or(-1),
            mck_io_num: mclk.map(|mclk| mclk.into_ref().pin()).unwrap_or(-1),
            ws_io_num: ws.into_ref().pin(),
        };

        // Safety: &pin_cfg is a valid pointer to an i2s_pin_config_t.
        unsafe {
            esp!(i2s_set_pin(port, &pin_cfg))?;
        }

        Ok(Self {
            i2s: port as u8,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sRxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new TDM mode driver for the given I2S peripheral with only the receive channel open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let chan_cfg = config.channel_cfg.as_sdk(port);

        let mut rx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe { esp!(i2s_new_channel(&chan_cfg, null_mut(), &mut rx_chan_handle,))? };

        if rx_chan_handle.is_null() {
            panic!("Expected non-null rx channel handle");
        }

        // Allocate the internal channel struct.
        let rx = I2sChannel::new(port as u8, rx_chan_handle);

        // At this point, returning early will drop the rx channel, closing it properly.

        // Create the channel configuration.
        let dout: Option<PeripheralRef<'d, AnyIOPin>> = None;
        let tdm_config = config.as_sdk(
            bclk.into_ref(),
            din.map(|d_in| d_in.into_ref()),
            dout,
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: rx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &tdm_config is a valid pointer to an i2s_tdm_config_t.
        unsafe {
            // Open the RX channel.
            esp!(i2s_channel_init_tdm_mode(rx.chan_handle, &tdm_config))?;
        }

        // Now we leak the rx channel so it is no longer managed. This pins it in memory in a way that
        // is easily accessible to the ESP-IDF SDK.
        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            tx: None,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    /// Create a new TDM mode driver for the given I2S peripheral with only the receive channel open.
    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let mut driver_cfg = config.as_sdk();
        driver_cfg.mode |= i2s_mode_t_I2S_MODE_RX;

        // Safety: &driver_cfg is a valid pointer to an i2s_driver_config_t.
        unsafe {
            esp!(i2s_driver_install(port, &driver_cfg, 0, null_mut()))?;
        }

        // Set the pin configuration.
        let pin_cfg = i2s_pin_config_t {
            bck_io_num: bclk.into_ref().pin(),
            data_in_num: din.map(|din| din.into_ref().pin()).unwrap_or(-1),
            data_out_num: -1,
            mck_io_num: mclk.map(|mclk| mclk.into_ref().pin()).unwrap_or(-1),
            ws_io_num: ws.into_ref().pin(),
        };

        // Safety: &pin_cfg is a valid pointer to an i2s_pin_config_t.
        unsafe {
            esp!(i2s_set_pin(port, &pin_cfg))?;
        }

        Ok(Self {
            i2s: port as u8,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sTxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new TDM mode driver for the given I2S peripheral with only the transmit channel open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let chan_cfg = config.channel_cfg.as_sdk(port);

        let mut tx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe { esp!(i2s_new_channel(&chan_cfg, &mut tx_chan_handle, null_mut()))? };

        if tx_chan_handle.is_null() {
            panic!("Expected non-null tx channel handle");
        }

        // Allocate the internal channel struct.
        let tx = I2sChannel::new(port as u8, tx_chan_handle);

        // Create the channel configuration.
        let din: Option<PeripheralRef<'d, AnyIOPin>> = None;
        let tdm_config = config.as_sdk(
            bclk.into_ref(),
            din,
            dout.map(|d_out| d_out.into_ref()),
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: tx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &tdm_config is a valid pointer to an i2s_tdm_config_t.
        unsafe {
            // Open the TX channel.
            esp!(i2s_channel_init_tdm_mode(tx.chan_handle, &tdm_config))?;
        }

        // Now we leak the tx channel so it is no longer managed. This pins it in memory in a way that
        // is easily accessible to the ESP-IDF SDK.
        Ok(Self {
            i2s: port as u8,
            rx: None,
            tx: Some(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    /// Create a new TDM mode driver for the given I2S peripheral with only the transmit channel open.
    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let mut driver_cfg = config.as_sdk();
        driver_cfg.mode |= i2s_mode_t_I2S_MODE_TX;

        // Safety: &driver_cfg is a valid pointer to an i2s_driver_config_t.
        unsafe {
            esp!(i2s_driver_install(port, &driver_cfg, 0, null_mut()))?;
        }

        // Set the pin configuration.
        let pin_cfg = i2s_pin_config_t {
            bck_io_num: bclk.into_ref().pin(),
            data_in_num: -1,
            data_out_num: dout.map(|dout| dout.into_ref().pin()).unwrap_or(-1),
            mck_io_num: mclk.map(|mclk| mclk.into_ref().pin()).unwrap_or(-1),
            ws_io_num: ws.into_ref().pin(),
        };

        // Safety: &pin_cfg is a valid pointer to an i2s_pin_config_t.
        unsafe {
            esp!(i2s_set_pin(port, &pin_cfg))?;
        }

        Ok(Self {
            i2s: port as u8,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sRxSupported> I2sRxChannel<'d> for I2sTdmDriver<'d, Dir> {
    #[cfg(not(esp_idf_version_major = "4"))]
    unsafe fn rx_handle(&self) -> i2s_chan_handle_t {
        self.rx.as_ref().unwrap().chan_handle
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(feature = "riscv-ulp-hal"),
        feature = "alloc"
    ))]
    unsafe fn rx_subscribe(
        &mut self,
        rx_callback: Box<dyn FnMut(u8, I2sRxEvent) -> bool + 'static>,
    ) -> Result<(), EspError> {
        self.rx
            .as_mut()
            .unwrap()
            .rx_subscribe(Box::new(rx_callback))
    }
}

impl<'d, Dir: I2sTxSupported> I2sTxChannel<'d> for I2sTdmDriver<'d, Dir> {
    #[cfg(not(esp_idf_version_major = "4"))]
    unsafe fn tx_handle(&self) -> i2s_chan_handle_t {
        self.tx.as_ref().unwrap().chan_handle
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(feature = "riscv-ulp-hal"),
        feature = "alloc"
    ))]
    unsafe fn tx_subscribe(
        &mut self,
        tx_callback: Box<dyn FnMut(u8, I2sTxEvent) -> bool + 'static>,
    ) -> Result<(), EspError> {
        self.tx
            .as_mut()
            .unwrap()
            .tx_subscribe(Box::new(tx_callback))
    }
}

#[cfg(esp_idf_version_major = "4")]
impl<'d, Dir> Drop for I2sTdmDriver<'d, Dir> {
    fn drop(&mut self) {
        unsafe {
            let result = i2s_driver_uninstall(self.i2s as u32);
            if result != ESP_OK {
                // This isn't fatal so a panic isn't warranted, but we do want to be able to debug it.
                esp_log_write(
                    esp_log_level_t_ESP_LOG_ERROR,
                    LOG_TAG as *const u8 as *const i8,
                    b"Failed to delete RX channel: %s\0" as *const u8 as *const i8,
                    esp_err_to_name(result),
                );
            }
        }
    }
}
