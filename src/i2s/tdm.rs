//! Time-division multiplexing (TDM) support for I2S.
use super::*;
use crate::{gpio::*, peripheral::*};

use esp_idf_sys::*;

pub(super) mod config {
    #[allow(unused)]
    use crate::{gpio::*, i2s::config::*, peripheral::*};
    use core::{
        convert::TryFrom,
        ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, Not},
    };
    use esp_idf_sys::*;

    /// Automatic total number of slots, equivalent to the maximum active slot number.
    pub const TDM_AUTO_SLOT_NUM: u32 = 0;

    /// Automatic word-select signal width, equivalent to half the width of a frame.
    pub const TDM_AUTO_WS_WIDTH: u32 = 0;

    /// Time-division multiplexing (TDM) mode configuration for the I2S peripheral.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
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
            i2s_driver_config_t {
                mode: self.channel_cfg.role.as_sdk(),
                sample_rate: self.clk_cfg.sample_rate_hz,
                bits_per_sample: self.slot_cfg.data_bit_width.as_sdk(),
                channel_format: i2s_channel_fmt_t_I2S_CHANNEL_FMT_MULTIPLE, // mono mode doesn't make sense in TDM
                communication_format: self.slot_cfg.comm_fmt.as_sdk(),
                intr_alloc_flags: 1 << 1, // ESP_INTR_FLAG_LEVEL1
                dma_buf_count: self.channel_cfg.dma_buffer_count as i32,
                dma_buf_len: self.channel_cfg.frames_per_buffer as i32,
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
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
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
        /// Set the mclk_multiple to [`MclkMultiple::M384`] when using 24-bit data width. Otherwise, the sample rate
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
        #[allow(clippy::needless_update)]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_tdm_clk_config_t {
            i2s_tdm_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
                bclk_div: self.bclk_div,
                ..Default::default()
            }
        }
    }

    #[cfg(esp_idf_version_major = "4")]
    pub type TdmCommFormat = crate::i2s::std::config::StdCommFormat;

    /// TDM mode GPIO (general purpose input/output) configuration.
    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    pub struct TdmGpioConfig {
        /// Invert the BCLK signal.
        bclk_invert: bool,

        /// Invert the MCLK signal.
        mclk_invert: bool,

        /// Invert the WS signal.
        ws_invert: bool,
    }

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
        #[cfg(not(esp_idf_version_major = "4"))]
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

    /// TDM mode slot configuration.
    ///
    /// To create a slot configuration, use [`TdmSlotConfig::philips_slot_default`],
    /// [`TdmSlotConfig::pcm_short_slot_default`], [`TdmSlotConfig::pcm_long_slot_default`], or
    /// [`TdmSlotConfig::msb_slot_default`], then customize it as needed.
    ///
    /// In TDM mode, WS (word select, sometimes called LRCLK or left/right clock) becomes a frame synchronization
    /// signal that signals the first slot of a frame. The two sides of the TDM link must agree on the number
    /// of channels, data bit width, and frame synchronization pattern; this cannot be determined by examining the
    /// signal itself.
    ///
    /// The Philips default pulls the WS line low one BCK period before the first data bit of the first slot is
    /// sent and holds it low for 50% of the frame.
    ///
    #[doc = include_str!("tdm_slot_philips.svg")]
    ///
    /// MSB (most-significant bit) mode is similar to Philips mode, except the WS line is pulled low at the same time
    /// the first data bit of the first slot is sent. It is held low for 50% of the frame.
    ///
    #[doc = include_str!("tdm_slot_msb.svg")]
    ///
    /// PCM (pulse-code modulation) short mode pulls the WS line *high* one BCK period before the first data bit of
    /// the first slot is sent, keeps it high for one BCK, then pulls it low for the remainder of the frame.
    #[doc = include_str!("tdm_slot_pcm_short.svg")]
    /// PCM long mode pulls the WS line *high* one BCK period before the first data bit of the first slot is sent,
    /// keeps it high until just before the last data bit of the first slot is sent, then pulls it low for the
    /// remainder of the frame.
    #[doc = include_str!("tdm_slot_pcm_long.svg")]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    ///
    /// Diagrams from _ESP-IDF Programming Guide_; rendered by Wavedrom.
    pub struct TdmSlotConfig {
        /// I2S sample data bit width (valid data bits per sample).
        data_bit_width: DataBitWidth,

        /// I2S slot bit width (total bits per slot).
        slot_bit_width: SlotBitWidth,

        /// Which slots are active in the TDM frame.
        slot_mask: TdmSlotMask,

        /// The word select (WS) signal width, in terms of the bit clock (BCK) periods.
        #[cfg(not(esp_idf_version_major = "4"))]
        ws_width: u32,

        /// The word select signal polarity; `true` enables the high level first.
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
        #[must_use]
        pub fn data_bit_width(mut self, data_bit_width: DataBitWidth) -> Self {
            self.data_bit_width = data_bit_width;
            self
        }

        /// Update the slot bit width on this TDM slot configuration.
        ///
        /// This is normally set to [`SlotBitWidth::Auto`] to match `[data_bit_width][TdmSlotConfig::data_bit_width()]`.
        #[inline(always)]
        #[must_use]
        pub fn slot_bit_width(mut self, slot_bit_width: SlotBitWidth) -> Self {
            self.slot_bit_width = slot_bit_width;
            self
        }

        /// Update the slot mask on this TDM slot configuration.
        #[inline(always)]
        #[must_use]
        pub fn slot_mask(mut self, slot_mask: TdmSlotMask) -> Self {
            self.slot_mask = slot_mask;
            self
        }

        /// Update the word select signal width on this TDM slot configuration.
        ///
        /// This sets the number of bits to keep the word select signal active at the start of each frame. If this is
        /// set to 0 ([`TDM_AUTO_WS_WIDTH`]), the word select signal will be kept active for half of the frame.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        #[must_use]
        pub fn ws_width(mut self, ws_width: u32) -> Self {
            self.ws_width = ws_width;
            self
        }

        /// Update the word select signal polarity on this TDM slot configuration.
        ///
        /// Setting this to `true` will make the word select (WS) signal active high at the start (PCM modes).
        /// Setting this to `false` will make the WS signal active low at the start (Philips and MSB modes).
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        #[must_use]
        pub fn ws_polarity(mut self, ws_polarity: bool) -> Self {
            self.ws_polarity = ws_polarity;
            self
        }

        /// Update the bit shift flag on this TDM slot configuration.
        ///
        /// Setting this to `true` will activate the word select (WS) signal lone BCK period before the first data bit
        /// of the first slot is sent (Philips and PCM modes). Setting this to `false` will activate the WS
        /// signal at the same time the first data bit of the first slot is sent (MSB mode).
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        #[must_use]
        pub fn bit_shift(mut self, bit_shift: bool) -> Self {
            self.bit_shift = bit_shift;
            self
        }

        /// Update the communication format on this TDM slot configuration.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        #[must_use]
        pub fn comm_fmt(mut self, comm_fmt: TdmCommFormat) -> Self {
            self.comm_fmt = comm_fmt;
            self
        }

        /// Update the left-alignment flag on this TDM slot configuration.
        ///
        /// This only has an effect when `[slot_bit_width][TdmSlotMask::slot_bit_width()]` is greater than
        /// `[data_bit_width][TdmSlotMask::data_bit_width()]`. Setting this to `true` will left-align the data in the slot and
        /// fill the right-most bits (usually the least-significant bits) with zeros. Setting this to `false` will right-align the
        /// data in the slot and fill the left-most bits (usually the most-significant bits) with zeros.
        #[inline(always)]
        #[must_use]
        pub fn left_align(mut self, left_align: bool) -> Self {
            self.left_align = left_align;
            self
        }

        /// Update the big-endian flag on this TDM slot configuration.
        ///
        /// This affects the interpretation of the data when `[data_bit_width][TdmSlotMask::data_bit_width()]` is
        /// greater than 8. Setting this to
        /// `true` will interpret the data as big-endian. Setting this to `false` will interpret the data as
        /// little-endian (the default, and the native endian-ness of all ESP32 microcontrollers).
        #[inline(always)]
        #[must_use]
        pub fn big_endian(mut self, big_endian: bool) -> Self {
            self.big_endian = big_endian;
            self
        }

        /// Update the LSB-first flag on this TDM slot configuration.
        ///
        /// Setting this to `true` will transmit data LSB-first (no known modes do this). Setting this to `false`
        /// will transmit data MSB-first (the default for all known modes).
        #[inline(always)]
        #[must_use]
        pub fn bit_order_lsb(mut self, bit_order_lsb: bool) -> Self {
            self.bit_order_lsb = bit_order_lsb;
            self
        }

        /// Update the skip mask flag on this TDM slot configuration.
        ///
        /// Setting this to `true` will ignore `[slot_mask][TdmSlotMask::slot_mask()]` and transmit all slots. Setting this to `false` will
        /// respect the slot mask.
        #[inline(always)]
        #[must_use]
        pub fn skip_mask(mut self, skip_mask: bool) -> Self {
            self.skip_mask = skip_mask;
            self
        }

        /// Update the total number of slots on this TDM slot configuration.
        ///
        /// Setting this to 0 ([`TDM_AUTO_SLOT_NUM`]) will automatically set the total number of slots to the
        /// the number of active slots in `[slot_mask][TdmSlotMask::slot_mask()]`.
        #[inline(always)]
        #[must_use]
        pub fn total_slots(mut self, total_slots: u32) -> Self {
            self.total_slots = total_slots;
            self
        }

        /// Configure in Philips format with the active slots enabled by the specified mask.
        #[inline(always)]
        #[must_use]
        pub fn philips_slot_default(bits_per_sample: DataBitWidth, slot_mask: TdmSlotMask) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
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
        #[must_use]
        pub fn msb_slot_default(bits_per_sample: DataBitWidth, slot_mask: TdmSlotMask) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
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
        #[inline(always)]
        #[must_use]
        pub fn pcm_short_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
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
        #[inline(always)]
        #[must_use]
        pub fn pcm_long_slot_default(
            bits_per_sample: DataBitWidth,
            slot_mask: TdmSlotMask,
        ) -> Self {
            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
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
                slot_mode: SlotMode::Stereo.as_sdk(), // mono mode doesn't make sense in TDM
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

    /// An individual TDM slot.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum TdmSlot {
        /// TDM slot #0
        Slot0,

        /// TDM slot #1
        Slot1,

        /// TDM slot #2
        Slot2,

        /// TDM slot #3
        Slot3,

        /// TDM slot #4
        Slot4,

        /// TDM slot #5
        Slot5,

        /// TDM slot #6
        Slot6,

        /// TDM slot #7
        Slot7,

        /// TDM slot #8
        Slot8,

        /// TDM slot #9
        Slot9,

        /// TDM slot #10
        Slot10,

        /// TDM slot #11
        Slot11,

        /// TDM slot #12
        Slot12,

        /// TDM slot #13
        Slot13,

        /// TDM slot #14
        Slot14,

        /// TDM slot #15
        Slot15,
    }

    /// Mask of TDM slots to enable.
    #[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
    pub struct TdmSlotMask(u16);

    /// Attempt to convert from a `u8` to a `TdmSlot`.
    impl TryFrom<u8> for TdmSlot {
        type Error = EspError;

        fn try_from(slot: u8) -> Result<Self, Self::Error> {
            match slot {
                0 => Ok(Self::Slot0),
                1 => Ok(Self::Slot1),
                2 => Ok(Self::Slot2),
                3 => Ok(Self::Slot3),
                4 => Ok(Self::Slot4),
                5 => Ok(Self::Slot5),
                6 => Ok(Self::Slot6),
                7 => Ok(Self::Slot7),
                8 => Ok(Self::Slot8),
                9 => Ok(Self::Slot9),
                10 => Ok(Self::Slot10),
                11 => Ok(Self::Slot11),
                12 => Ok(Self::Slot12),
                13 => Ok(Self::Slot13),
                14 => Ok(Self::Slot14),
                15 => Ok(Self::Slot15),
                _ => Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap()),
            }
        }
    }

    /// Convert a `TdmSlot` to a `u8`.
    impl From<TdmSlot> for u8 {
        fn from(slot: TdmSlot) -> u8 {
            match slot {
                TdmSlot::Slot0 => 0,
                TdmSlot::Slot1 => 1,
                TdmSlot::Slot2 => 2,
                TdmSlot::Slot3 => 3,
                TdmSlot::Slot4 => 4,
                TdmSlot::Slot5 => 5,
                TdmSlot::Slot6 => 6,
                TdmSlot::Slot7 => 7,
                TdmSlot::Slot8 => 8,
                TdmSlot::Slot9 => 9,
                TdmSlot::Slot10 => 10,
                TdmSlot::Slot11 => 11,
                TdmSlot::Slot12 => 12,
                TdmSlot::Slot13 => 13,
                TdmSlot::Slot14 => 14,
                TdmSlot::Slot15 => 15,
            }
        }
    }

    /// Convert a `TdmSlot` into a `TdmSlotMask`.
    impl From<TdmSlot> for TdmSlotMask {
        #[inline(always)]
        fn from(slot: TdmSlot) -> TdmSlotMask {
            TdmSlotMask(1 << u8::from(slot))
        }
    }

    /// Bitwise AND a`TdmSlot` with another `TdmSlot` to produce a `TdmSlotMask`.
    ///
    /// If the slots are the same, the result is a `TdmSlotMask` containing that slot.
    /// Otherwise, the result is an empty slot mask.
    impl BitAnd<TdmSlot> for TdmSlot {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitand(self, rhs: Self) -> Self::Output {
            TdmSlotMask::from(self) & TdmSlotMask::from(rhs)
        }
    }

    /// Bitwise AND a `TdmSlot` with a `TdmSlotMask` to produce a `TdmSlotMask`.
    ///
    /// If the slot mask contains the slot, the result is a `TdmSlotMask` containing that slot.
    /// Otherwise, the result is an empty slot mask.
    impl BitAnd<TdmSlotMask> for TdmSlot {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitand(self, rhs: TdmSlotMask) -> Self::Output {
            TdmSlotMask::from(self) & rhs
        }
    }

    /// Bitwise AND a `TdmSlotMask` with a `TdmSlot` to produce a `TdmSlotMask`.
    ///
    /// If the slot mask contains the slot, the result is a `TdmSlotMask` containing that slot.
    /// Otherwise, the result is an empty slot mask.
    impl BitAnd<TdmSlot> for TdmSlotMask {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitand(self, rhs: TdmSlot) -> Self::Output {
            self & TdmSlotMask::from(rhs)
        }
    }

    /// Bitwise AND a `TdmSlotMask` with another `TdmSlotMask` to produce a `TdmSlotMask`.
    ///
    /// The result is a slot mask containing the slots that are common to both slot masks.
    impl BitAnd<TdmSlotMask> for TdmSlotMask {
        type Output = Self;

        #[inline(always)]
        fn bitand(self, rhs: Self) -> Self::Output {
            Self(self.0 & rhs.0)
        }
    }

    /// Bitwise AND a `TdmSlotMask` with a `TdmSlot` and assign the result to `self`.
    ///
    /// If the slot mask contains the slot, the result is a `TdmSlotMask` containing that slot.
    /// Otherwise, the result is an empty slot mask.
    impl BitAndAssign<TdmSlot> for TdmSlotMask {
        #[inline(always)]
        fn bitand_assign(&mut self, rhs: TdmSlot) {
            self.0 &= TdmSlotMask::from(rhs).0;
        }
    }

    /// Bitwise AND a `TdmSlotMask` with another `TdmSlotMask` and assign the result to `self`.
    ///
    /// The result is a slot mask containing the slots that are common to both slot masks.
    impl BitAndAssign<TdmSlotMask> for TdmSlotMask {
        #[inline(always)]
        fn bitand_assign(&mut self, rhs: Self) {
            self.0 &= rhs.0;
        }
    }

    /// Bitwise OR a`TdmSlot` with another `TdmSlot` to produce a `TdmSlotMask`.
    ///
    /// The result is a `TdmSlotMask` containing both slots.
    impl BitOr<TdmSlot> for TdmSlot {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitor(self, rhs: Self) -> Self::Output {
            TdmSlotMask::from(self) | TdmSlotMask::from(rhs)
        }
    }

    /// Bitwise OR a`TdmSlot` with a `TdmSlotMask` to produce a `TdmSlotMask`.
    ///
    /// The result is a `TdmSlotMask` containing the slot and all slots in the slot mask.
    impl BitOr<TdmSlotMask> for TdmSlot {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitor(self, rhs: TdmSlotMask) -> Self::Output {
            TdmSlotMask::from(self) | rhs
        }
    }

    /// Bitwise OR a`TdmSlotMask` with a `TdmSlot` to produce a `TdmSlotMask`.
    ///
    /// The result is a `TdmSlotMask` containing the slot and all slots in the slot mask.
    impl BitOr<TdmSlot> for TdmSlotMask {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn bitor(self, rhs: TdmSlot) -> Self::Output {
            self | TdmSlotMask::from(rhs)
        }
    }

    /// Bitwise OR a`TdmSlotMask` with another `TdmSlotMask` to produce a `TdmSlotMask`.
    ///
    /// The result is a `TdmSlotMask` containing the slots in either slot mask.
    impl BitOr<TdmSlotMask> for TdmSlotMask {
        type Output = Self;

        #[inline(always)]
        fn bitor(self, rhs: Self) -> Self::Output {
            Self(self.0 | rhs.0)
        }
    }

    /// Bitwise OR a`TdmSlotMask` with a `TdmSlot` and assign the result to `self`.
    ///
    /// The result is a `TdmSlotMask` containing the slot and all slots in the slot mask.
    impl BitOrAssign<TdmSlot> for TdmSlotMask {
        #[inline(always)]
        fn bitor_assign(&mut self, rhs: TdmSlot) {
            self.0 |= TdmSlotMask::from(rhs).0;
        }
    }

    /// Bitwise OR a`TdmSlotMask` with another `TdmSlotMask` and assign the result to `self.
    ///
    /// The result is a `TdmSlotMask` containing the slots in either slot mask.
    impl BitOrAssign<TdmSlotMask> for TdmSlotMask {
        #[inline(always)]
        fn bitor_assign(&mut self, rhs: Self) {
            self.0 |= rhs.0;
        }
    }

    /// Produce the bitwise NOT of a `TdmSlot` to produce a `TdmSlotMask` containing all slots
    /// except the original slot.
    impl Not for TdmSlot {
        type Output = TdmSlotMask;

        #[inline(always)]
        fn not(self) -> Self::Output {
            !TdmSlotMask::from(self)
        }
    }

    /// Produce the bitwise NOT of a `TdmSlotMask` to produce a `TdmSlotMask` containing all slots
    /// except the slots in the original slot mask.
    impl Not for TdmSlotMask {
        type Output = Self;

        fn not(self) -> Self::Output {
            Self(!self.0)
        }
    }

    impl TdmSlotMask {
        /// Creates a `TdmSlotMask` from the raw bit mask value.
        #[inline(always)]
        pub fn from_mask_value(value: u16) -> Self {
            Self(value)
        }

        /// Indicates whether this slot mask is empty.
        #[inline(always)]
        pub fn is_empty(&self) -> bool {
            self.0 == 0
        }

        /// Returns the number of slots in the slot mask.
        #[inline(always)]
        pub fn len(&self) -> usize {
            self.0.count_ones() as usize
        }

        /// Returns the mask value as a `u16`.
        #[inline(always)]
        pub fn mask_value(&self) -> u16 {
            self.0
        }

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

impl<'d, Dir> I2sDriver<'d, Dir> {
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    fn internal_new_tdm<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: &config::TdmConfig,
        rx: bool,
        tx: bool,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let chan_cfg = config.channel_cfg.as_sdk(I2S::port());

        let this = Self::internal_new::<I2S>(&chan_cfg, rx, tx)?;

        // Create the channel configuration.
        let tdm_config = config.as_sdk(
            bclk.into_ref(),
            din.map(|d_in| d_in.into_ref()),
            dout.map(|d_out| d_out.into_ref()),
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        if rx {
            unsafe {
                // Open the RX channel.
                esp!(i2s_channel_init_tdm_mode(this.rx_handle, &tdm_config))?;
            }
        }

        if tx {
            unsafe {
                // Open the TX channel.
                esp!(i2s_channel_init_tdm_mode(this.tx_handle, &tdm_config))?;
            }
        }

        Ok(this)
    }

    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    fn internal_new_tdm<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: &config::TdmConfig,
        rx: bool,
        tx: bool,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let mut driver_cfg = config.as_sdk();

        if rx {
            driver_cfg.mode |= i2s_mode_t_I2S_MODE_RX;
        }

        if tx {
            driver_cfg.mode |= i2s_mode_t_I2S_MODE_TX;
        }

        let this = Self::internal_new::<I2S>(&driver_cfg)?;

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
            esp!(i2s_set_pin(this.port as _, &pin_cfg))?;
        }

        Ok(this)
    }
}

impl<'d> I2sDriver<'d, I2sBiDir> {
    /// Create a new TDM mode driver for the given I2S peripheral with both the receive and transmit channels open.
    #[cfg(not(any(esp32, esp32s2)))]
    #[cfg_attr(feature = "nightly", doc(cfg(not(any(esp32, esp32s2)))))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tdm_bidir<I2S: I2s>(
        i2s: impl Peripheral<P = I2S> + 'd,
        config: &config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: impl Peripheral<P = impl InputPin> + 'd,
        dout: impl Peripheral<P = impl OutputPin> + 'd,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        Self::internal_new_tdm(
            i2s,
            config,
            true,
            true,
            bclk,
            Some(din),
            Some(dout),
            mclk,
            ws,
        )
    }
}

impl<'d> I2sDriver<'d, I2sRx> {
    /// Create a new TDM mode driver for the given I2S peripheral with only the receive channel open.
    #[cfg(not(any(esp32, esp32s2)))]
    #[cfg_attr(feature = "nightly", doc(cfg(not(any(esp32, esp32s2)))))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tdm_rx<I2S: I2s>(
        i2s: impl Peripheral<P = I2S> + 'd,
        config: &config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: impl Peripheral<P = impl InputPin> + 'd,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        Self::internal_new_tdm(
            i2s,
            config,
            true,
            false,
            bclk,
            Some(din),
            AnyIOPin::none(),
            mclk,
            ws,
        )
    }
}

impl<'d> I2sDriver<'d, I2sTx> {
    /// Create a new TDM mode driver for the given I2S peripheral with only the transmit channel open.
    #[cfg(not(any(esp32, esp32s2)))]
    #[cfg_attr(feature = "nightly", doc(cfg(not(any(esp32, esp32s2)))))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tdm_tx<I2S: I2s>(
        i2s: impl Peripheral<P = I2S> + 'd,
        config: &config::TdmConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        dout: impl Peripheral<P = impl OutputPin> + 'd,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        Self::internal_new_tdm(
            i2s,
            config,
            false,
            true,
            bclk,
            AnyIOPin::none(),
            Some(dout),
            mclk,
            ws,
        )
    }
}
