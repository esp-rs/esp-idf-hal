//! Standard mode driver for the ESP32 I2S peripheral.
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

    /// The standard mode configuration for the I2S peripheral.
    pub struct StdConfig {
        /// The base channel configuration.
        pub(super) channel_cfg: Config,

        /// Standard mode channel clock configuration.
        clk_cfg: StdClkConfig,

        /// Standard mode channel slot configuration.
        slot_cfg: StdSlotConfig,

        /// Standard mode channel GPIO configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        gpio_cfg: StdGpioConfig,
    }

    impl StdConfig {
        /// Create a new standard mode channel configuration from the given clock configuration, slot configuration,
        /// and GPIO configuration.
        pub fn new(
            channel_cfg: Config,
            clk_cfg: StdClkConfig,
            slot_cfg: StdSlotConfig,
            #[cfg(not(esp_idf_version_major = "4"))] gpio_cfg: StdGpioConfig,
        ) -> Self {
            Self {
                channel_cfg,
                clk_cfg,
                slot_cfg,
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg,
            }
        }

        /// Create a new standard mode channel configuration for the Philips I2S protocol with the specified sample
        /// rate and bits per sample, in stereo mode, with MCLK set to 256 times the sample rate.
        #[inline(always)]
        pub fn philips(sample_rate_hz: u32, bits_per_sample: DataBitWidth) -> Self {
            Self {
                channel_cfg: Config::default(),
                clk_cfg: StdClkConfig::from_sample_rate_hz(sample_rate_hz),
                slot_cfg: StdSlotConfig::philips_slot_default(bits_per_sample, SlotMode::Stereo),
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg: StdGpioConfig::default(),
            }
        }

        /// Create a new standard mode channel configuration for the PCM I2S protocol with the specified sample rate
        /// and bits per sample, in stereo mode, with MCLK set to 256 times the sample rate.
        #[inline(always)]
        pub fn pcm(sample_rate_hz: u32, bits_per_sample: DataBitWidth) -> Self {
            Self {
                channel_cfg: Config::default(),
                clk_cfg: StdClkConfig::from_sample_rate_hz(sample_rate_hz),
                slot_cfg: StdSlotConfig::pcm_slot_default(bits_per_sample, SlotMode::Stereo),
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg: StdGpioConfig::default(),
            }
        }

        /// Create a new standard mode channel configuration for the MSB I2S protocol with the specified sample rate
        /// and bits per sample, in stereo mode, with MCLK set to 256 times the sample rate.
        #[inline(always)]
        pub fn msb(sample_rate_hz: u32, bits_per_sample: DataBitWidth) -> Self {
            Self {
                channel_cfg: Config::default(),
                clk_cfg: StdClkConfig::from_sample_rate_hz(sample_rate_hz),
                slot_cfg: StdSlotConfig::msb_slot_default(bits_per_sample, SlotMode::Stereo),
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg: StdGpioConfig::default(),
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_std_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(crate) fn as_sdk<'d>(
            &self,
            bclk: PeripheralRef<'d, impl InputPin + OutputPin>,
            din: Option<PeripheralRef<'d, impl InputPin>>,
            dout: Option<PeripheralRef<'d, impl OutputPin>>,
            mclk: Option<PeripheralRef<'d, impl InputPin + OutputPin>>,
            ws: PeripheralRef<'d, impl InputPin + OutputPin>,
        ) -> i2s_std_config_t {
            i2s_std_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(bclk, din, dout, mclk, ws),
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_driver_config_t` representation.
        ///
        /// # Note
        /// The mode field is not fully set by this function. Only the controller/target field is set. Before using,
        /// the following bits must be considered: `I2S_MODE_TX`, `I2S_MODE_RX`, `I2S_MODE_DAC_BUILT_IN`, and
        /// `I2S_MODE_ADC_BUILT_IN`. (`I2S_MODE_PDM` should not be used here.)
        #[cfg(esp_idf_version_major = "4")]
        pub(crate) fn as_sdk(&self) -> i2s_driver_config_t {
            let chan_fmt = match self.slot_cfg.slot_mode {
                SlotMode::Stereo => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                SlotMode::Mono => match self.slot_cfg.slot_mask {
                    StdSlotMask::Both => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                    StdSlotMask::Left => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_LEFT,
                    StdSlotMask::Right => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_RIGHT,
                },
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

                // The following are TDM-only fields and are not present on chips that don't support TDM mode.
                // There's no cfg option for this (it's a constant in esp-idf-sys).
                #[cfg(not(any(esp32, esp32s2)))]
                chan_mask: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                total_chan: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: self.slot_cfg.left_align,
                #[cfg(not(any(esp32, esp32s2)))]
                big_edin: self.slot_cfg.big_endian,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_msb: !self.slot_cfg.bit_order_lsb,
                #[cfg(not(any(esp32, esp32s2)))]
                skip_msk: true,
            }
        }
    }

    /// Standard mode channel clock configuration.
    #[derive(Clone)]
    pub struct StdClkConfig {
        /// I2S sample rate.
        sample_rate_hz: u32,

        /// Clock source.
        clk_src: ClockSource,

        /// The multiple of MCLK to the sample rate.
        mclk_multiple: MclkMultiple,
    }

    impl StdClkConfig {
        /// Create a standard clock configuration with the specified rate (in Hz), clock source, and MCLK multiple of
        /// the sample rate.
        #[inline(always)]
        pub fn new(sample_rate_hz: u32, clk_src: ClockSource, mclk_multiple: MclkMultiple) -> Self {
            Self {
                sample_rate_hz,
                clk_src,
                mclk_multiple,
            }
        }

        /// Create a standard clock configuration with the specified rate in Hz. This will set the clock source to
        /// PLL_F160M and the MCLK multiple to 256 times the sample rate.
        ///
        /// # Note
        /// Set the mclk_multiple to [MclkMultiple::M384] when using 24-bit data width. Otherwise, the sample rate
        /// might be imprecise since the BCLK division is not an integer.
        #[inline(always)]
        pub fn from_sample_rate_hz(rate: u32) -> Self {
            Self {
                sample_rate_hz: rate,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
            }
        }

        /// Set the clock source on this standard clock configuration.
        #[inline(always)]
        pub fn clk_src(mut self, clk_src: ClockSource) -> Self {
            self.clk_src = clk_src;
            self
        }

        /// Set the MCLK multiple on this standard clock configuration.
        #[inline(always)]
        pub fn mclk_multiple(mut self, mclk_multiple: MclkMultiple) -> Self {
            self.mclk_multiple = mclk_multiple;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_std_clk_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_std_clk_config_t {
            i2s_std_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
            }
        }
    }

    /// The communication format used by the v4 driver.
    #[cfg(esp_idf_version_major = "4")]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub enum StdCommFormat {
        /// Standard I2S/Philips format.
        Philips,

        /// MSB-aligned format (data present at first bit clock).
        Msb,

        /// PCM short standard. Word select is one bit clock.
        PcmShort,

        /// PCM long standard. Word select is the same as the data bit width.
        PcmLong,
    }

    #[cfg(esp_idf_version_major = "4")]
    impl StdCommFormat {
        #[inline(always)]
        pub(in crate::i2s) fn as_sdk(&self) -> i2s_comm_format_t {
            match self {
                Self::Philips => i2s_comm_format_t_I2S_COMM_FORMAT_STAND_I2S,
                Self::Msb => i2s_comm_format_t_I2S_COMM_FORMAT_STAND_MSB,
                Self::PcmShort => i2s_comm_format_t_I2S_COMM_FORMAT_PCM_SHORT,
                Self::PcmLong => i2s_comm_format_t_I2S_COMM_FORMAT_PCM_LONG,
            }
        }
    }

    /// Standard mode GPIO (general purpose input/output) configuration.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[derive(Default)]
    pub struct StdGpioConfig {
        /// Invert the BCLK signal.
        bclk_invert: bool,

        /// Invert the MCLK signal.
        mclk_invert: bool,

        /// Invert the WS signal.
        ws_invert: bool,
    }

    #[cfg(not(esp_idf_version_major = "4"))]
    impl StdGpioConfig {
        /// Create a new standard mode GPIO configuration with the specified inversion flags for BCLK, MCLK, and WS.
        pub fn new(bclk_invert: bool, mclk_invert: bool, ws_invert: bool) -> Self {
            Self {
                bclk_invert,
                mclk_invert,
                ws_invert,
            }
        }

        /// Set the BCLK inversion flag on this standard GPIO configuration.
        #[inline(always)]
        pub fn bclk_invert(mut self, bclk_invert: bool) -> Self {
            self.bclk_invert = bclk_invert;
            self
        }

        /// Set the MCLK inversion flag on this standard GPIO configuration.
        #[inline(always)]
        pub fn mclk_invert(mut self, mclk_invert: bool) -> Self {
            self.mclk_invert = mclk_invert;
            self
        }

        /// Set the WS inversion flag on this standard GPIO configuration.
        #[inline(always)]
        pub fn ws_invert(mut self, ws_invert: bool) -> Self {
            self.ws_invert = ws_invert;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_std_gpio_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub(crate) fn as_sdk<'d>(
            &self,
            bclk: PeripheralRef<'d, impl InputPin + OutputPin>,
            din: Option<PeripheralRef<'d, impl InputPin>>,
            dout: Option<PeripheralRef<'d, impl OutputPin>>,
            mclk: Option<PeripheralRef<'d, impl InputPin + OutputPin>>,
            ws: PeripheralRef<'d, impl InputPin + OutputPin>,
        ) -> i2s_std_gpio_config_t {
            let invert_flags = i2s_std_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_std_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.mclk_invert as u32,
                    self.bclk_invert as u32,
                    self.ws_invert as u32,
                ),
                ..Default::default()
            };

            i2s_std_gpio_config_t {
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

    /// Standard mode channel slot configuration.
    ///
    /// To create a slot configuration, use [StdSlotConfig::philips_slot_default], [StdSlotConfig::pcm_slot_default], or
    /// [StdSlotConfig::msb_slot_default], then customize it as needed.
    pub struct StdSlotConfig {
        /// I2S sample data bit width (valid data bits per sample).
        data_bit_width: DataBitWidth,

        /// I2S slot bit width (total bits per slot).
        slot_bit_width: SlotBitWidth,

        /// Mono or stereo mode operation.
        slot_mode: SlotMode,

        /// Are we using the left, right, or both data slots?
        slot_mask: StdSlotMask,

        /// The word select (WS) signal width, in terms of the bit clock (BCK) periods.
        #[cfg(not(esp_idf_version_major = "4"))]
        ws_width: u32,

        /// The word select signal polarity; true enables the light lever first.
        #[cfg(not(esp_idf_version_major = "4"))]
        ws_polarity: bool,

        /// Set to enable the additional bit-shift needed in Philips mode.
        #[cfg(not(esp_idf_version_major = "4"))]
        bit_shift: bool,

        /// ESP32/ESP32S2 only: place the right slot data in the MSB in the FIFO.
        #[cfg(all(any(esp32, esp32s2), not(esp_idf_version_major = "4")))]
        msb_right: bool,

        /// The communication format used by the driver.
        #[cfg(esp_idf_version_major = "4")]
        comm_fmt: StdCommFormat,

        /// Non-ESP32/ESP32S2: enable left-alignment
        #[cfg(not(any(esp32, esp32s2)))]
        left_align: bool,

        /// Non-ESP32/ESP32S2: Enable big-endian.
        #[cfg(not(any(esp32, esp32s2)))]
        big_endian: bool,

        /// Non-ESP32/ESP32S2: Enable LSB-first.
        #[cfg(not(any(esp32, esp32s2)))]
        bit_order_lsb: bool,
    }

    impl StdSlotConfig {
        /// Update the data bit width on this standard slot configuration.
        #[inline(always)]
        pub fn data_bit_width(mut self, data_bit_width: DataBitWidth) -> Self {
            self.data_bit_width = data_bit_width;
            self
        }

        /// Update the slot bit width on this standard slot configuration.
        #[inline(always)]
        pub fn slot_bit_width(mut self, slot_bit_width: SlotBitWidth) -> Self {
            self.slot_bit_width = slot_bit_width;
            self
        }

        /// Update the slot mode and mask on this standard slot configuration.
        #[inline(always)]
        pub fn slot_mode_mask(mut self, slot_mode: SlotMode, slot_mask: StdSlotMask) -> Self {
            self.slot_mode = slot_mode;
            self.slot_mask = slot_mask;
            self
        }

        /// Update the word select signal width on this standard slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn ws_width(mut self, ws_width: u32) -> Self {
            self.ws_width = ws_width;
            self
        }

        /// Update the word select signal polarity on this standard slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn ws_polarity(mut self, ws_polarity: bool) -> Self {
            self.ws_polarity = ws_polarity;
            self
        }

        /// Update the bit shift flag on this standard slot configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub fn bit_shift(mut self, bit_shift: bool) -> Self {
            self.bit_shift = bit_shift;
            self
        }

        /// Update the MSB-right flag on this standard slot configuration.
        #[cfg(all(any(esp32, esp32s2), not(esp_idf_version_major = "4")))]
        #[inline(always)]
        pub fn msb_right(mut self, msb_right: bool) -> Self {
            self.msb_right = msb_right;
            self
        }

        /// Update the communication format on this standard slot configuration.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub fn comm_fmt(mut self, comm_fmt: StdCommFormat) -> Self {
            self.comm_fmt = comm_fmt;
            self
        }

        /// Update the left-alignment flag on this standard slot configuration.
        #[cfg(not(any(esp32, esp32s2)))]
        #[inline(always)]
        pub fn left_align(mut self, left_align: bool) -> Self {
            self.left_align = left_align;
            self
        }

        /// Update the big-endian flag on this standard slot configuration.
        #[cfg(not(any(esp32, esp32s2)))]
        #[inline(always)]
        pub fn big_endian(mut self, big_endian: bool) -> Self {
            self.big_endian = big_endian;
            self
        }

        /// Update the LSB-first flag on this standard slot configuration.
        #[cfg(not(any(esp32, esp32s2)))]
        #[inline(always)]
        pub fn bit_order_lsb(mut self, bit_order_lsb: bool) -> Self {
            self.bit_order_lsb = bit_order_lsb;
            self
        }

        /// Configure in Philips format in 2 slots.
        pub fn philips_slot_default(bits_per_sample: DataBitWidth, slot_mode: SlotMode) -> Self {
            let slot_mask = if slot_mode == SlotMode::Mono && cfg!(any(esp32, esp32s2)) {
                StdSlotMask::Left
            } else {
                StdSlotMask::Both
            };

            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: bits_per_sample.into(),
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: false,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: true,
                #[cfg(all(esp32, not(esp_idf_version_major = "4")))]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(all(esp32s2, not(esp_idf_version_major = "4")))]
                msb_right: true,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: StdCommFormat::Philips,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: false,
            }
        }

        /// Configure in PCM (short) format in 2 slots.
        pub fn pcm_slot_default(bits_per_sample: DataBitWidth, slot_mode: SlotMode) -> Self {
            let slot_mask = if slot_mode == SlotMode::Mono && cfg!(any(esp32, esp32s2)) {
                StdSlotMask::Left
            } else {
                StdSlotMask::Both
            };

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
                bit_shift: true,
                #[cfg(all(esp32, not(esp_idf_version_major = "4")))]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(all(esp32s2, not(esp_idf_version_major = "4")))]
                msb_right: true,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: StdCommFormat::PcmShort,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: false,
            }
        }

        /// Configure in MSB format in 2 slots.
        pub fn msb_slot_default(bits_per_sample: DataBitWidth, slot_mode: SlotMode) -> Self {
            let slot_mask = if slot_mode == SlotMode::Mono && cfg!(any(esp32, esp32s2)) {
                StdSlotMask::Left
            } else {
                StdSlotMask::Both
            };

            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_width: bits_per_sample.into(),
                #[cfg(not(esp_idf_version_major = "4"))]
                ws_polarity: false,
                #[cfg(not(esp_idf_version_major = "4"))]
                bit_shift: false,
                #[cfg(all(esp32, not(esp_idf_version_major = "4")))]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(all(esp32s2, not(esp_idf_version_major = "4")))]
                msb_right: true,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: StdCommFormat::Msb,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: false,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_std_slot_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub(crate) fn as_sdk(&self) -> i2s_std_slot_config_t {
            i2s_std_slot_config_t {
                data_bit_width: self.data_bit_width.as_sdk(),
                slot_bit_width: self.slot_bit_width.as_sdk(),
                slot_mode: self.slot_mode.as_sdk(),
                slot_mask: self.slot_mask.as_sdk(),
                ws_width: self.ws_width,
                ws_pol: self.ws_polarity,
                bit_shift: self.bit_shift,
                #[cfg(any(esp32, esp32s2))]
                msb_right: self.msb_right,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: self.left_align,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: self.big_endian,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: self.bit_order_lsb,
            }
        }
    }

    // The derived Clone appears to have problems with some of the cfg attributes in rust-analyzer.
    impl Clone for StdSlotConfig {
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
                #[cfg(all(any(esp32, esp32s2), not(esp_idf_version_major = "4")))]
                msb_right: self.msb_right,
                #[cfg(esp_idf_version_major = "4")]
                comm_fmt: self.comm_fmt,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: self.left_align,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: self.big_endian,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: self.bit_order_lsb,
            }
        }
    }

    /// I2S slot selection in standard mode.
    ///
    /// The default is `StdSlotMask::Both`.
    ///
    /// # Note
    /// This has different meanings in transmit vs receive mode, and stereo vs mono mode. This may have different
    /// behaviors on different targets. For details, refer to the I2S API reference.
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum StdSlotMask {
        /// I2S transmits or receives the left slot.
        Left,

        /// I2S transmits or receives the right slot.
        Right,

        /// I2S transmits or receives both slots.
        Both,
    }

    impl Default for StdSlotMask {
        #[inline(always)]
        fn default() -> Self {
            Self::Both
        }
    }

    impl StdSlotMask {
        /// Convert to the ESP-IDF SDK `i2s_std_slot_mask_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_std_slot_mask_t {
            match self {
                Self::Left => 1 << 0,
                Self::Right => 1 << 1,
                Self::Both => (1 << 0) | (1 << 1),
            }
        }
    }
}

/// The I2S standard mode driver.
pub struct I2sStdDriver<'d, Dir> {
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

unsafe impl<'d, Dir> Send for I2sStdDriver<'d, Dir> {}
unsafe impl<'d, Dir> Sync for I2sStdDriver<'d, Dir> {}

impl<'d, Dir> I2sPort for I2sStdDriver<'d, Dir> {
    fn port(&self) -> i2s_port_t {
        self.i2s as u32
    }
}

impl<'d, Dir: I2sRxSupported + I2sTxSupported> I2sStdDriver<'d, Dir> {
    /// Create a new standard mode driver for the given I2S peripheral with both the receive and transmit channels open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_bidir<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
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

        // Create the channel configuration.
        let std_config = config.as_sdk(
            bclk.into_ref(),
            din.map(|d_in| d_in.into_ref()),
            dout.map(|d_out| d_out.into_ref()),
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: rx/tx.chan_handle are valid, non-null i2s_chan_handle_t,
        // and &std_config is a valid pointer to an i2s_std_config_t.
        unsafe {
            // Open the RX channel.
            esp!(i2s_channel_init_std_mode(rx.chan_handle, &std_config))?;

            // Open the TX channel.
            esp!(i2s_channel_init_std_mode(tx.chan_handle, &std_config))?;
        }

        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            tx: Some(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_bidir<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        din: Option<impl Peripheral<P = impl InputPin> + 'd>,
        dout: Option<impl Peripheral<P = impl OutputPin> + 'd>,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let mut driver_cfg = config.as_sdk();
        driver_cfg.mode |= i2s_mode_t_I2S_MODE_TX | i2s_mode_t_I2S_MODE_RX;

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

impl<'d, Dir: I2sRxSupported> I2sStdDriver<'d, Dir> {
    /// Create a new standard mode driver for the given I2S peripheral with only the receive channel open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
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

        // Create the channel configuration.
        let dout: Option<PeripheralRef<'d, AnyIOPin>> = None;
        let std_config: i2s_std_config_t = config.as_sdk(
            bclk.into_ref(),
            din.map(|d_in| d_in.into_ref()),
            dout,
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: rx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &std_config is a valid pointer to an i2s_std_config_t.
        unsafe {
            // Open the RX channel.
            esp!(i2s_channel_init_std_mode(rx.chan_handle, &std_config))?;
        }

        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            tx: None,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
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

impl<'d, Dir: I2sTxSupported> I2sStdDriver<'d, Dir> {
    /// Create a new standard mode driver for the given I2S peripheral with only the transmit channel open.
    #[cfg(not(esp_idf_version_major = "4"))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
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
        let std_config: i2s_std_config_t = config.as_sdk(
            bclk.into_ref(),
            din,
            dout.map(|d_out| d_out.into_ref()),
            mclk.map(|m_clk| m_clk.into_ref()),
            ws.into_ref(),
        );

        // Safety: tx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &std_config is a valid pointer to an i2s_std_config_t.
        unsafe {
            // Open the TX channel.
            esp!(i2s_channel_init_std_mode(tx.chan_handle, &std_config))?;
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

    #[cfg(esp_idf_version_major = "4")]
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::StdConfig,
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

impl<'d, Dir: I2sRxSupported> I2sRxChannel<'d> for I2sStdDriver<'d, Dir> {
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

impl<'d, Dir: I2sTxSupported> I2sTxChannel<'d> for I2sStdDriver<'d, Dir> {
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
impl<'d, Dir> Drop for I2sStdDriver<'d, Dir> {
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
