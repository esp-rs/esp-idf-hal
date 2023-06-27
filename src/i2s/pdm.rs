//! Pulse density modulation (PDM) driver for the ESP32 I2S peripheral.
use super::*;
use crate::{gpio::*, peripheral::Peripheral};
use core::{marker::PhantomData, ptr::null_mut};
use esp_idf_sys::*;

pub(super) mod config {
    #[allow(unused)]
    use crate::{gpio::*, i2s::config::*, peripheral::*};
    use esp_idf_sys::*;

    /// I2S pulse density modulation (PDM) downsampling mode.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    pub enum PdmDownsample {
        /// Downsample 8 samples.
        #[default]
        Samples8,

        /// Downsample 16 samples.
        Samples16,

        /// Maximum downsample rate.
        Max,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    impl PdmDownsample {
        /// Convert to the ESP-IDF SDK `i2s_pdm_downsample_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_dsr_t {
            match self {
                Self::Samples8 => 0,
                Self::Samples16 => 1,
                Self::Max => 2,
            }
        }
    }

    /// THe I2S pulse density modulation (PDM) mode receive clock configuration for the I2S peripheral.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    pub struct PdmRxClkConfig {
        /// The sample rate in Hz.
        pub(super) sample_rate_hz: u32,

        /// The clock source.
        clk_src: ClockSource,

        /// The multiple of the MCLK signal to the sample rate.
        mclk_multiple: MclkMultiple,

        /// Downsampling rate mode.
        pub(super) downsample_mode: PdmDownsample,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    impl PdmRxClkConfig {
        /// Create a PDM clock configuration with the specified sample rate in Hz. This will set the clock source to
        /// PLL_F160M, the MCLK multiple to 256 times the sample rate, and the downsampling mode to 8 samples.
        #[inline(always)]
        pub fn from_sample_rate_hz(rate: u32) -> Self {
            Self {
                sample_rate_hz: rate,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
                downsample_mode: PdmDownsample::Samples8,
            }
        }

        /// Set the clock source on this PDM receive clock configuration.
        #[inline(always)]
        pub fn clk_src(mut self, clk_src: ClockSource) -> Self {
            self.clk_src = clk_src;
            self
        }

        /// Set the MCLK multiple on this PDM receive clock configuration.
        #[inline(always)]
        pub fn mclk_multiple(mut self, mclk_multiple: MclkMultiple) -> Self {
            self.mclk_multiple = mclk_multiple;
            self
        }

        /// Set the downsampling mode on this PDM receive clock configuration.
        #[inline(always)]
        pub fn downsample_mode(mut self, downsample_mode: PdmDownsample) -> Self {
            self.downsample_mode = downsample_mode;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_rx_clk_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_rx_clk_config_t {
            i2s_pdm_rx_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
                dn_sample_mode: self.downsample_mode.as_sdk(),
            }
        }
    }

    /// The I2S pulse density modulation (PDM) mode receive configuration for the I2S peripheral.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    pub struct PdmRxConfig {
        /// The base channel configuration.
        pub(super) channel_cfg: Config,

        /// PDM mode channel receive clock configuration.
        pub(super) clk_cfg: PdmRxClkConfig,

        /// PDM mode channel slot configuration.
        pub(super) slot_cfg: PdmRxSlotConfig,

        /// PDM mode channel GPIO configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub(super) gpio_cfg: PdmRxGpioConfig,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    impl PdmRxConfig {
        /// Create a new PDM mode receive configuration from the specified clock, slot, and GPIO configurations.
        pub fn new(
            channel_cfg: Config,
            clk_cfg: PdmRxClkConfig,
            slot_cfg: PdmRxSlotConfig,
            #[cfg(not(esp_idf_version_major = "4"))] gpio_cfg: PdmRxGpioConfig,
        ) -> Self {
            Self {
                channel_cfg,
                clk_cfg,
                slot_cfg,
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg,
            }
        }

        /// Convert this PDM mode receive configuration into the ESP-IDF SDK `i2s_pdm_rx_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            din: PeripheralRef<'d, impl InputPin>,
        ) -> i2s_pdm_rx_config_t {
            i2s_pdm_rx_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(clk, din),
            }
        }

        /// Convert this PDM mode receive configuration into the ESP-IDF SDK `i2s_pdm_rx_config_t` representation.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        #[inline(always)]
        pub(super) fn as_sdk_multi<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dins: &[PeripheralRef<'d, impl InputPin>],
        ) -> i2s_pdm_rx_config_t {
            i2s_pdm_rx_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk_multi(clk, dins),
            }
        }

        /// Convert this PDM mode receive configuration into the ESP-IDF SDK `i2s_driver_config_t` representation.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_driver_config_t {
            let chan_fmt = match self.slot_cfg.slot_mode {
                SlotMode::Stereo => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                SlotMode::Mono => match self.slot_cfg.slot_mask {
                    PdmSlotMask::Both => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                    PdmSlotMask::Left => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_LEFT,
                    PdmSlotMask::Right => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_RIGHT,
                },
            };

            i2s_driver_config_t {
                mode: self.channel_cfg.role.as_sdk()
                    | i2s_mode_t_I2S_MODE_RX
                    | i2s_mode_t_I2S_MODE_PDM,
                sample_rate: self.clk_cfg.sample_rate_hz,
                bits_per_sample: 16, // fixed for PDM,
                channel_format: chan_fmt,
                communication_format: 0,  // ?
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
                bits_per_chan: 16, // fixed for PDM

                // The following are TDM-only fields and are not present on chips that don't support TDM mode.
                // There's no cfg option for this (it's a constant in esp-idf-sys).
                #[cfg(not(any(esp32, esp32s2)))]
                chan_mask: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                total_chan: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_edin: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_msb: false,
                #[cfg(not(any(esp32, esp32s2)))]
                skip_msk: true,
            }
        }
    }

    /// PDM mode GPIO (general purpose input/output) receive configuration.
    // Not used in v4
    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    #[derive(Default)]
    pub struct PdmRxGpioConfig {
        /// Whether the clock output is inverted.
        pub(super) clk_inv: bool,
    }

    /// The maximum number of data input pins that can be used in PDM mode.
    #[cfg(esp32)]
    pub const SOC_I2S_PDM_MAX_RX_LINES: usize = 1;

    /// The maximum number of data input pins that can be used in PDM mode.
    #[cfg(esp32s3)]
    pub const SOC_I2S_PDM_MAX_RX_LINES: usize = 4;

    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    impl PdmRxGpioConfig {
        /// Create a new PDM mode GPIO receive configuration with the specified inversion flag for the clock output.
        #[inline(always)]
        pub fn new(clk_inv: bool) -> Self {
            Self { clk_inv }
        }

        /// Set the clock inversion flag on this PDM GPIO configuration.
        #[inline(always)]
        pub fn clk_invert(mut self, clk_inv: bool) -> Self {
            self.clk_inv = clk_inv;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_rx_gpio_config_t` representation.
        #[cfg(any(
            esp_idf_version_major = "4",
            all(esp_idf_version_major = "5", esp_idf_version_minor = "0")
        ))]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            din: PeripheralRef<'d, impl InputPin>,
        ) -> i2s_pdm_rx_gpio_config_t {
            let invert_flags = i2s_pdm_rx_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_pdm_rx_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.clk_inv as u32,
                ),
                ..Default::default()
            };

            i2s_pdm_rx_gpio_config_t {
                clk: clk.pin(),
                din: din.pin(),
                invert_flags,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_rx_gpio_config_t` representation.
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            din: PeripheralRef<'d, impl InputPin>,
        ) -> i2s_pdm_rx_gpio_config_t {
            let mut dins: [gpio_num_t; SOC_I2S_PDM_MAX_RX_LINES] = [-1; SOC_I2S_PDM_MAX_RX_LINES];
            dins[0] = din.pin();

            let pins = i2s_pdm_rx_gpio_config_t__bindgen_ty_1 { dins };

            let invert_flags = i2s_pdm_rx_gpio_config_t__bindgen_ty_2 {
                _bitfield_1: i2s_pdm_rx_gpio_config_t__bindgen_ty_2::new_bitfield_1(
                    self.clk_inv as u32,
                ),
                ..Default::default()
            };

            i2s_pdm_rx_gpio_config_t {
                clk: clk.pin(),
                __bindgen_anon_1: pins,
                invert_flags,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_rx_gpio_config_t` representation.
        ///
        /// This will ignore any din pins beyond [SOC_I2S_PDM_MAX_RX_LINES].
        #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
        pub(crate) fn as_sdk_multi<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dins: &[PeripheralRef<'d, impl InputPin>],
        ) -> i2s_pdm_rx_gpio_config_t {
            let mut din_pins: [gpio_num_t; SOC_I2S_PDM_MAX_RX_LINES] =
                [-1; SOC_I2S_PDM_MAX_RX_LINES];

            for (i, din) in dins.iter().enumerate() {
                if i >= SOC_I2S_PDM_MAX_RX_LINES {
                    break;
                }

                din_pins[i] = din.pin();
            }

            let pins = i2s_pdm_rx_gpio_config_t__bindgen_ty_1 { dins: din_pins };

            let invert_flags = i2s_pdm_rx_gpio_config_t__bindgen_ty_2 {
                _bitfield_1: i2s_pdm_rx_gpio_config_t__bindgen_ty_2::new_bitfield_1(
                    self.clk_inv as u32,
                ),
                ..Default::default()
            };

            i2s_pdm_rx_gpio_config_t {
                clk: clk.pin(),
                __bindgen_anon_1: pins,
                invert_flags,
            }
        }
    }

    /// PDM mode channel receive slot configuration.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    #[derive(Clone)]
    pub struct PdmRxSlotConfig {
        /// I2S sample data bit width (valid data bits per sample).
        pub(super) data_bit_width: DataBitWidth,

        /// I2s slot bit width (total bits per slot).
        pub(super) slot_bit_width: SlotBitWidth,

        /// Mono or stereo mode operation.
        pub(super) slot_mode: SlotMode,

        /// Are we using the left, right, or both data slots?
        pub(super) slot_mask: PdmSlotMask,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
    impl PdmRxSlotConfig {
        /// Configure the PDM mode channel receive slot configuration for the specified bits per sample and slot mode
        /// in 2 slots.
        pub fn from_bits_per_sample_and_slot_mode(
            bits_per_sample: DataBitWidth,
            slot_mode: SlotMode,
        ) -> Self {
            let slot_mask = if slot_mode == SlotMode::Mono {
                PdmSlotMask::Left
            } else {
                PdmSlotMask::Both
            };

            Self {
                data_bit_width: bits_per_sample,
                slot_bit_width: SlotBitWidth::Auto,
                slot_mode,
                slot_mask,
            }
        }

        /// Update the data bit width on this PDM receive slot configuration.
        #[inline(always)]
        pub fn data_bit_width(mut self, data_bit_width: DataBitWidth) -> Self {
            self.data_bit_width = data_bit_width;
            self
        }

        /// Update the slot bit width on this PDM receive slot configuration.
        #[inline(always)]
        pub fn slot_bit_width(mut self, slot_bit_width: SlotBitWidth) -> Self {
            self.slot_bit_width = slot_bit_width;
            self
        }

        /// Update the slot mode and mask on this PDM receive slot configuration.
        #[inline(always)]
        pub fn slot_mode_mask(mut self, slot_mode: SlotMode, slot_mask: PdmSlotMask) -> Self {
            self.slot_mode = slot_mode;
            self.slot_mask = slot_mask;
            self
        }

        /// Convert this PDM mode channel receive slot configuration into the ESP-IDF SDK `i2s_pdm_rx_slot_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_rx_slot_config_t {
            i2s_pdm_rx_slot_config_t {
                data_bit_width: self.data_bit_width.as_sdk(),
                slot_bit_width: self.slot_bit_width.as_sdk(),
                slot_mode: self.slot_mode.as_sdk(),
                slot_mask: self.slot_mask.as_sdk(),
            }
        }
    }

    /// Pulse density modulation (PDM) transmit signal scaling mode.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PdmSignalScale {
        /// Divide the PDM signal by 2.
        Div2,

        /// No scaling.
        None,

        /// Multiply the PDM signal by 2.
        Mul2,

        /// Multiply the PDM signal by 4.
        Mul4,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl Default for PdmSignalScale {
        #[inline(always)]
        fn default() -> Self {
            Self::None
        }
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl PdmSignalScale {
        /// Convert to the ESP-IDF SDK `i2s_pdm_signal_scale_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PDM is implemented.
        pub(crate) fn as_sdk(&self) -> i2s_pdm_sig_scale_t {
            match self {
                Self::Div2 => 0,
                Self::None => 1,
                Self::Mul2 => 2,
                Self::Mul4 => 3,
            }
        }
    }

    /// I2S slot selection in PDM mode.
    ///
    /// The default is `PdmSlotMask::Both`.
    ///
    /// # Note
    /// This has different meanings in transmit vs receive mode, and stereo vs mono mode. This may have different
    /// behaviors on different targets. For details, refer to the I2S API reference.
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PdmSlotMask {
        /// I2S transmits or receives the left slot.
        Left,

        /// I2S transmits or receives the right slot.
        Right,

        /// I2S transmits or receives both slots.
        Both,
    }

    impl Default for PdmSlotMask {
        #[inline(always)]
        fn default() -> Self {
            Self::Both
        }
    }

    impl PdmSlotMask {
        /// Convert to the ESP-IDF SDK `i2s_pdm_slot_mask_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        #[allow(unused)]
        pub(crate) fn as_sdk(&self) -> i2s_pdm_slot_mask_t {
            match self {
                Self::Left => 1 << 0,
                Self::Right => 1 << 1,
                Self::Both => (1 << 0) | (1 << 1),
            }
        }
    }

    /// The I2s pulse density modulation (PDM) mode transmit clock configuration.
    ///
    /// # Note
    /// The PDM transmit clock can only be set to the following two upsampling rate configurations:
    /// * `upsampling_fp = 960`, `upsampling_fs = sample_rate_hz / 100`. In this case, `Fpdm = 128 * 48000 = 6.144 MHz`.
    /// * `upsampling_fp = 960`, `upsampling_fs = 480`. In this case, `Fpdm = 128 * sample_rate_hz`.
    ///
    /// If the PDM receiver does not use the PDM serial clock, the first configuration should be used. Otherwise,
    /// use the second configuration.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    pub struct PdmTxClkConfig {
        /// I2S sample rate in Hz.
        pub(super) sample_rate_hz: u32,

        /// The clock source.
        pub(super) clk_src: ClockSource,

        /// The multiple of MCLK to the sample rate.
        pub(super) mclk_multiple: MclkMultiple,

        /// Upsampling `fp` parameter.
        upsample_fp: u32,

        /// Upsampling `fs` parameter.
        upsample_fs: u32,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl PdmTxClkConfig {
        /// Create a new PDM mode transmit clock configuration from the specified sample rate in Hz. This will set the
        /// clock source to PLL_F160M, the MCLK multiple to 256 times the sample rate, `upsample_fp` to 960, and
        /// `upsample_fs` to 480.
        #[inline(always)]
        pub fn from_sample_rate_hz(sample_rate_hz: u32) -> Self {
            Self {
                sample_rate_hz,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
                upsample_fp: 960,
                upsample_fs: 480,
            }
        }

        /// Set the sample rate on this PDM mode transmit clock configuration.
        #[inline(always)]
        pub fn sample_rate_hz(mut self, sample_rate_hz: u32) -> Self {
            self.sample_rate_hz = sample_rate_hz;
            self
        }

        /// Set the clock source on this PDM mode transmit clock configuration.
        #[inline(always)]
        pub fn clk_src(mut self, clk_src: ClockSource) -> Self {
            self.clk_src = clk_src;
            self
        }

        /// Set the MCLK multiple on this PDM mode transmit clock configuration.
        #[inline(always)]
        pub fn mclk_multiple(mut self, mclk_multiple: MclkMultiple) -> Self {
            self.mclk_multiple = mclk_multiple;
            self
        }

        /// Set the upsampling parameters on this PDM mode transmit clock configuration.
        #[inline(always)]
        pub fn upsample(mut self, upsample_fp: u32, upsample_fs: u32) -> Self {
            self.upsample_fp = upsample_fp;
            self.upsample_fs = upsample_fs;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_tx_clk_config_t` representation.
        #[allow(clippy::needless_update)]
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_tx_clk_config_t {
            i2s_pdm_tx_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
                up_sample_fp: self.upsample_fp,
                up_sample_fs: self.upsample_fs,
                ..Default::default() // bclk_div in ESP IDF > 5.1
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_tx_upsample_cfg_t` representation.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_tx_upsample_cfg_t {
            i2s_pdm_tx_upsample_cfg_t {
                sample_rate: self.sample_rate_hz as i32,
                fp: self.upsample_fp as i32,
                fs: self.upsample_fs as i32,
            }
        }
    }

    /// The I2S pulse density modulation (PDM) mode transmit configuration for the I2S peripheral.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    pub struct PdmTxConfig {
        /// The base channel configuration.
        pub(super) channel_cfg: Config,

        /// PDM mode channel transmit clock configuration.
        pub(super) clk_cfg: PdmTxClkConfig,

        /// PDM mode channel transmit slot configuration.
        pub(super) slot_cfg: PdmTxSlotConfig,

        /// PDM mode channel transmit GPIO configuration.
        #[cfg(not(esp_idf_version_major = "4"))]
        pub(super) gpio_cfg: PdmTxGpioConfig,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl PdmTxConfig {
        /// Create a new PDM mode transmit configuration from the specified clock, slot, and GPIO configurations.
        pub fn new(
            channel_cfg: Config,
            clk_cfg: PdmTxClkConfig,
            slot_cfg: PdmTxSlotConfig,
            #[cfg(not(esp_idf_version_major = "4"))] gpio_cfg: PdmTxGpioConfig,
        ) -> Self {
            Self {
                channel_cfg,
                clk_cfg,
                slot_cfg,
                #[cfg(not(esp_idf_version_major = "4"))]
                gpio_cfg,
            }
        }

        /// Convert to the ESP-IDF `i2s_pdm_tx_config_t` representation.
        #[cfg(all(not(esp_idf_version_major = "4"), not(esp_idf_soc_i2s_hw_version_2)))]
        #[inline(always)]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dout: PeripheralRef<'d, impl OutputPin>,
        ) -> i2s_pdm_tx_config_t {
            i2s_pdm_tx_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(clk, dout),
            }
        }

        /// Convert to the ESP-IDF `i2s_pdm_tx_config_t` representation.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dout: PeripheralRef<'d, impl OutputPin>,
            dout2: Option<PeripheralRef<'d, impl OutputPin>>,
        ) -> i2s_pdm_tx_config_t {
            i2s_pdm_tx_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(clk, dout, dout2),
            }
        }

        /// Convert to the ESP-IDF `i2s_driver_config_t` representation.
        #[cfg(esp_idf_version_major = "4")]
        pub(crate) fn as_sdk(&self) -> i2s_driver_config_t {
            let chan_fmt = match self.slot_cfg.slot_mode {
                SlotMode::Stereo => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                SlotMode::Mono => match self.slot_cfg.slot_mask {
                    PdmSlotMask::Both => i2s_channel_fmt_t_I2S_CHANNEL_FMT_RIGHT_LEFT,
                    PdmSlotMask::Left => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_LEFT,
                    PdmSlotMask::Right => i2s_channel_fmt_t_I2S_CHANNEL_FMT_ONLY_RIGHT,
                },
            };

            i2s_driver_config_t {
                mode: self.channel_cfg.role.as_sdk()
                    | i2s_mode_t_I2S_MODE_TX
                    | i2s_mode_t_I2S_MODE_PDM,
                sample_rate: self.clk_cfg.sample_rate_hz,
                bits_per_sample: 16, // fixed for PDM,
                channel_format: chan_fmt,
                communication_format: 0,  // ?
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
                bits_per_chan: 16, // fixed for PDM

                // The following are TDM-only fields and are not present on chips that don't support TDM mode.
                // There's no cfg option for this (it's a constant in esp-idf-sys).
                #[cfg(not(any(esp32, esp32s2)))]
                chan_mask: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                total_chan: 0,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_edin: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_msb: false,
                #[cfg(not(any(esp32, esp32s2)))]
                skip_msk: true,
            }
        }
    }

    /// PDM mode GPIO (general purpose input/output) transmit configuration.
    // Not used in v4.
    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    #[derive(Default)]
    pub struct PdmTxGpioConfig {
        /// Whether the clock output is inverted.
        pub(super) clk_inv: bool,
    }

    /// The maximum number of data output pins that can be used in PDM mode.
    #[cfg(esp32)]
    pub const SOC_I2S_PDM_MAX_TX_LINES: usize = 1;

    /// The maximum number of data input pins that can be used in PDM mode.
    #[cfg(any(esp32c3, esp32c6, esp32h2))]
    pub const SOC_I2S_PDM_MAX_TX_LINES: usize = 2;

    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    impl PdmTxGpioConfig {
        /// Create a new PDM mode GPIO transmit configuration with the specified inversion flag for the clock output.
        #[inline(always)]
        pub fn new(clk_inv: bool) -> Self {
            Self { clk_inv }
        }

        /// Set the clock inversion flag on this PDM GPIO transmit configuration.
        #[inline(always)]
        pub fn clk_invert(mut self, clk_inv: bool) -> Self {
            self.clk_inv = clk_inv;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_tx_gpio_config_t` representation.
        #[cfg(esp_idf_soc_i2s_hw_version_1)]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dout: PeripheralRef<'d, impl OutputPin>,
        ) -> i2s_pdm_tx_gpio_config_t {
            let invert_flags = i2s_pdm_tx_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_pdm_tx_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.clk_inv as u32,
                ),
                ..Default::default()
            };
            i2s_pdm_tx_gpio_config_t {
                clk: clk.pin(),
                dout: dout.pin(),
                invert_flags,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_pdm_tx_gpio_config_t` representation.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        pub(crate) fn as_sdk<'d>(
            &self,
            clk: PeripheralRef<'d, impl OutputPin>,
            dout: PeripheralRef<'d, impl OutputPin>,
            dout2: Option<PeripheralRef<'d, impl OutputPin>>,
        ) -> i2s_pdm_tx_gpio_config_t {
            let invert_flags = i2s_pdm_tx_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_pdm_tx_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.clk_inv as u32,
                ),
                ..Default::default()
            };
            let dout2 = if let Some(dout2) = dout2 {
                dout2.pin()
            } else {
                -1
            };

            i2s_pdm_tx_gpio_config_t {
                clk: clk.pin(),
                dout: dout.pin(),
                dout2,
                invert_flags,
            }
        }
    }

    /// I2S pulse density modulation (PDM) transmit line mode
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum PdmTxLineMode {
        /// Standard PDM format output: left and right slot data on a single line.
        OneLineCodec,

        /// PDM DAC format output: left or right slot data on a single line.
        OneLineDac,

        /// PDM DAC format output: left and right slot data on separate lines.
        TwoLineDac,
    }

    impl Default for PdmTxLineMode {
        #[inline(always)]
        fn default() -> Self {
            Self::OneLineCodec
        }
    }

    impl PdmTxLineMode {
        /// Convert this to the ESP-IDF SDK `i2s_pdm_tx_line_mode_t` representation.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_tx_line_mode_t {
            match self {
                Self::OneLineCodec => i2s_pdm_tx_line_mode_t_I2S_PDM_TX_ONE_LINE_CODEC,
                Self::OneLineDac => i2s_pdm_tx_line_mode_t_I2S_PDM_TX_ONE_LINE_DAC,
                Self::TwoLineDac => i2s_pdm_tx_line_mode_t_I2S_PDM_TX_TWO_LINE_DAC,
            }
        }
    }

    /// I2S pulse density modulation (PDM) transmit slot configuration.
    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    pub struct PdmTxSlotConfig {
        // data_bit_width and slot_bit_width are omitted; they are always 16 bits.
        /// Mono or stereo mode operation.
        pub(super) slot_mode: SlotMode,

        /// Slot mask to choose the left or right slot.
        #[cfg(not(esp_idf_soc_i2s_hw_version_2))]
        pub(super) slot_mask: PdmSlotMask,

        /// Sigma-delta filter prescale.
        sd_prescale: u32,

        /// Sigma-delta filter saling value.
        sd_scale: PdmSignalScale,

        /// High-pass filter scaling value.
        hp_scale: PdmSignalScale,

        /// Low-pass filter scaling value
        lp_scale: PdmSignalScale,

        /// Sinc-filter scaling value.
        sinc_scale: PdmSignalScale,

        /// PDM transmit line mode.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        line_mode: PdmTxLineMode,

        /// High-pass filter enable
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        hp_enable: bool,

        /// High-pass filter cutoff frequence.
        /// The range of this is 23.3Hz to 185Hz.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        hp_cutoff_freq: f32,

        /// Sigma-delta filter dither.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        sd_dither: u32,

        /// Sigma-delta filter dither 2.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        sd_dither2: u32,
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl Default for PdmTxSlotConfig {
        #[inline(always)]
        fn default() -> Self {
            Self::from_slot_mode(SlotMode::Stereo)
        }
    }

    #[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
    impl PdmTxSlotConfig {
        /// Configure the PDM mode channel transmit slot configuration for the specified slot mode in 2 slots.
        ///
        /// This sets the sigma-delta, low-pass, and sinc scaling to None.
        ///
        /// For hardware version 1, the high-pass filter scaling is set to None.
        ///
        /// For hardware version 2, the high-pass filter is enabled, scaled to dividing by 2 and set to 35.5 Hz.
        #[inline(always)]
        pub fn from_slot_mode(slot_mode: SlotMode) -> Self {
            Self {
                slot_mode,
                #[cfg(not(esp_idf_soc_i2s_hw_version_2))]
                slot_mask: PdmSlotMask::Both,
                sd_prescale: 0,
                sd_scale: PdmSignalScale::None,
                #[cfg(not(esp_idf_soc_i2s_hw_version_2))]
                hp_scale: PdmSignalScale::None,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                hp_scale: PdmSignalScale::Div2,
                lp_scale: PdmSignalScale::None,
                sinc_scale: PdmSignalScale::None,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                line_mode: PdmTxLineMode::OneLineCodec,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                hp_enable: true,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                hp_cutoff_freq: 32.5,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                sd_dither: 0,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                sd_dither2: 1,
            }
        }

        /// Sets the slot mode on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn slot_mode(mut self, slot_mode: SlotMode) -> Self {
            self.slot_mode = slot_mode;
            self
        }

        /// Sets the slot mask on this PDM transmit slot configuration.
        #[cfg(esp_idf_soc_i2s_hw_version_1)]
        #[inline(always)]
        pub fn slot_mask(mut self, slot_mask: PdmSlotMask) -> Self {
            self.slot_mask = slot_mask;
            self
        }

        /// Sets the sigma-delta filter prescale on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn sd_prescale(mut self, sd_prescale: u32) -> Self {
            self.sd_prescale = sd_prescale;
            self
        }

        /// Sets the sigma-delta filter scaling on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn sd_scale(mut self, sd_scale: PdmSignalScale) -> Self {
            self.sd_scale = sd_scale;
            self
        }

        /// Sets the high-pass filter scaling on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn hp_scale(mut self, hp_scale: PdmSignalScale) -> Self {
            self.hp_scale = hp_scale;
            self
        }

        /// Sets the low-pass filter scaling on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn lp_scale(mut self, lp_scale: PdmSignalScale) -> Self {
            self.lp_scale = lp_scale;
            self
        }

        /// Sets the sinc filter scaling on this PDM transmit slot configuration.
        #[inline(always)]
        pub fn sinc_scale(mut self, sinc_scale: PdmSignalScale) -> Self {
            self.sinc_scale = sinc_scale;
            self
        }

        /// Sets the PDM transmit line mode on this PDM transmit slot configuration.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub fn line_mode(mut self, line_mode: PdmTxLineMode) -> Self {
            self.line_mode = line_mode;
            self
        }

        /// Sets the high-pass filter enable on this PDM transmit slot configuration.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub fn hp_enable(mut self, hp_enable: bool) -> Self {
            self.hp_enable = hp_enable;
            self
        }

        /// Sets the high-pass filter cutoff frequency on this PDM transmit slot configuration.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub fn hp_cutoff_freq(mut self, hp_cutoff_freq: f32) -> Self {
            self.hp_cutoff_freq = hp_cutoff_freq;
            self
        }

        /// Sets the sigma-delta filter dither on this PDM transmit slot configuration.
        #[cfg(esp_idf_soc_i2s_hw_version_2)]
        #[inline(always)]
        pub fn sd_dither(mut self, sd_dither: u32, sd_dither2: u32) -> Self {
            self.sd_dither = sd_dither;
            self.sd_dither2 = sd_dither2;
            self
        }

        /// Convert this to the ESP-IDF SDK `i2s_pdm_tx_slot_config_t` type.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_pdm_tx_slot_config_t {
            i2s_pdm_tx_slot_config_t {
                data_bit_width: DataBitWidth::Bits16.as_sdk(),
                slot_bit_width: SlotBitWidth::Bits16.as_sdk(),
                slot_mode: self.slot_mode.as_sdk(),
                #[cfg(esp_idf_soc_i2s_hw_version_1)]
                slot_mask: self.slot_mask.as_sdk(),
                sd_prescale: self.sd_prescale,
                sd_scale: self.sd_scale.as_sdk(),
                hp_scale: self.hp_scale.as_sdk(),
                lp_scale: self.lp_scale.as_sdk(),
                sinc_scale: self.sinc_scale.as_sdk(),
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                line_mode: self.line_mode.as_sdk(),
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                hp_en: self.hp_enable,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                hp_cut_off_freq_hz: self.hp_cutoff_freq,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                sd_dither: self.sd_dither,
                #[cfg(esp_idf_soc_i2s_hw_version_2)]
                sd_dither2: self.sd_dither2,
            }
        }
    }
}

/// The I2S pulse density modulation (PDM) driver.
pub struct I2sPdmDriver<'d, Dir> {
    /// The Rx channel, possibly None.
    #[cfg(all(esp_idf_soc_i2s_supports_pdm_rx, feature = "alloc"))]
    rx: Option<I2sChannel<I2sRxEvent>>,

    /// The Rx channel, possibly None.
    #[cfg(all(esp_idf_soc_i2s_supports_pdm_rx, not(feature = "alloc")))]
    rx: Option<I2sChannel>,

    /// The Tx channel, possibly None.
    #[cfg(all(esp_idf_soc_i2s_supports_pdm_tx, feature = "alloc"))]
    tx: Option<I2sChannel<I2sTxEvent>>,

    /// The Tx channel, possibly None.
    #[cfg(all(esp_idf_soc_i2s_supports_pdm_tx, not(feature = "alloc")))]
    tx: Option<I2sChannel>,

    /// The I2S peripheral number. Either 0 or 1 (ESP32 and ESP32S3 only).
    i2s: u8,

    /// Driver lifetime -- mimics the lifetime of the peripheral.
    _p: PhantomData<&'d ()>,

    /// Directionality -- mimics the directionality of the peripheral.
    _dir: PhantomData<Dir>,
}

unsafe impl<'d, Dir> Send for I2sPdmDriver<'d, Dir> {}
unsafe impl<'d, Dir> Sync for I2sPdmDriver<'d, Dir> {}

impl<'d, Dir> I2sPort for I2sPdmDriver<'d, Dir> {
    /// Returns the I2S port number of this driver.
    fn port(&self) -> i2s_port_t {
        self.i2s as u32
    }
}

#[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
impl<'d, Dir: I2sRxSupported> I2sPdmDriver<'d, Dir> {
    /// Create a new pulse density modulation (PDM) mode driver for the given I2S peripheral with only the receive
    /// channel open.
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        rx_cfg: config::PdmRxConfig,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        din: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let chan_cfg = rx_cfg.channel_cfg.as_sdk(port);

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
        let rx_cfg = rx_cfg.as_sdk(clk.into_ref(), din.into_ref());

        // Safety: rx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &rx_cfg is a valid pointer to an i2s_pdm_rx_config_t.
        unsafe {
            // Open the RX channel.
            esp!(esp_idf_sys::i2s_channel_init_pdm_rx_mode(
                rx.chan_handle,
                &rx_cfg
            ))?;
        }

        // Now we leak the rx channel so it is no longer managed. This pins it in memory in a way that
        // is easily accessible to the ESP-IDF SDK.
        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
            tx: None,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }

    /// Create a new pulse density modulation (PDM) mode driver for the given I2S peripheral with only the receive
    /// channel open using multiple DIN pins to receive data.
    #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx_multi<I2S, I2SP, CLK, CLKP, DIN, DINP, const DINC: usize>(
        _i2s: I2SP,
        rx_cfg: config::PdmRxConfig,
        clk: CLKP,
        dins: [DINP; DINC],
    ) -> Result<Self, EspError>
    where
        I2SP: Peripheral<P = I2S> + 'd,
        I2S: I2s,
        CLKP: Peripheral<P = CLK> + 'd,
        CLK: OutputPin,
        DINP: Peripheral<P = DIN> + 'd,
        DIN: InputPin + Sized,
    {
        let port = I2S::port();
        let chan_cfg = rx_cfg.channel_cfg.as_sdk(port);

        let mut rx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe { esp!(i2s_new_channel(&chan_cfg, null_mut(), &mut rx_chan_handle,))? };

        if rx_chan_handle.is_null() {
            panic!("Expected non-null rx channel handle");
        }

        // Allocate the internal channel struct.
        let rx = I2sChannel::new(port as u8, rx_chan_handle);

        // Safety: assume_init is safe to call because we are only claiming to have "initialized" the
        // MaybeUninit, not the PeripheralRef itself.
        let mut dins_ref: [MaybeUninit<crate::peripheral::PeripheralRef<'d, DIN>>; DINC] =
            unsafe { MaybeUninit::uninit().assume_init() };
        for (i, din) in IntoIterator::into_iter(dins).enumerate() {
            dins_ref[i].write(din.into_ref());
        }

        // Safety: everything is initialized, so we can safely transmute the array to the initialized type.
        let dins_ref = unsafe {
            core::mem::transmute_copy::<_, [crate::peripheral::PeripheralRef<'d, DIN>; DINC]>(
                &dins_ref,
            )
        };

        // Create the channel configuration.
        let rx_cfg = rx_cfg.as_sdk_multi(clk.into_ref(), &dins_ref);

        // Safety: rx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &rx_cfg is a valid pointer to an i2s_pdm_rx_config_t.
        unsafe {
            // Open the RX channel.
            esp!(esp_idf_sys::i2s_channel_init_pdm_rx_mode(
                rx.chan_handle,
                &rx_cfg
            ))?;
        }

        Ok(Self {
            i2s: port as u8,
            rx: Some(rx),
            #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
            tx: None,
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

#[cfg(all(esp_idf_version_major = "4", any(esp32, esp32s3)))]
impl<'d, Dir: I2sRxSupported> I2sPdmDriver<'d, Dir> {
    /// Create a new pulse density modulation (PDM) mode driver for the given I2S peripheral with the receive channel
    /// open.
    #[allow(clippy::too_many_arguments)]
    pub fn new_rx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        rx_cfg: config::PdmRxConfig,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        din: impl Peripheral<P = impl InputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let driver_cfg = rx_cfg.as_sdk();

        // Safety: &driver_cfg is a valid pointer to an i2s_driver_config_t.
        unsafe {
            esp!(i2s_driver_install(port, &driver_cfg, 0, null_mut()))?;
        }

        // Set the rate and downsampling configuration.
        let downsample = rx_cfg.clk_cfg.downsample_mode.as_sdk();
        unsafe {
            esp!(i2s_set_pdm_rx_down_sample(port, downsample))?;
        }

        // Set the pin configuration.
        let pin_cfg = i2s_pin_config_t {
            bck_io_num: clk.into_ref().pin(),
            data_in_num: din.into_ref().pin(),
            data_out_num: -1,
            mck_io_num: -1,
            ws_io_num: -1,
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

#[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
impl<'d, Dir: I2sTxSupported> I2sPdmDriver<'d, Dir> {
    /// Create a new pulse density modulation (PDM) mode driver for the given I2S peripheral with only the transmit
    /// channel open.
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        tx_cfg: config::PdmTxConfig,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        dout: impl Peripheral<P = impl OutputPin> + 'd,
        #[cfg(esp_idf_soc_i2s_hw_version_2)] dout2: Option<
            impl Peripheral<P = impl OutputPin> + 'd,
        >,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let chan_cfg = tx_cfg.channel_cfg.as_sdk(port);

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
        let tx_cfg = tx_cfg.as_sdk(
            clk.into_ref(),
            dout.into_ref(),
            #[cfg(esp_idf_soc_i2s_hw_version_2)]
            dout2.map(|dout2| dout2.into_ref()),
        );

        // Safety: tx.chan_handle is a valid, non-null i2s_chan_handle_t,
        // and &tx_cfg is a valid pointer to an i2s_pdm_tx_config_t.
        unsafe {
            // Open the TX channel.
            esp!(esp_idf_sys::i2s_channel_init_pdm_tx_mode(
                tx.chan_handle,
                &tx_cfg
            ))?;
        }

        Ok(Self {
            i2s: port as u8,
            #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
            rx: None,
            tx: Some(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

#[cfg(all(esp_idf_version_major = "4", any(esp32, esp32s3, esp32c3, esp32c6)))]
impl<'d, Dir: I2sTxSupported> I2sPdmDriver<'d, Dir> {
    /// Create a new pulse density modulation (PDM) mode driver for the given I2S peripheral with both the receive and
    /// transmit channels open.
    #[allow(clippy::too_many_arguments)]
    pub fn new_tx<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        tx_cfg: config::PdmTxConfig,
        clk: impl Peripheral<P = impl OutputPin> + 'd,
        dout: impl Peripheral<P = impl OutputPin> + 'd,
    ) -> Result<Self, EspError> {
        let port = I2S::port();
        let driver_cfg = tx_cfg.as_sdk();

        // Safety: &driver_cfg is a valid pointer to an i2s_driver_config_t.
        unsafe {
            esp!(i2s_driver_install(port, &driver_cfg, 0, null_mut()))?;
        }

        // Set the upsampling configuration.
        let upsample = tx_cfg.clk_cfg.as_sdk();
        unsafe {
            esp!(i2s_set_pdm_tx_up_sample(port, &upsample))?;
        }

        // Set the pin configuration.
        let pin_cfg = i2s_pin_config_t {
            bck_io_num: clk.into_ref().pin(),
            data_in_num: -1,
            data_out_num: dout.into_ref().pin(),
            mck_io_num: -1,
            ws_io_num: -1,
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

#[cfg(any(esp_idf_soc_i2s_supports_pdm_rx, esp32, esp32s3))]
impl<'d, Dir: I2sRxSupported> I2sRxChannel<'d> for I2sPdmDriver<'d, Dir> {
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
        rx_callback: impl FnMut(u8, I2sRxEvent) -> bool + 'static,
    ) -> Result<(), EspError> {
        self.rx.as_mut().unwrap().rx_subscribe(rx_callback)
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(feature = "riscv-ulp-hal"),
        feature = "alloc"
    ))]
    fn rx_unsubscribe(&mut self) -> Result<(), EspError> {
        self.rx.as_mut().unwrap().rx_unsubscribe()
    }
}

#[cfg(any(esp_idf_soc_i2s_supports_pdm_tx, esp32, esp32s3, esp32c3, esp32c6))]
impl<'d, Dir: I2sTxSupported> I2sTxChannel<'d> for I2sPdmDriver<'d, Dir> {
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
        tx_callback: impl FnMut(u8, I2sTxEvent) -> bool + 'static,
    ) -> Result<(), EspError> {
        self.tx.as_mut().unwrap().tx_subscribe(tx_callback)
    }

    #[cfg(all(
        not(esp_idf_version_major = "4"),
        not(feature = "riscv-ulp-hal"),
        feature = "alloc"
    ))]
    fn tx_unsubscribe(&mut self) -> Result<(), EspError> {
        self.tx.as_mut().unwrap().tx_unsubscribe()
    }
}

#[cfg(esp_idf_version_major = "4")]
impl<'d, Dir> Drop for I2sPdmDriver<'d, Dir> {
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
