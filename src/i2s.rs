//! Driver for the Inter-IC Sound (I2S) peripheral(s).

use crate::{
    gpio::{IOPin, Pin},
    peripheral::Peripheral,
};
use core::{
    convert::TryInto,
    ffi::c_void,
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    ptr::null_mut,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};
use esp_idf_sys::{
    esp, esp_err_to_name, esp_log_level_t_ESP_LOG_ERROR, esp_log_write, i2s_chan_config_t,
    i2s_chan_handle_t, i2s_channel_disable, i2s_channel_enable, i2s_channel_init_std_mode,
    i2s_channel_read, i2s_channel_register_event_callback, i2s_channel_write, i2s_del_channel,
    i2s_event_callbacks_t, i2s_event_data_t, i2s_new_channel, i2s_port_t, i2s_role_t, EspError,
    ESP_ERR_NOT_FOUND,
};

/// I2S peripheral in controller (master) role, bclk and ws signal will be set to output.
const I2S_ROLE_CONTROLLER: i2s_role_t = 0;

/// I2S peripheral in target (slave) role, bclk and ws signal will be set to input.
const I2S_ROLE_TARGET: i2s_role_t = 1;

/// Logging tag.
const LOG_TAG: &[u8; 17] = b"esp-idf-hal::i2s\0";

/// I2S configuration
pub mod config {
    use super::{I2S_ROLE_CONTROLLER, I2S_ROLE_TARGET};
    use crate::{
        gpio::{IOPin, InputPin, OutputPin, Pin},
        peripheral::PeripheralRef,
    };
    use core::convert::TryFrom;
    use esp_idf_sys::{
        i2s_chan_config_t, i2s_clock_src_t, i2s_data_bit_width_t, i2s_mclk_multiple_t,
        i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_128, i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256,
        i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_384, i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_512,
        i2s_port_t, i2s_role_t, i2s_slot_bit_width_t, i2s_slot_mode_t, i2s_std_clk_config_t,
        i2s_std_config_t, i2s_std_gpio_config_t, i2s_std_gpio_config_t__bindgen_ty_1,
        i2s_std_slot_config_t, i2s_std_slot_mask_t, EspError, ESP_ERR_INVALID_ARG,
    };

    /// Possible types of I2S channel configurations.
    pub enum ChanConfig<'a, Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Standard mode channel configuration.
        Std(&'a StdChanConfig<Bclk, Din, Dout, Mclk, Ws>),
    }

    /// I2S driver channels to open
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum ChannelOpen {
        /// Open both RX and TX channels
        Both,

        /// Open RX channel only
        Rx,

        /// Open TX channel only
        Tx,
    }

    impl Default for ChannelOpen {
        #[inline(always)]
        fn default() -> Self {
            Self::Both
        }
    }

    /// I2S clock source
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum ClockSource {
        /// Use PLL_F160M as the source clock
        Pll160M,

        /// Use APLL as the source clock
        #[cfg(any(esp32, esp32s2))]
        Apll,

        /// Use XTAL as the source clock
        #[cfg(any(esp32c3, esp32s3))]
        Xtal,
    }

    impl Default for ClockSource {
        #[inline(always)]
        fn default() -> Self {
            Self::Pll160M
        }
    }

    impl ClockSource {
        pub(super) fn as_sdk(&self) -> i2s_clock_src_t {
            match self {
                Self::Pll160M => esp_idf_sys::soc_module_clk_t_SOC_MOD_CLK_PLL_F160M,
                #[cfg(any(esp32, esp32s2))]
                Self::Apll => esp_idf_sys::soc_module_clk_t_SOC_MOD_CLK_APLL,
                #[cfg(any(esp32c3, esp32s3))]
                Self::Xtal => esp_idf_sys::soc_module_clk_t_SOC_MOD_CLK_XTAL,
            }
        }
    }

    /// I2S controller channel configuration.
    ///
    /// To create a custom configuration, use the builder pattern built-in to this struct. For example:
    /// ```
    /// use esp_idf_hal::i2s::config::{ChannelOpen, Config, Role};
    /// let config = Config::default().role(Role::Target).channels(ChannelOpen::Rx);
    /// ```
    ///
    /// The default configuration is:
    /// * Role ([Config::role]): [Role::Controller] (master)
    /// * DMA buffer number/descriptor number ([Config::dma_desc]): 6
    /// * I2S frames in one DMA buffer ([Config::dma_frame]): 240
    /// * Auto clear ([Config::auto_clear]): false
    /// * Channels to open ([Config::channels]): [ChannelOpen::Both]
    #[derive(Clone)]
    pub struct Config {
        /// The role of this channel: controller (master) or target (slave)
        pub(super) role: Role,

        /// The DMA buffer number to use (also the DMA descriptor number).
        pub(super) dma_desc: u32,

        /// The number of I2S frames in one DMA buffer.
        pub(super) dma_frame: u32,

        /// If true, the transmit buffer will be automatically cleared upon sending.
        pub(super) auto_clear: bool,

        /// The channels to open.
        pub(super) channels: ChannelOpen,
    }

    impl Default for Config {
        #[inline(always)]
        fn default() -> Self {
            Self {
                role: Role::Controller,
                dma_desc: 6,
                dma_frame: 240,
                auto_clear: false,
                channels: ChannelOpen::Both,
            }
        }
    }

    impl Config {
        #[inline(always)]
        pub fn new() -> Self {
            Default::default()
        }

        /// Set the role of this channel: controller (master) or target (slave)
        #[must_use]
        #[inline(always)]
        pub fn role(mut self, role: Role) -> Self {
            self.role = role;
            self
        }

        /// Set the DMA buffer to use.
        #[must_use]
        #[inline(always)]
        pub fn dma_desc(mut self, dma_desc: u32) -> Self {
            self.dma_desc = dma_desc;
            self
        }

        /// Set the I2S frame number in one DMA buffer.
        #[must_use]
        #[inline(always)]
        pub fn dma_frame(mut self, dma_frame: u32) -> Self {
            self.dma_frame = dma_frame;
            self
        }

        /// Set if the transmit buffer will be automatically cleared upon sending.
        #[must_use]
        #[inline(always)]
        pub fn auto_clear(mut self, auto_clear: bool) -> Self {
            self.auto_clear = auto_clear;
            self
        }

        /// Set the channels to open.
        #[must_use]
        #[inline(always)]
        pub fn channels(mut self, channels: ChannelOpen) -> Self {
            self.channels = channels;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_chan_config_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self, id: i2s_port_t) -> i2s_chan_config_t {
            i2s_chan_config_t {
                id,
                role: self.role.as_sdk(),
                dma_desc_num: self.dma_desc,
                dma_frame_num: self.dma_frame,
                auto_clear: self.auto_clear,
            }
        }
    }

    /// Available data bit width in one slot.
    #[derive(Clone, Copy, Eq, Ord, PartialEq, PartialOrd)]
    pub enum DataBitWidth {
        /// Channel data bit width is 8 bits.
        Bits8,

        /// Channel data bit width is 16 bits.
        Bits16,

        /// Channel data bit width is 24 bits.
        Bits24,

        /// Channel data bit width is 32 bits.
        Bits32,
    }

    impl From<DataBitWidth> for u32 {
        #[inline(always)]
        fn from(value: DataBitWidth) -> Self {
            match value {
                DataBitWidth::Bits8 => 8,
                DataBitWidth::Bits16 => 16,
                DataBitWidth::Bits24 => 24,
                DataBitWidth::Bits32 => 32,
            }
        }
    }

    impl DataBitWidth {
        /// Convert to the ESP-IDF SDK `i2s_data_bit_width_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_data_bit_width_t {
            match self {
                Self::Bits8 => 8,
                Self::Bits16 => 16,
                Self::Bits24 => 24,
                Self::Bits32 => 32,
            }
        }
    }

    impl TryFrom<usize> for DataBitWidth {
        type Error = EspError;

        fn try_from(value: usize) -> Result<Self, Self::Error> {
            match value {
                8 => Ok(Self::Bits8),
                16 => Ok(Self::Bits16),
                24 => Ok(Self::Bits24),
                32 => Ok(Self::Bits32),
                _ => Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap()),
            }
        }
    }

    /// The multiple of MCLK to the sample rate.
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum MclkMultiple {
        /// MCLK = sample rate * 128
        M128,

        /// MCLK = sample rate * 256
        M256,

        /// MCLK = sample rate * 384
        M384,

        /// MCLK = sample rate * 512
        M512,
    }

    impl MclkMultiple {
        /// Convert to the ESP-IDF SDK `i2s_mclk_multiple_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_mclk_multiple_t {
            match self {
                Self::M128 => i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_128,
                Self::M256 => i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256,
                Self::M384 => i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_384,
                Self::M512 => i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_512,
            }
        }
    }

    impl From<MclkMultiple> for u32 {
        #[inline(always)]
        fn from(mclk_multiple: MclkMultiple) -> Self {
            match mclk_multiple {
                MclkMultiple::M128 => 128,
                MclkMultiple::M256 => 256,
                MclkMultiple::M384 => 384,
                MclkMultiple::M512 => 512,
            }
        }
    }

    /// A/U-law compression/decompression configuration.
    #[cfg(esp_idf_soc_i2s_supports_pcm)]
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PcmCompress {
        /// Disable A/U-law compression/decompression.
        Disable,

        /// A-law decompression.
        ADecompress,

        /// A-law compression.
        ACompress,

        /// U-law decompression.
        UDecompress,

        /// U-law compression.
        UCompress,
    }

    #[cfg(esp_idf_soc_i2s_supports_pcm)]
    impl Default for PcmCompress {
        #[inline(always)]
        fn default() -> Self {
            Self::Disable
        }
    }

    #[cfg(esp_idf_soc_i2s_supports_pcm)]
    impl PcmCompress {
        /// Convert to the ESP-IDF SDK `i2s_pcm_compress_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PCM is implemented.
        pub(super) fn as_sdk(&self) -> esp_idf_sys::i2s_pcm_compress_t {
            match self {
                Self::Disable => 0,
                Self::ADecompress => 1,
                Self::ACompress => 2,
                Self::UDecompress => 3,
                Self::UCompress => 4,
            }
        }
    }

    /// I2S pulse density modulation (PDM) downsampling mode.
    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum PdmDownsample {
        /// Downsample 8 samples.
        Samples8,

        /// Downsample 16 samples.
        Samples16,

        /// Maximum downsample rate.
        Max,
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    impl Default for PdmDownsample {
        #[inline(always)]
        fn default() -> Self {
            Self::Samples8
        }
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_rx)]
    impl PdmDownsample {
        /// Convert to the ESP-IDF SDK `i2s_pdm_downsample_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PDM is implemented.
        pub(super) fn as_sdk(&self) -> esp_idf_sys::i2s_pdm_dsr_t {
            match self {
                Self::Samples8 => 0,
                Self::Samples16 => 1,
                Self::Max => 2,
            }
        }
    }

    /// Pulse density modulation (PDM) transmit signal scaling mode.
    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
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

    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    impl Default for PdmSignalScale {
        #[inline(always)]
        fn default() -> Self {
            Self::None
        }
    }

    #[cfg(esp_idf_soc_i2s_supports_pdm_tx)]
    impl PdmSignalScale {
        /// Convert to the ESP-IDF SDK `i2s_pdm_signal_scale_t` representation.
        #[inline(always)]
        #[allow(unused)] // TODO: remove when PDM is implemented.
        pub(crate) fn as_sdk(&self) -> esp_idf_sys::i2s_pdm_sig_scale_t {
            match self {
                Self::Div2 => 0,
                Self::None => 1,
                Self::Mul2 => 2,
                Self::Mul4 => 3,
            }
        }
    }

    /// I2S channel operating role
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum Role {
        /// Controller (master)
        Controller,

        /// Target (slave)
        Target,
    }

    impl Default for Role {
        #[inline(always)]
        fn default() -> Self {
            Self::Controller
        }
    }

    impl Role {
        /// Convert to the ESP-IDF SDK `i2s_role_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_role_t {
            match self {
                Self::Controller => I2S_ROLE_CONTROLLER,
                Self::Target => I2S_ROLE_TARGET,
            }
        }
    }

    /// Standard mode channel configuration.
    pub struct StdChanConfig<Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Standard mode channel clock configuration.
        pub clk_cfg: StdClkConfig,

        /// Standard mode channel slot configuration.
        pub slot_cfg: StdSlotConfig,

        /// Standard mode channel GPIO configuration.
        pub gpio_cfg: StdGpioConfig<Bclk, Din, Dout, Mclk, Ws>,
    }

    impl<Bclk, Din, Dout, Mclk, Ws> StdChanConfig<Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Create a new standard mode channel configuration from the given clock configuration, slot configuration,
        /// and GPIO configuration.
        pub fn new(
            clk_cfg: StdClkConfig,
            slot_cfg: StdSlotConfig,
            gpio_cfg: StdGpioConfig<Bclk, Din, Dout, Mclk, Ws>,
        ) -> Self {
            Self {
                clk_cfg,
                slot_cfg,
                gpio_cfg,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_std_config_t` representation.
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_std_config_t {
            i2s_std_config_t {
                clk_cfg: self.clk_cfg.as_sdk(),
                slot_cfg: self.slot_cfg.as_sdk(),
                gpio_cfg: self.gpio_cfg.as_sdk(),
            }
        }
    }

    impl<'a, Bclk, Din, Dout, Mclk, Ws> From<&'a StdChanConfig<Bclk, Din, Dout, Mclk, Ws>>
        for ChanConfig<'a, Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        #[inline(always)]
        fn from(cfg: &'a StdChanConfig<Bclk, Din, Dout, Mclk, Ws>) -> Self {
            Self::Std(cfg)
        }
    }

    /// Standard mode channel clock configuration.
    #[derive(Clone)]
    pub struct StdClkConfig {
        /// I2S sample rate.
        pub sample_rate_hz: u32,

        /// Clock source.
        pub clk_src: ClockSource,

        /// The multiple of MCLK to the sample rate.
        pub mclk_multiple: MclkMultiple,
    }

    impl StdClkConfig {
        /// Create a standard clock configuration with the specified rate in Hz.
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

        /// Convert to the ESP-IDF SDK `i2s_std_clk_config_t` representation.
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_std_clk_config_t {
            i2s_std_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
            }
        }
    }

    /// Standard mode GPIO (general purpose input/output) configuration.
    pub struct StdGpioConfig<Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// MCLK (master/controller clock) pin. Optional output.
        mclk: Mclk,

        /// BCLK (bit clock) pin. Input in target/slave mode, output in controller/master mode.
        bclk: PeripheralRef<'static, Bclk>,

        /// WS (word select) pin. Input in target/slave mode, output in controller/master mode.
        ws: PeripheralRef<'static, Ws>,

        /// Data output pin.
        data_out: Dout,

        /// Data input pin.
        data_in: Din,

        /// Invert the MCLK signal.
        mclk_invert: bool,

        /// Invert the BCLK signal.
        bclk_invert: bool,

        /// Invert the WS signal.
        ws_invert: bool,
    }

    impl<Bclk, Din, Dout, Mclk, Ws> StdGpioConfig<Bclk, Din, Dout, Mclk, Ws>
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Convert to the ESP-IDF SDK `i2s_std_gpio_config_t` representation.
        pub(crate) fn as_sdk(&self) -> i2s_std_gpio_config_t {
            let invert_flags = i2s_std_gpio_config_t__bindgen_ty_1 {
                _bitfield_1: i2s_std_gpio_config_t__bindgen_ty_1::new_bitfield_1(
                    self.mclk_invert as u32,
                    self.bclk_invert as u32,
                    self.ws_invert as u32,
                ),
                ..Default::default()
            };

            i2s_std_gpio_config_t {
                mclk: self.mclk.pin(),
                bclk: self.bclk.pin(),
                ws: self.ws.pin(),
                dout: self.data_out.pin(),
                din: self.data_in.pin(),
                invert_flags,
            }
        }
    }

    /// Incremental builder for a standard mode GPIO configuration.
    pub struct StdGpioConfigBuilder<Bclk, Din, Dout, Mclk, Ws> {
        bclk: Bclk,
        data_in: Din,
        data_out: Dout,
        mclk: Mclk,
        ws: Ws,
        bclk_invert: bool,
        mclk_invert: bool,
        ws_invert: bool,
    }

    impl Default for StdGpioConfigBuilder<(), (), (), (), ()> {
        fn default() -> Self {
            Self {
                bclk: (),
                data_in: (),
                data_out: (),
                mclk: (),
                ws: (),
                bclk_invert: false,
                mclk_invert: false,
                ws_invert: false,
            }
        }
    }

    impl<Din, Dout, Mclk, Ws> StdGpioConfigBuilder<(), Din, Dout, Mclk, Ws> {
        /// Set the bit clock (BCK) pin.
        ///
        /// This must be called before calling [StdGpioConfigBuilder::build].
        #[must_use]
        #[inline(always)]
        pub fn bclk<Bclk: Pin + IOPin + 'static>(
            self,
            bclk: Bclk,
        ) -> StdGpioConfigBuilder<PeripheralRef<'static, Bclk>, Din, Dout, Mclk, Ws> {
            StdGpioConfigBuilder {
                bclk: bclk.into_ref(),
                data_in: self.data_in,
                data_out: self.data_out,
                mclk: self.mclk,
                ws: self.ws,
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    impl<Bclk, Dout, Mclk, Ws> StdGpioConfigBuilder<Bclk, (), Dout, Mclk, Ws> {
        /// Set the data input (DIN) pin.
        #[must_use]
        #[inline(always)]
        pub fn data_in<Din: Pin + InputPin + 'static>(
            self,
            data_in: Din,
        ) -> StdGpioConfigBuilder<Bclk, PeripheralRef<'static, Din>, Dout, Mclk, Ws> {
            StdGpioConfigBuilder {
                bclk: self.bclk,
                data_in: data_in.into_ref(),
                data_out: self.data_out,
                mclk: self.mclk,
                ws: self.ws,
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    impl<Bclk, Din, Mclk, Ws> StdGpioConfigBuilder<Bclk, Din, (), Mclk, Ws> {
        /// Set the data output (DOUT) pin.
        #[must_use]
        #[inline(always)]
        pub fn data_out<Dout: Pin + OutputPin + 'static>(
            self,
            data_out: Dout,
        ) -> StdGpioConfigBuilder<Bclk, Din, PeripheralRef<'static, Dout>, Mclk, Ws> {
            StdGpioConfigBuilder {
                bclk: self.bclk,
                data_in: self.data_in,
                data_out: data_out.into_ref(),
                mclk: self.mclk,
                ws: self.ws,
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    impl<Bclk, Din, Dout, Ws> StdGpioConfigBuilder<Bclk, Din, Dout, (), Ws> {
        /// Set the master clock (MCK) pin.
        #[must_use]
        #[inline(always)]
        pub fn mclk<Mclk: Pin + OutputPin + 'static>(
            self,
            mclk: Mclk,
        ) -> StdGpioConfigBuilder<Bclk, Din, Dout, PeripheralRef<'static, Mclk>, Ws> {
            StdGpioConfigBuilder {
                bclk: self.bclk,
                data_in: self.data_in,
                data_out: self.data_out,
                mclk: mclk.into_ref(),
                ws: self.ws,
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    impl<Bclk, Din, Dout, Mclk> StdGpioConfigBuilder<Bclk, Din, Dout, Mclk, ()> {
        /// Set the word select (WS) pin.
        ///
        /// This must be called before calling [StdGpioConfigBuilder::build].
        #[must_use]
        #[inline(always)]
        pub fn ws<Ws: Pin + IOPin + 'static>(
            self,
            ws: Ws,
        ) -> StdGpioConfigBuilder<Bclk, Din, Dout, Mclk, PeripheralRef<'static, Ws>> {
            StdGpioConfigBuilder {
                bclk: self.bclk,
                data_in: self.data_in,
                data_out: self.data_out,
                mclk: self.mclk,
                ws: ws.into_ref(),
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    impl<Bclk, Din, Dout, Mclk, Ws> StdGpioConfigBuilder<Bclk, Din, Dout, Mclk, Ws> {
        /// Set the inversion state of the bit clock (BCK) pin.
        #[must_use]
        #[inline(always)]
        pub fn bclk_invert(mut self, bclk_invert: bool) -> Self {
            self.bclk_invert = bclk_invert;
            self
        }

        /// Set the inversion state of the master clock (MCK) pin.
        #[must_use]
        #[inline(always)]
        pub fn mclk_invert(mut self, mclk_invert: bool) -> Self {
            self.mclk_invert = mclk_invert;
            self
        }

        /// Set the inversion state of the word select (WS) pin.
        #[must_use]
        #[inline(always)]
        pub fn ws_invert(mut self, ws_invert: bool) -> Self {
            self.ws_invert = ws_invert;
            self
        }
    }

    impl<Bclk, Din, Dout, Mclk, Ws>
        StdGpioConfigBuilder<
            PeripheralRef<'static, Bclk>,
            Din,
            Dout,
            Mclk,
            PeripheralRef<'static, Ws>,
        >
    where
        Bclk: Pin + IOPin + 'static,
        Din: MaybePin + 'static,
        Dout: MaybePin + 'static,
        Mclk: MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Create a new standard mode GPIO configuration.
        ///
        /// This will panic if the BCK or WS pins have not been set.
        pub fn build(self) -> StdGpioConfig<Bclk, Din, Dout, Mclk, Ws> {
            StdGpioConfig {
                bclk: self.bclk,
                data_in: self.data_in,
                data_out: self.data_out,
                mclk: self.mclk,
                ws: self.ws,
                bclk_invert: self.bclk_invert,
                mclk_invert: self.mclk_invert,
                ws_invert: self.ws_invert,
            }
        }
    }

    /// Standard mode channel slot configuration.
    #[derive(Clone)]
    pub struct StdSlotConfig {
        /// I2S sample data bit width (valid data bits per sample).
        pub data_bit_width: DataBitWidth,

        /// I2S slot bit width (total bits per slot).
        pub slot_bit_width: SlotBitWidth,

        /// Mono or stereo mode operation.
        pub slot_mode: SlotMode,

        /// Are we using the left, right, or both data slots?
        pub slot_mask: StdSlotMask,

        /// The word select (WS) signal width, in terms of the bit clock (BCK) periods.
        pub ws_width: u32,

        /// The word select signal polarity; true enables the light lever first.
        pub ws_polarity: bool,

        /// Set to enable the additional bit-shift needed in Philips mode.
        pub bit_shift: bool,

        /// ESP32/ESP32S2 only: place the right slot data in the MSB in the FIFO.
        #[cfg(any(esp32, esp32s2))]
        pub msb_right: bool,

        /// Non-ESP32/ESP32S2: enable left-alignment
        #[cfg(not(any(esp32, esp32s2)))]
        pub left_align: bool,

        /// Non-ESP32/ESP32S2: Enable big-endian.
        #[cfg(not(any(esp32, esp32s2)))]
        pub big_endian: bool,

        /// Non-ESP32/ESP32S2: Enable LSB-first.
        #[cfg(not(any(esp32, esp32s2)))]
        pub bit_order_lsb: bool,
    }

    impl StdSlotConfig {
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
                ws_width: bits_per_sample.into(),
                ws_polarity: false,
                bit_shift: true,
                #[cfg(esp32)]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(esp32s2)]
                msb_right: true,
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
                ws_width: 1,
                ws_polarity: true,
                bit_shift: true,
                #[cfg(esp32)]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(esp32s2)]
                msb_right: true,
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
                ws_width: bits_per_sample.into(),
                ws_polarity: false,
                bit_shift: false,
                #[cfg(esp32)]
                msb_right: bits_per_sample <= DataBitWidth::Bits16,
                #[cfg(esp32s2)]
                msb_right: true,
                #[cfg(not(any(esp32, esp32s2)))]
                left_align: false,
                #[cfg(not(any(esp32, esp32s2)))]
                big_endian: false,
                #[cfg(not(any(esp32, esp32s2)))]
                bit_order_lsb: false,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_std_slot_config_t` representation.
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

    /// I2S slot selection in standard mode.
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
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_std_slot_mask_t {
            match self {
                Self::Left => 1 << 0,
                Self::Right => 1 << 1,
                Self::Both => (1 << 0) | (1 << 1),
            }
        }
    }

    /// The total slot bit width in one slot.
    ///
    /// This is not necessarily the number of data bits in one slot. A slot may have additional bits padded
    /// to fill out the slot.
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum SlotBitWidth {
        /// Slot bit width is automatically set to the data bit width.
        Auto,

        /// Slot bit width is 8 bits.
        Bits8,

        /// Slot bit width is 16 bits.
        Bits16,

        /// Slot bit width is 24 bits.
        Bits24,

        /// Slot bit width is 32 bits.
        Bits32,
    }

    impl Default for SlotBitWidth {
        #[inline(always)]
        fn default() -> Self {
            Self::Auto
        }
    }

    impl SlotBitWidth {
        /// Convert this to the underlying SDK type.
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_slot_bit_width_t {
            match self {
                Self::Auto => 0,
                Self::Bits8 => 8,
                Self::Bits16 => 16,
                Self::Bits24 => 24,
                Self::Bits32 => 32,
            }
        }
    }

    impl TryFrom<u32> for SlotBitWidth {
        type Error = EspError;

        fn try_from(value: u32) -> Result<Self, Self::Error> {
            match value {
                0 => Ok(Self::Auto),
                8 => Ok(Self::Bits8),
                16 => Ok(Self::Bits16),
                24 => Ok(Self::Bits24),
                32 => Ok(Self::Bits32),
                _ => Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap()),
            }
        }
    }

    /// I2S channel slot mode.
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum SlotMode {
        /// Mono mode:
        /// * When transmitting, transmit the same data in all slots.
        /// * When receiving, only receive data from the first slot.
        Mono,

        /// Stereo mode:
        /// * When transmitting, transmit different data in each slot.
        /// * When receiving, receive data from all slots.
        Stereo,
    }

    impl Default for SlotMode {
        #[inline(always)]
        fn default() -> Self {
            Self::Stereo
        }
    }

    impl SlotMode {
        /// Convert this to the underlying SDK type.
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_slot_mode_t {
            match self {
                Self::Mono => 0,
                Self::Stereo => 1,
            }
        }
    }

    pub trait MaybePin {
        fn pin(&self) -> i32;
    }

    impl MaybePin for () {
        fn pin(&self) -> i32 {
            -1
        }
    }

    impl<T: Pin> MaybePin for PeripheralRef<'static, T> {
        fn pin(&self) -> i32 {
            <T as Pin>::pin(self)
        }
    }
}

pub trait I2s: Send {
    fn port() -> i2s_port_t;
}

/// The inner details about an I2S driver. This is allocated separately to prevent moving the atomics around.
struct I2sDriverInternal {
    i2s: u8,
    config: config::Config,
    rx_chan_handle: i2s_chan_handle_t,
    tx_chan_handle: i2s_chan_handle_t,
    rx_chan_available: AtomicBool,
    tx_chan_available: AtomicBool,
}

/// Allow references for the driver to be send across threads.
///
/// This is necessary becuase i2s_chan_handle_t is a pointer, which is not Sync. However, the ESP-IDF API is thread
/// safe here.
///
/// Note that we cannot implement Send. We do *not* want this struct to be sent between threads.
unsafe impl Sync for I2sDriverInternal {}

static mut I2S_DRIVERS: [MaybeUninit<I2sDriverInternal>; 2] =
    [MaybeUninit::uninit(), MaybeUninit::uninit()];

/// The I2S driver for an I2S peripheral.
pub struct I2sDriver<'d> {
    internal: ManuallyDrop<Box<I2sDriverInternal>>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> I2sDriver<'d> {
    pub fn new<I2S: I2s>(
        _i2s: impl Peripheral<P = I2S> + 'd,
        config: config::Config,
    ) -> Result<Self, EspError> {
        let port = I2S::port();

        let internal: *mut I2sDriverInternal = unsafe { I2S_DRIVERS[port as usize].as_mut_ptr() };

        // Initialize the interal side of the driver.

        // Safety: We know that internal is properly aligned and non-null.
        // We are initializing i2s here.
        unsafe {
            (*internal).i2s = port as u8;
        }

        let ll_config: i2s_chan_config_t = config.as_sdk(port);

        // Safety: We initialize tx_chan_handle and rx_chan_handle to null where necessary, and return _pointers_ to
        // them, not the uninitialized values, otherwise.
        let (rx_p, tx_p): (*mut i2s_chan_handle_t, *mut i2s_chan_handle_t) = unsafe {
            match config.channels {
                config::ChannelOpen::Both => (
                    &mut (*internal).rx_chan_handle,
                    &mut (*internal).tx_chan_handle,
                ),
                config::ChannelOpen::Rx => {
                    (*internal).tx_chan_handle = null_mut();
                    (&mut (*internal).rx_chan_handle, null_mut())
                }
                config::ChannelOpen::Tx => {
                    (*internal).rx_chan_handle = null_mut();
                    (null_mut(), &mut (*internal).tx_chan_handle)
                }
            }
        };

        // We're done with config, so move it into the internal driver struct.
        unsafe {
            (*internal).config = config;
        }

        // Safety: &ll_config is a valid pointer to an i2s_chan_config_t. rx_p and tx_p are either valid pointers to
        // the internal.rx_chan_handle and internal.tx_chan_handle, or null.
        unsafe { esp!(i2s_new_channel(&ll_config, tx_p, rx_p))? };

        // At this point, everything except the available atomics is initialized. Initialize the atomics with Release
        // ordering to ensure the above values are visible to other threads.
        unsafe {
            (*internal)
                .rx_chan_available
                .store(!rx_p.is_null(), Ordering::Release);
        }
        unsafe {
            (*internal)
                .tx_chan_available
                .store(!tx_p.is_null(), Ordering::Release);
        }

        // Safety: Box would drop the internal driver (which is statically allocated), but we leak the box by wrapping
        // it in a ManuallyDrop that we never drop.
        let internal = ManuallyDrop::new(unsafe { Box::from_raw(internal) });

        Ok(Self {
            internal,
            _p: PhantomData,
        })
    }

    /// Open the receive channel.
    ///
    /// The returned channel will be in the `READY` state: initialized but not yet started.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_NOT_FOUND` if the channel is already open or the I2S peripheral
    /// was not configured to receive data.
    ///
    /// # TODO
    /// This currenly only supports the standard configuration mode.
    pub fn open_rx_channel<'c, 'cb, C, Mclk, Bclk, Ws, Dout, Din>(
        &mut self,
        config: C,
    ) -> Result<I2sRxChannel<'cb, 'd>, EspError>
    where
        C: Into<config::ChanConfig<'c, Bclk, Din, Dout, Mclk, Ws>>,
        Bclk: Pin + IOPin + 'static,
        Din: config::MaybePin + 'static,
        Dout: config::MaybePin + 'static,
        Mclk: config::MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        // Make sure the channel is available. Acquire to ensure the release in ::new is visible to us.
        if !self
            .internal
            .rx_chan_available
            .swap(false, Ordering::Acquire)
        {
            return Err(EspError::from(ESP_ERR_NOT_FOUND).unwrap());
        }

        let chan_config = config.into();
        match chan_config {
            config::ChanConfig::Std(config) => {
                let ll_config = config.as_sdk();
                match unsafe {
                    esp!(i2s_channel_init_std_mode(
                        self.internal.rx_chan_handle,
                        &ll_config
                    ))
                } {
                    Ok(()) => Ok(I2sRxChannel::new(
                        self.internal.rx_chan_handle,
                        &self.internal.rx_chan_available,
                    )),
                    Err(e) => {
                        // Release to match our acquire above.
                        self.internal
                            .rx_chan_available
                            .store(true, Ordering::Release);
                        Err(e)
                    }
                }
            }
        }
    }

    /// Open the transmit channel.
    ///
    /// The returned channel will be in the `READY` state: initialized but not yet started.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_NOT_FOUND` if the channel is already open or the I2S peripheral
    /// was not configured to transmit data.
    ///
    /// # TODO
    /// This currenly only supports the standard configuration mode.
    pub fn open_tx_channel<'c, 'cb, C, Mclk, Bclk, Ws, Dout, Din>(
        &mut self,
        config: C,
    ) -> Result<I2sTxChannel<'cb, 'd>, EspError>
    where
        C: Into<config::ChanConfig<'c, Bclk, Din, Dout, Mclk, Ws>>,
        Bclk: Pin + IOPin + 'static,
        Din: config::MaybePin + 'static,
        Dout: config::MaybePin + 'static,
        Mclk: config::MaybePin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        // Make sure the channel is available. Acquire to ensure the release in ::new is visible to us.
        if !self
            .internal
            .tx_chan_available
            .swap(false, Ordering::Acquire)
        {
            return Err(EspError::from(ESP_ERR_NOT_FOUND).unwrap());
        }

        let chan_config = config.into();
        match chan_config {
            config::ChanConfig::Std(config) => {
                let ll_config = config.as_sdk();
                match unsafe {
                    esp!(i2s_channel_init_std_mode(
                        self.internal.tx_chan_handle,
                        &ll_config
                    ))
                } {
                    Ok(()) => Ok(I2sTxChannel::new(
                        self.internal.tx_chan_handle,
                        &self.internal.tx_chan_available,
                    )),
                    Err(e) => {
                        // Release to match our acquire above.
                        self.internal
                            .tx_chan_available
                            .store(true, Ordering::Release);
                        Err(e)
                    }
                }
            }
        }
    }
}

impl<'a> Drop for I2sDriver<'a> {
    fn drop(&mut self) {
        // Channels have been dropped, so we can delete them.
        unsafe {
            if !self.internal.rx_chan_handle.is_null() {
                let result = i2s_del_channel(self.internal.rx_chan_handle);
                if result != 0 {
                    // This isn't fatal, so a panic isn't warranted, but we do want to be able to debug it.
                    esp_log_write(
                        esp_log_level_t_ESP_LOG_ERROR,
                        LOG_TAG as *const u8 as *const i8,
                        b"Failed to delete RX channel: %s\0" as *const u8 as *const i8,
                        esp_err_to_name(result),
                    );
                }
            }

            if !self.internal.tx_chan_handle.is_null() {
                let result = i2s_del_channel(self.internal.tx_chan_handle);
                if result != 0 {
                    // This isn't fatal, so a panic isn't warranted, but we do want to be able to debug it.
                    esp_log_write(
                        esp_log_level_t_ESP_LOG_ERROR,
                        LOG_TAG as *const u8 as *const i8,
                        b"Failed to delete TX channel: %s\0" as *const u8 as *const i8,
                        esp_err_to_name(result),
                    );
                }
            }
        }
    }
}

/// Core functions common to receive and transmit channels.
pub trait I2sChannel {
    /// Return the underlying handle for the channel.
    ///
    /// # Safety
    /// The returned handle is only valid for the lifetime of the channel. The handle must not be disposed of outside
    /// of the I2sChannel.
    unsafe fn handle(&self) -> i2s_chan_handle_t;

    /// Enable the I2S channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `READY` state: initialized (as returned by
    /// [I2sDriver::open_rx_channel] or [I2sDriver::open_tx_channel]), but not yet started, or disabled from the
    /// `RUNNING` state via [I2sChannel::disable]. The channel will enter the `RUNNING` state if it is enabled
    /// successfully.
    ///
    /// Enabling the channel will start I2S communications on the hardware. BCLK and WS signals will be generated if
    /// this is a controller. MCLK will be generated once initialization is finished.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `READY` state.
    fn enable(&mut self) -> Result<(), EspError>;

    /// Disable the I2S channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `RUNNING` state: the channel has been previously enabled
    /// via a call to [I2sChannel::enable]. The channel will enter the `READY` state if it is disabled successfully.
    ///
    /// Disabling the channel will stop I2S communications on the hardware. BCLK and WS signals will stop being
    /// generated if this is a controller. MCLK will continue to be generated.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `RUNNING` state.
    fn disable(&mut self) -> Result<(), EspError>;
}

pub struct I2sRxChannel<'cb, 'd> {
    handle: i2s_chan_handle_t,
    available: *const AtomicBool,
    callback: Option<&'cb dyn I2sRxCallback>,
    _p: PhantomData<&'d mut ()>,
}

unsafe impl<'cb, 'd> Sync for I2sRxChannel<'cb, 'd> {}

impl<'cb, 'd> Drop for I2sRxChannel<'cb, 'd> {
    fn drop(&mut self) {
        // Mark the channel as being available to open again.
        unsafe {
            (*self.available).store(true, Ordering::Release);
        }
    }
}

impl<'cb, 'd> I2sChannel for I2sRxChannel<'cb, 'd> {
    #[inline(always)]
    unsafe fn handle(&self) -> i2s_chan_handle_t {
        self.handle
    }

    #[inline(always)]
    fn enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.handle)) }
    }

    #[inline(always)]
    fn disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_disable(self.handle)) }
    }
}

impl<'cb, 'd> I2sRxChannel<'cb, 'd>
where
    'd: 'cb,
{
    /// Create a new I2S receive channel.
    fn new(handle: i2s_chan_handle_t, available: *const AtomicBool) -> Self {
        Self {
            handle,
            available,
            callback: None,
            _p: PhantomData,
        }
    }

    /// Read data from the channel.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes read, or an [EspError] if an error occurred.
    pub fn read(&mut self, buffer: &mut [u8], timeout_ms: Duration) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);

        unsafe {
            esp!(i2s_channel_read(
                self.handle,
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
                &mut bytes_read,
                timeout_ms
            ))?
        }

        Ok(bytes_read)
    }

    /// Read data from the channel into an uninitalized buffer.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes read, or an [EspError] if an error occurred.
    ///
    /// # Safety
    /// Upon a successful return with `Ok(n_read)`, `buffer[..n_read]` will be initialized.
    pub fn read_uninit(
        &mut self,
        buffer: &mut [MaybeUninit<u8>],
        timeout_ms: Duration,
    ) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);

        unsafe {
            esp!(i2s_channel_read(
                self.handle,
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
                &mut bytes_read,
                timeout_ms
            ))?
        }

        Ok(bytes_read)
    }

    /// Set callbacks for the channel.
    ///
    /// # Note
    /// This can be called only when the channel is in a `READY` state, before it is running.
    ///
    /// The callbacks will be called from an interrupt handler. These callbacks should be short and are restricted
    /// from performing any non-ISR safe operations.
    ///
    /// `CONFIG_I2S_ISR_IRAM_SAFE` is not currently supported. This requires that the callback and the functions called
    /// by it must be in IRAM. Rust has no way to copy function bodies (in particular, it does not provide the means
    /// to get the size of the function body), so this is not possible.
    pub fn set_callback_handler(
        &mut self,
        callback: Option<&'cb dyn I2sRxCallback>,
    ) -> Result<(), EspError> {
        unsafe {
            self.callback = callback;
            let callbacks = i2s_event_callbacks_t {
                on_recv: if self.callback.is_none() {
                    None
                } else {
                    Some(dispatch_on_recv)
                },
                on_recv_q_ovf: if self.callback.is_none() {
                    None
                } else {
                    Some(dispatch_on_recv_q_ovf)
                },
                on_sent: None,
                on_send_q_ovf: None,
            };

            esp!(i2s_channel_register_event_callback(
                self.handle,
                &callbacks,
                self as *const Self as *const c_void as *mut c_void
            ))
        }
    }
}

/// C-facing ISR dispatcher for on_recv callbacks.
extern "C" fn dispatch_on_recv(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    let channel: &I2sRxChannel<'static, 'static> = unsafe { &*(user_ctx as *const I2sRxChannel) };
    if let Some(callback) = channel.callback {
        unsafe { callback.on_receive(channel, &*(event as *const i2s_event_data_t)) }
    } else {
        false
    }
}

/// C-facing ISR dispatcher for on_recv_q_ovf callbacks.
extern "C" fn dispatch_on_recv_q_ovf(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    let channel: &I2sRxChannel<'static, 'static> = unsafe { &*(user_ctx as *mut I2sRxChannel) };
    if let Some(callback) = channel.callback {
        unsafe { callback.on_receive_queue_overflow(channel, &*(event as *const i2s_event_data_t)) }
    } else {
        false
    }
}

pub struct I2sTxChannel<'cb, 'd> {
    handle: i2s_chan_handle_t,
    available: *const AtomicBool,
    callback: Option<&'cb dyn I2sTxCallback>,
    _p: PhantomData<&'d mut ()>,
}

impl<'cb, 'd> Drop for I2sTxChannel<'cb, 'd> {
    fn drop(&mut self) {
        // Mark the channel as being available to open again.
        unsafe {
            (*self.available).store(true, Ordering::Release);
        }
    }
}

impl<'cb, 'd> I2sChannel for I2sTxChannel<'cb, 'd> {
    #[inline(always)]
    unsafe fn handle(&self) -> i2s_chan_handle_t {
        self.handle
    }

    #[inline(always)]
    fn enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.handle)) }
    }

    #[inline(always)]
    fn disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_disable(self.handle)) }
    }
}

impl<'cb, 'd> I2sTxChannel<'cb, 'd>
where
    'd: 'cb,
{
    /// Create a new I2S receive channel.
    fn new(handle: i2s_chan_handle_t, available: *const AtomicBool) -> Self {
        Self {
            handle,
            available,
            callback: None,
            _p: PhantomData,
        }
    }

    /// Preload data into the transmit channel DMA buffer.
    ///
    /// This may be called only when the channel is in the `READY` state: initialized but not yet started.
    ///
    /// This is used to preload data into the DMA buffer so that valid data can be transmitted immediately after the
    /// channel is enabled via [I2sChannel::enable]. If this function is not called before enabling the channel,
    /// empty data will be transmitted.
    ///
    /// This function can be called multiple times before enabling the channel. Additional calls will concatenate the
    /// data to the end of the buffer until the buffer is full.
    ///
    /// # Returns
    /// This returns the number of bytes that have been loaded into the buffer. If this is less than the length of
    /// the data provided, the buffer is full and no more data can be loaded.
    #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
    pub fn preload_data(&mut self, data: &[u8]) -> Result<usize, EspError> {
        let mut bytes_loaded: usize = 0;

        unsafe {
            esp!(esp_idf_sys::i2s_channel_preload_data(
                self.handle,
                data.as_ptr(),
                data.len(),
                &mut bytes_loaded as *mut usize
            ));
        }

        Ok(bytes_loaded)
    }

    /// Write data to the channel.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes sent. This may be less than the length of the data provided.
    pub fn write(&mut self, data: &[u8], timeout: Duration) -> Result<usize, EspError> {
        let timeout_ms: u32 = timeout.as_millis().try_into().unwrap_or(u32::MAX);
        let mut bytes_written: usize = 0;

        unsafe {
            esp!(i2s_channel_write(
                self.handle,
                data.as_ptr() as *const c_void,
                data.len(),
                &mut bytes_written,
                timeout_ms
            ))?;
        }

        Ok(bytes_written)
    }

    /// Set callbacks for the channel.
    ///
    /// # Note
    /// This can be called only when the channel is in a `READY` state, before it is running.
    ///
    /// The callbacks will be called from an interrupt handler. These callbacks should be short and are restricted
    /// from performing any non-ISR safe operations.
    ///
    /// `CONFIG_I2S_ISR_IRAM_SAFE` is not currently supported. This requires that the callback and the functions called
    /// by it must be in IRAM. Rust has no way to copy function bodies (in particular, it does not provide the means
    /// to get the size of the function body), so this is not possible.
    pub fn set_callback_handler(
        &mut self,
        callback: Option<&'d dyn I2sTxCallback>,
    ) -> Result<(), EspError> {
        unsafe {
            self.callback = callback;
            let callbacks = i2s_event_callbacks_t {
                on_recv: None,
                on_recv_q_ovf: None,
                on_sent: if self.callback.is_none() {
                    None
                } else {
                    Some(dispatch_on_sent)
                },
                on_send_q_ovf: if self.callback.is_none() {
                    None
                } else {
                    Some(dispatch_on_send_q_ovf)
                },
            };

            esp!(i2s_channel_register_event_callback(
                self.handle,
                &callbacks,
                self as *const Self as *const c_void as *mut c_void
            ))
        }
    }
}

/// C-facing ISR dispatcher for on_sent callbacks.
extern "C" fn dispatch_on_sent(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    let channel: &I2sTxChannel<'static, 'static> = unsafe { &*(user_ctx as *mut I2sTxChannel) };
    if let Some(callback) = channel.callback {
        unsafe { callback.on_sent(channel, &*(event as *const i2s_event_data_t)) }
    } else {
        false
    }
}

/// C-facing ISR dispatcher for on_send_q_ovf callbacks.
extern "C" fn dispatch_on_send_q_ovf(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    let channel: &I2sTxChannel<'static, 'static> = unsafe { &*(user_ctx as *const I2sTxChannel) };
    if let Some(callback) = channel.callback {
        unsafe { callback.on_send_queue_overflow(channel, &*(event as *const i2s_event_data_t)) }
    } else {
        false
    }
}

/// Callback handler for a receiver channel.
///
/// This runs in an interrupt context.
pub trait I2sRxCallback {
    /// Called when an I2S receive event occurs.
    ///
    /// # Parameters
    /// * `rx_channel`: The channel on which the event occurred.
    /// * `event`: Data associated with the event, including the DMA buffer address and the size that just finished
    ///   receiving data.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken upby this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_receive(&self, rx_channel: &I2sRxChannel<'_, '_>, event: &I2sEvent) -> bool {
        false
    }

    /// Called when an I2S receiving queue overflows.
    ///
    /// # Parameters
    /// * `rx_channel`: The channel on which the event occurred.
    /// * `event`: Data associated with the event, including the buffer size that has been overwritten.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken upby this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_receive_queue_overflow(
        &self,
        rx_channel: &I2sRxChannel<'_, '_>,
        event: &I2sEvent,
    ) -> bool {
        false
    }
}

/// Callback handler for a transmitter channel.
///
/// This runs in an interrupt context.
pub trait I2sTxCallback {
    /// Called when an I2S DMA buffer is finished sending.
    ///
    /// # Parameters
    /// * `tx_channel`: The channel on which the event occurred.
    /// * `event`: Data associated with the event, including the DMA buffer address and the size that just finished
    ///   sending data.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken upby this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_sent(&self, tx_channel: &I2sTxChannel<'_, '_>, event: &I2sEvent) -> bool {
        false
    }

    /// Called when an I2S sending queue overflows.
    ///
    /// # Parameters
    /// * `tx_channel`: The channel on which the event occurred.
    /// * `event`: Data associated with the event, including the buffer size that has been overwritten.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken upby this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_send_queue_overflow(&self, tx_channel: &I2sTxChannel<'_, '_>, event: &I2sEvent) -> bool {
        false
    }
}

pub type I2sEvent = i2s_event_data_t;

macro_rules! impl_i2s {
    ($i2s:ident: $port:expr) => {
        crate::impl_peripheral!($i2s);

        impl I2s for $i2s {
            #[inline(always)]
            fn port() -> i2s_port_t {
                $port
            }
        }
    };
}

impl_i2s!(I2S0: 0);
#[cfg(any(esp32, esp32s3))]
impl_i2s!(I2S1: 1);
