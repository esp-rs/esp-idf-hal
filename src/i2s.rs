use crate::{
    gpio::{IOPin, InputPin, OutputPin, Pin},
    peripheral::{Peripheral},
};
use core::{
    marker::PhantomData,
    mem::{ManuallyDrop, MaybeUninit},
    ptr::null_mut,
    sync::atomic::{AtomicBool, Ordering},
};
use esp_idf_sys::{
    esp, i2s_chan_config_t, i2s_chan_handle_t, i2s_channel_disable, i2s_channel_enable, i2s_channel_init_std_mode,
    i2s_new_channel, i2s_port_t, i2s_role_t, EspError, ESP_ERR_NOT_FOUND,
};

/// I2S peripheral in controller (master) role, bclk and ws signal will be set to output.
const I2S_ROLE_CONTROLLER: i2s_role_t = 0;

/// I2S peripheral in target (slave) role, bclk and ws signal will be set to input.
const I2S_ROLE_TARGET: i2s_role_t = 1;

/// I2S configuration
pub mod config {
    use super::{I2S_ROLE_CONTROLLER, I2S_ROLE_TARGET};
    use crate::{
        gpio::{IOPin, InputPin, OutputPin, Pin},
        peripheral::{Peripheral, PeripheralRef},
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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
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

    /// I2S channel configuration
    #[derive(Clone, Default)]
    pub struct Config {
        /// The role of this channel: controller (master) or target (slave)
        role: Role,

        /// The DMA buffer to use.
        dma_desc: u32,

        /// The I2S frame number in one DMA buffer.
        dma_frame: u32,

        /// If true, the transmit buffer will be automatically cleared upon sending.
        auto_clear: bool,

        /// The channels to open.
        pub(super) channels: ChannelOpen,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        /// Set the role of this channel: controller (master) or target (slave)
        #[must_use]
        pub fn role(mut self, role: Role) -> Self {
            self.role = role;
            self
        }

        /// Set the DMA buffer to use.
        #[must_use]
        pub fn dma_desc(mut self, dma_desc: u32) -> Self {
            self.dma_desc = dma_desc;
            self
        }

        /// Set the I2S frame number in one DMA buffer.
        #[must_use]
        pub fn dma_frame(mut self, dma_frame: u32) -> Self {
            self.dma_frame = dma_frame;
            self
        }

        /// Set if the transmit buffer will be automatically cleared upon sending.
        #[must_use]
        pub fn auto_clear(mut self, auto_clear: bool) -> Self {
            self.auto_clear = auto_clear;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_chan_config_t` representation.
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
    #[derive(Clone, Copy, Eq, PartialEq)]
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

    impl DataBitWidth {
        /// Convert to the ESP-IDF SDK `i2s_data_bit_width_t` representation.
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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// Convert to the ESP-IDF SDK `i2s_std_config_t` representation.
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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
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
        pub fn from_sample_rate_hz(rate: u32) -> Self {
            Self {
                sample_rate_hz: rate,
                clk_src: ClockSource::default(),
                mclk_multiple: MclkMultiple::M256,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_std_clk_config_t` representation.
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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        /// MCLK (master/controller clock) pin. Optional output.
        mclk: Option<PeripheralRef<'static, Mclk>>,

        /// BCLK (bit clock) pin. Input in target/slave mode, output in controller/master mode.
        bclk: PeripheralRef<'static, Bclk>,

        /// WS (word select) pin. Input in target/slave mode, output in controller/master mode.
        ws: PeripheralRef<'static, Ws>,

        /// Data output pin.
        data_out: Option<PeripheralRef<'static, Dout>>,

        /// Data input pin.
        data_in: Option<PeripheralRef<'static, Din>>,

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
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        #[allow(clippy::too_many_arguments)]
        pub fn new<BclkP, DinP, DoutP, MclkP, WsP>(
            bclk: BclkP,
            data_in: Option<DinP>,
            data_out: Option<DoutP>,
            mclk: Option<MclkP>,
            ws: WsP,
            bclk_invert: bool,
            mclk_invert: bool,
            ws_invert: bool,
        ) -> Self
        where
            BclkP: Peripheral<P = Bclk> + 'static,
            DinP: Peripheral<P = Din> + 'static,
            DoutP: Peripheral<P = Dout> + 'static,
            MclkP: Peripheral<P = Mclk> + 'static,
            WsP: Peripheral<P = Ws> + 'static,
        {
            Self {
                mclk: mclk.map(|mclk| mclk.into_ref()),
                bclk: bclk.into_ref(),
                ws: ws.into_ref(),
                data_out: data_out.map(|data_out| data_out.into_ref()),
                data_in: data_in.map(|data_in| data_in.into_ref()),
                mclk_invert,
                bclk_invert,
                ws_invert,
            }
        }

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
                mclk: if let Some(mclk) = &self.mclk {
                    mclk.pin()
                } else {
                    -1
                },
                bclk: self.bclk.pin(),
                ws: self.ws.pin(),
                dout: if let Some(data_out) = &self.data_out {
                    data_out.pin()
                } else {
                    -1
                },
                din: if let Some(data_in) = &self.data_in {
                    data_in.pin()
                } else {
                    -1
                },
                invert_flags,
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
        pub lsb_first: bool,
    }

    impl StdSlotConfig {
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
                lsb_first: self.lsb_first,
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
        pub(crate) fn as_sdk(&self) -> i2s_slot_mode_t {
            match self {
                Self::Mono => 0,
                Self::Stereo => 1,
            }
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

static mut I2S_DRIVERS: [MaybeUninit<I2sDriverInternal>; 2] = [MaybeUninit::uninit(), MaybeUninit::uninit()];

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
        unsafe { (*internal).i2s = port as u8; }

        let ll_config: i2s_chan_config_t = config.as_sdk(port);

        // Safety: We initialize tx_chan_handle and rx_chan_handle to null where necessary, and return _pointers_ to
        // them, not the uninitialized values, otherwise.
        let (rx_p, tx_p): (*mut i2s_chan_handle_t, *mut i2s_chan_handle_t) = unsafe {
            match config.channels {
                config::ChannelOpen::Both => (&mut (*internal).rx_chan_handle, &mut (*internal).tx_chan_handle),
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
        unsafe { (*internal).config = config; }

        // Safety: &ll_config is a valid pointer to an i2s_chan_config_t. rx_p and tx_p are either valid pointers to
        // the internal.rx_chan_handle and internal.tx_chan_handle, or null.
        unsafe { esp!(i2s_new_channel(&ll_config, rx_p, tx_p))? };

        // At this point, everything except the available atomics is initialized. Initialize the atomics with Release
        // ordering to ensure the above values are visible to other threads.
        unsafe { (*internal).rx_chan_available.store(! rx_p.is_null(), Ordering::Release); }
        unsafe { (*internal).tx_chan_available.store(! tx_p.is_null(), Ordering::Release); }

        // Safety: Box would drop the internal driver (which is statically allocated), but we leak the box by wrapping
        // it in a ManuallyDrop that we never drop.
        let internal = ManuallyDrop::new(unsafe { Box::from_raw(internal) });

        Ok(Self {
            internal,
            _p: PhantomData,
        })
    }

    pub fn open_rx_channel<'c, C, Mclk, Bclk, Ws, Dout, Din>(
        &mut self,
        config: C,
    ) -> Result<I2sRxChannel<'d>, EspError>
    where
        C: Into<config::ChanConfig<'c, Bclk, Din, Dout, Mclk, Ws>>,
        Bclk: Pin + IOPin + 'static,
        Din: Pin + InputPin + 'static,
        Dout: Pin + OutputPin + 'static,
        Mclk: Pin + OutputPin + 'static,
        Ws: Pin + IOPin + 'static,
    {
        // Make sure the channel is available. Acquire to ensure the release in ::new is visible to us.
        if !self.internal.rx_chan_available.swap(false, Ordering::Acquire) {
            return Err(EspError::from(ESP_ERR_NOT_FOUND).unwrap());
        }

        let chan_config = config.into();
        match chan_config {
            config::ChanConfig::Std(config) => {
                let ll_config = config.as_sdk();
                match unsafe { esp!(i2s_channel_init_std_mode(self.internal.rx_chan_handle, &ll_config)) } {
                    Ok(()) => Ok(I2sRxChannel {
                        handle: self.internal.rx_chan_handle,
                        available: &self.internal.rx_chan_available,
                        _p: PhantomData,
                    }),
                    Err(e) => {
                        // Release to match our acquire above.
                        self.internal.rx_chan_available.store(true, Ordering::Release);
                        Err(e)
                    }
                }
            }
        }
    }

    pub fn get_tx_channel(&mut self) -> Result<I2sTxChannel<'d>, EspError> {
        todo!()
    }
}

pub trait I2sChannel {
    fn enable(&mut self) -> Result<(), EspError>;
    fn disable(&mut self) -> Result<(), EspError>;
}

pub struct I2sRxChannel<'d> {
    handle: i2s_chan_handle_t,
    available: *const AtomicBool,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Drop for I2sRxChannel<'d> {
    fn drop(&mut self) {
        // Mark the channel as being available to open again.
        unsafe { (*self.available).store(true, Ordering::Release); }
    }
}

impl<'d> I2sChannel for I2sRxChannel<'d> {
    fn enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.handle)) }
    }

    fn disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_disable(self.handle)) }
    }
}

pub struct I2sTxChannel<'d> {
    driver: *mut I2sDriver<'d>,
    _p: PhantomData<&'d mut ()>,
}

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
