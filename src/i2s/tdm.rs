//! Time-division multiplexing (TDM) support for I2S.
use super::{
    I2s, I2sChannel, I2sRxCallback, I2sRxChannel, I2sRxSupported, I2sTxCallback, I2sTxChannel,
    I2sTxSupported,
};
use crate::{
    gpio::{AnyIOPin, InputPin, OutputPin},
    peripheral::{Peripheral, PeripheralRef},
};
use core::{marker::PhantomData, ptr::null_mut};
use esp_idf_sys::{
    esp, i2s_chan_config_t, i2s_chan_handle_t, i2s_channel_init_tdm_mode, i2s_new_channel,
    i2s_port_t, EspError,
};

pub(super) mod config {
    use crate::{
        gpio::{InputPin, OutputPin},
        i2s::config::{ClockSource, Config, DataBitWidth, MclkMultiple, SlotBitWidth, SlotMode},
        peripheral::PeripheralRef,
    };
    use esp_idf_sys::{
        i2s_tdm_clk_config_t, i2s_tdm_config_t, i2s_tdm_gpio_config_t,
        i2s_tdm_gpio_config_t__bindgen_ty_1, i2s_tdm_slot_config_t, i2s_tdm_slot_mask_t,
    };

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
            gpio_cfg: TdmGpioConfig,
        ) -> Self {
            Self {
                channel_cfg,
                clk_cfg,
                slot_cfg,
                gpio_cfg,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_config_t` representation.
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
    }

    impl TdmClkConfig {
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

        /// Convert to the ESP-IDF SDK `i2s_tdm_clk_config_t` representation.
        #[inline(always)]
        pub(crate) fn as_sdk(&self) -> i2s_tdm_clk_config_t {
            i2s_tdm_clk_config_t {
                sample_rate_hz: self.sample_rate_hz,
                clk_src: self.clk_src.as_sdk(),
                mclk_multiple: self.mclk_multiple.as_sdk(),
            }
        }
    }

    /// TDM mode GPIO (general purpose input/output) configuration.
    #[derive(Default)]
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
    #[derive(Clone)]
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
        ws_width: u32,

        /// The word select signal polarity; true enables the light lever first.
        ws_polarity: bool,

        /// Set to enable the additional bit-shift needed in Philips mode.
        bit_shift: bool,

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
        #[inline(always)]
        pub fn ws_width(mut self, ws_width: u32) -> Self {
            self.ws_width = ws_width;
            self
        }

        /// Update the word select signal polarity on this TDM slot configuration.
        #[inline(always)]
        pub fn ws_polarity(mut self, ws_polarity: bool) -> Self {
            self.ws_polarity = ws_polarity;
            self
        }

        /// Update the bit shift flag on this TDM slot configuration.
        #[inline(always)]
        pub fn bit_shift(mut self, bit_shift: bool) -> Self {
            self.bit_shift = bit_shift;
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
                ws_width: TDM_AUTO_WS_WIDTH,
                ws_polarity: false,
                bit_shift: true,
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
                ws_width: TDM_AUTO_WS_WIDTH,
                ws_polarity: false,
                bit_shift: false,
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
                ws_width: 1,
                ws_polarity: true,
                bit_shift: false,
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
                ws_width: bits_per_sample.into(),
                ws_polarity: true,
                bit_shift: false,
                left_align: false,
                big_endian: false,
                bit_order_lsb: false,
                skip_mask: false,
                total_slots: TDM_AUTO_SLOT_NUM,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_tdm_slot_config_t` representation.
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

    /// Mask of TDM slots to enable.
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub struct TdmSlotMask(u16);

    impl Default for TdmSlotMask {
        fn default() -> Self {
            Self(0)
        }
    }

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
        pub(super) fn as_sdk(&self) -> i2s_tdm_slot_mask_t {
            self.0 as i2s_tdm_slot_mask_t
        }
    }
}

/// The I2S TDM mode driver.
pub struct I2sTdmDriver<'d, Dir> {
    /// The Rx channel, possibly null.
    rx: *mut I2sChannel<dyn I2sRxCallback>,

    /// The Tx channel, possibly null.
    tx: *mut I2sChannel<dyn I2sTxCallback>,

    /// The I2S peripheral number. Either 0 or 1 (ESP32 and ESP32S3 only).
    i2s: u8,

    /// Driver lifetime -- mimics the lifetime of the peripheral.
    _p: PhantomData<&'d ()>,

    /// Directionality -- mimics the directionality of the peripheral.
    _dir: PhantomData<Dir>,
}

unsafe impl<'d, Dir> Send for I2sTdmDriver<'d, Dir> {}
unsafe impl<'d, Dir> Sync for I2sTdmDriver<'d, Dir> {}

impl<'d, Dir> I2sTdmDriver<'d, Dir> {
    /// Returns the I2S port number of this driver.
    pub fn port(&self) -> i2s_port_t {
        self.i2s as u32
    }
}

impl<'d, Dir: I2sRxSupported + I2sTxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new TDM mode driver for the given I2S peripheral with both the receive and transmit channels open.
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
        let chan_cfg: i2s_chan_config_t = config.channel_cfg.as_sdk(port);

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
        let rx = Box::new(I2sChannel {
            chan_handle: rx_chan_handle,
            callback: None,
            i2s: port as u8,
        });

        let tx = Box::new(I2sChannel {
            chan_handle: tx_chan_handle,
            callback: None,
            i2s: port as u8,
        });

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

        // Now we leak the rx and tx channels so they are no longer managed. This pins them in memory in a way that
        // is easily accessible to the ESP-IDF SDK.
        Ok(Self {
            i2s: port as u8,
            rx: Box::leak(rx),
            tx: Box::leak(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sRxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new standard mode driver for the given I2S peripheral with only the receive channel open.
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
        let chan_cfg: i2s_chan_config_t = config.channel_cfg.as_sdk(port);

        let mut rx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe { esp!(i2s_new_channel(&chan_cfg, null_mut(), &mut rx_chan_handle,))? };

        if rx_chan_handle.is_null() {
            panic!("Expected non-null rx channel handle");
        }

        // Allocate the internal channel struct.
        let rx = Box::new(I2sChannel {
            chan_handle: rx_chan_handle,
            callback: None,
            i2s: port as u8,
        });

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
            rx: Box::leak(rx),
            tx: null_mut(),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sTxSupported> I2sTdmDriver<'d, Dir> {
    /// Create a new standard mode driver for the given I2S peripheral with only the transmit channel open.
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
        let chan_cfg: i2s_chan_config_t = config.channel_cfg.as_sdk(port);

        let mut tx_chan_handle: i2s_chan_handle_t = null_mut();

        // Safety: &chan_cfg is a valid pointer to an i2s_chan_config_t.
        // rx and tx are out pointers.
        unsafe { esp!(i2s_new_channel(&chan_cfg, &mut tx_chan_handle, null_mut()))? };

        if tx_chan_handle.is_null() {
            panic!("Expected non-null tx channel handle");
        }

        // Allocate the internal channel struct.
        let tx = Box::new(I2sChannel {
            chan_handle: tx_chan_handle,
            callback: None,
            i2s: port as u8,
        });

        // At this point, returning early will drop the tx channel, closing it properly.

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
            rx: null_mut(),
            tx: Box::leak(tx),
            _p: PhantomData,
            _dir: PhantomData,
        })
    }
}

impl<'d, Dir: I2sRxSupported> I2sRxChannel<'d> for I2sTdmDriver<'d, Dir> {
    unsafe fn rx_handle(&self) -> i2s_chan_handle_t {
        (*self.rx).chan_handle
    }

    fn set_rx_callback<Rx: I2sRxCallback + 'static>(
        &mut self,
        rx_callback: Rx,
    ) -> Result<(), EspError> {
        unsafe { &mut *self.rx }.set_rx_callback(Box::new(rx_callback))
    }
}

impl<'d, Dir: I2sTxSupported> I2sTxChannel<'d> for I2sTdmDriver<'d, Dir> {
    unsafe fn tx_handle(&self) -> i2s_chan_handle_t {
        (*self.tx).chan_handle
    }

    fn set_tx_callback<Tx: I2sTxCallback + 'static>(
        &mut self,
        tx_callback: Tx,
    ) -> Result<(), EspError> {
        unsafe { &mut *self.tx }.set_tx_callback(Box::new(tx_callback))
    }
}
