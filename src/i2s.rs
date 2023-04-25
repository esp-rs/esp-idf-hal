//! Driver for the Inter-IC Sound (I2S) peripheral(s).

use core::{convert::TryInto, ffi::c_void, mem::MaybeUninit, time::Duration};
use esp_idf_sys::{esp, i2s_port_t, EspError};

#[cfg(not(esp_idf_version_major = "4"))]
use {
    core::ptr::null_mut,
    esp_idf_sys::{
        esp_err_to_name, esp_log_level_t_ESP_LOG_ERROR, esp_log_write, i2s_chan_handle_t,
        i2s_channel_disable, i2s_channel_enable, i2s_channel_read,
        i2s_channel_register_event_callback, i2s_channel_write, i2s_del_channel,
        i2s_event_callbacks_t, i2s_event_data_t, ESP_OK,
    },
};

#[cfg(esp_idf_version_major = "4")]
use esp_idf_sys::{configTICK_RATE_HZ, i2s_read, i2s_start, i2s_stop, i2s_write};

mod pcm;

// For v5+, we rely configuration options for PDM/TDM support.
// For v4, we have to examine the chip type.
#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        any(esp_idf_soc_i2s_supports_pdm_rx, esp_idf_soc_i2s_supports_pdm_tx)
    ),
    all(esp_idf_version_major = "4", any(esp32, esp32s3, esp32c3, esp32c6))
))]
mod pdm;

mod std;

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_soc_i2s_supports_tdm),
    all(esp_idf_version_major = "4", any(esp32s3, esp32c3, esp32c6))
))]
mod tdm;

/// Logging tag.
#[allow(unused)]
const LOG_TAG: &[u8; 17] = b"esp-idf-hal::i2s\0";

/// I2S configuration
pub mod config {
    pub use super::pcm::config::*;
    #[cfg(any(
        all(
            not(esp_idf_version_major = "4"),
            any(esp_idf_soc_i2s_supports_pdm_rx, esp_idf_soc_i2s_supports_pdm_tx)
        ),
        all(esp_idf_version_major = "4", any(esp32, esp32s3, esp32c3, esp32c6))
    ))]
    pub use super::pdm::config::*;
    pub use super::std::config::*;

    #[cfg(any(
        all(not(esp_idf_version_major = "4"), esp_idf_soc_i2s_supports_tdm),
        all(esp_idf_version_major = "4", any(esp32s3, esp32c3, esp32c6))
    ))]
    pub use super::tdm::config::*;
    use core::convert::TryFrom;
    use esp_idf_sys::{
        i2s_mclk_multiple_t, i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_128,
        i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_256, i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_384,
        EspError, ESP_ERR_INVALID_ARG,
    };

    #[cfg(not(esp_idf_version_major = "4"))]
    use esp_idf_sys::{
        i2s_chan_config_t, i2s_clock_src_t, i2s_data_bit_width_t,
        i2s_mclk_multiple_t_I2S_MCLK_MULTIPLE_512, i2s_port_t, i2s_role_t, i2s_slot_bit_width_t,
        i2s_slot_mode_t,
    };

    #[cfg(esp_idf_version_major = "4")]
    use esp_idf_sys::{
        i2s_bits_per_chan_t, i2s_bits_per_sample_t, i2s_mode_t, i2s_mode_t_I2S_MODE_MASTER,
        i2s_mode_t_I2S_MODE_SLAVE,
    };

    /// I2S clock source
    #[derive(Clone, Copy, Eq, PartialEq)]
    pub enum ClockSource {
        /// Use PLL_F160M as the source clock
        Pll160M,

        /// Use APLL as the source clock
        #[cfg(any(esp32, esp32s2))]
        Apll,
    }

    impl Default for ClockSource {
        #[inline(always)]
        fn default() -> Self {
            Self::Pll160M
        }
    }

    impl ClockSource {
        #[cfg(not(esp_idf_version_major = "4"))]
        pub(super) fn as_sdk(&self) -> i2s_clock_src_t {
            match self {
                Self::Pll160M => esp_idf_sys::soc_module_clk_t_SOC_MOD_CLK_PLL_F160M,
                #[cfg(any(esp32, esp32s2))]
                Self::Apll => esp_idf_sys::soc_module_clk_t_SOC_MOD_CLK_APLL,
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
    #[derive(Clone)]
    pub struct Config {
        /// The role of this channel: controller (master) or target (slave)
        pub(super) role: Role,

        /// The DMA buffer number to use (also the DMA descriptor number).
        pub(super) dma_desc: u32,

        /// The number of I2S frames in one DMA buffer.
        pub(super) frames: u32,

        /// If true, the transmit buffer will be automatically cleared upon sending.
        pub(super) auto_clear: bool,
    }

    impl Default for Config {
        #[inline(always)]
        fn default() -> Self {
            Self {
                role: Role::Controller,
                dma_desc: 6,
                frames: 240,
                auto_clear: false,
            }
        }
    }

    impl Config {
        #[inline(always)]
        /// Create a new Config with the specified role, DMA descriptor, frame count, and auto clear setting.
        pub fn new(role: Role, dma_desc: u32, frames: u32, auto_clear: bool) -> Self {
            Self {
                role,
                dma_desc,
                frames,
                auto_clear,
            }
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

        /// Set the number of I2S frames in one DMA buffer.
        #[must_use]
        #[inline(always)]
        pub fn frames(mut self, frames: u32) -> Self {
            self.frames = frames;
            self
        }

        /// Set if the transmit buffer will be automatically cleared upon sending.
        #[must_use]
        #[inline(always)]
        pub fn auto_clear(mut self, auto_clear: bool) -> Self {
            self.auto_clear = auto_clear;
            self
        }

        /// Convert to the ESP-IDF SDK `i2s_chan_config_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self, id: i2s_port_t) -> i2s_chan_config_t {
            i2s_chan_config_t {
                id,
                role: self.role.as_sdk(),
                dma_desc_num: self.dma_desc,
                dma_frame_num: self.frames,
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
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_data_bit_width_t {
            match self {
                Self::Bits8 => 8,
                Self::Bits16 => 16,
                Self::Bits24 => 24,
                Self::Bits32 => 32,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_bits_per_sample_t` representation.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_bits_per_sample_t {
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
        #[cfg(not(esp_idf_version_major = "4"))]
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
                #[cfg(not(esp_idf_version_major = "4"))]
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
                #[cfg(not(esp_idf_version_major = "4"))]
                MclkMultiple::M512 => 512,
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

    /// I2S peripheral in controller (master) role, bclk and ws signal will be set to output.
    #[cfg(not(esp_idf_version_major = "4"))]
    const I2S_ROLE_CONTROLLER: i2s_role_t = 0;

    /// I2S peripheral in target (slave) role, bclk and ws signal will be set to input.
    #[cfg(not(esp_idf_version_major = "4"))]
    const I2S_ROLE_TARGET: i2s_role_t = 1;

    impl Role {
        /// Convert to the ESP-IDF SDK `i2s_role_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_role_t {
            match self {
                Self::Controller => I2S_ROLE_CONTROLLER,
                Self::Target => I2S_ROLE_TARGET,
            }
        }

        /// Convert to the ESP-IDF SDK `i2s_mode_t` representation.
        #[cfg(esp_idf_version_major = "4")]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_mode_t {
            match self {
                Self::Controller => i2s_mode_t_I2S_MODE_MASTER,
                Self::Target => i2s_mode_t_I2S_MODE_SLAVE,
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

    #[cfg(not(esp_idf_version_major = "4"))]
    type SlotBitWidthSdkType = i2s_slot_bit_width_t;

    #[cfg(esp_idf_version_major = "4")]
    type SlotBitWidthSdkType = i2s_bits_per_chan_t;

    impl SlotBitWidth {
        /// Convert this to the ESP-IDF SDK `i2s_slot_bit_width_t`/`i2s_bits_per_chan_t` representation.
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> SlotBitWidthSdkType {
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
        /// Convert this to the ESP-IDF SDK `i2s_slot_mode_t` representation.
        #[cfg(not(esp_idf_version_major = "4"))]
        #[inline(always)]
        pub(super) fn as_sdk(&self) -> i2s_slot_mode_t {
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

pub trait I2sPort {
    /// Returns the I2S port number of this driver.
    fn port(&self) -> i2s_port_t;
}

/// Functions for receive channels.
pub trait I2sRxChannel<'d>: I2sPort {
    /// Return the underlying handle for the channel.
    ///
    /// # Safety
    /// The returned handle is only valid for the lifetime of the channel. The handle must not be disposed of outside
    /// of the driver.
    #[cfg(not(esp_idf_version_major = "4"))]
    unsafe fn rx_handle(&self) -> i2s_chan_handle_t;

    /// Enable the I2S receive channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `READY` state: initialized but not yet started from a driver
    /// constructor, or disabled from the `RUNNING` state via [I2sRxChannel::rx_disable]. The channel will enter the
    /// `RUNNING` state if it is enabled successfully.
    ///
    /// Enabling the channel will start I2S communications on the hardware. BCLK and WS signals will be generated if
    /// this is a controller. MCLK will be generated once initialization is finished.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `READY` state.
    #[cfg(esp_idf_version_major = "4")]
    fn rx_enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_start(self.port())) }
    }

    /// Enable the I2S receive channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `READY` state: initialized but not yet started from a driver
    /// constructor, or disabled from the `RUNNING` state via [I2sRxChannel::rx_disable]. The channel will enter the
    /// `RUNNING` state if it is enabled successfully.
    ///
    /// Enabling the channel will start I2S communications on the hardware. BCLK and WS signals will be generated if
    /// this is a controller. MCLK will be generated once initialization is finished.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `READY` state.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn rx_enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.rx_handle())) }
    }

    /// Disable the I2S receive channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `RUNNING` state: the channel has been previously enabled
    /// via a call to [I2sRxChannel::rx_enable]. The channel will enter the `READY` state if it is disabled
    /// successfully.
    ///
    /// Disabling the channel will stop I2S communications on the hardware. BCLK and WS signals will stop being
    /// generated if this is a controller. MCLK will continue to be generated.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `RUNNING` state.
    #[cfg(esp_idf_version_major = "4")]
    fn rx_disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_stop(self.port())) }
    }

    /// Disable the I2S receive channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `RUNNING` state: the channel has been previously enabled
    /// via a call to [I2sRxChannel::rx_enable]. The channel will enter the `READY` state if it is disabled
    /// successfully.
    ///
    /// Disabling the channel will stop I2S communications on the hardware. BCLK and WS signals will stop being
    /// generated if this is a controller. MCLK will continue to be generated.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `RUNNING` state.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn rx_disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_disable(self.rx_handle())) }
    }

    /// Read data from the channel.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes read, or an [EspError] if an error occurred.
    #[cfg(esp_idf_version_major = "4")]
    fn read(&mut self, buffer: &mut [u8], timeout_ms: Duration) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);
        let ticks_to_wait = timeout_ms * configTICK_RATE_HZ / 1000;
        unsafe {
            esp!(i2s_read(
                self.port(),
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
                &mut bytes_read,
                ticks_to_wait
            ))?
        }

        Ok(bytes_read)
    }

    /// Read data from the channel.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes read, or an [EspError] if an error occurred.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn read(&mut self, buffer: &mut [u8], timeout_ms: Duration) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);

        unsafe {
            esp!(i2s_channel_read(
                self.rx_handle(),
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
    #[cfg(esp_idf_version_major = "4")]
    fn read_uninit(
        &mut self,
        buffer: &mut [MaybeUninit<u8>],
        timeout_ms: Duration,
    ) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);
        let ticks_to_wait = timeout_ms * configTICK_RATE_HZ / 1000;

        unsafe {
            esp!(i2s_read(
                self.port(),
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
                &mut bytes_read,
                ticks_to_wait
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
    #[cfg(not(esp_idf_version_major = "4"))]
    fn read_uninit(
        &mut self,
        buffer: &mut [MaybeUninit<u8>],
        timeout_ms: Duration,
    ) -> Result<usize, EspError> {
        let mut bytes_read: usize = 0;
        let timeout_ms: u32 = timeout_ms.as_millis().try_into().unwrap_or(u32::MAX);

        unsafe {
            esp!(i2s_channel_read(
                self.rx_handle(),
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
                &mut bytes_read,
                timeout_ms
            ))?
        }

        Ok(bytes_read)
    }

    /// Sets the receive callback for the channel.
    ///
    /// This may be called only when the channel is in the `REGISTERED` or `RUNNING` state.
    ///
    /// # Bugs
    /// This functionality appears to be fundamentally broken in ESP-IDF 5.0.*. This function is invoked by
    /// [`i2s_dma_rx_callback`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L562).
    /// It expects the callback (or another mechanism) to mark the DMA buffer as ready for reuse, as
    /// [`i2s_channel_read` does by calling `xQueueReceive`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L1076).
    /// Otherwise, the queue eventually fills up and [`xQueueIsQueueFullFromISR` returns true and the recieve buffer overflows](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L494).
    ///
    /// However, it is impossible for non ESP-IDF code to call `xQueueReceive` on the queue because the queue is hidden behind the
    /// opaque [`i2s_chan_handle_t` object](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/include/driver/i2s_types.h#L66).
    /// The actual definition is private in [`i2s_channel_obj_t.msg_queue`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_private.h#L71-L103).
    ///
    /// This requires a fix in the ESP-IDF SDK to work properly.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn set_rx_callback<Rx: I2sRxCallback + 'static>(
        &mut self,
        callback: Rx,
    ) -> Result<(), EspError>;
}

/// Functions for transmit channels.
pub trait I2sTxChannel<'d>: I2sPort {
    /// Return the underlying handle for the transmit channel.
    ///
    /// # Safety
    /// The returned handle is only valid for the lifetime of the channel. The handle must not be disposed of outside
    /// of the driver.
    #[cfg(not(esp_idf_version_major = "4"))]
    unsafe fn tx_handle(&self) -> i2s_chan_handle_t;

    /// Enable the I2S transmit channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `READY` state: initialized but not yet started from a driver
    /// constructor, or disabled from the `RUNNING` state via [I2sTxChannel::tx_disable]. The channel will enter the
    /// `RUNNING` state if it is enabled successfully.
    ///
    /// Enabling the channel will start I2S communications on the hardware. BCLK and WS signals will be generated if
    /// this is a controller. MCLK will be generated once initialization is finished.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `READY` state.
    #[cfg(esp_idf_version_major = "4")]
    fn tx_enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_start(self.port())) }
    }

    /// Enable the I2S transmit channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `READY` state: initialized but not yet started from a driver
    /// constructor, or disabled from the `RUNNING` state via [I2sTxChannel::tx_disable]. The channel will enter the
    /// `RUNNING` state if it is enabled successfully.
    ///
    /// Enabling the channel will start I2S communications on the hardware. BCLK and WS signals will be generated if
    /// this is a controller. MCLK will be generated once initialization is finished.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `READY` state.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn tx_enable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_enable(self.tx_handle())) }
    }

    /// Disable the I2S transmit channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `RUNNING` state: the channel has been previously enabled
    /// via a call to [I2sTxChannel::tx_enable]. The channel will enter the `READY` state if it is disabled
    /// successfully.
    ///
    /// Disabling the channel will stop I2S communications on the hardware. BCLK and WS signals will stop being
    /// generated if this is a controller. MCLK will continue to be generated.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `RUNNING` state.
    #[cfg(esp_idf_version_major = "4")]
    fn tx_disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_stop(self.port())) }
    }

    /// Disable the I2S transmit channel.
    ///
    /// # Note
    /// This can only be called when the channel is in the `RUNNING` state: the channel has been previously enabled
    /// via a call to [I2sTxChannel::tx_enable]. The channel will enter the `READY` state if it is disabled
    /// successfully.
    ///
    /// Disabling the channel will stop I2S communications on the hardware. BCLK and WS signals will stop being
    /// generated if this is a controller. MCLK will continue to be generated.
    ///
    /// # Errors
    /// This will return an [EspError] with `ESP_ERR_INVALID_STATE` if the channel is not in the `RUNNING` state.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn tx_disable(&mut self) -> Result<(), EspError> {
        unsafe { esp!(i2s_channel_disable(self.tx_handle())) }
    }

    /// Preload data into the transmit channel DMA buffer.
    ///
    /// This may be called only when the channel is in the `READY` state: initialized but not yet started.
    ///
    /// This is used to preload data into the DMA buffer so that valid data can be transmitted immediately after the
    /// channel is enabled via [I2sTxChannel::tx_enable]. If this function is not called before enabling the channel,
    /// empty data will be transmitted.
    ///
    /// This function can be called multiple times before enabling the channel. Additional calls will concatenate the
    /// data to the end of the buffer until the buffer is full.
    ///
    /// # Returns
    /// This returns the number of bytes that have been loaded into the buffer. If this is less than the length of
    /// the data provided, the buffer is full and no more data can be loaded.
    #[cfg(all(esp_idf_version_major = "5", not(esp_idf_version_minor = "0")))]
    fn preload_data(&mut self, data: &[u8]) -> Result<usize, EspError> {
        let mut bytes_loaded: usize = 0;

        unsafe {
            esp!(esp_idf_sys::i2s_channel_preload_data(
                self.tx_handle(),
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
    #[cfg(esp_idf_version_major = "4")]
    fn write(&mut self, data: &[u8], timeout: Duration) -> Result<usize, EspError> {
        let mut bytes_written: usize = 0;
        let timeout_ms: u32 = timeout.as_millis().try_into().unwrap_or(u32::MAX);
        let ticks_to_wait = timeout_ms * configTICK_RATE_HZ / 1000;
        unsafe {
            esp!(i2s_write(
                self.port(),
                data.as_ptr() as *mut c_void,
                data.len(),
                &mut bytes_written,
                ticks_to_wait,
            ))?;
        }

        Ok(bytes_written)
    }

    /// Write data to the channel.
    ///
    /// This may be called only when the channel is in the `RUNNING` state.
    ///
    /// # Returns
    /// This returns the number of bytes sent. This may be less than the length of the data provided.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn write(&mut self, data: &[u8], timeout: Duration) -> Result<usize, EspError> {
        let mut bytes_written: usize = 0;
        let timeout_ms: u32 = timeout.as_millis().try_into().unwrap_or(u32::MAX);
        unsafe {
            esp!(i2s_channel_write(
                self.tx_handle(),
                data.as_ptr() as *mut c_void,
                data.len(),
                &mut bytes_written,
                timeout_ms
            ))?;
        }

        Ok(bytes_written)
    }

    /// Sets the transmit callback for the channel.
    ///
    /// This may be called only when the channel is in the `REGISTERED` or `RUNNING` state.
    ///
    /// # Bugs
    /// This functionality appears to be fundamentally broken in ESP-IDF 5.0.*. This function is invoked by
    /// [`i2s_dma_tx_callback`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L521).
    /// It expects the callback (or another mechanism) to mark the DMA buffer as ready for reuse, as
    /// [`i2s_channel_write` does by calling `xQueueReceive`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L1038).
    /// Otherwise, the queue eventually fills up and [`xQueueIsQueueFullFromISR` returns true and the recieve buffer overflows](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_common.c#L523).
    ///
    /// However, it is impossible for non ESP-IDF code to call `xQueueReceive` on the queue because the queue is hidden behind the
    /// opaque [`i2s_chan_handle_t` object](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/include/driver/i2s_types.h#L66).
    /// The actual definition is private in [`i2s_channel_obj_t.msg_queue`](https://github.com/espressif/esp-idf/blob/v5.0.1/components/driver/i2s/i2s_private.h#L71-L103).
    ///
    /// This requires a fix in the ESP-IDF SDK to work properly.
    #[cfg(not(esp_idf_version_major = "4"))]
    fn set_tx_callback<Tx: I2sTxCallback + 'static>(
        &mut self,
        callback: Tx,
    ) -> Result<(), EspError>;
}

/// Marker trait indicating that a driver supports the [I2sRx] trait.
pub trait I2sRxSupported {}

/// Concrete implementation of [I2sRxSupported] for use in clients.
///
/// Example usage:
/// ```
/// use esp_idf_hal::i2s::{config::{StdModeConfig, DataBitWidth}, gpio::*};
/// let std_config = StdModeConfig::philips(48000, DataBitWidth::Bits16);
/// let periperhals = Peripherals::take().unwrap();
/// let bclk = peripherals.pins.gpio1;
/// let din = peripherals.pins.gpio4;
/// let mclk = AnyIOPin::none();
/// let ws = peripherals.pins.gpio2;
/// let i2s = I2sStdModeDriver::<I2sRx>::new_rx(periperhals.i2s0, std_config, bclk, Some(din), mclk, ws, None).unwrap();
/// ```
pub struct I2sRx {}
impl I2sRxSupported for I2sRx {}

/// Marker trait indicating that a driver supports the [I2sTx] trait.
pub trait I2sTxSupported {}

/// Concrete implementation of [I2sTxSupported] for use in clients.
///
/// Example usage:
/// ```
/// use esp_idf_hal::i2s::{config::{StdModeConfig, DataBitWidth}, gpio::*};
/// let std_config = StdModeConfig::philips(48000, DataBitWidth::Bits16);
/// let periperhals = Peripherals::take().unwrap();
/// let bclk = peripherals.pins.gpio1;
/// let dout = peripherals.pins.gpio6;
/// let mclk = AnyIOPin::none();
/// let ws = peripherals.pins.gpio2;
/// let i2s = I2sStdModeDriver::<I2sTx>::new_tx(periperhals.i2s0, std_config, bclk, Some(dout), mclk, ws, None).unwrap();
/// ```
pub struct I2sTx {}
impl I2sTxSupported for I2sTx {}

/// Concrete implementation of both [I2sRxSupported] and [I2sTxSupported] for use in clients.
///
/// Example usage:
/// ```
/// use esp_idf_hal::i2s::{config::{StdModeConfig, DataBitWidth}, gpio::*, peripherals::Peripherals};
/// let std_config = StdModeConfig::philips(48000, DataBitWidth::Bits16);
/// let periperhals = Peripherals::take().unwrap();
/// let bclk = peripherals.pins.gpio1;
/// let din = peripherals.pins.gpio4;
/// let dout = peripherals.pins.gpio6;
/// let mclk = AnyIOPin::none();
/// let ws = peripherals.pins.gpio2;
/// let i2s = I2sStdModeDriver::<I2sBiDir>::new_bidir(periperhals.i2s0, std_config, bclk, Some(din), Some(dout), mclk, ws, None, None).unwrap();
/// ```
pub struct I2sBiDir {}
impl I2sRxSupported for I2sBiDir {}
impl I2sTxSupported for I2sBiDir {}

/// Callback handler for a receiver channel.
///
/// This runs in an interrupt context.
#[cfg(not(esp_idf_version_major = "4"))]
pub trait I2sRxCallback {
    /// Called when an I2S receive event occurs.
    ///
    /// # Parameters
    /// * `port`: The driver port (I2S peripheral number) associated with the event.
    /// * `event`: Data associated with the event, including the DMA buffer address and the size that just finished
    ///   receiving data.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken up by this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_receive(&mut self, port: u8, event: &I2sRawEvent) -> bool {
        false
    }

    /// Called when an I2S receiving queue overflows.
    ///
    /// # Parameters
    /// * `port`: The driver port (I2S peripheral number) associated with the event.
    /// * `event`: Data associated with the event, including the buffer size that has been overwritten.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken up by this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_receive_queue_overflow(&mut self, port: u8, event: &I2sRawEvent) -> bool {
        false
    }
}

/// Callback handler for a transmitter channel.
///
/// This runs in an interrupt context.
#[cfg(not(esp_idf_version_major = "4"))]
pub trait I2sTxCallback {
    /// Called when an I2S DMA buffer is finished sending.
    ///
    /// # Parameters
    /// * `port`: The driver port (I2S peripheral number) associated with the event.
    /// * `event`: Data associated with the event, including the DMA buffer address and the size that just finished
    ///   sending data.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken up by this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_sent(&mut self, port: u8, event: &I2sRawEvent) -> bool {
        false
    }

    /// Called when an I2S sending queue overflows.
    ///
    /// # Parameters
    /// * `port`: The driver port (I2S peripheral number) associated with the event.
    /// * `event`: Data associated with the event, including the buffer size that has been overwritten.
    ///
    /// # Returns
    /// Returns `true` if a high priority task has been woken up by this callback function, `false` otherwise.
    #[allow(unused_variables)]
    fn on_send_queue_overflow(&mut self, port: u8, event: &I2sRawEvent) -> bool {
        false
    }
}

/// Internals of the I2S driver for a channel. This is pinned for the lifetime of the driver for
/// interrupts to function properly.
#[cfg(not(esp_idf_version_major = "4"))]
struct I2sChannel<Callback: ?Sized> {
    /// The channel handle.
    chan_handle: i2s_chan_handle_t,

    /// The interrupt handler for channel events.
    callback: Option<Box<Callback>>,

    /// The port number.
    i2s: u8,
}

#[cfg(not(esp_idf_version_major = "4"))]
unsafe impl<Callback: ?Sized> Sync for I2sChannel<Callback> {}

// No Send impl -- this *must* be pinned in memory for the callback to function properly.

#[cfg(not(esp_idf_version_major = "4"))]
impl<Callback: ?Sized> Drop for I2sChannel<Callback> {
    fn drop(&mut self) {
        if !self.chan_handle.is_null() {
            unsafe {
                // Safety: chan_handle is a valid, non-null i2s_chan_handle_t.
                let result = i2s_del_channel(self.chan_handle);
                if result != ESP_OK {
                    // This isn't fatal so a panic isn't warranted, but we do want to be able to debug it.
                    esp_log_write(
                        esp_log_level_t_ESP_LOG_ERROR,
                        LOG_TAG as *const u8 as *const i8,
                        b"Failed to delete channel: %s\0" as *const u8 as *const i8,
                        esp_err_to_name(result),
                    );
                }
                self.chan_handle = null_mut();
            }
        }
    }
}

#[cfg(not(esp_idf_version_major = "4"))]
impl<Callback: I2sRxCallback + ?Sized> I2sChannel<Callback> {
    /// Utility function to set RX callbacks for drivers.
    fn set_rx_callback(&mut self, callback: Box<Callback>) -> Result<(), EspError> {
        let callbacks = i2s_event_callbacks_t {
            on_recv: Some(dispatch_on_recv),
            on_recv_q_ovf: Some(dispatch_on_recv_q_ovf),
            on_sent: None,
            on_send_q_ovf: None,
        };

        // Safety: internal is a valid pointer to I2sStdDriverInternal and is initialized.
        unsafe {
            esp!(i2s_channel_register_event_callback(
                self.chan_handle,
                &callbacks,
                self as *mut Self as *mut c_void
            ))?;

            self.callback = Some(callback);
        }

        Ok(())
    }
}

#[cfg(not(esp_idf_version_major = "4"))]
impl<Callback: I2sTxCallback + ?Sized> I2sChannel<Callback> {
    /// Utility function to set TX callbacks for drivers.
    fn set_tx_callback(&mut self, callback: Box<Callback>) -> Result<(), EspError> {
        let callbacks = i2s_event_callbacks_t {
            on_recv: None,
            on_recv_q_ovf: None,
            on_sent: Some(dispatch_on_sent),
            on_send_q_ovf: Some(dispatch_on_send_q_ovf),
        };

        // Safety: internal is a valid pointer to I2sStdDriverInternal and is initialized.
        unsafe {
            let e = i2s_channel_register_event_callback(
                self.chan_handle,
                &callbacks,
                self as *mut Self as *mut c_void,
            );

            if e != ESP_OK {
                println!("Failed to register TX callback: error code {e}");
                esp!(e)?;
            }

            self.callback = Some(callback);
        }

        Ok(())
    }
}

/// C-facing ISR dispatcher for on_recv callbacks.
#[cfg(not(esp_idf_version_major = "4"))]
extern "C" fn dispatch_on_recv(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    // Safety: user_ctx is always a pointer to I2sStdDriverChannel.
    let internal: &mut I2sChannel<dyn I2sRxCallback> =
        unsafe { &mut *(user_ctx as *mut I2sChannel<dyn I2sRxCallback>) };
    if let Some(callback) = &mut internal.callback {
        callback.on_receive(internal.i2s, unsafe {
            &*(event as *const i2s_event_data_t)
        })
    } else {
        false
    }
}

/// C-facing ISR dispatcher for on_recv_q_ovf callbacks.
#[cfg(not(esp_idf_version_major = "4"))]
extern "C" fn dispatch_on_recv_q_ovf(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    // Safety: user_ctx is always a pointer to I2sStdDriverChannel.
    let internal: &mut I2sChannel<dyn I2sRxCallback> =
        unsafe { &mut *(user_ctx as *mut I2sChannel<dyn I2sRxCallback>) };
    if let Some(callback) = &mut internal.callback {
        callback.on_receive_queue_overflow(internal.i2s, unsafe {
            &*(event as *const i2s_event_data_t)
        })
    } else {
        false
    }
}

/// C-facing ISR dispatcher for on_sent callbacks.
#[cfg(not(esp_idf_version_major = "4"))]
extern "C" fn dispatch_on_sent(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    // Safety: user_ctx is always a pointer to I2sStdDriverChannel.
    let internal: &mut I2sChannel<dyn I2sTxCallback> =
        unsafe { &mut *(user_ctx as *mut I2sChannel<dyn I2sTxCallback>) };
    if let Some(callback) = &mut internal.callback {
        callback.on_sent(internal.i2s, unsafe {
            &*(event as *const i2s_event_data_t)
        })
    } else {
        false
    }
}

/// C-facing ISR dispatcher for on_send_q_ovf callbacks.
#[cfg(not(esp_idf_version_major = "4"))]
extern "C" fn dispatch_on_send_q_ovf(
    _handle: i2s_chan_handle_t,
    event: *mut i2s_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    // Safety: user_ctx is always a pointer to I2sStdDriverChannel.
    let internal: &mut I2sChannel<dyn I2sTxCallback> =
        unsafe { &mut *(user_ctx as *mut I2sChannel<dyn I2sTxCallback>) };
    if let Some(callback) = &mut internal.callback {
        callback.on_send_queue_overflow(internal.i2s, unsafe {
            &*(event as *const i2s_event_data_t)
        })
    } else {
        false
    }
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

#[cfg(not(esp_idf_version_major = "4"))]
pub type I2sRawEvent = i2s_event_data_t;

#[cfg(any(
    all(
        not(esp_idf_version_major = "4"),
        any(esp_idf_soc_i2s_supports_pdm_rx, esp_idf_soc_i2s_supports_pdm_tx)
    ),
    all(esp_idf_version_major = "4", any(esp32, esp32s3, esp32c3, esp32c6))
))]
pub use self::pdm::*;

pub use self::std::*;

#[cfg(any(
    all(not(esp_idf_version_major = "4"), esp_idf_soc_i2s_supports_tdm),
    all(esp_idf_version_major = "4", any(esp32s3, esp32c3, esp32c6))
))]
pub use self::tdm::*;

impl_i2s!(I2S0: 0);
#[cfg(any(esp32, esp32s3))]
impl_i2s!(I2S1: 1);
