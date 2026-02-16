//! DSI peripheral control
//!
//! Interface to the [LCD (Liquid Crystal Display)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/lcd.html)
//! peripheral, supporting DSI (Display Serial Interface) panels.
//!
//! This module provides safe Rust wrappers for the LCD API, which is available
//! on ESP32-P4 and later chips.
//!
//! The driver uses a typestate pattern: [`LcdDriver<NoDpiPanel>`] is returned from
//! [`LcdDriver::new()`] and supports command/control operations. After calling
//! [`LcdDriver::create_dpi_panel()`] or [`LcdDriver::set_dpi_panel()`], it transitions
//! to [`LcdDriver<WithDpiPanel>`] which supports drawing operations.
//!
//! # Example
//!
//! Initialize a DSI LCD panel and draw a bitmap:
//! ```no_run
//! use esp_idf_hal::dsi::*;
//! use esp_idf_hal::peripherals::Peripherals;
//! use esp_idf_sys::*;
//!
//! let peripherals = Peripherals::take()?;
//!
//! let video_timing = VideoTiming::new(1024, 600)
//!     .hsync_back_porch(100)
//!     .hsync_pulse_width(100)
//!     .hsync_front_porch(100)
//!     .vsync_back_porch(10)
//!     .vsync_pulse_width(10)
//!     .vsync_front_porch(10);
//!
//! let config = LcdConfig::new(
//!     video_timing,
//!     2,     // num_data_lanes
//!     1250,  // lane_bit_rate_mbps
//!     48,    // dpi_clock_freq_mhz
//!     PixelFormat::Rgb888,
//! );
//!
//! // Create driver (NoDpiPanel state)
//! let mut lcd = LcdDriver::new(peripherals.dsi, &config)?;
//!
//! // Create and configure vendor control panel
//! let mut dev_config: esp_lcd_panel_dev_config_t = lcd.config().into();
//! dev_config.reset_gpio_num = 5;
//! let mut control_panel: PanelHandle = core::ptr::null_mut();
//! unsafe {
//!     esp!(esp_lcd_new_panel_ili9881c(lcd.dbi_io_handle(), &dev_config, &mut control_panel))?;
//! }
//! lcd.set_control_panel(control_panel)?;
//! lcd.reset()?;
//! lcd.init()?;
//! lcd.set_display_on(true)?;
//!
//! // Transition to WithDpiPanel state
//! let lcd = lcd.create_dpi_panel()?;
//!
//! // Draw a bitmap
//! let bitmap = vec![0u8; 1024 * 600 * 3];
//! lcd.draw_bitmap(0, 0, 1024, 600, &bitmap)?;
//! ```

#[cfg(not(feature = "alloc"))]
compile_error!("DSI module requires the `alloc` feature to be enabled");

use core::ffi::c_void;
use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use core::ptr::NonNull;

extern crate alloc;

use alloc::boxed::Box;

use crate::sys::EspError;
use esp_idf_sys::*;

// ---------------------------------------------------------------------------
// Type aliases for ESP-IDF handle types
// ---------------------------------------------------------------------------

/// Handle to a MIPI-DSI bus, obtained from `esp_lcd_new_dsi_bus`.
pub type DsiBusHandle = esp_lcd_dsi_bus_handle_t;

/// Handle to a panel IO interface (DBI), obtained from `esp_lcd_new_panel_io_dbi`.
pub type PanelIoHandle = esp_lcd_panel_io_handle_t;

/// Handle to an LCD panel (control or DPI), obtained from vendor-specific constructors
/// or `esp_lcd_new_panel_dpi`.
pub type PanelHandle = esp_lcd_panel_handle_t;

// ---------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------

/// DSI configuration types
pub mod config {
    use super::*;

    /// Complete LCD configuration
    ///
    /// Stores the underlying ESP-IDF C configuration structs directly, ensuring
    /// that any field changes on the C side cause a compile error rather than
    /// silently breaking. Builder methods provide typed access for optional settings.
    ///
    /// # Required Parameters
    ///
    /// The constructor requires all display-specific values since these differ per panel:
    /// - Video timing (resolution and sync porches)
    /// - Number of DSI data lanes
    /// - Lane bit rate
    /// - DPI clock frequency
    /// - Pixel format
    pub struct LcdConfig {
        pub(crate) bus_config: esp_lcd_dsi_bus_config_t,
        pub(crate) dbi_config: esp_lcd_dbi_io_config_t,
        pub(crate) dpi_config: esp_lcd_dpi_panel_config_t,
        pixel_format: PixelFormat,
    }

    impl LcdConfig {
        /// Create a new LCD configuration
        ///
        /// All display-specific parameters are required since they differ per panel.
        ///
        /// # Arguments
        ///
        /// * `video_timing` - Video timing configuration (resolution and sync timings)
        /// * `num_data_lanes` - Number of DSI data lanes (typically 1, 2, or 4)
        /// * `lane_bit_rate_mbps` - Lane bit rate in Mbps (e.g. 1000, 1250)
        /// * `dpi_clock_freq_mhz` - DPI clock frequency in MHz (e.g. 48, 52)
        /// * `pixel_format` - Pixel format for the display
        pub fn new(
            video_timing: VideoTiming,
            num_data_lanes: u8,
            lane_bit_rate_mbps: u32,
            dpi_clock_freq_mhz: u32,
            pixel_format: PixelFormat,
        ) -> Self {
            let bus_config = esp_lcd_dsi_bus_config_t {
                bus_id: 0,
                num_data_lanes,
                phy_clk_src: soc_module_clk_t_SOC_MOD_CLK_PLL_F160M as u32,
                lane_bit_rate_mbps,
            };

            let dbi_config = esp_lcd_dbi_io_config_t {
                virtual_channel: 0,
                lcd_cmd_bits: 8,
                lcd_param_bits: 8,
            };

            let color_format: lcd_color_format_t = pixel_format.to_color_format();
            let esp_timing: esp_lcd_video_timing_t = video_timing.into();

            let mut dpi_config = esp_lcd_dpi_panel_config_t {
                virtual_channel: 0,
                dpi_clk_src: soc_periph_mipi_dsi_dpi_clk_src_t_MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                pixel_format: pixel_format.to_rgb_pixel_format(),
                in_color_format: color_format,
                out_color_format: color_format,
                num_fbs: 1,
                video_timing: esp_timing,
                dpi_clock_freq_mhz,
                ..unsafe { core::mem::zeroed() }
            };
            dpi_config.flags.set_use_dma2d(1);

            Self {
                bus_config,
                dbi_config,
                dpi_config,
                pixel_format,
            }
        }

        // Builder methods for optional settings

        /// Set the DSI bus ID (default: 0)
        #[must_use]
        pub fn bus_id(mut self, bus_id: i32) -> Self {
            self.bus_config.bus_id = bus_id;
            self
        }

        /// Set the PHY clock source
        #[must_use]
        pub fn phy_clk_src(mut self, phy_clk_src: u32) -> Self {
            self.bus_config.phy_clk_src = phy_clk_src;
            self
        }

        /// Set the virtual channel ID (default: 0)
        ///
        /// This sets the virtual channel for both DBI IO and DPI panel.
        #[must_use]
        pub fn virtual_channel(mut self, virtual_channel: u8) -> Self {
            self.dbi_config.virtual_channel = virtual_channel;
            self.dpi_config.virtual_channel = virtual_channel;
            self
        }

        /// Set the LCD command bit-width (default: 8)
        #[must_use]
        pub fn lcd_cmd_bits(mut self, lcd_cmd_bits: i32) -> Self {
            self.dbi_config.lcd_cmd_bits = lcd_cmd_bits;
            self
        }

        /// Set the LCD parameter bit-width (default: 8)
        #[must_use]
        pub fn lcd_param_bits(mut self, lcd_param_bits: i32) -> Self {
            self.dbi_config.lcd_param_bits = lcd_param_bits;
            self
        }

        /// Set the DPI clock source
        #[must_use]
        pub fn dpi_clk_src(mut self, dpi_clk_src: u32) -> Self {
            self.dpi_config.dpi_clk_src = dpi_clk_src;
            self
        }

        /// Set the number of frame buffers (default: 1)
        #[must_use]
        pub fn num_fbs(mut self, num_fbs: u8) -> Self {
            self.dpi_config.num_fbs = num_fbs;
            self
        }

        /// Enable or disable DMA2D for pixel data transfer (default: enabled)
        #[must_use]
        pub fn use_dma2d(mut self, use_dma2d: bool) -> Self {
            self.dpi_config.flags.set_use_dma2d(use_dma2d as u32);
            self
        }

        // Accessor methods

        /// Get a reference to the underlying DSI bus configuration
        pub fn bus_config(&self) -> &esp_lcd_dsi_bus_config_t {
            &self.bus_config
        }

        /// Get a reference to the underlying DBI IO configuration
        pub fn dbi_config(&self) -> &esp_lcd_dbi_io_config_t {
            &self.dbi_config
        }

        /// Get a reference to the underlying DPI panel configuration
        pub fn dpi_config(&self) -> &esp_lcd_dpi_panel_config_t {
            &self.dpi_config
        }

        /// Get the pixel format
        pub fn pixel_format(&self) -> PixelFormat {
            self.pixel_format
        }
    }

    impl Clone for LcdConfig {
        fn clone(&self) -> Self {
            Self {
                bus_config: self.bus_config,
                dbi_config: self.dbi_config,
                dpi_config: self.dpi_config,
                pixel_format: self.pixel_format,
            }
        }
    }

    impl core::fmt::Debug for LcdConfig {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("LcdConfig")
                .field("pixel_format", &self.pixel_format)
                .field("num_data_lanes", &self.bus_config.num_data_lanes)
                .field("lane_bit_rate_mbps", &self.bus_config.lane_bit_rate_mbps)
                .field("dpi_clock_freq_mhz", &self.dpi_config.dpi_clock_freq_mhz)
                .field("h_size", &self.dpi_config.video_timing.h_size)
                .field("v_size", &self.dpi_config.video_timing.v_size)
                .finish_non_exhaustive()
        }
    }

    impl From<&LcdConfig> for esp_lcd_panel_dev_config_t {
        fn from(config: &LcdConfig) -> Self {
            let bits_per_pixel = config.pixel_format.bits_per_pixel();
            let rgb_order = config.pixel_format.rgb_element_order();

            let mut panel_config: esp_lcd_panel_dev_config_t = unsafe { core::mem::zeroed() };

            panel_config.bits_per_pixel = bits_per_pixel;
            panel_config.__bindgen_anon_1.rgb_ele_order = rgb_order.into();

            panel_config
        }
    }

    /// Video timing configuration
    #[derive(Debug, Clone, Copy)]
    pub struct VideoTiming {
        pub h_size: u32,
        pub v_size: u32,
        pub hsync_back_porch: u32,
        pub hsync_pulse_width: u32,
        pub hsync_front_porch: u32,
        pub vsync_back_porch: u32,
        pub vsync_pulse_width: u32,
        pub vsync_front_porch: u32,
    }

    impl VideoTiming {
        /// Create a new video timing configuration
        pub const fn new(h_size: u32, v_size: u32) -> Self {
            Self {
                h_size,
                v_size,
                hsync_back_porch: 0,
                hsync_pulse_width: 0,
                hsync_front_porch: 0,
                vsync_back_porch: 0,
                vsync_pulse_width: 0,
                vsync_front_porch: 0,
            }
        }

        #[must_use]
        pub fn hsync_back_porch(mut self, hsync_back_porch: u32) -> Self {
            self.hsync_back_porch = hsync_back_porch;
            self
        }

        #[must_use]
        pub fn hsync_pulse_width(mut self, hsync_pulse_width: u32) -> Self {
            self.hsync_pulse_width = hsync_pulse_width;
            self
        }

        #[must_use]
        pub fn hsync_front_porch(mut self, hsync_front_porch: u32) -> Self {
            self.hsync_front_porch = hsync_front_porch;
            self
        }

        #[must_use]
        pub fn vsync_back_porch(mut self, vsync_back_porch: u32) -> Self {
            self.vsync_back_porch = vsync_back_porch;
            self
        }

        #[must_use]
        pub fn vsync_pulse_width(mut self, vsync_pulse_width: u32) -> Self {
            self.vsync_pulse_width = vsync_pulse_width;
            self
        }

        #[must_use]
        pub fn vsync_front_porch(mut self, vsync_front_porch: u32) -> Self {
            self.vsync_front_porch = vsync_front_porch;
            self
        }
    }

    impl From<VideoTiming> for esp_lcd_video_timing_t {
        fn from(timing: VideoTiming) -> Self {
            esp_lcd_video_timing_t {
                h_size: timing.h_size,
                v_size: timing.v_size,
                hsync_back_porch: timing.hsync_back_porch,
                hsync_pulse_width: timing.hsync_pulse_width,
                hsync_front_porch: timing.hsync_front_porch,
                vsync_back_porch: timing.vsync_back_porch,
                vsync_pulse_width: timing.vsync_pulse_width,
                vsync_front_porch: timing.vsync_front_porch,
            }
        }
    }

    /// Pixel format
    ///
    /// Encodes both the bit depth/format and the RGB element ordering.
    /// The format name (e.g., `Rgb888`, `Bgr888`) specifies both the
    /// pixel format and the color component order.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum PixelFormat {
        /// 24-bit RGB format with RGB component order
        Rgb888,
        /// 24-bit RGB format with BGR component order
        Bgr888,
        /// 16-bit RGB format with RGB component order
        Rgb565,
        /// 16-bit RGB format with BGR component order
        Bgr565,
        /// 18-bit RGB format with RGB component order
        Rgb666,
        /// 18-bit RGB format with BGR component order
        Bgr666,
    }

    impl PixelFormat {
        /// Get the bits per pixel for this format
        pub const fn bits_per_pixel(&self) -> u32 {
            match self {
                PixelFormat::Rgb888 | PixelFormat::Bgr888 => 24,
                PixelFormat::Rgb565 | PixelFormat::Bgr565 => 16,
                PixelFormat::Rgb666 | PixelFormat::Bgr666 => 18,
            }
        }

        /// Get the bytes per pixel for this format
        pub const fn bytes_per_pixel(&self) -> usize {
            match self {
                PixelFormat::Rgb888
                | PixelFormat::Bgr888
                | PixelFormat::Rgb666
                | PixelFormat::Bgr666 => 3,
                PixelFormat::Rgb565 | PixelFormat::Bgr565 => 2,
            }
        }

        /// Get the RGB element order for this format
        pub const fn rgb_element_order(&self) -> RgbElementOrder {
            match self {
                PixelFormat::Rgb888 | PixelFormat::Rgb565 | PixelFormat::Rgb666 => {
                    RgbElementOrder::Rgb
                }
                PixelFormat::Bgr888 | PixelFormat::Bgr565 | PixelFormat::Bgr666 => {
                    RgbElementOrder::Bgr
                }
            }
        }

        /// Convert to `lcd_color_rgb_pixel_format_t`
        pub fn to_rgb_pixel_format(&self) -> lcd_color_rgb_pixel_format_t {
            match self {
                PixelFormat::Rgb888 | PixelFormat::Bgr888 => {
                    lcd_color_rgb_pixel_format_t_LCD_COLOR_PIXEL_FORMAT_RGB888
                }
                PixelFormat::Rgb565 | PixelFormat::Bgr565 => {
                    lcd_color_rgb_pixel_format_t_LCD_COLOR_PIXEL_FORMAT_RGB565
                }
                PixelFormat::Rgb666 | PixelFormat::Bgr666 => {
                    lcd_color_rgb_pixel_format_t_LCD_COLOR_PIXEL_FORMAT_RGB666
                }
            }
        }

        /// Convert to `lcd_color_format_t`
        pub fn to_color_format(&self) -> lcd_color_format_t {
            match self {
                PixelFormat::Rgb888 | PixelFormat::Bgr888 => {
                    lcd_color_format_t_LCD_COLOR_FMT_RGB888
                }
                PixelFormat::Rgb565 | PixelFormat::Bgr565 => {
                    lcd_color_format_t_LCD_COLOR_FMT_RGB565
                }
                PixelFormat::Rgb666 | PixelFormat::Bgr666 => {
                    lcd_color_format_t_LCD_COLOR_FMT_RGB666
                }
            }
        }
    }

    // Note: We can't implement From<PixelFormat> for both lcd_color_rgb_pixel_format_t and
    // lcd_color_format_t because they're both type aliases for u32, which would cause a conflict.
    // Use to_rgb_pixel_format() and to_color_format() methods instead.

    /// RGB element order
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum RgbElementOrder {
        Rgb,
        Bgr,
    }

    impl From<RgbElementOrder> for lcd_rgb_element_order_t {
        fn from(order: RgbElementOrder) -> Self {
            match order {
                RgbElementOrder::Rgb => lcd_rgb_element_order_t_LCD_RGB_ELEMENT_ORDER_RGB,
                RgbElementOrder::Bgr => lcd_rgb_element_order_t_LCD_RGB_ELEMENT_ORDER_BGR,
            }
        }
    }

    /// RGB data endianness
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum RgbDataEndian {
        BigEndian,
        LittleEndian,
    }

    impl From<RgbDataEndian> for lcd_rgb_data_endian_t {
        fn from(endian: RgbDataEndian) -> Self {
            match endian {
                RgbDataEndian::BigEndian => lcd_rgb_data_endian_t_LCD_RGB_DATA_ENDIAN_BIG,
                RgbDataEndian::LittleEndian => lcd_rgb_data_endian_t_LCD_RGB_DATA_ENDIAN_LITTLE,
            }
        }
    }
}

// Re-export config types at module level
pub use config::*;

// ---------------------------------------------------------------------------
// DSI peripheral
// ---------------------------------------------------------------------------

crate::impl_peripheral!(DSI);

// ---------------------------------------------------------------------------
// Typestate types
// ---------------------------------------------------------------------------

/// Marker trait for LCD driver state
pub trait DpiPanelState {}

/// No DPI panel has been created yet.
///
/// Command-level operations (`tx_param`, `rx_param`, `tx_color`) and
/// control panel operations (`reset`, `init`, `set_display_on`) are available.
pub struct NoDpiPanel;
impl DpiPanelState for NoDpiPanel {}

/// A DPI panel is active.
///
/// Drawing and display manipulation methods (`draw_bitmap`, `invert_colors`,
/// `mirror_x`, `mirror_y`, `swap_xy`, `set_gap`) are available.
pub struct WithDpiPanel;
impl DpiPanelState for WithDpiPanel {}

// ---------------------------------------------------------------------------
// Internal DPI panel data
// ---------------------------------------------------------------------------

struct DpiPanel {
    handle: NonNull<c_void>,
    /// Heap-allocated DPI config (only set when we created the panel via `create_dpi_panel`)
    config_box: Option<*mut esp_lcd_dpi_panel_config_t>,
    /// Binary semaphore signaled by `on_color_trans_done` when DMA2D finishes
    /// copying the caller's buffer into the internal frame buffer.
    draw_done_sem: SemaphoreHandle_t,
}

/// FreeRTOS `queueQUEUE_TYPE_BINARY_SEMAPHORE` (C macro, not exported by bindgen).
const QUEUE_TYPE_BINARY_SEMAPHORE: u8 = 3;

/// Create a FreeRTOS binary semaphore.
///
/// The semaphore starts in the "empty" state so the first `take` will block
/// until someone `give`s it.
fn create_binary_semaphore() -> Result<SemaphoreHandle_t, EspError> {
    let sem = unsafe { xQueueGenericCreate(1, 0, QUEUE_TYPE_BINARY_SEMAPHORE) };
    if sem.is_null() {
        Err(EspError::from_infallible::<ESP_ERR_NO_MEM>())
    } else {
        Ok(sem)
    }
}

/// Register the `on_color_trans_done` callback on a DPI panel.
///
/// The callback gives `sem` from ISR context when the DMA2D transfer that
/// copies the caller's buffer into the internal frame buffer completes.
fn register_draw_done_cb(
    panel_handle: PanelHandle,
    sem: SemaphoreHandle_t,
) -> Result<(), EspError> {
    unsafe {
        let cbs = esp_lcd_dpi_panel_event_callbacks_t {
            on_color_trans_done: Some(on_color_trans_done_isr),
            ..core::mem::zeroed()
        };
        esp!(esp_lcd_dpi_panel_register_event_callbacks(
            panel_handle,
            &cbs,
            sem as *mut c_void,
        ))
    }
}

/// ISR callback: signals the binary semaphore when DMA2D finishes.
unsafe extern "C" fn on_color_trans_done_isr(
    _panel: esp_lcd_panel_handle_t,
    _edata: *mut esp_lcd_dpi_panel_event_data_t,
    user_ctx: *mut c_void,
) -> bool {
    let sem = user_ctx as SemaphoreHandle_t;
    let mut higher_prio_woken: BaseType_t = 0;
    xQueueGiveFromISR(sem, &mut higher_prio_woken);
    higher_prio_woken != 0
}

// ---------------------------------------------------------------------------
// VendorPanel trait
// ---------------------------------------------------------------------------

/// Trait for vendor-specific LCD panel drivers.
///
/// Vendor panel drivers create a control panel handle from a DBI IO interface.
/// The control panel is used for initialization commands (reset, init, display on/off).
///
/// # Example
///
/// ```no_run
/// use esp_idf_hal::dsi::*;
/// use esp_idf_sys::*;
///
/// struct Ili9881c {
///     reset_gpio: i32,
/// }
///
/// impl VendorPanel for Ili9881c {
///     fn create(
///         &self,
///         io_handle: PanelIoHandle,
///         dev_config: &esp_lcd_panel_dev_config_t,
///     ) -> Result<PanelHandle, esp_idf_hal::sys::EspError> {
///         let mut handle: PanelHandle = core::ptr::null_mut();
///         unsafe {
///             esp!(esp_lcd_new_panel_ili9881c(io_handle, dev_config, &mut handle))?;
///         }
///         Ok(handle)
///     }
/// }
/// ```
pub trait VendorPanel {
    /// Create a vendor-specific control panel.
    ///
    /// # Arguments
    ///
    /// * `io_handle` - The DBI IO handle for sending commands
    /// * `dev_config` - Panel device configuration (pixel format, reset GPIO, etc.)
    fn create(
        &self,
        io_handle: PanelIoHandle,
        dev_config: &esp_lcd_panel_dev_config_t,
    ) -> Result<PanelHandle, EspError>;
}

// ---------------------------------------------------------------------------
// LcdDriver
// ---------------------------------------------------------------------------

/// LCD driver with typestate for DPI panel availability
///
/// The type parameter `S` tracks whether a DPI panel has been created:
/// - [`NoDpiPanel`]: only command/control operations are available
/// - [`WithDpiPanel`]: drawing operations are also available
pub struct LcdDriver<'d, S: DpiPanelState = NoDpiPanel> {
    bus_handle: NonNull<c_void>,
    dbi_io_handle: NonNull<c_void>,
    control_panel_handle: Option<PanelHandle>,
    dpi_panel: Option<DpiPanel>,
    config: LcdConfig,
    _p: PhantomData<&'d mut ()>,
    _state: PhantomData<S>,
}

// ---------------------------------------------------------------------------
// Methods available in both states
// ---------------------------------------------------------------------------

impl<'d, S: DpiPanelState> LcdDriver<'d, S> {
    /// Get the raw DSI bus handle
    ///
    /// Useful for vendor-specific drivers that need the DSI bus handle.
    pub fn bus_handle(&self) -> DsiBusHandle {
        self.bus_handle.as_ptr().cast()
    }

    /// Get the raw DBI IO handle
    ///
    /// The DBI IO handle can be used to send commands to the LCD panel controller
    /// (e.g., for manufacturer-specific initialization sequences).
    pub fn dbi_io_handle(&self) -> PanelIoHandle {
        self.dbi_io_handle.as_ptr().cast()
    }

    /// Get a reference to the stored configuration
    pub fn config(&self) -> &LcdConfig {
        &self.config
    }

    /// Get the raw control panel handle, if one has been set
    pub fn control_panel_handle(&self) -> Option<PanelHandle> {
        self.control_panel_handle
    }

    /// Set the control panel handle
    ///
    /// This must be called before using control panel operations (`reset`, `init`, etc.).
    /// The handle is typically created using vendor-specific functions
    /// (e.g., `esp_lcd_new_panel_ili9881c`) with the DBI IO handle from `dbi_io_handle()`.
    ///
    /// Returns the previous control panel handle if one was set.
    pub fn set_control_panel(
        &mut self,
        control_panel: PanelHandle,
    ) -> Result<Option<PanelHandle>, EspError> {
        if control_panel.is_null() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
        }
        Ok(self.control_panel_handle.replace(control_panel))
    }

    /// Create and set a vendor-specific control panel.
    ///
    /// Convenience method that calls [`VendorPanel::create()`] with the driver's
    /// DBI IO handle and a device config derived from the stored [`LcdConfig`],
    /// then sets the resulting handle as the control panel.
    ///
    /// For more control over the `esp_lcd_panel_dev_config_t` (e.g., setting
    /// `reset_gpio_num` or `vendor_config`), use [`Self::with_vendor_panel_config()`]
    /// or the manual flow with `dbi_io_handle()`, `config()`, and `set_control_panel()`.
    pub fn with_vendor_panel<V: VendorPanel>(&mut self, vendor: &V) -> Result<(), EspError> {
        let dev_config: esp_lcd_panel_dev_config_t = (&self.config).into();
        self.with_vendor_panel_config(vendor, &dev_config)
    }

    /// Create and set a vendor-specific control panel with custom device config.
    pub fn with_vendor_panel_config<V: VendorPanel>(
        &mut self,
        vendor: &V,
        dev_config: &esp_lcd_panel_dev_config_t,
    ) -> Result<(), EspError> {
        let handle = vendor.create(self.dbi_io_handle(), dev_config)?;
        self.set_control_panel(handle)?;
        Ok(())
    }

    /// Transmit LCD command and corresponding parameters via DBI IO
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `param` - Buffer containing the command parameters, or an empty slice if no parameters
    pub fn tx_param(&self, lcd_cmd: i32, param: &[u8]) -> Result<(), EspError> {
        unsafe {
            let param_ptr = if param.is_empty() {
                core::ptr::null()
            } else {
                param.as_ptr() as *const c_void
            };
            esp!(esp_lcd_panel_io_tx_param(
                self.dbi_io_handle(),
                lcd_cmd,
                param_ptr,
                param.len(),
            ))?;
        }
        Ok(())
    }

    /// Receive LCD command parameters via DBI IO
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `param` - Buffer to receive the command parameters
    pub fn rx_param(&self, lcd_cmd: i32, param: &mut [u8]) -> Result<(), EspError> {
        unsafe {
            esp!(esp_lcd_panel_io_rx_param(
                self.dbi_io_handle(),
                lcd_cmd,
                param.as_mut_ptr() as *mut c_void,
                param.len(),
            ))?;
        }
        Ok(())
    }

    /// Transmit LCD color data via DBI IO
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `color` - Buffer containing the color data to transmit
    pub fn tx_color(&self, lcd_cmd: i32, color: &[u8]) -> Result<(), EspError> {
        unsafe {
            esp!(esp_lcd_panel_io_tx_color(
                self.dbi_io_handle(),
                lcd_cmd,
                color.as_ptr() as *const c_void,
                color.len(),
            ))?;
        }
        Ok(())
    }

    // Control panel operations (require set_control_panel to have been called)

    fn require_control_panel(&self) -> Result<PanelHandle, EspError> {
        self.control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
    }

    /// Reset the panel
    ///
    /// Requires that `set_control_panel()` has been called first.
    pub fn reset(&self) -> Result<(), EspError> {
        let panel = self.require_control_panel()?;
        unsafe { esp!(esp_lcd_panel_reset(panel)) }
    }

    /// Initialize the panel
    ///
    /// Requires that `set_control_panel()` has been called first.
    pub fn init(&self) -> Result<(), EspError> {
        let panel = self.require_control_panel()?;
        unsafe { esp!(esp_lcd_panel_init(panel)) }
    }

    /// Turn the display on or off
    ///
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_on(&self, on: bool) -> Result<(), EspError> {
        let panel = self.require_control_panel()?;
        unsafe { esp!(esp_lcd_panel_disp_on_off(panel, on)) }
    }

    /// Turn the display off
    ///
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_off(&self, off: bool) -> Result<(), EspError> {
        let panel = self.require_control_panel()?;
        unsafe { esp!(esp_lcd_panel_disp_off(panel, off)) }
    }

    /// Enter or exit sleep mode (low power mode)
    ///
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_sleep(&self, sleep: bool) -> Result<(), EspError> {
        let panel = self.require_control_panel()?;
        unsafe { esp!(esp_lcd_panel_disp_sleep(panel, sleep)) }
    }
}

// ---------------------------------------------------------------------------
// NoDpiPanel methods
// ---------------------------------------------------------------------------

impl<'d> LcdDriver<'d, NoDpiPanel> {
    /// Create a new DSI LCD driver
    ///
    /// This initializes the DSI bus and DBI IO interface. After creation:
    /// 1. Optionally set a vendor-specific control panel using `set_control_panel()`
    /// 2. Initialize the panel (`reset()`, `init()`, `set_display_on()`)
    /// 3. Create the DPI panel using `create_dpi_panel()` or `set_dpi_panel()`
    ///
    /// # Example
    ///
    /// ```no_run
    /// use esp_idf_hal::dsi::*;
    /// use esp_idf_hal::peripherals::Peripherals;
    /// use esp_idf_sys::*;
    ///
    /// let peripherals = Peripherals::take().unwrap();
    ///
    /// let video_timing = VideoTiming::new(1024, 600)
    ///     .hsync_back_porch(100)
    ///     .hsync_pulse_width(100)
    ///     .hsync_front_porch(100)
    ///     .vsync_back_porch(10)
    ///     .vsync_pulse_width(10)
    ///     .vsync_front_porch(10);
    ///
    /// let config = LcdConfig::new(
    ///     video_timing, 2, 1250, 48, PixelFormat::Rgb888,
    /// );
    ///
    /// let mut driver = LcdDriver::new(peripherals.dsi, &config)?;
    ///
    /// // Set up vendor control panel, then transition:
    /// // let driver = driver.create_dpi_panel()?;
    /// ```
    pub fn new(_dsi: DSI<'d>, config: &LcdConfig) -> Result<Self, EspError> {
        unsafe {
            // Create DSI bus
            let mut bus_handle: DsiBusHandle = core::ptr::null_mut();
            esp!(esp_lcd_new_dsi_bus(&config.bus_config, &mut bus_handle))?;

            let bus_handle = NonNull::new(bus_handle as *mut c_void)
                .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;

            // Create DBI IO interface
            let mut dbi_io: PanelIoHandle = core::ptr::null_mut();
            let result = esp!(esp_lcd_new_panel_io_dbi(
                bus_handle.as_ptr().cast(),
                &config.dbi_config,
                &mut dbi_io,
            ));
            if let Err(e) = result {
                let _ = esp_lcd_del_dsi_bus(bus_handle.as_ptr().cast());
                return Err(e);
            }

            let dbi_io_handle = match NonNull::new(dbi_io as *mut c_void) {
                Some(h) => h,
                None => {
                    let _ = esp_lcd_del_dsi_bus(bus_handle.as_ptr().cast());
                    return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
                }
            };

            Ok(Self {
                bus_handle,
                dbi_io_handle,
                control_panel_handle: None,
                dpi_panel: None,
                config: config.clone(),
                _p: PhantomData,
                _state: PhantomData,
            })
        }
    }

    /// Create a DPI panel for drawing operations
    ///
    /// Consumes the driver and returns a new driver in the [`WithDpiPanel`] state.
    /// This should be called after the vendor-specific control panel has been
    /// initialized (`set_control_panel()`, `reset()`, `init()`, `set_display_on()`).
    pub fn create_dpi_panel(self) -> Result<LcdDriver<'d, WithDpiPanel>, EspError> {
        unsafe {
            let dpi_config = Box::new(self.config.dpi_config);
            let dpi_config_ptr = Box::into_raw(dpi_config);

            let mut panel_handle: PanelHandle = core::ptr::null_mut();
            let result = esp!(esp_lcd_new_panel_dpi(
                self.bus_handle.as_ptr().cast(),
                dpi_config_ptr,
                &mut panel_handle,
            ));

            if let Err(e) = result {
                let _ = Box::from_raw(dpi_config_ptr);
                return Err(e);
                // self drops here, cleaning up bus + DBI IO + control panel
            }

            let handle = match NonNull::new(panel_handle as *mut c_void) {
                Some(h) => h,
                None => {
                    let _ = Box::from_raw(dpi_config_ptr);
                    return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
                }
            };

            // Create semaphore + callback so draw_bitmap can wait for DMA completion
            let sem = create_binary_semaphore().map_err(|e| {
                let _ = esp_lcd_panel_del(panel_handle);
                let _ = Box::from_raw(dpi_config_ptr);
                e
            })?;
            register_draw_done_cb(panel_handle, sem).map_err(|e| {
                vQueueDelete(sem);
                let _ = esp_lcd_panel_del(panel_handle);
                let _ = Box::from_raw(dpi_config_ptr);
                e
            })?;

            // Prevent Drop from running on self (we're moving fields to the new driver)
            let this = ManuallyDrop::new(self);

            Ok(LcdDriver {
                bus_handle: this.bus_handle,
                dbi_io_handle: this.dbi_io_handle,
                control_panel_handle: this.control_panel_handle,
                dpi_panel: Some(DpiPanel {
                    handle,
                    config_box: Some(dpi_config_ptr),
                    draw_done_sem: sem,
                }),
                config: core::ptr::read(&this.config),
                _p: PhantomData,
                _state: PhantomData,
            })
        }
    }

    /// Set an externally-created DPI panel handle
    ///
    /// Use this instead of `create_dpi_panel()` when a vendor-specific driver
    /// (like Waveshare) creates the DPI panel internally.
    pub fn set_dpi_panel(
        self,
        dpi_panel: PanelHandle,
    ) -> Result<LcdDriver<'d, WithDpiPanel>, EspError> {
        let handle = NonNull::new(dpi_panel as *mut c_void)
            .ok_or(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;

        // Create semaphore + callback so draw_bitmap can wait for DMA completion
        let sem = create_binary_semaphore()?;
        if let Err(e) = register_draw_done_cb(dpi_panel, sem) {
            unsafe { vQueueDelete(sem) };
            return Err(e);
        }

        // Prevent Drop from running on self
        let this = ManuallyDrop::new(self);

        unsafe {
            Ok(LcdDriver {
                bus_handle: this.bus_handle,
                dbi_io_handle: this.dbi_io_handle,
                control_panel_handle: this.control_panel_handle,
                dpi_panel: Some(DpiPanel {
                    handle,
                    config_box: None, // not created by us
                    draw_done_sem: sem,
                }),
                config: core::ptr::read(&this.config),
                _p: PhantomData,
                _state: PhantomData,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// WithDpiPanel methods
// ---------------------------------------------------------------------------

impl<'d> LcdDriver<'d, WithDpiPanel> {
    /// Get the DPI panel handle
    pub fn panel_handle(&self) -> PanelHandle {
        // Safety: in WithDpiPanel state, dpi_panel is always Some
        self.dpi_panel.as_ref().unwrap().handle.as_ptr().cast()
    }

    /// Get raw pointers to the DPI panel's internal frame buffer(s).
    ///
    /// Returns up to `N` frame buffer pointers (where `N` = [`num_fbs`](LcdConfig::num_fbs)
    /// at config time). Each buffer is laid out in row-major order with no
    /// padding: `offset = (y * width + x) * bytes_per_pixel`.
    ///
    /// With a single buffer, the DPI peripheral continuously scans it;
    /// writing directly may cause tearing. With two or more buffers, write
    /// to the back buffer and swap via [`draw_bitmap()`](Self::draw_bitmap)
    /// for tear-free updates.
    ///
    /// For a higher-level API, see [`display()`](Self::display) which
    /// handles double buffering automatically.
    pub fn frame_buffer(&self) -> Result<*mut u8, EspError> {
        let mut fb: *mut c_void = core::ptr::null_mut();
        unsafe {
            esp!(esp_lcd_dpi_panel_get_frame_buffer(
                self.panel_handle(),
                1,
                &mut fb,
            ))?;
        }
        Ok(fb as *mut u8)
    }

    /// Get both frame buffer pointers for double buffering.
    ///
    /// Requires [`num_fbs`](LcdConfig::num_fbs) >= 2 at config time.
    pub fn frame_buffers(&self) -> Result<(*mut u8, *mut u8), EspError> {
        let mut fb0: *mut c_void = core::ptr::null_mut();
        let mut fb1: *mut c_void = core::ptr::null_mut();
        unsafe {
            esp!(esp_lcd_dpi_panel_get_frame_buffer(
                self.panel_handle(),
                2,
                &mut fb0,
                &mut fb1,
            ))?;
        }
        Ok((fb0 as *mut u8, fb1 as *mut u8))
    }

    /// Draw a bitmap to the display (asynchronous DMA2D transfer).
    ///
    /// Starts a DMA2D transfer and returns immediately. The caller **must**
    /// ensure that `data` remains valid until the transfer completes.
    /// Call [`wait_for_draw()`](Self::wait_for_draw) to block until the
    /// transfer is finished.
    ///
    /// # Coordinate System
    ///
    /// Uses **half-open intervals**: `[x1, x2) × [y1, y2)`.
    /// For a 1024×600 display, use `x2=1024, y2=600` (not 1023, 599).
    ///
    /// # Bitmap Layout
    ///
    /// The bitmap must be laid out in **row-major order** with:
    /// - Width = `(x2 - x1)` pixels per row
    /// - No padding or stride between rows
    ///
    /// The total buffer size depends on the pixel format:
    /// - RGB888/BGR888: `(x2 - x1) * (y2 - y1) * 3` bytes
    /// - RGB565/BGR565: `(x2 - x1) * (y2 - y1) * 2` bytes
    /// - RGB666/BGR666: `(x2 - x1) * (y2 - y1) * 3` bytes
    pub fn draw_bitmap(
        &self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        data: &[u8],
    ) -> Result<(), EspError> {
        unsafe {
            esp!(esp_lcd_panel_draw_bitmap(
                self.panel_handle(),
                x1,
                y1,
                x2,
                y2,
                data.as_ptr() as *const c_void,
            ))
        }
    }

    /// Block until the previous [`draw_bitmap()`](Self::draw_bitmap) DMA2D
    /// transfer finishes copying the caller's buffer into the internal frame
    /// buffer.
    ///
    /// After this returns it is safe to free or reuse the buffer that was
    /// passed to `draw_bitmap()`, and to issue the next `draw_bitmap()` call.
    pub fn wait_for_draw(&self) {
        let dpi = self.dpi_panel.as_ref().unwrap();
        unsafe {
            xQueueSemaphoreTake(dpi.draw_done_sem, u32::MAX);
        }
    }

    /// Invert the display colors
    pub fn invert_colors(&self, invert: bool) -> Result<(), EspError> {
        unsafe { esp!(esp_lcd_panel_invert_color(self.panel_handle(), invert)) }
    }

    /// Mirror the display horizontally
    pub fn mirror_x(&self, mirror: bool) -> Result<(), EspError> {
        unsafe { esp!(esp_lcd_panel_mirror(self.panel_handle(), mirror, false)) }
    }

    /// Mirror the display vertically
    pub fn mirror_y(&self, mirror: bool) -> Result<(), EspError> {
        unsafe { esp!(esp_lcd_panel_mirror(self.panel_handle(), false, mirror)) }
    }

    /// Swap the X and Y axes
    pub fn swap_xy(&self, swap: bool) -> Result<(), EspError> {
        unsafe { esp!(esp_lcd_panel_swap_xy(self.panel_handle(), swap)) }
    }

    /// Set the gap between pixels
    pub fn set_gap(&self, x_gap: i32, y_gap: i32) -> Result<(), EspError> {
        unsafe { esp!(esp_lcd_panel_set_gap(self.panel_handle(), x_gap, y_gap)) }
    }

    /// Create an `embedded-graphics` [`DrawTarget`] wrapper for this driver.
    ///
    /// With two or more frame buffers ([`num_fbs`](LcdConfig::num_fbs) >= 2),
    /// the wrapper uses double buffering: all draw calls write to an
    /// invisible back buffer, and [`flush()`](eg::LcdDisplay::flush) swaps
    /// it to the screen atomically (no tearing).
    ///
    /// With a single frame buffer, draws go directly to the live buffer
    /// (faster, but may tear).
    ///
    /// The color type `C` must match the pixel format configured in [`LcdConfig`].
    #[cfg(feature = "embedded-graphics")]
    pub fn display<C: eg::LcdColor>(&mut self) -> Result<eg::LcdDisplay<'_, 'd, C>, EspError> {
        let num_fbs = self.config.dpi_config.num_fbs;
        if num_fbs >= 2 {
            let (fb0, fb1) = self.frame_buffers()?;
            Ok(eg::LcdDisplay {
                driver: self,
                fb: fb1,       // back buffer — draw here
                fb_front: fb0, // front buffer — currently displayed
                _color: PhantomData,
            })
        } else {
            let fb = self.frame_buffer()?;
            Ok(eg::LcdDisplay {
                driver: self,
                fb,
                fb_front: core::ptr::null_mut(), // single-buffer mode
                _color: PhantomData,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// Drop
// ---------------------------------------------------------------------------

impl<'d, S: DpiPanelState> Drop for LcdDriver<'d, S> {
    fn drop(&mut self) {
        unsafe {
            // Clean up DPI panel
            if let Some(ref dpi) = self.dpi_panel {
                let _ = esp_lcd_panel_del(dpi.handle.as_ptr().cast());
                if let Some(config_ptr) = dpi.config_box {
                    if !config_ptr.is_null() {
                        let _ = Box::from_raw(config_ptr);
                    }
                }
                if !dpi.draw_done_sem.is_null() {
                    vQueueDelete(dpi.draw_done_sem);
                }
            }

            // Clean up control panel
            if let Some(control) = self.control_panel_handle {
                if !control.is_null() {
                    let _ = esp_lcd_panel_del(control);
                }
            }

            // Clean up DBI IO
            let _ = esp_lcd_panel_io_del(self.dbi_io_handle.as_ptr().cast());

            // Clean up DSI bus
            let _ = esp_lcd_del_dsi_bus(self.bus_handle.as_ptr().cast());
        }
    }
}

unsafe impl<'d, S: DpiPanelState> Send for LcdDriver<'d, S> {}

// ---------------------------------------------------------------------------
// embedded-graphics support
// ---------------------------------------------------------------------------

#[cfg(feature = "embedded-graphics")]
pub mod eg {
    use super::*;

    use embedded_graphics_core::draw_target::DrawTarget;
    use embedded_graphics_core::geometry::{OriginDimensions, Size};
    use embedded_graphics_core::pixelcolor::{
        Bgr565, Bgr888, PixelColor, Rgb565, Rgb888, RgbColor,
    };
    use embedded_graphics_core::primitives::Rectangle;
    use embedded_graphics_core::Pixel;

    /// Trait for pixel color types usable with the LCD driver.
    ///
    /// Maps an `embedded-graphics` color type to the corresponding LCD [`PixelFormat`]
    /// and provides byte-level conversion.
    pub trait LcdColor: PixelColor + RgbColor {
        /// The LCD pixel format corresponding to this color type.
        fn pixel_format() -> PixelFormat;

        /// Bytes per pixel.
        fn bytes_per_pixel() -> usize;

        /// Write this pixel's bytes to `buf` at the given byte offset.
        fn write_to(&self, buf: &mut [u8], offset: usize);
    }

    impl LcdColor for Rgb888 {
        fn pixel_format() -> PixelFormat {
            PixelFormat::Rgb888
        }
        fn bytes_per_pixel() -> usize {
            3
        }
        fn write_to(&self, buf: &mut [u8], offset: usize) {
            buf[offset] = self.r();
            buf[offset + 1] = self.g();
            buf[offset + 2] = self.b();
        }
    }

    impl LcdColor for Bgr888 {
        fn pixel_format() -> PixelFormat {
            PixelFormat::Bgr888
        }
        fn bytes_per_pixel() -> usize {
            3
        }
        fn write_to(&self, buf: &mut [u8], offset: usize) {
            buf[offset] = self.b();
            buf[offset + 1] = self.g();
            buf[offset + 2] = self.r();
        }
    }

    impl LcdColor for Rgb565 {
        fn pixel_format() -> PixelFormat {
            PixelFormat::Rgb565
        }
        fn bytes_per_pixel() -> usize {
            2
        }
        fn write_to(&self, buf: &mut [u8], offset: usize) {
            // Pack as RRRRRGGG_GGGBBBBB using the logical color components
            let r = (self.r() >> 3) as u16;
            let g = (self.g() >> 2) as u16;
            let b = (self.b() >> 3) as u16;
            let raw: u16 = (r << 11) | (g << 5) | b;
            buf[offset..offset + 2].copy_from_slice(&raw.to_ne_bytes());
        }
    }

    impl LcdColor for Bgr565 {
        fn pixel_format() -> PixelFormat {
            PixelFormat::Bgr565
        }
        fn bytes_per_pixel() -> usize {
            2
        }
        fn write_to(&self, buf: &mut [u8], offset: usize) {
            // Pack as BBBBBGGG_GGGRRRRR
            let b = (self.b() >> 3) as u16;
            let g = (self.g() >> 2) as u16;
            let r = (self.r() >> 3) as u16;
            let raw: u16 = (b << 11) | (g << 5) | r;
            buf[offset..offset + 2].copy_from_slice(&raw.to_ne_bytes());
        }
    }

    /// `embedded-graphics` [`DrawTarget`] wrapper for [`LcdDriver`].
    ///
    /// Created via [`LcdDriver::display()`]. Writes pixels directly to the
    /// DPI panel's internal frame buffer for maximum throughput (no DMA
    /// overhead per draw call).
    ///
    /// With double buffering ([`num_fbs`](LcdConfig::num_fbs) >= 2), all
    /// draws go to an invisible back buffer. Call [`flush()`](Self::flush)
    /// after composing each frame to swap it to the screen atomically.
    ///
    /// The color type `C` must match the pixel format configured in [`LcdConfig`].
    pub struct LcdDisplay<'a, 'd, C: LcdColor> {
        pub(crate) driver: &'a mut LcdDriver<'d, WithDpiPanel>,
        /// Back buffer — all draws go here.
        pub(crate) fb: *mut u8,
        /// Front buffer (null in single-buffer mode).
        pub(crate) fb_front: *mut u8,
        pub(crate) _color: PhantomData<C>,
    }

    impl<C: LcdColor> OriginDimensions for LcdDisplay<'_, '_, C> {
        fn size(&self) -> Size {
            let timing = &self.driver.config().dpi_config().video_timing;
            Size::new(timing.h_size, timing.v_size)
        }
    }

    impl<C: LcdColor> LcdDisplay<'_, '_, C> {
        /// Return the back buffer as a mutable byte slice.
        fn fb_slice(&mut self) -> &mut [u8] {
            let size = self.size();
            let len = size.width as usize * size.height as usize * C::bytes_per_pixel();
            unsafe { core::slice::from_raw_parts_mut(self.fb, len) }
        }

        /// Total framebuffer size in bytes.
        fn fb_size(&self) -> usize {
            let size = self.size();
            size.width as usize * size.height as usize * C::bytes_per_pixel()
        }

        /// Returns `true` if double buffering is active.
        pub fn is_double_buffered(&self) -> bool {
            !self.fb_front.is_null()
        }

        /// Swap the back buffer to the screen.
        ///
        /// In double-buffer mode, this tells the DPI driver to display the
        /// back buffer (the driver handles cache sync internally) and then
        /// switches the draw target to the other buffer.
        ///
        /// In single-buffer mode, this flushes the CPU cache so the DPI
        /// peripheral sees the updated pixels.
        pub fn flush(&mut self) -> Result<(), EspError> {
            if self.is_double_buffered() {
                let size = self.size();
                let fb_size = self.fb_size();
                let back_buf = unsafe { core::slice::from_raw_parts(self.fb, fb_size) };

                // The DPI driver detects this is an internal framebuffer
                // and does a zero-copy pointer swap + cache sync.
                self.driver
                    .draw_bitmap(0, 0, size.width as i32, size.height as i32, back_buf)?;
                self.driver.wait_for_draw();

                // Swap: old back becomes front, old front becomes back.
                core::mem::swap(&mut self.fb, &mut self.fb_front);
            } else {
                // Single-buffer mode: no-op for now. Cache sync requires
                // esp_cache_msync which needs an upstream esp-idf-sys change.
            }
            Ok(())
        }
    }

    impl<C: LcdColor> DrawTarget for LcdDisplay<'_, '_, C> {
        type Color = C;
        type Error = EspError;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            let bpp = C::bytes_per_pixel();
            let size = self.size();
            let stride = size.width as usize * bpp;
            let fb = self.fb_slice();

            for Pixel(point, color) in pixels {
                if point.x >= 0
                    && point.y >= 0
                    && (point.x as u32) < size.width
                    && (point.y as u32) < size.height
                {
                    let offset = point.y as usize * stride + point.x as usize * bpp;
                    C::write_to(&color, fb, offset);
                }
            }
            Ok(())
        }

        fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Self::Color>,
        {
            let bounding =
                Rectangle::new(embedded_graphics_core::geometry::Point::zero(), self.size());

            let area = area.intersection(&bounding);
            let width = area.size.width as usize;
            let height = area.size.height as usize;

            if width == 0 || height == 0 {
                return Ok(());
            }

            let bpp = C::bytes_per_pixel();
            let stride = self.size().width as usize * bpp;
            let x_start = area.top_left.x as usize;
            let y_start = area.top_left.y as usize;
            let fb = self.fb_slice();

            let mut i = 0;
            for color in colors.into_iter().take(width * height) {
                let px = i % width;
                let py = i / width;
                let offset = (y_start + py) * stride + (x_start + px) * bpp;
                C::write_to(&color, fb, offset);
                i += 1;
            }
            Ok(())
        }

        fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
            let bounding =
                Rectangle::new(embedded_graphics_core::geometry::Point::zero(), self.size());

            let area = area.intersection(&bounding);
            let width = area.size.width as usize;
            let height = area.size.height as usize;

            if width == 0 || height == 0 {
                return Ok(());
            }

            let bpp = C::bytes_per_pixel();
            let stride = self.size().width as usize * bpp;
            let x_start = area.top_left.x as usize;
            let y_start = area.top_left.y as usize;

            // Build one row of identical pixels
            let row_bytes = width * bpp;
            let mut row = alloc::vec![0u8; row_bytes];
            for i in 0..width {
                C::write_to(&color, &mut row, i * bpp);
            }

            // Copy the row pattern directly into the framebuffer
            let fb = self.fb_slice();
            for y in 0..height {
                let offset = (y_start + y) * stride + x_start * bpp;
                fb[offset..offset + row_bytes].copy_from_slice(&row);
            }
            Ok(())
        }

        fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
            let size = self.size();
            let rect = Rectangle::new(embedded_graphics_core::geometry::Point::zero(), size);
            self.fill_solid(&rect, color)
        }
    }
}
