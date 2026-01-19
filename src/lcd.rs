//! LCD peripheral control
//!
//! Interface to the [LCD (Liquid Crystal Display)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/lcd.html)
//! peripheral, supporting DSI (Display Serial Interface) panels.
//!
//! This module provides safe Rust wrappers for the LCD API, which is available
//! on ESP32-P4 and later chips.
//!
//! # Example
//!
//! Initialize a DSI LCD panel and draw a bitmap:
//! ```
//! use esp_idf_hal::lcd::*;
//! use esp_idf_hal::peripherals::Peripherals;
//!
//! let peripherals = Peripherals::take()?;
//!
//! // Configure LCD
//! let video_timing = VideoTiming::new(1024, 600)
//!     .hsync_back_porch(100)
//!     .hsync_pulse_width(100)
//!     .hsync_front_porch(100)
//!     .vsync_back_porch(10)
//!     .vsync_pulse_width(10)
//!     .vsync_front_porch(10);
//!
//! let config = LcdConfig::new(video_timing)
//!     .num_data_lanes(2)
//!     .lane_bit_rate_mbps(1250)
//!     .dpi_clock_freq_mhz(48)
//!     .pixel_format(PixelFormat::Rgb888);
//!
//! use esp_idf_sys::*;
//!
//! // Create driver with DSI bus and DBI IO
//! let mut lcd = LcdDriver::new(peripherals.lcd, &config)?;
//!
//! // Create vendor control panel
//! let mut dev_config: esp_lcd_panel_dev_config_t = lcd.config().into();
//! dev_config.reset_gpio_num = 5;
//! let mut control_panel: esp_lcd_panel_handle_t = core::ptr::null_mut();
//! unsafe {
//!     esp!(esp_lcd_new_panel_ili9881c(lcd.dbi_io_handle(), &dev_config, &mut control_panel))?;
//! }
//! lcd.set_control_panel(control_panel)?;
//!
//! // Initialize the panel
//! lcd.reset()?;
//! lcd.init()?;
//! lcd.set_display_on(true)?;
//!
//! // Create DPI panel for drawing (uses config from new())
//! lcd.create_dpi_panel()?;
//!
//! // Draw a bitmap
//! // NOTE: draw_bitmap uses half-open intervals [x1, x2) × [y1, y2),
//! // meaning x2 and y2 are NOT inclusive. For a 1024×600 display,
//! // pass x2=1024, y2=600 (not 1023, 599) to cover the full screen.
//! // The framebuffer must be laid out row-major with width = (x2 - x1) pixels.
//! let framebuffer = vec![0u8; 1024 * 600 * 3]; // RGB888: 3 bytes per pixel
//! lcd.draw_bitmap(0, 0, 1024, 600, &framebuffer)?;
//! ```

#[cfg(not(feature = "alloc"))]
compile_error!("LCD module requires the `alloc` feature to be enabled");

use core::ffi::c_void;
use core::marker::PhantomData;

extern crate alloc;

use alloc::boxed::Box;

use crate::sys::EspError;
use esp_idf_sys::*;

/// LCD configuration types
pub mod config {
    use super::*;

    /// Complete LCD configuration
    ///
    /// This single struct consolidates all LCD-related configurations. It can produce
    /// the underlying ESP-IDF C structs as needed, eliminating the need for separate
    /// config structs for DSI bus, DBI IO, and DPI panel.
    #[derive(Debug, Clone)]
    pub struct LcdConfig {
        // DSI bus settings
        pub bus_id: i32,
        pub num_data_lanes: u8,
        pub phy_clk_src: u32,
        pub lane_bit_rate_mbps: u32,

        // DBI IO settings
        pub virtual_channel: u8,
        pub lcd_cmd_bits: i32,
        pub lcd_param_bits: i32,

        // DPI panel settings
        pub dpi_clk_src: u32,
        pub pixel_format: PixelFormat,
        pub num_fbs: u8,
        pub video_timing: VideoTiming,
        pub dpi_clock_freq_mhz: u32,
        pub use_dma2d: bool,
    }

    impl LcdConfig {
        /// Create a new LCD configuration with default values
        ///
        /// # Arguments
        ///
        /// * `video_timing` - Video timing configuration (resolution and sync timings)
        pub fn new(video_timing: VideoTiming) -> Self {
            Self {
                // DSI bus defaults
                bus_id: 0,
                num_data_lanes: 2,
                phy_clk_src: soc_module_clk_t_SOC_MOD_CLK_PLL_F160M as u32,
                lane_bit_rate_mbps: 1250,

                // DBI IO defaults
                virtual_channel: 0,
                lcd_cmd_bits: 8,
                lcd_param_bits: 8,

                // DPI panel defaults
                dpi_clk_src: soc_periph_mipi_dsi_dpi_clk_src_t_MIPI_DSI_DPI_CLK_SRC_DEFAULT,
                pixel_format: PixelFormat::Rgb888,
                num_fbs: 1,
                video_timing,
                dpi_clock_freq_mhz: 48,
                use_dma2d: true,
            }
        }

        // Builder methods for DSI bus settings
        #[must_use]
        pub fn bus_id(mut self, bus_id: i32) -> Self {
            self.bus_id = bus_id;
            self
        }

        #[must_use]
        pub fn num_data_lanes(mut self, num_data_lanes: u8) -> Self {
            self.num_data_lanes = num_data_lanes;
            self
        }

        #[must_use]
        pub fn phy_clk_src(mut self, phy_clk_src: u32) -> Self {
            self.phy_clk_src = phy_clk_src;
            self
        }

        #[must_use]
        pub fn lane_bit_rate_mbps(mut self, lane_bit_rate_mbps: u32) -> Self {
            self.lane_bit_rate_mbps = lane_bit_rate_mbps;
            self
        }

        // Builder methods for DBI IO settings
        #[must_use]
        pub fn virtual_channel(mut self, virtual_channel: u8) -> Self {
            self.virtual_channel = virtual_channel;
            self
        }

        #[must_use]
        pub fn lcd_cmd_bits(mut self, lcd_cmd_bits: i32) -> Self {
            self.lcd_cmd_bits = lcd_cmd_bits;
            self
        }

        #[must_use]
        pub fn lcd_param_bits(mut self, lcd_param_bits: i32) -> Self {
            self.lcd_param_bits = lcd_param_bits;
            self
        }

        // Builder methods for DPI panel settings
        #[must_use]
        pub fn dpi_clk_src(mut self, dpi_clk_src: u32) -> Self {
            self.dpi_clk_src = dpi_clk_src;
            self
        }

        #[must_use]
        pub fn pixel_format(mut self, pixel_format: PixelFormat) -> Self {
            self.pixel_format = pixel_format;
            self
        }

        #[must_use]
        pub fn num_fbs(mut self, num_fbs: u8) -> Self {
            self.num_fbs = num_fbs;
            self
        }

        #[must_use]
        pub fn dpi_clock_freq_mhz(mut self, dpi_clock_freq_mhz: u32) -> Self {
            self.dpi_clock_freq_mhz = dpi_clock_freq_mhz;
            self
        }

        #[must_use]
        pub fn use_dma2d(mut self, use_dma2d: bool) -> Self {
            self.use_dma2d = use_dma2d;
            self
        }
    }

    impl From<&LcdConfig> for esp_lcd_panel_dev_config_t {
        fn from(config: &LcdConfig) -> Self {
            let bits_per_pixel = config.pixel_format.bits_per_pixel();
            let rgb_order = config.pixel_format.rgb_element_order();

            // Zero-initialize the struct to handle unions/bitfields properly
            let mut panel_config: esp_lcd_panel_dev_config_t = unsafe { core::mem::zeroed() };

            // Set non-union fields
            // reset_gpio_num, data_endian, vendor_config, and flags are left as defaults
            // (zeroed values). Users should set these directly on the struct if needed.
            panel_config.bits_per_pixel = bits_per_pixel;

            // Set union fields through the union accessor
            // rgb_ele_order is in __bindgen_anon_1 union
            // Note: pixel_format is converted via method since both format types alias u32
            panel_config.__bindgen_anon_1.rgb_ele_order = rgb_order.into();

            panel_config
        }
    }

    impl From<&LcdConfig> for esp_lcd_dpi_panel_config_t {
        fn from(config: &LcdConfig) -> Self {
            let video_timing: esp_lcd_video_timing_t = config.video_timing.into();
            let color_format: lcd_color_format_t = config.pixel_format.to_color_format();

            let mut dpi_config = esp_lcd_dpi_panel_config_t {
                virtual_channel: config.virtual_channel,
                dpi_clk_src: config.dpi_clk_src,
                pixel_format: config.pixel_format.to_rgb_pixel_format(), // deprecated but set for compatibility
                in_color_format: color_format,
                out_color_format: color_format, // Output format matches input format
                num_fbs: config.num_fbs,
                video_timing,
                dpi_clock_freq_mhz: config.dpi_clock_freq_mhz,
                ..unsafe { core::mem::zeroed() }
            };

            // Set flags.use_dma2d
            dpi_config.flags.set_use_dma2d(config.use_dma2d as u32);

            dpi_config
        }
    }

    impl From<&LcdConfig> for esp_lcd_dsi_bus_config_t {
        fn from(config: &LcdConfig) -> Self {
            esp_lcd_dsi_bus_config_t {
                bus_id: config.bus_id,
                num_data_lanes: config.num_data_lanes,
                phy_clk_src: config.phy_clk_src,
                lane_bit_rate_mbps: config.lane_bit_rate_mbps,
            }
        }
    }

    impl From<&LcdConfig> for esp_lcd_dbi_io_config_t {
        fn from(config: &LcdConfig) -> Self {
            esp_lcd_dbi_io_config_t {
                virtual_channel: config.virtual_channel,
                lcd_cmd_bits: config.lcd_cmd_bits,
                lcd_param_bits: config.lcd_param_bits,
            }
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
    }

    impl PixelFormat {
        /// Convert to `lcd_color_rgb_pixel_format_t` (for panel device config)
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

        /// Convert to `lcd_color_format_t` (for DPI panel config)
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

crate::impl_peripheral!(LCD);

/// LCD driver
pub struct LcdDriver<'d> {
    panel_handle: Option<esp_lcd_panel_handle_t>, // DPI panel handle (None if not yet created)
    bus_handle: esp_lcd_dsi_bus_handle_t,
    dbi_io_handle: esp_lcd_panel_io_handle_t,
    control_panel_handle: Option<esp_lcd_panel_handle_t>,
    dpi_config: Option<*mut esp_lcd_dpi_panel_config_t>, // DPI config pointer (None if DPI panel not created)
    config: LcdConfig, // Stored config for later use (e.g., create_dpi_panel)
    _p: PhantomData<&'d mut ()>,
}

impl<'d> LcdDriver<'d> {
    /// Create a new DSI LCD driver
    ///
    /// This initializes the DSI bus and DBI IO interface. After creation, you can:
    /// 1. Optionally set a vendor-specific control panel using `set_control_panel()`
    /// 2. Create the DPI panel using `create_dpi_panel()` (or set it if vendor provided one)
    ///
    /// The configuration is stored in the driver and used when creating the DPI panel.
    ///
    /// This method follows the ESP-IDF initialization flow:
    /// 1. Create DSI bus (`esp_lcd_new_dsi_bus`)
    /// 2. Create DBI IO (`esp_lcd_new_panel_io_dbi`) - for sending commands
    ///
    /// # Arguments
    ///
    /// * `config` - Complete LCD configuration (bus, DBI, and DPI settings)
    ///
    /// # Example
    ///
    /// Standard initialization with vendor control panel:
    /// ```no_run
    /// use esp_idf_hal::lcd::*;
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
    /// let config = LcdConfig::new(video_timing)
    ///     .num_data_lanes(2)
    ///     .lane_bit_rate_mbps(1250)
    ///     .dpi_clock_freq_mhz(48)
    ///     .pixel_format(PixelFormat::Rgb888);
    ///
    /// // Create driver with DSI bus and DBI IO
    /// let mut driver = LcdDriver::new(peripherals.lcd, &config)?;
    ///
    /// // Create vendor control panel
    /// let mut dev_config: esp_lcd_panel_dev_config_t = driver.config().into();
    /// dev_config.reset_gpio_num = 5; // Set reset pin if needed
    /// let mut control_panel: esp_lcd_panel_handle_t = core::ptr::null_mut();
    /// unsafe {
    ///     esp!(esp_lcd_new_panel_ili9881c(driver.dbi_io_handle(), &dev_config, &mut control_panel))?;
    /// }
    /// driver.set_control_panel(control_panel)?;
    ///
    /// // Create DPI panel (uses config from new())
    /// driver.create_dpi_panel()?;
    /// ```
    ///
    /// Vendor driver that creates both panels (e.g., Waveshare):
    /// ```no_run
    /// let mut driver = LcdDriver::new(peripherals.lcd, &config)?;
    ///
    /// let panel_dev_config: esp_lcd_panel_dev_config_t = driver.config().into();
    /// let (control_panel, dpi_panel) = create_waveshare_driver(
    ///     driver.bus_handle(),
    ///     driver.dbi_io_handle(),
    ///     &panel_dev_config
    /// )?;
    ///
    /// driver.set_control_panel(control_panel)?;
    /// driver.set_dpi_panel(dpi_panel)?;
    /// ```
    pub fn new(_lcd: LCD<'d>, config: &LcdConfig) -> Result<Self, EspError> {
        unsafe {
            // Create DSI bus configuration from consolidated config
            let sys_bus_config: esp_lcd_dsi_bus_config_t = config.into();

            // Validate configuration before calling ESP-IDF function
            if sys_bus_config.num_data_lanes == 0 || sys_bus_config.num_data_lanes > 4 {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
            }
            if sys_bus_config.lane_bit_rate_mbps == 0 {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
            }

            // Create DSI bus handle
            let mut bus_handle: esp_lcd_dsi_bus_handle_t = core::ptr::null_mut();
            let esp_result = esp_lcd_new_dsi_bus(&sys_bus_config, &mut bus_handle);
            if esp_result != ESP_OK {
                let err = EspError::from(esp_result as i32)
                    .unwrap_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
                return Err(err);
            }

            if bus_handle.is_null() {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
            }

            // Create DBI IO interface for sending commands to the panel
            // This is step 2 in the ESP-IDF documentation flow
            // The DBI IO handle is used to create vendor-specific control panels
            let sys_dbi_config: esp_lcd_dbi_io_config_t = config.into();

            let mut mipi_dbi_io: esp_lcd_panel_io_handle_t = core::ptr::null_mut();
            esp!(esp_lcd_new_panel_io_dbi(
                bus_handle,
                &sys_dbi_config,
                &mut mipi_dbi_io
            ))?;

            if mipi_dbi_io.is_null() {
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
            }

            Ok(Self {
                panel_handle: None,
                bus_handle,
                dbi_io_handle: mipi_dbi_io,
                control_panel_handle: None,
                dpi_config: None,
                config: config.clone(),
                _p: PhantomData,
            })
        }
    }

    /// Get the raw panel handle
    ///
    /// # Note
    ///
    /// This exposes the underlying ESP-IDF DPI panel handle for advanced use cases.
    /// Returns `None` if no DPI panel has been set (can be set via `set_dpi_panel()`).
    /// Prefer using the HAL methods when possible.
    pub fn panel_handle(&self) -> Option<esp_lcd_panel_handle_t> {
        self.panel_handle
    }

    /// Get the raw bus handle
    ///
    /// # Note
    ///
    /// This exposes the underlying ESP-IDF handle for advanced use cases.
    /// Useful for vendor-specific drivers that need the DSI bus handle.
    pub fn bus_handle(&self) -> esp_lcd_dsi_bus_handle_t {
        self.bus_handle
    }

    /// Get the raw DBI IO handle
    ///
    /// # Note
    ///
    /// This exposes the underlying ESP-IDF handle for advanced use cases.
    /// The DBI IO handle can be used to send commands to the LCD panel controller
    /// (e.g., for manufacturer-specific initialization sequences).
    ///
    /// This handle is typically used to create vendor-specific control panels
    /// (e.g., `esp_lcd_new_panel_ili9881c`) before creating the DPI panel.
    pub fn dbi_io_handle(&self) -> esp_lcd_panel_io_handle_t {
        self.dbi_io_handle
    }

    /// Get a reference to the stored configuration
    ///
    /// This is useful for vendor-specific drivers that need to convert the config
    /// to C structs (e.g., `esp_lcd_panel_dev_config_t`).
    ///
    /// # Example
    ///
    /// ```no_run
    /// let panel_dev_config: esp_lcd_panel_dev_config_t = driver.config().into();
    /// ```
    pub fn config(&self) -> &LcdConfig {
        &self.config
    }

    /// Transmit LCD command and corresponding parameters via DBI IO
    ///
    /// This function sends a command and its parameters to the LCD panel controller
    /// through the DBI IO interface. Commands are sent using polling transactions
    /// and the function waits for completion before returning.
    ///
    /// If any queued transactions sent by `tx_color()` are still pending when this
    /// function is called, it will wait until they are finished and the queue is
    /// empty before sending the command(s).
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `param` - Buffer containing the command parameters, or an empty slice if no parameters
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Send a command with parameters
    /// let params = [0x01, 0x02, 0x03];
    /// driver.tx_param(0x2A, &params)?;
    ///
    /// // Send a command without parameters
    /// driver.tx_param(0x29, &[])?;
    ///
    /// // Send only parameters (no command)
    /// driver.tx_param(-1, &params)?;
    /// ```
    pub fn tx_param(&self, lcd_cmd: i32, param: &[u8]) -> Result<(), EspError> {
        unsafe {
            let param_ptr = if param.is_empty() {
                core::ptr::null()
            } else {
                param.as_ptr() as *const c_void
            };
            esp!(esp_lcd_panel_io_tx_param(
                self.dbi_io_handle,
                lcd_cmd,
                param_ptr,
                param.len(),
            ))?;
        }
        Ok(())
    }

    /// Receive LCD command parameters via DBI IO
    ///
    /// This function sends a command and receives the corresponding parameters
    /// from the LCD panel controller through the DBI IO interface. Commands are
    /// sent using polling transactions and the function waits for completion.
    ///
    /// If any queued transactions sent by `tx_color()` are still pending when this
    /// function is called, it will wait until they are finished and the queue is
    /// empty before sending the command(s).
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `param` - Buffer to receive the command parameters
    ///
    /// # Returns
    ///
    /// Returns an error if read is not supported by the transport or if parameters
    /// are invalid.
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Read parameters for a command
    /// let mut params = [0u8; 4];
    /// driver.rx_param(0x0A, &mut params)?;
    /// ```
    pub fn rx_param(&self, lcd_cmd: i32, param: &mut [u8]) -> Result<(), EspError> {
        unsafe {
            esp!(esp_lcd_panel_io_rx_param(
                self.dbi_io_handle,
                lcd_cmd,
                param.as_mut_ptr() as *mut c_void,
                param.len(),
            ))?;
        }
        Ok(())
    }

    /// Transmit LCD color data via DBI IO
    ///
    /// This function sends color data to the LCD panel controller through the DBI IO
    /// interface. Color data transfers are queued and may be asynchronous, allowing
    /// multiple transfers to be queued.
    ///
    /// # Arguments
    ///
    /// * `lcd_cmd` - The specific LCD command, or `-1` if no command is needed
    /// * `color` - Buffer containing the color data to transmit
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Send color data with a command
    /// let color_data = [0xFF, 0x00, 0x00]; // Red pixel
    /// driver.tx_color(0x2C, &color_data)?;
    ///
    /// // Send color data without a command
    /// driver.tx_color(-1, &color_data)?;
    /// ```
    pub fn tx_color(&self, lcd_cmd: i32, color: &[u8]) -> Result<(), EspError> {
        unsafe {
            esp!(esp_lcd_panel_io_tx_color(
                self.dbi_io_handle,
                lcd_cmd,
                color.as_ptr() as *const c_void,
                color.len(),
            ))?;
        }
        Ok(())
    }

    /// Set the control panel handle
    ///
    /// This **must** be called before using any control panel operations (reset, init, etc.).
    /// The control panel handle is typically created using vendor-specific functions
    /// (e.g., `esp_lcd_new_panel_ili9881c`) with the DBI IO handle from `dbi_io_handle()`.
    /// Use `config()` to get the configuration for converting to `esp_lcd_panel_dev_config_t`.
    /// This method also allows you to change the control panel if needed.
    ///
    /// # Arguments
    ///
    /// * `control_panel` - The control panel handle to use for initialization and control operations.
    ///   Must not be null.
    ///
    /// # Returns
    ///
    /// Returns an error if the control panel handle is null, or the previous control panel handle
    /// if one was set (or `None` if this is the first time).
    pub fn set_control_panel(
        &mut self,
        control_panel: esp_lcd_panel_handle_t,
    ) -> Result<Option<esp_lcd_panel_handle_t>, EspError> {
        if control_panel.is_null() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
        }
        Ok(self.control_panel_handle.replace(control_panel))
    }

    /// Get the raw control panel handle
    ///
    /// # Note
    ///
    /// This exposes the underlying ESP-IDF control panel handle if one has been set.
    ///
    /// The control panel handle is used for vendor-specific initialization and
    /// control operations, while the DPI panel (from `panel_handle()`) is used
    /// for pixel data operations.
    ///
    /// Returns `None` if no control panel has been set yet.
    pub fn control_panel_handle(&self) -> Option<esp_lcd_panel_handle_t> {
        self.control_panel_handle
    }

    /// Create a DPI panel for drawing operations
    ///
    /// This should be called after the vendor-specific control panel has been set up
    /// and initialized (after calling `set_control_panel()`, `reset()`, `init()`, and
    /// `set_display_on()`). This follows the ESP-IDF initialization flow where the DPI
    /// panel is created after the control panel initialization.
    ///
    /// Uses the configuration that was provided during `new()`.
    ///
    /// # Returns
    ///
    /// Returns an error if a DPI panel has already been created, or if the DPI panel
    /// creation fails.
    pub fn create_dpi_panel(&mut self) -> Result<(), EspError> {
        if self.panel_handle.is_some() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        unsafe {
            // Create DPI panel configuration from stored config
            // Allocate on heap to ensure it persists for the lifetime of the panel
            let dpi_config = Box::new(esp_lcd_dpi_panel_config_t::from(&self.config));

            // Validate timing values
            assert!(dpi_config.video_timing.h_size > 0, "h_size must be > 0");
            assert!(dpi_config.video_timing.v_size > 0, "v_size must be > 0");
            assert!(
                dpi_config.video_timing.hsync_pulse_width > 0,
                "hsync_pulse_width must be > 0"
            );
            assert!(
                dpi_config.video_timing.vsync_pulse_width > 0,
                "vsync_pulse_width must be > 0"
            );
            assert!(
                dpi_config.dpi_clock_freq_mhz > 0,
                "dpi_clock_freq_mhz must be > 0"
            );

            let dpi_config_ptr = Box::into_raw(dpi_config);

            // Create panel handle using standard ESP-IDF DPI panel API
            // esp_lcd_new_panel_dpi takes the DSI bus handle and DPI config directly
            let mut panel_handle: esp_lcd_panel_handle_t = core::ptr::null_mut();
            let result = esp!(esp_lcd_new_panel_dpi(
                self.bus_handle,
                dpi_config_ptr,
                &mut panel_handle
            ));
            if let Err(e) = result {
                // Clean up on error
                let _ = Box::from_raw(dpi_config_ptr);
                return Err(e);
            }

            if panel_handle.is_null() {
                // Clean up on error
                let _ = Box::from_raw(dpi_config_ptr);
                return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
            }

            self.panel_handle = Some(panel_handle);
            self.dpi_config = Some(dpi_config_ptr);
        }

        Ok(())
    }

    /// Set the DPI panel handle
    ///
    /// This is useful when using vendor-specific drivers (like Waveshare) that create
    /// the DPI panel internally. After creating the vendor driver, you can set the
    /// DPI panel handle here so it can be used for drawing operations.
    ///
    /// This method should be used instead of `create_dpi_panel()` when the vendor driver
    /// handles DPI panel creation internally.
    ///
    /// # Arguments
    ///
    /// * `dpi_panel` - The DPI panel handle created by the vendor driver.
    ///   Must not be null.
    ///
    /// # Returns
    ///
    /// Returns an error if the DPI panel handle is null, or the previous DPI panel handle
    /// if one was set (or `None` if this is the first time).
    pub fn set_dpi_panel(
        &mut self,
        dpi_panel: esp_lcd_panel_handle_t,
    ) -> Result<Option<esp_lcd_panel_handle_t>, EspError> {
        if dpi_panel.is_null() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
        }
        Ok(self.panel_handle.replace(dpi_panel))
    }

    /// Draw a bitmap to the display
    ///
    /// The bitmap data format must match the `pixel_format` specified in the `LcdConfig`
    /// used when creating the driver. The default is RGB888 (3 bytes per pixel), but
    /// other formats like RGB565 (2 bytes) or RGB666 are also supported.
    ///
    /// # Coordinate System
    ///
    /// This function uses **half-open intervals**: `[x1, x2) × [y1, y2)`.
    /// This means:
    /// - `x1` and `y1` are **inclusive** (start coordinates)
    /// - `x2` and `y2` are **exclusive** (end coordinates, not included)
    ///
    /// For example, to draw the full screen of a 1024×600 display:
    /// - Pass `x1=0, y1=0, x2=1024, y2=600` (not `x2=1023, y2=599`)
    ///
    /// # Framebuffer Layout
    ///
    /// The framebuffer must be laid out in **row-major order** with:
    /// - Width = `(x2 - x1)` pixels per row
    /// - Height = `(y2 - y1)` rows
    /// - No padding or stride between rows
    /// - Contiguous pixel data: `[row0_pixel0, row0_pixel1, ..., row0_pixelN, row1_pixel0, ...]`
    ///
    /// The total buffer size depends on the pixel format:
    /// - RGB888/BGR888: `(x2 - x1) * (y2 - y1) * 3` bytes
    /// - RGB565/BGR565: `(x2 - x1) * (y2 - y1) * 2` bytes
    /// - RGB666/BGR666: `(x2 - x1) * (y2 - y1) * 3` bytes (18-bit packed)
    pub fn draw_bitmap(
        &self,
        x1: i32,
        y1: i32,
        x2: i32,
        y2: i32,
        data: &[u8],
    ) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_draw_bitmap(
                panel,
                x1,
                y1,
                x2,
                y2,
                data.as_ptr() as *const c_void,
            ))?;
        }
        Ok(())
    }

    /// Reset the panel
    ///
    /// Uses the control panel for the reset operation.
    /// Requires that `set_control_panel()` has been called first.
    pub fn reset(&self) -> Result<(), EspError> {
        let panel = self
            .control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_reset(panel))?;
        }
        Ok(())
    }

    /// Initialize the panel
    ///
    /// Uses the control panel for the initialization operation.
    /// Requires that `set_control_panel()` has been called first.
    pub fn init(&self) -> Result<(), EspError> {
        let panel = self
            .control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_init(panel))?;
        }
        Ok(())
    }

    /// Turn the display on or off
    ///
    /// Uses the control panel for the display on/off operation.
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_on(&self, on: bool) -> Result<(), EspError> {
        let panel = self
            .control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_disp_on_off(panel, on))?;
        }
        Ok(())
    }

    /// Enter or exit sleep mode (low power mode)
    ///
    /// When in sleep mode, the internal frame buffer is preserved but the display
    /// enters a low power state. This is different from turning the display off.
    ///
    /// Uses the control panel for the sleep operation.
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_sleep(&self, sleep: bool) -> Result<(), EspError> {
        let panel = self
            .control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_disp_sleep(panel, sleep))?;
        }
        Ok(())
    }

    /// Invert the display colors
    ///
    /// Requires that a DPI panel has been set (via `create_dpi_panel()` or `set_dpi_panel()`).
    pub fn invert_colors(&self, invert: bool) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_invert_color(panel, invert))?;
        }
        Ok(())
    }

    /// Mirror the display horizontally
    ///
    /// Requires that a DPI panel has been set (via `create_dpi_panel()` or `set_dpi_panel()`).
    pub fn mirror_x(&self, mirror: bool) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_mirror(panel, mirror, false))?;
        }
        Ok(())
    }

    /// Mirror the display vertically
    ///
    /// Requires that a DPI panel has been set (via `create_dpi_panel()` or `set_dpi_panel()`).
    pub fn mirror_y(&self, mirror: bool) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_mirror(panel, false, mirror))?;
        }
        Ok(())
    }

    /// Swap the X and Y axes
    ///
    /// Requires that a DPI panel has been set (via `create_dpi_panel()` or `set_dpi_panel()`).
    pub fn swap_xy(&self, swap: bool) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_swap_xy(panel, swap))?;
        }
        Ok(())
    }

    /// Set the gap between pixels
    ///
    /// Requires that a DPI panel has been set (via `create_dpi_panel()` or `set_dpi_panel()`).
    pub fn set_gap(&self, x_gap: i32, y_gap: i32) -> Result<(), EspError> {
        let panel = self
            .panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_set_gap(panel, x_gap, y_gap))?;
        }
        Ok(())
    }

    /// Turn the display off
    ///
    /// Uses the control panel for the display off operation.
    /// Requires that `set_control_panel()` has been called first.
    pub fn set_display_off(&self, off: bool) -> Result<(), EspError> {
        let panel = self
            .control_panel_handle
            .ok_or_else(|| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;
        unsafe {
            esp!(esp_lcd_panel_disp_off(panel, off))?;
        }
        Ok(())
    }
}

impl<'d> Drop for LcdDriver<'d> {
    fn drop(&mut self) {
        unsafe {
            // Clean up DPI panel if we created it
            if let Some(panel) = self.panel_handle {
                if !panel.is_null() {
                    let _ = esp_lcd_panel_del(panel);
                }
            }
            // Clean up DPI config if we allocated it
            if let Some(dpi_config) = self.dpi_config {
                if !dpi_config.is_null() {
                    let _ = Box::from_raw(dpi_config);
                }
            }
        }
    }
}

unsafe impl<'d> Send for LcdDriver<'d> {}
