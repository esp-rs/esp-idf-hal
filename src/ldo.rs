//! Low Dropout Voltage Regulator (LDO) peripheral control
//!
//! Interface to the [Low Dropout Voltage Regulator (LDO)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32p4/api-reference/peripherals/ldo_regulator.html)
//!
//! This module provides safe Rust wrappers for the LDO API, which is available
//! on ESP32-P4 and later chips.
//!
//! # Example
//!
//! Configure LDO for MIPI D-PHY power supply (2.5V):
//! ```
//! use esp_idf_hal::ldo::*;
//! use esp_idf_hal::delay::FreeRtos;
//!
//! // Configure LDO channel for MIPI D-PHY
//! let ldo_config = LdoChannelConfig::new(3, 2500);  // channel_id, voltage_mv (2.5V)
//! // Or use builder pattern:
//! // let ldo_config = LdoChannelConfig::new(3, 2500)
//! //     .adjustable(true);  // Enable dynamic voltage adjustment
//!
//! let mut ldo_channel = LdoChannel::new(&ldo_config)?;
//! // Channel is automatically enabled when acquired
//!
//! // Wait for stabilization
//! FreeRtos::delay_ms(20);
//! ```

#![cfg(esp32p4)]

use core::marker::PhantomData;

use esp_idf_sys::*;

/// Types for configuring the LDO peripheral
pub mod config {
    /// LDO channel configuration
    #[derive(Debug, Clone, Eq, PartialEq, Hash)]
    pub struct LdoChannelConfig {
        pub voltage_mv: i32,
        /// Enable dynamic voltage adjustment via `adjust_voltage()`
        pub adjustable: bool,
        /// Allow hardware (e.g., eFuse) to control the channel
        pub owned_by_hw: bool,
    }

    impl LdoChannelConfig {
        /// Create a new LDO channel configuration
        pub const fn new(voltage_mv: i32) -> Self {
            Self {
                voltage_mv,
                adjustable: false,
                owned_by_hw: false,
            }
        }

        #[must_use]
        pub fn voltage_mv(mut self, voltage_mv: i32) -> Self {
            self.voltage_mv = voltage_mv;
            self
        }

        #[must_use]
        pub fn adjustable(mut self, adjustable: bool) -> Self {
            self.adjustable = adjustable;
            self
        }

        #[must_use]
        pub fn owned_by_hw(mut self, owned_by_hw: bool) -> Self {
            self.owned_by_hw = owned_by_hw;
            self
        }
    }
}

/// An LDO channel
pub struct LdoChannel<'d> {
    handle: esp_ldo_channel_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> LdoChannel<'d> {
    /// Create (acquire) a new LDO channel with the given configuration
    pub fn new<LDO: Ldo + 'd>(
        _ldo: LDO,
        config: &config::LdoChannelConfig,
    ) -> Result<Self, EspError> {
        let mut ldo_config = esp_ldo_channel_config_t::default();
        ldo_config.chan_id = LDO::channel();
        ldo_config.voltage_mv = config.voltage_mv;
        ldo_config.flags = esp_ldo_channel_config_t_ldo_extra_flags {
            _bitfield_1: esp_ldo_channel_config_t_ldo_extra_flags::new_bitfield_1(
                config.adjustable as u32,
                config.owned_by_hw as u32,
                0, // bypass field (deprecated, always set to 0)
            ),
            ..Default::default()
        };

        let mut handle: esp_ldo_channel_handle_t = core::ptr::null_mut();
        esp!(unsafe { esp_ldo_acquire_channel(&ldo_config, &mut handle) })?;

        if handle.is_null() {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        Ok(Self {
            handle,
            _p: PhantomData,
        })
    }

    /// Get the raw LDO channel handle
    ///
    /// # Note
    ///
    /// This exposes the underlying ESP-IDF handle for advanced use cases.
    /// Prefer using the HAL methods when possible.
    pub fn handle(&self) -> esp_ldo_channel_handle_t {
        self.handle
    }

    /// Adjust the output voltage of the LDO channel
    pub fn adjust_voltage(&mut self, voltage_mv: i32) -> Result<(), EspError> {
        esp!(unsafe { esp_ldo_channel_adjust_voltage(self.handle, voltage_mv) })?;
        Ok(())
    }
}

impl Drop for LdoChannel<'_> {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            let _ = esp!(unsafe { esp_ldo_release_channel(self.handle) });
        }
    }
}

unsafe impl Send for LdoChannel<'_> {}

// Re-export config types at module level
pub use config::*;

pub trait Ldo {
    fn channel() -> i32;
}

macro_rules! impl_ldo {
    ($ldo:ident: $channel:expr) => {
        crate::impl_peripheral!($ldo);

        impl Ldo for $ldo<'_> {
            #[inline(always)]
            fn channel() -> i32 {
                $channel
            }
        }
    };
}

#[cfg(esp32p4)]
impl_ldo!(LDO1: 1);
#[cfg(esp32p4)]
impl_ldo!(LDO2: 2);
#[cfg(esp32p4)]
impl_ldo!(LDO3: 3);
#[cfg(esp32p4)]
impl_ldo!(LDO4: 4);
