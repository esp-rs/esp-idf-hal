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
//! use esp_idf_hal::peripherals::Peripherals;
//!
//! let peripherals = Peripherals::take()?;
//!
//! // Configure LDO channel for MIPI D-PHY (LDO3 is adjustable)
//! let ldo_config = LdoChannelConfig::new(2500);  // voltage_mv (2.5V)
//!
//! let mut ldo_channel = LdoChannel::new(&peripherals.ldo3, &ldo_config)?;
//! // Channel is automatically enabled when acquired
//!
//! // Adjust voltage (only available for Adjustable channels)
//! ldo_channel.adjust_voltage(3000)?;  // Change to 3.0V
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
        /// Allow hardware (e.g., eFuse) to control the channel
        pub owned_by_hw: bool,
    }

    impl LdoChannelConfig {
        /// Create a new LDO channel configuration
        pub const fn new(voltage_mv: i32) -> Self {
            Self {
                voltage_mv,
                owned_by_hw: false,
            }
        }

        #[must_use]
        pub fn voltage_mv(mut self, voltage_mv: i32) -> Self {
            self.voltage_mv = voltage_mv;
            self
        }

        #[must_use]
        pub fn owned_by_hw(mut self, owned_by_hw: bool) -> Self {
            self.owned_by_hw = owned_by_hw;
            self
        }
    }
}

pub trait VoltageType {
    const IS_ADJUSTABLE: bool;
}

pub struct Adjustable;
pub struct Fixed;

impl VoltageType for Adjustable {
    const IS_ADJUSTABLE: bool = true;
}
impl VoltageType for Fixed {
    const IS_ADJUSTABLE: bool = false;
}

/// An LDO channel
pub struct LdoChannel<'d, V: VoltageType> {
    handle: esp_ldo_channel_handle_t,
    _p: PhantomData<&'d mut ()>,
    _v: PhantomData<V>,
}

impl<'d, V: VoltageType> LdoChannel<'d, V> {
    /// Create (acquire) a new LDO channel with the given configuration
    pub fn new<LDO: Ldo<VoltageType = V> + 'd>(
        _ldo: LDO,
        config: &config::LdoChannelConfig,
    ) -> Result<LdoChannel<'d, V>, EspError> {
        let mut ldo_config = esp_ldo_channel_config_t::default();
        ldo_config.chan_id = LDO::channel();
        ldo_config.voltage_mv = config.voltage_mv;
        ldo_config.flags = esp_ldo_channel_config_t_ldo_extra_flags {
            _bitfield_1: esp_ldo_channel_config_t_ldo_extra_flags::new_bitfield_1(
                V::IS_ADJUSTABLE as u32,
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

        Ok(LdoChannel {
            handle,
            _p: PhantomData,
            _v: PhantomData,
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
}

impl<'d> LdoChannel<'d, Adjustable> {
    /// Adjust the output voltage of the LDO channel
    pub fn adjust_voltage(&mut self, voltage_mv: i32) -> Result<(), EspError> {
        esp!(unsafe { esp_ldo_channel_adjust_voltage(self.handle, voltage_mv) })?;
        Ok(())
    }
}

impl<'d, V: VoltageType> Drop for LdoChannel<'d, V> {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            let _ = esp!(unsafe { esp_ldo_release_channel(self.handle) });
        }
    }
}

unsafe impl<'d, V: VoltageType> Send for LdoChannel<'d, V> {}

// Re-export config types at module level
pub use config::*;

pub trait Ldo {
    type VoltageType: VoltageType;
    fn channel() -> i32;
}

macro_rules! impl_ldo {
    ($ldo:ident: $channel:expr) => {
        pub struct $ldo<'a, V: VoltageType>(
            ::core::marker::PhantomData<&'a mut ()>,
            ::core::marker::PhantomData<V>,
        );

        impl<'a, V: VoltageType> $ldo<'a, V> {
            /// Unsafely create an instance of this peripheral out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline(always)]
            pub unsafe fn steal() -> Self {
                Self(::core::marker::PhantomData, ::core::marker::PhantomData)
            }

            /// Creates a new peripheral reference with a shorter lifetime.
            ///
            /// Use this method if you would like to keep working with the peripheral after
            /// you dropped the driver that consumes this.
            ///
            /// # Safety
            ///
            /// You must ensure that you are not using reborrowed peripherals in drivers which are
            /// forgotten via `core::mem::forget`.
            #[inline]
            #[allow(dead_code)]
            pub unsafe fn reborrow(&mut self) -> $ldo<'_, V> {
                Self(::core::marker::PhantomData, ::core::marker::PhantomData)
            }
        }

        unsafe impl<'a, V: VoltageType> Send for $ldo<'a, V> {}

        impl<'a, V: VoltageType> Ldo for $ldo<'a, V> {
            type VoltageType = V;
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
