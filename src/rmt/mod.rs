pub mod config;
pub mod encoder;

mod tx_channel;
pub use tx_channel::*;
mod rx_channel;
pub use rx_channel::*;
mod sync_manager;
pub use sync_manager::*;

use core::ptr;

use esp_idf_sys::*;

use crate::rmt::config::CarrierConfig;

fn assert_not_in_isr() {
    if crate::interrupt::active() {
        panic!("This function cannot be called from an ISR");
    }
}

pub trait RmtChannel {
    /// Returns the underlying `rmt_channel_handle_t`.
    fn handle(&self) -> rmt_channel_handle_t;

    #[must_use]
    fn is_enabled(&self) -> bool;

    unsafe fn set_internal_enabled(&mut self, is_enabled: bool);

    /// Must be called in advance before transmitting or receiving RMT symbols.
    /// For TX channels, enabling a channel enables a specific interrupt and
    /// prepares the hardware to dispatch transactions.
    ///
    /// For RX channels, enabling a channel enables an interrupt, but the receiver
    /// is not started during this time, as the characteristics of the incoming
    /// signal have yet to be specified.
    ///
    /// The receiver is started in [`RxChannel::receive`].
    fn enable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_enable(self.handle()) })?;

        // SAFETY: The channel has been enabled -> it is safe to mark it as enabled.
        unsafe { self.set_internal_enabled(true) };
        Ok(())
    }

    /// Disables the interrupt and clearing any pending interrupts. The transmitter
    /// and receiver are disabled as well.
    ///
    /// # Note
    ///
    /// This function will release a power management (PM) lock that might be
    /// installed during channel allocation
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Disable RMT channel failed because of invalid argument
    /// - `ESP_ERR_INVALID_STATE`: Disable RMT channel failed because it's not enabled yet
    /// - `ESP_FAIL`: Disable RMT channel failed because of other error
    fn disable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_disable(self.handle()) })?;
        // SAFETY: The channel has been disable -> it is safe to mark it as disabled.
        unsafe { self.set_internal_enabled(false) };
        Ok(())
    }

    /// Apply (de)modulation feature for the channel.
    ///
    /// If `carrier_config` is `None`, the carrier (de)modulation will be disabled.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Apply carrier failed because of invalid argument
    /// - `ESP_FAIL`: Apply carrier failed because of other error
    fn apply_carrier(&mut self, carrier_config: Option<&CarrierConfig>) -> Result<(), EspError> {
        let mut sys_carrier = None;
        if let Some(CarrierConfig {
            frequency,
            duty_cycle,
            polarity_active_low,
            always_on,
        }) = carrier_config
        {
            sys_carrier = Some(rmt_carrier_config_t {
                frequency_hz: (*frequency).into(),
                duty_cycle: duty_cycle.0 as f32 / 100.0,
                flags: rmt_carrier_config_t__bindgen_ty_1 {
                    _bitfield_1: rmt_carrier_config_t__bindgen_ty_1::new_bitfield_1(
                        *polarity_active_low as u32,
                        *always_on as u32,
                    ),
                    ..Default::default()
                },
            })
        }

        esp!(unsafe {
            rmt_apply_carrier(
                self.handle(),
                sys_carrier.as_ref().map_or(ptr::null(), |c| c as *const _),
            )
        })
    }
}

// TODO: esp32c2 does not define a clock source?

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum ClockSource {
    #[default]
    Default,
    #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
    APB,
    #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
    RcFast,
    #[cfg(any(esp32, esp32s2))]
    RefTick,
    #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
    XTAL,
    #[cfg(any(esp32c5, esp32c6, esp32p4))]
    PLLF80M,
}

impl From<ClockSource> for rmt_clock_source_t {
    fn from(clock: ClockSource) -> Self {
        match clock {
            ClockSource::Default => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_DEFAULT,
            #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
            ClockSource::APB => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_APB,
            #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
            ClockSource::RcFast => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_RC_FAST,
            #[cfg(any(esp32, esp32s2))]
            ClockSource::RefTick => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_REF_TICK,
            #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
            ClockSource::XTAL => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_XTAL,
            #[cfg(any(esp32c5, esp32c6, esp32p4))]
            ClockSource::PLLF80M => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_PLL_F80M,
        }
    }
}
