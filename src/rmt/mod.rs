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
use crate::rmt_legacy::Pulse;

pub trait RmtChannel {
    /// Returns the underlying `rmt_channel_handle_t`.
    fn handle(&self) -> rmt_channel_handle_t;

    /// Must be called in advance before transmitting or receiving RMT symbols.
    /// For TX channels, enabling a channel enables a specific interrupt and
    /// prepares the hardware to dispatch transactions. For RX channels, enabling
    /// a channel enables an interrupt, but the receiver is not started during
    /// this time, as the characteristics of the incoming signal have yet to be
    /// specified.
    ///
    /// The receiver is started in rmt_receive().
    fn enable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_enable(self.handle()) })
    }

    /// Disables the interrupt and clearing any pending interrupts. The transmitter
    /// and receiver are disabled as well.
    fn disable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_disable(self.handle()) })
    }

    /// Apply modulation feature for the channel.
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
    PllF80m,
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
            ClockSource::PllF80m => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_PLL_F80M,
        }
    }
}

// TODO: Decide on how to adjust the signal trait.
//
// Why it needs an adjustment?
//
// The original api only accepted a pointer to an array of rmt symbols,
// the new api supports a pointer to any kind of data, as long as it is
// supported by an encoder.
//
// To accommodate this change, the Signal trait has been adjusted to
// support a generic type T.
//
// Depending on the final implementation, one could keep backward compatibility
// with the old api by defaulting T to rmt_item32_t (I am not a fan of this)
//
// Potential problems:
// - Make sure that Signal owns its data, so the code can assume that when it is
//   passed an owned value that implements Signal, that the slice it references is
//   a part of it.
pub trait Signal<T> {
    // TODO: associated type instead of generic?
    fn as_slice(&self) -> &[T];
}

#[derive(Clone, Copy)]
pub struct Symbol(rmt_symbol_word_t);

impl Symbol {
    /// Create a symbol from a pair of half-cycles.
    pub fn new(level0: Pulse, level1: Pulse) -> Self {
        let item = rmt_symbol_word_t {
            __bindgen_anon_1: Default::default(),
        };
        let mut this = Self(item);
        this.update(level0, level1);
        this
    }

    /// Mutate this symbol to store a different pair of half-cycles.
    pub fn update(&mut self, level0: Pulse, level1: Pulse) {
        // SAFETY: We're overriding all 32 bits, so it doesn't matter what was here before.
        let inner = unsafe { &mut self.0.__bindgen_anon_1 };
        inner.set_level0(level0.pin_state as u16);
        inner.set_duration0(level0.ticks.ticks());
        inner.set_level1(level1.pin_state as u16);
        inner.set_duration1(level1.ticks.ticks());
    }
}

// TODO: is it safe to transmute between rmt_item32_t and rmt_symbol_word_t?

impl Signal<rmt_symbol_word_t> for Symbol {
    fn as_slice(&self) -> &[rmt_symbol_word_t] {
        std::slice::from_ref(&self.0)
    }
}

impl Signal<rmt_symbol_word_t> for [rmt_symbol_word_t] {
    fn as_slice(&self) -> &[rmt_symbol_word_t] {
        self
    }
}

impl Signal<rmt_symbol_word_t> for &[rmt_symbol_word_t] {
    fn as_slice(&self) -> &[rmt_symbol_word_t] {
        self
    }
}
