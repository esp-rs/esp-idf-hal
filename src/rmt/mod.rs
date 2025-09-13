pub mod config;
pub mod encoder;

mod tx_channel;
pub use tx_channel::*;

use esp_idf_sys::*;

use crate::rmt_legacy::Pulse;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SourceClock {
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

impl From<SourceClock> for rmt_clock_source_t {
    fn from(clock: SourceClock) -> Self {
        match clock {
            SourceClock::Default => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_DEFAULT,
            #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
            SourceClock::APB => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_APB,
            #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
            SourceClock::RcFast => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_RC_FAST,
            #[cfg(any(esp32, esp32s2))]
            SourceClock::RefTick => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_REF_TICK,
            #[cfg(any(esp32c3, esp32c5, esp32c6, esp32h2, esp32h4, esp32p4, esp32s3))]
            SourceClock::XTAL => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_XTAL,
            #[cfg(any(esp32c5, esp32c6, esp32p4))]
            SourceClock::PllF80m => soc_periph_rmt_clk_src_t_RMT_CLK_SRC_PLL_F80M,
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
        *self
    }
}
