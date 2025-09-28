#[cfg(feature = "alloc")]
mod rx_channel;
mod tx_channel;
#[cfg(feature = "alloc")]
pub use rx_channel::*;
pub use tx_channel::*;

#[cfg(feature = "alloc")]
pub(super) mod tx_queue;
