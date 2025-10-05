use core::fmt;
use core::pin::Pin;

use esp_idf_sys::EspError;

use crate::rmt::config::TransmitConfig;
use crate::rmt::encoder::Encoder;
use crate::rmt::{AsyncTxChannelDriver, TxQueue};

/// An asynchronous wrapper around the blocking [`TxQueue`].
///
/// # Drop Behavior
///
/// When an `AsyncTxQueue` is dropped, it will block until all of its ongoing transmissions are complete.
/// This ensures that no transmissions are left incomplete when the queue is no longer in use.
pub struct AsyncTxQueue<'c, 'd, E: Encoder, S, const N: usize> {
    queue: TxQueue<'c, 'd, E, S, N>,
}

impl<'c, 'd, E: Encoder, S, const N: usize> AsyncTxQueue<'c, 'd, E, S, N> {
    pub(crate) fn new(queue: TxQueue<'c, 'd, E, S, N>) -> Self {
        Self { queue }
    }

    /// Returns an immutable pinned reference to the underlying `TxQueue`.
    pub fn queue(self: Pin<&Self>) -> Pin<&TxQueue<'c, 'd, E, S, N>> {
        unsafe { self.map_unchecked(|this| &this.queue) }
    }

    /// Returns a mutable pinned reference to the underlying `TxQueue`.
    pub fn queue_mut(self: Pin<&mut Self>) -> Pin<&mut TxQueue<'c, 'd, E, S, N>> {
        // SAFETY: The queue is pinned when self is pinned
        unsafe { self.map_unchecked_mut(|this| &mut this.queue) }
    }

    pub fn channel(self: Pin<&mut Self>) -> &mut AsyncTxChannelDriver<'d> {
        self.queue_mut().channel().to_async()
    }
}

impl<'c, 'd, E: Encoder, S: AsRef<[E::Item]> + 'c, const N: usize> AsyncTxQueue<'c, 'd, E, S, N> {
    /// Pushes a new signal to be transmitted using the next available encoder.
    ///
    /// If all encoders are busy, this will wait until one is available.
    ///
    /// # Cancel Safety
    ///
    /// If the future is cancelled before completion, no transmission will be started.
    pub async fn push(
        mut self: Pin<&mut Self>,
        mut signal: S,
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        loop {
            match self
                .as_mut()
                .queue_mut()
                .push_internal(signal, config, false)
            {
                Ok(()) => return Ok(()),
                Err(Err(err)) => return Err(err),
                Err(Ok((original_signal, token))) => {
                    self.as_mut().channel().wait_for(token).await;

                    signal = original_signal;
                }
            }
        }
    }
}

// NOTE: On drop the underlying TxQueue will wait for all ongoing transmissions to finish -> no dedicated Drop impl needed

impl<'c, 'd, E: Encoder, S, const N: usize> fmt::Debug for AsyncTxQueue<'c, 'd, E, S, N>
where
    TxQueue<'c, 'd, E, S, N>: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AsyncTxQueue")
            .field("queue", &self.queue)
            .finish()
    }
}
