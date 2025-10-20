use core::fmt;

use alloc::vec::Vec;

use alloc::collections::VecDeque;
use esp_idf_sys::{EspError, ESP_ERR_TIMEOUT};

use super::tx_channel::TxChannelDriver;
use crate::rmt::config::TransmitConfig;
use crate::rmt::encoder::{Encoder, EncoderWrapper, RawEncoder};

// What kind of signal is necessary for the TxQueue to be safe?
//
// - From the signal, one should be able to get a pointer and a length to the buffer,
//   e.g. as_ref(&self) -> &[E::Item], from which one could get the ptr and len.
// - The signal must be guaranteed to be valid until the transmission is done,
//   this would be the same lifetime for which the TxQueue is valid.
// - The TxQueue will move the signal around, with some types like [E::Item; N] this would
//   invalidate the pointer to the data
//
// What are the options?
// - Only allow &'a [E::Item] where 'a is the lifetime of the TxQueue (assuming no mem::forget) or 'a = 'static
//   if mem::forget is used. This has the downside that one can not pass owned data like Vec or Box.
// - Keep an owned buffer like Vec<E::Item> or Box<[E::Item]> for each encoder, then clone the passed signal into it.
//   The benefits are that the user can pass any kind of signal that implements AsRef<[E::Item]>, but it would require
//   an initial allocation, copy the signal into the internal buffer, and E::Item would have to be Clone.
// - Implementing a custom queue where the stored elements are not moved until they are removed from the queue.
//   This is the likely the most performant option, but will require a lot of code to implement a custom queue
//   and might perform worse than the other options (another issue is fragmentation).
// - Define an unsafe trait that is implemented for types that are safe to move around like &[E::Item],
//   Vec<E::Item>, Box<[E::Item]>, ...
//   This allows the user to pass both owned and borrowed data. A disadvantage in comparison to the internal buffers
//   is that the user has to ensure that the data is valid until the transmission is done, and it might perform worse
//   when the buffers are not reused.

#[derive(Debug)]
pub(crate) struct EncoderBuffer<E: RawEncoder> {
    encoder: E,
    buffer: Vec<E::Item>,
}

impl<E: RawEncoder> EncoderBuffer<E> {
    pub fn new(encoder: E) -> Self {
        Self {
            encoder,
            buffer: Vec::new(),
        }
    }

    /// Updates the internal buffer to contain the given signal.
    ///
    /// It returns a mutable reference to the stored encoder and a reference to the now
    /// internally stored signal.
    fn update_from_slice(&mut self, signal: &[E::Item]) -> (&mut E, &[E::Item])
    where
        E::Item: Clone,
    {
        // Ensure that the buffer is large enough to hold the signal.
        //
        // The value will be overwritten anyway, so we can just clone the first element
        // to resize the buffer.
        self.buffer.resize(signal.len(), signal[0].clone());

        // Copy the signal into the buffer, ensuring that the signal will be valid for the entire
        // transmission.
        self.buffer[..signal.len()].clone_from_slice(signal);

        (&mut self.encoder, &self.buffer[..signal.len()])
    }
}

/// A queue to efficiently transmit multiple signals, reusing encoders when possible.
///
/// The driver has an internal queue from which it will copy the signals into the peripheral
/// memory. The signal and encoder must be valid until the transmission is done,
/// which can not be guaranteed when directly using the queue through [`TxChannelDriver::start_send`].
///
/// This type provides a safe interface to fill up the queue, reusing encoders when possible.
/// To do this, it keeps an allocated buffer for each encoder, to which the signal will be copied
/// before starting the transmission. This ensures that both the signal and encoder are valid
/// until the transmission is done.
///
/// # Drop behavior
///
/// When the `TxQueue` is dropped, it will wait for all transmissions to finish.
/// This ensures that the internal buffers are not dropped while they are still in use
/// by the peripheral.
pub struct TxQueue<'c, 'd, E: Encoder> {
    queue: VecDeque<EncoderBuffer<EncoderWrapper<E>>>,
    channel: &'c mut TxChannelDriver<'d>,
}

impl<'c, 'd, E: Encoder> TxQueue<'c, 'd, E> {
    pub(crate) fn new(
        queue: VecDeque<EncoderBuffer<EncoderWrapper<E>>>,
        channel: &'c mut TxChannelDriver<'d>,
    ) -> Self {
        assert!(
            !queue.is_empty(),
            "At least one encoder is required to encode a TxQueue"
        );

        Self { queue, channel }
    }

    /// Returns a mutable reference to the channel this queue is using.
    #[must_use]
    pub fn channel(&mut self) -> &mut TxChannelDriver<'d> {
        self.channel
    }
}

impl<'c, 'd, E: Encoder> TxQueue<'c, 'd, E> {
    /// Pushes a signal onto the transmission queue.
    ///
    /// The signal will be cloned into an internal buffer to ensure that it is valid for the entire
    /// transmission. These buffers will be reused for future transmissions after the current transmission
    /// is done.
    ///
    /// # Blocking behavior
    ///
    /// If the queue is full (i.e. all encoders are busy), this function will behave differently depending
    /// on whether [`TransmitConfig::queue_non_blocking`] is set or not:
    /// - If `queue_non_blocking` is `false`, it will block until one encoder is available.
    /// - If `queue_non_blocking` is `true`, it will return an error with code `ESP_ERR_TIMEOUT`.
    ///   There will be no state change, so the function can be called again immediately.
    ///
    /// If this queue has more encoders available than the channel queue size and `queue_non_blocking` is set,
    /// it will copy the signal into the internal buffer, but might fail (with a timeout) to start the
    /// transmission if the channel queue is full.
    ///
    /// # Panics
    ///
    /// If the signal is empty.
    pub fn push(&mut self, signal: &[E::Item], config: &TransmitConfig) -> Result<(), EspError>
    where
        E::Item: Clone,
    {
        assert!(!signal.is_empty(), "Can not send an empty signal");

        // Before a new transmission can be started, an encoder has to be available.
        //
        // Assuming the TxQueue has N encoders, it can immediately start sending if there are less than
        // N transmissions in progress on the channel.
        //
        // If there are N transmissions or more in progress, it would have to wait until there are less than
        // N transmissions in progress to guarantee that one encoder is available.
        while self.channel.queue_size() >= self.queue.len() {
            // If we should not block, replicate the error from esp-idf's send function
            if config.queue_non_blocking {
                return Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>());
            }

            // This waits for a transmission to finish
            crate::task::block_on(self.channel.wait_for_progress());
        }

        // This returns the next encoder and the buffer of the encoder.
        //
        // With the above waiting, it should be guaranteed that one encoder is available
        // which would be the first one in the queue (they are ordered from oldest first to newest last).
        let (next_encoder, buffer) = self
            .queue
            .front_mut()
            .expect("queue should never be empty")
            .update_from_slice(signal);

        unsafe { self.channel.start_send(next_encoder, buffer, config) }?;

        // If the channel queue is shorter than the number of encoders in this TxQueue,
        // and the non-blocking flag is set, it could happen that the start_send errors.
        //
        // To prevent loosing an encoder and its buffer, it will only be removed from
        // the front of the queue after a successful start_send.
        let buffer = self.queue.pop_front().unwrap();
        self.queue.push_back(buffer);

        Ok(())
    }
}

impl<'c, 'd, E: Encoder> Drop for TxQueue<'c, 'd, E> {
    fn drop(&mut self) {
        // This ensures that all transmissions are done before the internal buffers are dropped.
        let _ = self.channel.wait_all_done(None);
    }
}

impl<'c, 'd, E: Encoder> fmt::Debug for TxQueue<'c, 'd, E>
where
    E: fmt::Debug,
    E::Item: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxQueue")
            .field("queue", &self.queue)
            .field("channel", &self.channel)
            .finish()
    }
}
