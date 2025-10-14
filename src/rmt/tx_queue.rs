use core::fmt;

use alloc::vec::Vec;

use alloc::collections::VecDeque;
use esp_idf_sys::{EspError, ESP_ERR_TIMEOUT};

use super::tx_channel::TxChannelDriver;
use crate::rmt::config::TransmitConfig;
use crate::rmt::encoder::{into_raw, Encoder, EncoderWrapper};

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

pub unsafe trait MovableSignal<T> {
    fn as_ref(&self) -> &[T];
}

unsafe impl<T> MovableSignal<T> for &[T] {
    fn as_ref(&self) -> &[T] {
        self
    }
}

unsafe impl<T> MovableSignal<T> for &mut [T] {
    fn as_ref(&self) -> &[T] {
        self
    }
}

unsafe impl<T> MovableSignal<T> for Vec<T> {
    fn as_ref(&self) -> &[T] {
        self
    }
}

unsafe impl<T> MovableSignal<T> for alloc::boxed::Box<[T]> {
    fn as_ref(&self) -> &[T] {
        self
    }
}

unsafe impl<T, const N: usize> MovableSignal<T> for alloc::boxed::Box<[T; N]> {
    fn as_ref(&self) -> &[T] {
        &self[..]
    }
}

// TODO: One could have heapless::Deque as an alternative to VecDeque?
pub struct TxQueue<'c, 'd, E: Encoder> {
    queue: VecDeque<(EncoderWrapper<E>, Option<&'c [E::Item]>)>,
    channel: &'c mut TxChannelDriver<'d>,
}

impl<'c, 'd, E: Encoder> TxQueue<'c, 'd, E> {
    pub(crate) fn new(encoders: Vec<E>, channel: &'c mut TxChannelDriver<'d>) -> Self {
        assert!(
            !encoders.is_empty(),
            "At least one encoder is required to create a TxQueue"
        );

        Self {
            queue: encoders
                .into_iter()
                .map(into_raw)
                .map(|encoder| (encoder, None))
                .collect(),
            channel,
        }
    }

    /// Returns a mutable reference to the channel this queue is using.
    #[must_use]
    pub fn channel(&mut self) -> &mut TxChannelDriver<'d> {
        &mut self.channel
    }
}
// TODO: if we rewrite queue, do we need the send with an alloc?
// TODO: evaluate safety with mem::forget

impl<'c, 'd, E: Encoder> TxQueue<'c, 'd, E> {
    /// Pushes a signal onto the transmission queue.
    ///
    /// # Safety
    ///
    /// The queue will store the given reference to ensure that it lives until the transmission is done.
    /// If [`mem::forget`] is called on [`TxQueue`], the referenced buffer can be dropped before
    /// the transmission is done, leading to undefined behavior.
    ///
    /// The caller must ensure that `self` is not forgotten while there are still transmissions in progress.
    /// [`mem::forget`]: core::mem::forget
    pub unsafe fn push_nonstatic(
        &mut self,
        signal: &'c [E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        assert!(!signal.is_empty(), "Can not send an empty signal");

        // Before we can start a new transmission, we have to ensure that there is an encoder available.
        //
        // Assuming the TxQueue has N encoders, we can immediately start sending if there are less than
        // N transmissions in progress on the channel.
        //
        // If there are N transmissions or more in progress, we would have to wait until there are less than
        // N transmissions in progress to guarantee that one of our encoders is available.
        while self.channel.queue_size() >= self.queue.len() {
            // If we should not block, replicate the error from esp-idf's send function
            if config.queue_non_blocking {
                return Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>());
            }

            // This waits for a transmission to finish
            crate::task::block_on(self.channel.wait_for_progress());
        }

        // This returns the next encoder and the old signal (if any).
        //
        // With the above waiting, it should be guaranteed that one encoder is available
        // which would be the first one in the queue (they are ordered from oldest first to newest last).
        let (mut next_encoder, _old_signal) =
            self.queue.pop_front().expect("queue should never be empty");

        unsafe {
            self.channel
                .start_send(&mut next_encoder, signal.as_ref(), config)
        }?;

        // Store the encoder with the new signal in the queue (ensuring there is a reference to the signal)
        self.queue.push_back((next_encoder, Some(signal)));

        Ok(())
    }
}

// TODO: a push and a push_nonstatic?

impl<'c, 'd, E: Encoder> Drop for TxQueue<'c, 'd, E> {
    fn drop(&mut self) {
        let _ = self.channel.wait_all_done(None);
    }
}

// TODO: Show information about the elements in the queue?
impl<'c, 'd, E: Encoder> fmt::Debug for TxQueue<'c, 'd, E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxQueue")
            .field("queue", &self.queue.len())
            .field("channel", &self.channel)
            .finish()
    }
}
