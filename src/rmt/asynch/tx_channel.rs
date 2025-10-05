use core::pin::Pin;
use core::time::Duration;

use alloc::boxed::Box;

use esp_idf_sys::*;

use crate::gpio::OutputPin;
use crate::rmt::asynch::AsyncTxQueue;
#[cfg(feature = "alloc")]
use crate::rmt::blocking::TxChannelDriver;
use crate::rmt::config::{TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::{into_raw, Encoder, RawEncoder};
use crate::rmt::{assert_not_in_isr, RmtChannel, Token};
use crate::task::yield_now;

/// An asynchronous wrapper around the blocking [`TxChannelDriver`].
///
/// This allows using the rmt driver in asynchronous contexts,
/// without blocking an executor.
#[derive(Debug)]
#[repr(transparent)]
pub struct AsyncTxChannelDriver<'d> {
    pub(crate) driver: TxChannelDriver<'d>,
}

impl<'d> AsyncTxChannelDriver<'d> {
    /// Wraps an existing blocking driver into an asynchronous one.
    #[must_use]
    pub fn wrap(driver: TxChannelDriver<'d>) -> Self {
        Self { driver }
    }

    /// Returns a mutable reference to the underlying blocking driver.
    #[must_use]
    pub fn driver(&mut self) -> &mut TxChannelDriver<'d> {
        &mut self.driver
    }

    /// Creates a new RMT TX channel.
    ///
    /// # Note
    ///
    /// When multiple RMT channels are allocated at the same time,
    /// the group’s prescale is determined based on the resolution of
    /// the first channel. The driver then selects the appropriate prescale
    /// from low to high. To avoid prescale conflicts when allocating multiple
    /// channels, allocate channels in order of their target resolution,
    /// either from highest to lowest or lowest to highest.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context.
    pub fn new(pin: impl OutputPin + 'd, config: &TxChannelConfig) -> Result<Self, EspError> {
        Ok(Self::wrap(TxChannelDriver::new(pin, config)?))
    }

    /// Wait for all pending TX transactions to finish.
    ///
    /// # Note
    ///
    /// This function will block forever if the pending transaction can't
    /// be finished within a limited time (e.g. an infinite loop transaction).
    /// See also [`Self::disable`] for how to terminate a working channel.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Flush transactions failed because of invalid argument
    /// - `ESP_FAIL`: Flush transactions failed because of other error
    pub async fn wait_all_done(&mut self) -> Result<(), EspError> {
        loop {
            match self.driver().wait_all_done(Some(Duration::ZERO)) {
                Ok(()) => return Ok(()),
                Err(e) if e.code() == esp_idf_sys::ESP_ERR_TIMEOUT => {
                    // Yield to allow other tasks to run
                    yield_now().await;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Creates a transmission queue that manages the given encoders, and reuses them for new transmissions when possible.
    ///
    /// # Safety
    ///
    /// The pending transmissions are stored in the queue, which is why it must be pinned.
    ///
    /// The pinning ensures that the memory address of the signals and encoders does not change,
    /// before the transmission is done.
    ///
    /// To prevent invalid memory accesses, the queue will wait for all pending transmissions to finish
    /// in its drop implementation. If one were to [`mem::forget`] the queue with it on the stack,
    /// it could result in invalid memory accesses (undefined behaviour).
    ///
    /// Pinning the queue on the heap is one way to ensure that [`mem::forget`] does not result in
    /// undefined behaviour, for this reason, the safe wrapper [`Self::queue`] is provided.
    ///
    /// This function is provided to avoid the allocation. The caller has to pin it on the stack
    /// with [`pin!`].
    ///
    /// [`mem::forget`]: core::mem::forget
    /// [`pin!`]: core::pin::pin
    #[cfg(feature = "alloc")]
    pub unsafe fn raw_queue<E: Encoder, S, const N: usize>(
        &mut self,
        encoders: [E; N],
    ) -> AsyncTxQueue<'_, 'd, E, S, N> {
        AsyncTxQueue::new(self.driver().raw_queue(encoders))
    }

    /// Creates a transmission queue that manages the given encoders, and reuses them for new transmissions when possible.
    #[cfg(feature = "alloc")]
    pub fn queue<E: Encoder, S, const N: usize>(
        &mut self,
        encoders: [E; N],
    ) -> Pin<Box<AsyncTxQueue<'_, 'd, E, S, N>>> {
        // SAFETY: The queue is pinned on the heap, if it is forgotten,
        //         the memory the pending transmissions reference won't be freed.
        unsafe { Box::pin(self.raw_queue(encoders)) }
    }

    /// Starts transmitting the signal using the specified encoder and config.
    ///
    /// # Safety
    ///
    /// This function is a thin wrapper around the `rmt_transmit` function, it assumes that
    /// - the encoder is valid until the transmission is done (if not it is guaranteed to crash)
    /// - the signal is valid until the transmission is done
    /// - the encoder and signal are not modified during the transmission
    ///
    /// The caller must ensure that the encoder and signal live long enough and are not moved.
    ///
    /// # Cancel Safety
    ///
    /// If the future is still in progress, cancelling it will abort sending the transmission.
    /// There will be no leaked transmissions.
    pub async fn start_send<E: RawEncoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
        loop {
            match unsafe { self.driver().start_send(encoder, signal, config) } {
                Ok(token) => return Ok(token),
                // Invalid state is returned when the rmt driver queue is declared non-blocking and is full,
                // but also when the channel is not enabled.
                //
                // This retries only in the former case:
                Err(err) if err.code() == ESP_ERR_INVALID_STATE && self.is_enabled() => {
                    // The RMT driver is busy, wait a bit and try again
                    yield_now().await;
                }
                Err(err) => return Err(err),
            }
        }
    }

    /// Sends the given signal using the specified encoder and config.
    ///
    /// It will store the encoder and signal internally to ensure that they are not dropped
    /// before the transmission is done.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context.
    ///
    /// # Cancel Safety
    ///
    /// If the future is still in progress, cancelling it will abort sending the transmission.
    /// There will be no leaked transmissions.
    #[cfg(feature = "alloc")]
    pub async fn send<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        signal: impl AsRef<[E::Item]> + 'static,
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
        assert_not_in_isr();

        let signal = alloc::boxed::Box::pin(signal);
        let mut encoder = alloc::boxed::Box::pin(into_raw(encoder));

        // SAFETY: Both encoder and signal are pinned, and are stored in the channel to ensure that they live long enough
        let token = unsafe {
            // CANCEL-SAFETY: start_send is cancel-safe
            //             -> if this is cancelled, no transmission will be started
            //             -> it is safe to drop the encoder and signal.
            self.start_send(
                encoder.as_mut().get_unchecked_mut(),
                signal.as_ref().get_ref().as_ref(),
                config,
            )
            .await
        }?;

        self.driver().store_data(token, encoder, signal);

        Ok(token)
    }

    /// Sends the given signal using the specified encoder and config,
    /// then waits for the transmission to finish before returning.
    ///
    /// This a convenience function that combines `send` and `wait_for`.
    ///
    /// # Cancel Safety
    ///
    /// If the future is cancelled, before it starts sending, no transmission
    /// will be started.
    ///
    /// If the future is still in progress, cancelling it will abort waiting for
    /// the transmission to finish. The transmission will continue in the background,
    /// with no option to wait for it explicitly. One can wait for all pending transmissions
    /// see [`Self::wait_all_done`], disable the channel see [`TxChannelDriver::disable`] or
    /// wait for any subsequent transmission, which would wait for this one as well.
    ///
    /// There will be no undefined behavior or leaked memory. It will be cleaned
    /// up after a subsequent transmission is waited for or the channel is disabled.
    ///
    /// If this is not acceptable, consider using [`Self::send`] and [`Self::wait_for`]
    /// separately.
    #[cfg(feature = "alloc")]
    pub async fn send_and_wait<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        signal: impl AsRef<[E::Item]> + 'static,
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        // TODO: I think one could reduce the requirements here if one were to use the queue feature
        //       Would have the cost that it blocks on drop? Would make a select kind of pointless
        let token = self.send(encoder, signal, config).await?;
        self.wait_for(token).await;
        Ok(())
    }

    pub(super) async fn wait_for_with(channel: &mut TxChannelDriver<'d>, token: Token) {
        if !token.is_from(channel) {
            panic!("The given token {token:?} is not from this channel.");
        }

        // Only wait if the transmission is not already finished: (ensures that it does not wait when the channel is disabled)
        if !channel.is_finished(token) {
            channel.progress().wait_for(token.id).await;
        }

        channel.cleanup_data();
    }

    /// Waits for the transmission associated with the given token to finish.
    ///
    /// # Panics
    ///
    /// If the given token was not generated by this channel or is no longer valid.
    ///
    /// # Cancel Safety
    ///
    /// If the future is still in progress, cancelling it will simply abort waiting for
    /// it to finish. After cancelling, one can call this function again to wait for the
    /// transmission to finish.
    #[cfg(feature = "alloc")]
    pub async fn wait_for(&mut self, token: Token) {
        Self::wait_for_with(self.driver(), token).await;
    }
}

impl<'d> RmtChannel for AsyncTxChannelDriver<'d> {
    fn handle(&self) -> rmt_channel_handle_t {
        self.driver.handle()
    }

    fn is_enabled(&self) -> bool {
        self.driver.is_enabled()
    }

    unsafe fn set_internal_enabled(&mut self, is_enabled: bool) {
        self.driver.set_internal_enabled(is_enabled);
    }
}
