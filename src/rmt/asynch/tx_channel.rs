use core::time::Duration;

use crate::gpio::OutputPin;
use crate::rmt::blocking::TxChannelDriver;

use esp_idf_sys::*;

#[cfg(feature = "alloc")]
use crate::rmt::assert_not_in_isr;
use crate::rmt::config::{TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::Encoder;
use crate::rmt::RmtChannel;
use crate::rmt::Token;

#[derive(Debug)]
pub struct AsyncTxChannelDriver<'d> {
    driver: TxChannelDriver<'d>,
}

impl<'d> AsyncTxChannelDriver<'d> {
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
        Ok(Self {
            driver: TxChannelDriver::new(pin, config)?,
        })
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
            match self.driver.wait_all_done(Some(Duration::ZERO)) {
                Ok(()) => return Ok(()),
                Err(e) if e.code() == esp_idf_sys::ESP_ERR_TIMEOUT => {
                    // Yield to allow other tasks to run
                    crate::task::yield_now().await;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Returns a mutable reference to the underlying blocking TX channel driver.
    #[must_use]
    pub fn driver(&mut self) -> &mut TxChannelDriver<'d> {
        &mut self.driver
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
    pub async unsafe fn start_send<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
        loop {
            match unsafe { self.driver.start_send(encoder, signal, config) } {
                Ok(token) => return Ok(token),
                // Invalid state is returned when the rmt driver queue is declared non-blocking and is full,
                // but also when the channel is not enabled.
                //
                // This retries only in the former case:
                Err(err) if err.code() == ESP_ERR_INVALID_STATE && self.is_enabled() => {
                    // The RMT driver is busy, wait a bit and try again
                    crate::task::do_yield();
                    crate::task::yield_now().await;
                }
                Err(err) => return Err(err),
            }
        }
    }

    #[cfg(feature = "alloc")]
    pub async fn send<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        signal: impl AsRef<[E::Item]> + 'static,
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
        assert_not_in_isr();

        let signal = alloc::boxed::Box::pin(signal);
        let mut encoder = alloc::boxed::Box::pin(encoder);

        // SAFETY: Both encoder and signal are pinned, and are stored in the channel to ensure that they live long enough
        let token = unsafe {
            self.start_send(
                encoder.as_mut().get_unchecked_mut(),
                signal.as_ref().get_ref().as_ref(),
                config,
            )
            .await
        }?;

        // Store the data to keep it alive until the transmission is done
        self.driver.store_data(token, encoder, signal);

        Ok(token)
    }

    /// Sends the given signal using the specified encoder and config,
    /// then waits for the transmission to finish before returning.
    ///
    /// This a convenience function that combines `send` and `wait_for`.
    #[cfg(feature = "alloc")]
    pub async fn send_and_wait<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        signal: impl AsRef<[E::Item]> + 'static,
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        let token = self.send(encoder, signal, config).await?;
        self.wait_for(token).await;
        Ok(())
    }

    /// Waits for the transmission associated with the given token to finish.
    #[cfg(feature = "alloc")]
    pub async fn wait_for(&mut self, token: Token) {
        if !token.is_from(self) {
            panic!("The given token {token:?} is not from this channel.");
        }

        // Only wait if the transmission is not already finished: (ensures that it does not wait when the channel is disabled)
        if !self.driver.is_finished(token) {
            self.driver.progress().wait_for(token.id).await;
        }

        self.driver.cleanup_data();
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
