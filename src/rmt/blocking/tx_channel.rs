use core::time::Duration;

use crate::gpio::OutputPin;

use esp_idf_sys::*;

use crate::rmt::config::{TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::Encoder;
use crate::rmt::{AsyncTxChannelDriver, RmtChannel, Scope};

#[derive(Debug)]
pub struct TxChannelDriver<'d> {
    driver: AsyncTxChannelDriver<'d>,
}

impl<'d> TxChannelDriver<'d> {
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
            driver: AsyncTxChannelDriver::new(pin, config)?,
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
    /// - `ESP_ERR_TIMEOUT`: Flush transactions failed because of timeout
    /// - `ESP_FAIL`: Flush transactions failed because of other error
    pub fn wait_all_done(&self, timeout: Option<Duration>) -> Result<(), EspError> {
        esp!(unsafe {
            rmt_tx_wait_all_done(
                self.handle(),
                timeout.map_or(-1, |duration| duration.as_millis() as i32),
            )
        })
    }

    /// Creates a new scope in which multiple transmissions can be sent without awaiting the previous ones.
    ///
    /// After the closure returns, this function will wait for all transmissions to
    /// finish before returning.
    ///
    /// # Note
    ///
    /// If a transmission never ends (e.g. [`Loop::Endless`]) this function will never return.
    ///
    /// # Errors
    ///
    /// This function will return any error returned by the closure or by [`Self::wait_all_done`].
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context.
    pub fn scope<'env, F, R>(&'env mut self, f: F) -> Result<R, EspError>
    where
        for<'s, 'channel> F: FnOnce(&'s mut Scope<'channel, 'env>) -> Result<R, EspError>,
    {
        crate::task::block_on(self.driver.scope(async move |scope| f(scope)))
    }

    /// Sends the given signal using the specified encoder and config,
    /// then waits for the transmission to finish before returning.
    pub fn send_and_wait<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        self.scope(|scope| {
            scope.send_ref(encoder, signal, config)?;
            Ok(())
        })?;

        Ok(())
    }

    /// Add ISR handler for when a transmission is done.
    ///
    /// The callback will be called with the number of transmitted symbols, including one EOF symbol,
    /// which is appended by the driver to mark the end of the transmission. For a loop transmission,
    /// this value only counts for one round.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    ///
    /// # Safety
    ///
    /// Care should be taken not to call std, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// You are not allowed to block, but you are allowed to call FreeRTOS APIs with the FromISR suffix.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(
        &mut self,
        callback: impl Fn(rmt_tx_done_event_data_t) + Send + 'static,
    ) {
        self.driver.subscribe(callback);
    }

    /// Remove the ISR handler for when a transmission is done.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        self.driver.unsubscribe();
    }
}

impl<'d> RmtChannel for TxChannelDriver<'d> {
    fn handle(&self) -> rmt_channel_handle_t {
        self.driver.handle()
    }

    fn is_enabled(&self) -> bool {
        self.driver.is_enabled()
    }

    unsafe fn set_internal_enabled(&mut self, is_enabled: bool) {
        self.driver.set_internal_enabled(is_enabled)
    }
}
