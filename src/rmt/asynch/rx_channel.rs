use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::rmt::blocking::RxChannelDriver;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::{RmtChannel, Symbol};
use crate::task::{do_yield, yield_now};

#[derive(Debug)]
pub struct AsyncRxChannelDriver<'d> {
    driver: RxChannelDriver<'d>,
}

impl<'d> AsyncRxChannelDriver<'d> {
    /// Creates a new RMT RX channel.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Create RMT RX channel failed because of invalid argument
    /// - `ESP_ERR_NO_MEM`: Create RMT RX channel failed because out of memory
    /// - `ESP_ERR_NOT_FOUND`: Create RMT RX channel failed because all RMT channels are used up and no more free one
    /// - `ESP_ERR_NOT_SUPPORTED`: Create RMT RX channel failed because some feature is not supported by hardware, e.g. DMA feature is not supported by hardware
    /// - `ESP_FAIL`: Create RMT RX channel failed because of other error
    pub fn new(pin: impl InputPin + 'd, config: &RxChannelConfig) -> Result<Self, EspError> {
        Ok(Self {
            driver: RxChannelDriver::new(pin, config)?,
        })
    }

    /// Returns a mutable reference to the underlying blocking RX channel driver.
    #[must_use]
    pub fn driver(&mut self) -> &mut RxChannelDriver<'d> {
        &mut self.driver
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// For more details, see [`RxChannelDriver::receive`].
    ///
    /// # Cancel Safety
    ///
    /// If the future is cancelled before completion, the received data can be obtained on the next call
    /// to this function. Make sure that the buffer provided on the next call is large enough to hold
    /// the data.
    pub async fn receive(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let config_without_timeout = ReceiveConfig {
            timeout: None,
            ..config.clone()
        };

        loop {
            match self.driver.receive(buffer, &config_without_timeout) {
                Ok(n) => return Ok(n),
                Err(err) if err.code() == ESP_ERR_TIMEOUT => {
                    // Yield to allow other tasks to run before retrying.
                    do_yield();
                    yield_now().await;
                }
                Err(err) => return Err(err),
            }
        }
    }
}

impl<'d> RmtChannel for AsyncRxChannelDriver<'d> {
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
