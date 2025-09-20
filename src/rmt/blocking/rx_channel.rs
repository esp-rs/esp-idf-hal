use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::AsyncRxChannelDriver;
use crate::rmt::{RmtChannel, Symbol};
use crate::task::block_on;

#[derive(Debug)]
pub struct RxChannelDriver<'d> {
    driver: AsyncRxChannelDriver<'d>,
}

impl<'d> RxChannelDriver<'d> {
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
            driver: AsyncRxChannelDriver::new(pin, config)?,
        })
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// This function will wait until the receive operation is complete.
    pub fn receive_and_wait(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        block_on(self.driver.receive_and_wait(buffer, config))
    }

    #[cfg(feature = "alloc")]
    pub fn subscribe(
        &mut self,
        on_recv_done: impl FnMut(rmt_rx_done_event_data_t) + Send + 'static,
    ) {
        self.driver.subscribe(on_recv_done);
    }

    /// Remove the ISR handler for when a transmission is done.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        self.driver.unsubscribe();
    }
}

impl<'d> RmtChannel for RxChannelDriver<'d> {
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
