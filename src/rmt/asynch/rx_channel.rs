use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::rmt::blocking::RxChannelDriver;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::{RmtChannel, Symbol};
use crate::task::queue::Queue;
use crate::task::{do_yield, yield_now};

async fn queue_recv_front<T: Copy>(queue: &Queue<T>) -> T {
    loop {
        match queue.recv_front(0) {
            Some((value, _)) => return value,
            None => {
                do_yield();
                yield_now().await;
            }
        }
    }
}

async fn queue_send_back<T: Copy>(queue: &Queue<T>, item: T) -> bool {
    loop {
        match queue.send_back(item, 0) {
            Ok(value) => return value,
            Err(_) => {
                do_yield();
                yield_now().await;
            }
        }
    }
}

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
    /// This function will wait until the receive operation is complete.
    ///
    /// # Note
    ///
    /// If there are events from other receive operations, it will try to push them to
    /// the end of the queue. Make sure there is enough space in the queue, otherwise this
    /// might function might miss received symbols or never finish.
    ///
    /// # Safety
    ///
    /// A future can be discarded while it is still in progress. This could result in it writing to the buffer
    /// after the buffer is freed or got moved. The caller must guarantee that this future is polled to completion.
    pub async unsafe fn receive_and_wait(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let ptr = unsafe { self.driver().start_receive(buffer, config) }?;

        let queue = self.driver().queue();
        let mut read = 0;
        loop {
            let item = queue_recv_front(queue).await;

            if ptr != item.received_symbols {
                queue_send_back(queue, item).await;
                continue;
            }

            read += item.num_symbols;

            if item.flags.is_last() != 0 {
                break;
            }
        }

        Ok(read)
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
