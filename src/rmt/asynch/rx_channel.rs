use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem;
use core::pin::Pin;

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
    buffers: Vec<Pin<Box<[Symbol]>>>,
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
            buffers: Vec::new(),
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
    /// function might miss received symbols or never finish.
    pub async fn receive_and_wait(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        // It is unsafe to pass the user buffer to the driver, because a
        // future might be dropped while the driver is still using it.
        //
        // To avoid this, an internal buffer is used, that is later copied to the user buffer:
        self.buffers.push(Box::into_pin(
            vec![Symbol::default(); buffer.len()].into_boxed_slice(),
        ));

        let internal_buffer = unsafe { self.buffers.last_mut().unwrap_unchecked().as_mut() };

        let ptr = unsafe {
            self.driver
                .start_receive(internal_buffer.get_unchecked_mut(), config)
        }?;

        let queue = self.driver.queue();
        let mut read = 0;

        struct CleanupOnDrop<T> {
            has_completed: bool,
            queue: Queue<rmt_rx_done_event_data_t>,
            buffers: (*mut T, usize, usize),
        }

        impl<T> Drop for CleanupOnDrop<T> {
            fn drop(&mut self) {
                if self.has_completed {
                    return;
                }

                // TODO: figure out a way to actually await the completion here, the below might result in an infinite loop
                //       or some other bug.

                // If the future is cancelled, wait for the receive operation to complete
                while let Some((item, _)) = self.queue.recv_front(u32::MAX) {
                    if item.flags.is_last() != 0 {
                        break;
                    }
                }

                // This constructs self.buffers back into a Vec to remove that unused buffer.
                // self cannot have a regular reference to self.buffers, because it would conflict
                // with the code that copies the data to the user buffer.

                // SAFETY: The future got cancelled, and the above ensures that the driver is not using the buffer anymore.
                unsafe {
                    let mut buffers =
                        Vec::from_raw_parts(self.buffers.0, self.buffers.1, self.buffers.2);

                    buffers.pop();

                    // We don't want to drop `self.buffers`
                    mem::forget(buffers);
                }
            }
        }

        let mut completion = CleanupOnDrop {
            has_completed: false,
            // SAFETY: CleanupOnDrop is always dropped before self is dropped.
            queue: unsafe { Queue::new_borrowed(self.driver.queue().as_raw()) },
            buffers: (
                self.buffers.as_mut_ptr(),
                self.buffers.len(),
                self.buffers.capacity(),
            ),
        };

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

        completion.has_completed = true;

        let internal_buffer = unsafe { self.buffers.pop().unwrap_unchecked() };

        // Before returning, copy the data to the user buffer:
        buffer[..read].copy_from_slice(&internal_buffer[..read]);

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
