use core::fmt;
use core::marker::PhantomData;
use core::mem;
use core::ptr;

#[cfg(feature = "alloc")]
use alloc::boxed::Box;

use esp_idf_sys::*;

use crate::gpio::InputPin;
#[cfg(feature = "alloc")]
use crate::rmt::assert_not_in_isr;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::{RmtChannel, Symbol};
#[cfg(feature = "alloc")]
use crate::task::queue::Queue;

#[cfg(feature = "alloc")]
struct UserData {
    queue: Queue<rmt_rx_done_event_data_t>,
    callback: Option<Box<dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static>>,
}

pub struct RxChannelDriver<'d> {
    handle: rmt_channel_handle_t,
    is_enabled: bool,
    #[cfg(feature = "alloc")]
    user_data: Box<UserData>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> RxChannelDriver<'d> {
    #[cfg(feature = "alloc")]
    const RX_EVENT_CALLBACKS: rmt_rx_event_callbacks_t = rmt_rx_event_callbacks_t {
        on_recv_done: Some(Self::handle_isr),
    };
    #[cfg(feature = "alloc")]
    const RX_EVENT_CALLBACKS_DISABLE: rmt_rx_event_callbacks_t =
        rmt_rx_event_callbacks_t { on_recv_done: None };

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
        let sys_config = rmt_rx_channel_config_t {
            clk_src: config.clock_source.into(),
            resolution_hz: config.resolution.into(),
            mem_block_symbols: config.memory_access.symbols(),
            #[cfg(esp_idf_version_at_least_5_1_2)]
            intr_priority: config.interrupt_priority,
            flags: rmt_rx_channel_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_rx_channel_config_t__bindgen_ty_1::new_bitfield_1(
                    config.invert_in as u32,
                    config.memory_access.is_direct() as u32,
                    config.io_loop_back as u32,
                    #[cfg(esp_idf_version_at_least_5_4_0)]
                    {
                        config.allow_pd as u32
                    },
                ),
                ..Default::default()
            },
            gpio_num: pin.pin() as _,
        };

        if config.queue_size == 0 {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
        }

        let mut handle: rmt_channel_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_rx_channel(&sys_config, &mut handle) })?;

        #[cfg_attr(not(feature = "alloc"), allow(unused_mut))]
        let mut this = Self {
            is_enabled: false,
            handle,
            #[cfg(feature = "alloc")]
            user_data: Box::new(UserData {
                queue: Queue::new(config.queue_size),
                callback: None,
            }),
            _p: PhantomData,
        };

        #[cfg(feature = "alloc")]
        esp!(unsafe {
            rmt_rx_register_event_callbacks(
                handle,
                &Self::RX_EVENT_CALLBACKS,
                // SAFETY: The referenced data will not be moved and this is the only reference to it
                &raw mut this.user_data as *mut UserData as *mut core::ffi::c_void,
            )
        })?;

        Ok(this)
    }

    /// Initiate a receive job for RMT RX channel.
    ///
    /// This function is non-blocking, it initiates a new receive job and then returns
    /// the pointer it passed to the [`rmt_receive`] function. With this pointer one
    /// can identify the [`rmt_rx_done_event_data_t`] that corresponds to this receive
    /// operation.
    ///
    /// # ISR Safety
    ///
    /// If [`Self::is_enabled`] returns `true`, this function can be safely called from an ISR
    /// context.
    ///
    /// # Safety
    ///
    /// This function is a thin wrapper around the [`rmt_receive`] function, it assumes that
    /// the buffer is valid until all data has been received.
    pub unsafe fn start_receive(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<*mut rmt_symbol_word_t, EspError> {
        let sys_config = rmt_receive_config_t {
            signal_range_min_ns: config.signal_range_min.as_nanos() as _,
            signal_range_max_ns: config.signal_range_max.as_nanos() as _,
            #[cfg(esp_idf_version_at_least_5_3_0)]
            flags: rmt_receive_config_t_extra_rmt_receive_flags {
                _bitfield_1: rmt_receive_config_t_extra_rmt_receive_flags::new_bitfield_1(
                    config.enable_partial_rx as u32,
                ),
                ..Default::default()
            },
        };

        // Enable the channel if it is not enabled yet:
        if !self.is_enabled() {
            self.enable()?;
        }

        // Start a new receive operation, this does not block:
        let ptr = buffer.as_mut_ptr() as *mut rmt_symbol_word_t;
        esp!(unsafe {
            rmt_receive(
                self.handle,
                ptr as *mut _,
                mem::size_of_val(buffer),
                &sys_config,
            )
        })?;

        Ok(ptr)
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// This function will wait until the receive operation is complete.
    ///
    /// # Note
    ///
    /// If there are events from other receive operations, it will try to push them to
    /// the end of the queue. In case this fails, it will discard the event.
    #[cfg(feature = "alloc")]
    pub fn receive_and_wait(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let ptr = unsafe { self.start_receive(buffer, config) }?;

        let mut read = 0;
        while let Some((item, _)) = self.queue().recv_front(u32::MAX) {
            if ptr != item.received_symbols {
                // if it fails to send the unknown event to the back of the queue, just discard it.
                let _ = self.queue().send_back(item, u32::MAX);
                continue;
            }

            read += item.num_symbols;

            if item.flags.is_last() != 0 {
                break;
            }
        }

        Ok(read)
    }

    /// Returns a reference the queue to which events will be written.
    ///
    /// If the queue becomes full, it will discard the oldest events (the elements in the front).
    /// It is recommended to frequently poll the queue, to not miss any events.
    #[cfg(feature = "alloc")]
    #[must_use]
    pub fn queue(&self) -> &Queue<rmt_rx_done_event_data_t> {
        &self.user_data.queue
    }

    #[cfg(feature = "alloc")]
    pub fn subscribe(
        &mut self,
        on_recv_done: impl FnMut(rmt_rx_done_event_data_t) + Send + 'static,
    ) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Cannot subscribe to RX events while the channel is enabled");
        }

        self.user_data.callback = Some(Box::new(on_recv_done));
    }

    /// Remove the ISR handler for when a transmission is done.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Cannot unsubscribe from RX events while the channel is enabled");
        }

        self.user_data.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    #[cfg(feature = "alloc")]
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        event_data: *const rmt_rx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let data = event_data.read();
        let user_data = &mut *(user_data as *mut UserData);

        if let Some(handler) = user_data.callback.as_mut() {
            handler(data);
        }

        // timeout is only relevant for non-isr calls
        match user_data.queue.send_back(data, 0) {
            Ok(result) => result,
            // it might fail if the queue is full, in this case discard the oldest event:
            Err(_) => {
                user_data.queue.recv_front(0);

                user_data.queue.send_back(data, 0).unwrap()
            }
        }
    }
}

impl<'d> RmtChannel for RxChannelDriver<'d> {
    fn handle(&self) -> rmt_channel_handle_t {
        self.handle
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled
    }

    unsafe fn set_internal_enabled(&mut self, is_enabled: bool) {
        self.is_enabled = is_enabled;
    }
}

impl<'d> Drop for RxChannelDriver<'d> {
    fn drop(&mut self) {
        // Deleting the channel might fail if it is not disabled first.
        //
        // The result is ignored here, because there is nothing we can do about it.
        if self.is_enabled() {
            let _res = self.disable();
        }

        // Disable the callback to prevent use-after-free bugs:
        #[cfg(feature = "alloc")]
        unsafe {
            rmt_rx_register_event_callbacks(
                self.handle(),
                &Self::RX_EVENT_CALLBACKS_DISABLE,
                ptr::null_mut(),
            )
        };

        unsafe { rmt_del_channel(self.handle) };
    }
}

impl<'d> fmt::Debug for RxChannelDriver<'d> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RxChannelDriver")
            .field("is_enabled", &self.is_enabled)
            .field("handle", &self.handle)
            .finish()
    }
}
