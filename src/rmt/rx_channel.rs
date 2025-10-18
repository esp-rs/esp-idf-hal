use core::fmt;
use core::marker::PhantomData;
use core::mem;
use core::ptr;

use alloc::boxed::Box;
use alloc::vec::Vec;

use esp_idf_sys::*;

use crate::delay::TickType;
use crate::gpio::InputPin;
use crate::interrupt::asynch::HalIsrNotification;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::{assert_not_in_isr, RmtChannel, RxDoneEventData, Symbol};
use crate::task::queue::Queue;

struct UserData {
    queue: Queue<RxDoneEventData>,
    callback: Option<Box<dyn FnMut(RxDoneEventData) + Send + 'static>>,
    notif: HalIsrNotification,
}

pub struct RxChannelDriver<'d> {
    handle: rmt_channel_handle_t,
    is_enabled: bool,
    user_data: Box<UserData>,
    buffer: Vec<Symbol>,
    has_finished: bool,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> RxChannelDriver<'d> {
    const RX_EVENT_CALLBACKS: rmt_rx_event_callbacks_t = rmt_rx_event_callbacks_t {
        on_recv_done: Some(Self::handle_isr),
    };
    const RX_EVENT_CALLBACKS_DISABLE: rmt_rx_event_callbacks_t =
        rmt_rx_event_callbacks_t { on_recv_done: None };
    // There is no need for storing more than one event, because the driver will overwrite the buffer
    // when a new event arrives.
    const ISR_QUEUE_SIZE: usize = 1;

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

        let mut handle: rmt_channel_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_rx_channel(&sys_config, &mut handle) })?;

        let mut this = Self {
            is_enabled: false,
            handle,
            user_data: Box::new(UserData {
                queue: Queue::new(Self::ISR_QUEUE_SIZE),
                callback: None,
                notif: HalIsrNotification::new(),
            }),
            buffer: Vec::new(),
            has_finished: true,
            _p: PhantomData,
        };

        esp!(unsafe {
            rmt_rx_register_event_callbacks(
                handle,
                &Self::RX_EVENT_CALLBACKS,
                // SAFETY: The referenced data will not be moved and this is the only reference to it
                &raw mut (*this.user_data) as *mut core::ffi::c_void,
            )
        })?;

        Ok(this)
    }

    /// Returns whether the last receive operation has finished.
    ///
    /// If a timeout happend before the `receive` finished, this will return `false`
    /// and the data can be obtained on the next call to [`Self::receive`].
    ///
    /// For partial receives, this will return `false`, until all parts have been received.
    ///
    /// If this function returns `true`, the next call to [`Self::receive`]
    /// will start a new receive operation.
    #[must_use]
    pub const fn has_finished(&self) -> bool {
        self.has_finished
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// This function will wait until the ISR signals that it has received data. After that,
    /// it copies the data from the internal buffer to the user-provided buffer and returns
    /// the number of received symbols.
    ///
    /// # Timeouts
    ///
    /// A timeout can be specified in the [`ReceiveConfig::timeout`] field. If a timeout happens,
    /// it will return an error with the [`EspError::code`] [`ESP_ERR_TIMEOUT`].
    /// If the data arrives after the timeout, it can be obtained on the next call to this function.
    ///
    /// # Partial Receives
    ///
    /// If partial receives are enabled (see [`ReceiveConfig::enable_partial_rx`]), subsequent
    /// calls to this function will only start a new receive after all parts have been received.
    ///
    /// The driver will continue to write data into the internal buffer after this function returns.
    /// It is important that this function is called without much delay, to not miss any data.
    ///
    /// One can check whether a partial receive has finished with [`Self::has_finished`].
    pub fn receive(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let timeout = config.timeout.map(TickType::from);
        // If the previous receive operation has finished, start a new one:
        if self.has_finished {
            // Ensure the internal buffer is large enough:
            if self.buffer.len() < buffer.len() {
                self.buffer.resize(buffer.len(), Symbol::default());
            }

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
            let slice = self.buffer.as_mut_slice();
            // SAFETY: The previous receive operation has finished -> the buffer is not in use.
            // It is allocated -> it will be valid for the duration of the receive operation.
            // In case the channel is forgotten, the buffer will be leaked, but the driver can
            // continue writing to it and when the driver is dropped, it will be disabled first.
            esp!(unsafe {
                rmt_receive(
                    self.handle,
                    slice.as_mut_ptr() as *mut _,
                    mem::size_of_val::<[Symbol]>(slice),
                    &sys_config,
                )
            })?;
        }

        // The ISR will send an event through the queue when new data has been received.
        // For partial receives, it will reuse the initial buffer for subsequent parts.

        self.has_finished = false;

        let event_data = self.wait(timeout)?;

        #[cfg(esp_idf_version_at_least_5_3_0)]
        {
            self.has_finished = event_data.is_last;
        }
        #[cfg(not(esp_idf_version_at_least_5_3_0))]
        {
            self.has_finished = true;
        }

        let read = event_data.num_symbols;
        // Before returning, copy the data to the user buffer:
        buffer[..read].copy_from_slice(&self.buffer[..read]);

        Ok(read)
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
    pub async fn receive_async(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let config_without_timeout = ReceiveConfig {
            timeout: None,
            ..config.clone()
        };

        loop {
            match self.receive(buffer, &config_without_timeout) {
                Ok(n) => return Ok(n),
                Err(err) if err.code() == ESP_ERR_TIMEOUT => {
                    // Wait for the ISR to notify us that new data has arrived:
                    self.user_data.notif.wait().await;
                }
                Err(err) => return Err(err),
            }
        }
    }

    fn wait(&mut self, timeout: Option<TickType>) -> Result<RxDoneEventData, EspError> {
        let has_timeout = timeout.is_some();
        let ticks = timeout.map_or(u32::MAX, |tick| tick.ticks());
        loop {
            if let Some((data, _)) = self.user_data.queue.recv_front(ticks) {
                return Ok(data);
            }

            if has_timeout {
                return Err(EspError::from_infallible::<ESP_ERR_TIMEOUT>());
            }
        }
    }

    /// Subscribe to the ISR handler to get notified when a receive operation is done.
    ///
    /// There is only one callback possible, you can not subscribe multiple callbacks.
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
    pub unsafe fn subscribe(&mut self, on_recv_done: impl FnMut(RxDoneEventData) + Send + 'static) {
        assert_not_in_isr();

        if self.is_enabled() {
            panic!("Cannot subscribe to RX events while the channel is enabled");
        }

        self.user_data.callback = Some(Box::new(on_recv_done));
    }

    /// Remove the ISR handler for when a transmission is done.
    pub fn unsubscribe(&mut self) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Cannot unsubscribe from RX events while the channel is enabled");
        }

        self.user_data.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        event_data: *const rmt_rx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let data = RxDoneEventData::from(event_data.read());
        let user_data = &mut *(user_data as *mut UserData);

        if let Some(handler) = user_data.callback.as_mut() {
            handler(data);
        }

        let raw_queue = &user_data.queue;

        // timeout is only relevant for non-isr calls
        let result = match raw_queue.send_back(data, 0) {
            Ok(result) => result,
            // it might fail if the queue is full, in this case discard the oldest event:
            Err(_) => {
                raw_queue.recv_front(0);
                raw_queue.send_back(data, 0).unwrap()
            }
        };

        user_data.notif.notify_lsb();

        result
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

        if !self.is_enabled {
            // If the channel is disabled, any receive operation has been aborted
            // -> it is finished:
            self.has_finished = true;
        }
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
            .field("has_finished", &self.has_finished)
            .finish()
    }
}
