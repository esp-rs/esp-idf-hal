use core::marker::PhantomData;
use core::mem;
use core::pin::pin;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::interrupt;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::{RmtChannel, Symbol};

struct TransmissionProgress {
    #[cfg(feature = "alloc")]
    notif: crate::interrupt::asynch::HalIsrNotification,
    received_symbols: AtomicUsize,
}

impl TransmissionProgress {
    fn signal(&self, received_symbols: usize) {
        self.received_symbols
            .store(received_symbols, Ordering::SeqCst);
    }

    fn wait(&self) -> usize {
        while self.received_symbols.load(Ordering::SeqCst) != 0 {
            #[cfg(feature = "alloc")]
            crate::task::block_on(self.notif.wait());
            #[cfg(not(feature = "alloc"))]
            crate::task::do_yield();
        }

        self.received_symbols.load(Ordering::SeqCst)
    }
}

struct UserData<'a> {
    progress: &'a TransmissionProgress,
    callback: Option<&'a mut (dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static)>,
}

pub struct RxChannel<'d> {
    handle: rmt_channel_handle_t,
    is_enabled: bool,
    #[cfg(feature = "alloc")]
    callback: Option<alloc::boxed::Box<dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static>>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> RxChannel<'d> {
    const RX_EVENT_CALLBACKS: rmt_rx_event_callbacks_t = rmt_rx_event_callbacks_t {
        on_recv_done: Some(Self::handle_isr),
    };
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
            mem_block_symbols: config.memory_block_symbols,
            #[cfg(esp_idf_version_at_least_5_1_2)]
            intr_priority: config.interrupt_priority,
            flags: rmt_rx_channel_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_rx_channel_config_t__bindgen_ty_1::new_bitfield_1(
                    config.flags.invert_in as u32,
                    config.flags.with_dma as u32,
                    config.flags.io_loop_back as u32,
                    #[cfg(esp_idf_version_at_least_5_4_0)]
                    {
                        config.flags.allow_pd as u32
                    },
                ),
                ..Default::default()
            },
            gpio_num: pin.pin() as _,
        };
        let mut handle: rmt_channel_handle_t = ptr::null_mut();
        esp!(unsafe { rmt_new_rx_channel(&sys_config, &mut handle) })?;

        Ok(Self {
            is_enabled: false,
            handle,
            #[cfg(feature = "alloc")]
            callback: None,
            _p: PhantomData,
        })
    }

    /// A helper function that registers the ISR handler before calling the provided closure
    /// and always unregisters the ISR handler afterwards.
    fn with_callback<R>(
        handle: rmt_channel_handle_t,
        callback: Option<&mut (dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static)>,
        f: impl FnOnce(&TransmissionProgress) -> Result<R, EspError>,
    ) -> Result<R, EspError> {
        super::assert_not_in_isr();

        let progress = TransmissionProgress {
            #[cfg(feature = "alloc")]
            notif: crate::interrupt::asynch::HalIsrNotification::new(),
            received_symbols: AtomicUsize::new(0),
        };
        // This struct contains the data that will be passed to the ISR handler.
        // The C code wants a mutable pointer, therefore the data must be pinned, to ensure
        // that rust does not invalidate the pointer through moves:
        let mut user_data = pin!(UserData {
            progress: &progress,
            callback,
        });

        esp!(unsafe {
            rmt_rx_register_event_callbacks(
                handle,
                &Self::RX_EVENT_CALLBACKS,
                // SAFETY: The referenced data will not be moved and this is the only reference to it
                user_data.as_mut().get_unchecked_mut() as *mut UserData as *mut core::ffi::c_void,
            )
        })?;

        #[cfg(not(feature = "std"))]
        let result = f(&progress);
        #[cfg(feature = "std")]
        // in std environments, catch panics:
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&progress)));

        // Disable the callback to prevent use-after-free bugs:
        esp!(unsafe {
            rmt_rx_register_event_callbacks(
                handle,
                &Self::RX_EVENT_CALLBACKS_DISABLE,
                ptr::null_mut(),
            )
        })
        // SAFETY: This must always succeed, if it doesn't it might cause undefined behavior
        .expect("failed to disable RX callback, this is a bug in the esp-idf-hal library.");

        #[cfg(not(feature = "std"))]
        {
            result
        }
        #[cfg(feature = "std")]
        match result {
            Ok(value) => value,
            Err(error) => std::panic::resume_unwind(error),
        }
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// This function will wait until the receive operation is complete.
    pub fn receive(
        &mut self,
        buffer: &mut [Symbol],
        config: &ReceiveConfig,
    ) -> Result<usize, EspError> {
        let sys_config = rmt_receive_config_t {
            signal_range_min_ns: config.signal_range_min.as_nanos() as _,
            signal_range_max_ns: config.signal_range_max.as_nanos() as _,
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

        let handle = self.handle();
        Self::with_callback(
            handle,
            {
                #[cfg(feature = "alloc")]
                {
                    self.callback.as_mut().map(alloc::boxed::Box::as_mut)
                }
                #[cfg(not(feature = "alloc"))]
                {
                    None
                }
            },
            |progress| {
                // Start a new receive operation, this does not block:
                esp!(unsafe {
                    rmt_receive(
                        self.handle,
                        buffer.as_mut_ptr() as *mut _,
                        mem::size_of_val(buffer),
                        &sys_config,
                    )
                })?;

                // Wait for the end of the receive operation:
                Ok(progress.wait())
            },
        )
    }

    #[cfg(feature = "alloc")]
    pub fn subscribe(
        &mut self,
        on_recv_done: impl FnMut(rmt_rx_done_event_data_t) + Send + 'static,
    ) {
        super::assert_not_in_isr();
        if self.is_enabled() {
            panic!("Cannot subscribe to RX events while the channel is enabled");
        }

        // TODO: allocate callback in IRAM?
        // See https://github.com/esp-rs/esp-idf-hal/issues/486
        let on_recv_done: alloc::boxed::Box<dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static> =
            alloc::boxed::Box::new(on_recv_done);

        self.callback = Some(on_recv_done);
    }

    /// Remove the ISR handler for when a transmission is done.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        super::assert_not_in_isr();
        if self.is_enabled() {
            panic!("Cannot unsubscribe from RX events while the channel is enabled");
        }

        self.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        event_data: *const rmt_rx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let data = event_data.read();
        let user_data = &mut *(user_data as *mut UserData);
        let received_symbols = data.num_symbols;

        interrupt::with_isr_yield_signal(move || {
            if let Some(handler) = user_data.callback.as_mut() {
                handler(data);
            }

            user_data.progress.signal(received_symbols);
        })
    }
}

impl<'d> RmtChannel for RxChannel<'d> {
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

impl<'d> Drop for RxChannel<'d> {
    fn drop(&mut self) {
        // Deleting the channel might fail if it is not disabled first.
        //
        // The result is ignored here, because there is nothing we can do about it.
        if self.is_enabled() {
            let _res = self.disable();
        }

        unsafe { rmt_del_channel(self.handle) };
    }
}
