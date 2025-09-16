use core::marker::PhantomData;
use core::mem;
use core::num::NonZeroU32;

use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::interrupt;
use crate::rmt::config::{ReceiveConfig, RxChannelConfig};
use crate::rmt::RmtChannel;

pub struct RxChannel<'d> {
    handle: rmt_channel_handle_t,
    is_enabled: bool,
    on_recv_notifier: interrupt::asynch::HalIsrNotification,
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
        let mut handle: rmt_channel_handle_t = core::ptr::null_mut();
        esp!(unsafe { rmt_new_rx_channel(&sys_config, &mut handle) })?;

        let this = Self {
            is_enabled: false,
            handle,
            on_recv_notifier: interrupt::asynch::HalIsrNotification::new(),
            _p: PhantomData,
        };
        // The callback is necessary to receive the "done" event, therefore it is always registered.
        esp!(unsafe {
            rmt_rx_register_event_callbacks(
                handle,
                &Self::RX_EVENT_CALLBACKS,
                (&this.on_recv_notifier as *const _) as *mut _,
            )
        })?;

        Ok(this)
    }

    /// Receives RMT symbols into the provided buffer, returning the number of received symbols.
    ///
    /// This function will wait asynchronously until the receive operation is complete.
    pub async fn receive(
        &mut self,
        buffer: &mut [rmt_symbol_word_t],
        config: &ReceiveConfig,
    ) -> Result<u32, EspError> {
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
        // Start a new receive operation, this does not block:
        esp!(unsafe {
            rmt_receive(
                self.handle,
                buffer.as_mut_ptr() as *mut _,
                mem::size_of_val(buffer),
                &sys_config,
            )
        })?;

        Ok(self.on_recv_notifier.wait().await.get())
    }

    pub fn subscribe(
        &mut self,
        on_recv_done: impl FnMut(rmt_rx_done_event_data_t) + Send + 'static,
    ) {
        self.check();

        // TODO: allocate callback in IRAM?
        // See https://github.com/esp-rs/esp-idf-hal/issues/486
        let on_recv_done: alloc::boxed::Box<dyn FnMut(rmt_rx_done_event_data_t) + Send + 'static> =
            alloc::boxed::Box::new(on_recv_done);

        unsafe {
            ISR_HANDLERS[self.channel_id()] = Some(on_recv_done);
        }
    }

    /// Remove the ISR handler for when a transmission is done.
    pub fn unsubscribe(&mut self) {
        self.check();

        unsafe {
            ISR_HANDLERS[self.channel_id()] = None;
        }
    }

    fn channel_id(&self) -> usize {
        // rmt_channel_handle_t is a `typedef struct rmt_channel_t *rmt_channel_handle_t;`
        // which should correspond to a *mut rmt_channel_t
        unsafe { (self.handle as *mut rmt_channel_t).read() as usize }
    }

    fn check(&self) {
        if interrupt::active() {
            panic!("This function cannot be called from an ISR");
        }
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        channel: rmt_channel_handle_t,
        event_data: *const rmt_rx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        // rmt_channel_handle_t is a `typedef struct rmt_channel_t *rmt_channel_handle_t;`
        // which should correspond to a *mut rmt_channel_t
        let id = (channel as *mut rmt_channel_t).read() as usize;
        let data = event_data.read();
        let notifier = &*(user_data as *const interrupt::asynch::HalIsrNotification);
        let received_symbols = NonZeroU32::new_unchecked(data.num_symbols as u32);

        interrupt::with_isr_yield_signal(move || {
            if let Some(handler) = ISR_HANDLERS[id].as_mut() {
                handler(data);
            }

            notifier.notify(received_symbols);
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
        let _res = self.disable();

        unsafe {
            rmt_rx_register_event_callbacks(
                self.handle,
                &Self::RX_EVENT_CALLBACKS_DISABLE,
                core::ptr::null_mut(),
            )
        };

        unsafe {
            ISR_HANDLERS[self.channel_id()] = None;
        }
        self.on_recv_notifier.reset();

        unsafe { rmt_del_channel(self.handle) };
    }
}
type IsrHandler = Option<alloc::boxed::Box<dyn FnMut(rmt_rx_done_event_data_t)>>;
static mut ISR_HANDLERS: [IsrHandler; rmt_channel_t_RMT_CHANNEL_MAX as usize] =
    [const { None }; rmt_channel_t_RMT_CHANNEL_MAX as usize];
