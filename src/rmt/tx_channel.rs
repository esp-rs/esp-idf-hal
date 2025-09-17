use alloc::boxed::Box;
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::time::Duration;

use crate::gpio::OutputPin;
use crate::interrupt::asynch::HalIsrNotification;

use esp_idf_sys::*;

use crate::interrupt;
use crate::rmt::config::{Loop, TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::Encoder;
use crate::rmt::RmtChannel;

/// A handle to a sent transmission that can be used to wait for its completion.
pub struct TxHandle<'t> {
    id: usize,
    notif: &'t HalIsrNotification,
    finished_count: &'t AtomicUsize,
}

impl<'t> TxHandle<'t> {
    /// Waits asynchronously for the transmission to complete.
    ///
    /// # Note
    ///
    /// If the transmission never ends (e.g. [`Loop::Endless`]) this function will never return.
    pub async fn wait(&mut self) {
        // Calling self.notif.wait() is explicitly not allowed from an ISR context,
        // this makes sure that this is not called from an ISR:
        super::assert_not_in_isr();

        // The notif will notify every time a transmission is done, this might not
        // be the one we are waiting for, therefore the id is checked here:
        while self.finished_count.load(Ordering::SeqCst) <= self.id {
            self.notif.wait().await;
        }
    }

    /// Waits for the transmission to complete.
    ///
    /// # Note
    ///
    /// If the transmission never ends (e.g. [`Loop::Endless`]) this function will never return.
    pub fn wait_blocking(&mut self) {
        #[cfg(feature = "alloc")]
        crate::task::block_on(self.wait());
        #[cfg(not(feature = "alloc"))]
        while self.finished_count.load(Ordering::SeqCst) <= self.id {
            crate::task::do_yield();
        }
    }
}

/// A scope in which multiple transmissions can be sent.
pub struct Scope<'channel, 'env> {
    channel: &'channel TxChannel<'channel>,
    is_canceled: bool,
    _p: PhantomData<&'env mut ()>,
}

impl<'channel, 'env> Scope<'channel, 'env> {
    // SAFETY:
    //
    // It must be guaranteed that the encoder and signal live for the 'env lifetime.
    //
    // Make sure that the send function uses an `&mut self` reference and not `&self`,
    // otherwise the compiler might shorten the lifetime of the parameters:
    // https://doc.rust-lang.org/nomicon/subtyping.html#variance
    //
    // resulting in use-after-free bugs.

    /// Starts transmitting the given signal using the specified encoder and config.
    ///
    /// This is a safe API for [`TxChannel::start_send`], see its documentation for more details
    /// on the behavior.
    pub fn send<E: Encoder>(
        &mut self,
        encoder: &'env mut E,
        signal: &'env [E::Item],
        config: &TransmitConfig,
    ) -> Result<TxHandle<'channel>, EspError> {
        super::assert_not_in_isr();

        // SAFETY: The lifetimes on the encoder and signal ensure that they live until 'env
        unsafe { self.channel.start_send(encoder, signal, config) }
    }

    /// Indicates that the channel should be disabled (will cancel all pending transmissions)
    /// after the scope ends.
    ///
    /// # Note
    ///
    /// This does not immediately disable the channel, it will disable it after the scope ends.
    pub fn cancel(&mut self) {
        self.is_canceled = true;
    }
}

struct UserData {
    // TODO: is it okay to expose rmt_tx_done_event_data_t directly?
    #[cfg(feature = "alloc")]
    callback: Option<alloc::boxed::Box<dyn Fn(rmt_tx_done_event_data_t) + Send + 'static>>,
    next_tx_id: AtomicUsize,
    finished_count: AtomicUsize,
    notif: HalIsrNotification,
}

pub struct TxChannel<'d> {
    is_enabled: bool,
    handle: rmt_channel_handle_t,
    user_data: Box<UserData>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TxChannel<'d> {
    const TX_EVENT_CALLBACKS: rmt_tx_event_callbacks_t = rmt_tx_event_callbacks_t {
        on_trans_done: Some(Self::handle_isr),
    };
    const TX_EVENT_CALLBACKS_DISABLE: rmt_tx_event_callbacks_t = rmt_tx_event_callbacks_t {
        on_trans_done: None,
    };

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
        super::assert_not_in_isr();

        let sys_config: rmt_tx_channel_config_t = rmt_tx_channel_config_t {
            clk_src: config.clock_source.into(),
            resolution_hz: config.resolution.into(),
            mem_block_symbols: config.memory_block_symbols,
            trans_queue_depth: config.transaction_queue_depth,
            #[cfg(esp_idf_version_at_least_5_1_2)]
            intr_priority: config.interrupt_priority,
            flags: rmt_tx_channel_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_tx_channel_config_t__bindgen_ty_1::new_bitfield_1(
                    config.flags.invert_out as u32,
                    config.flags.with_dma as u32,
                    config.flags.io_loop_back as u32,
                    config.flags.io_od_mode as u32,
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
        esp!(unsafe { rmt_new_tx_channel(&sys_config, &mut handle) })?;

        let this = Self {
            is_enabled: false,
            handle,
            // TODO: technically must be pined?
            user_data: Box::new(UserData {
                #[cfg(feature = "alloc")]
                callback: None,
                next_tx_id: AtomicUsize::new(0),
                finished_count: AtomicUsize::new(0),
                notif: HalIsrNotification::new(),
            }),
            _p: PhantomData,
        };

        // The callback is used to detect when a transmission is finished.
        // Therefore it is always registered:
        esp!(unsafe {
            rmt_tx_register_event_callbacks(
                handle,
                &Self::TX_EVENT_CALLBACKS,
                // TODO: the Ok(this) likely moves this -> will invalidate the pointer...
                this.user_data.as_ref() as *const UserData as *mut core::ffi::c_void,
            )
        })?;

        Ok(this)
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
                self.handle,
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
        super::assert_not_in_isr();

        let (is_canceled, result) = {
            let channel: &TxChannel<'_> = &*self;
            let mut scope = Scope {
                channel: channel,
                is_canceled: false,
                _p: PhantomData,
            };

            let result = { f(&mut scope) };

            (scope.is_canceled, result)
        };

        if is_canceled {
            // If the scope was canceled, we don't wait for all done, instead disable the channel.
            self.disable()?;
        } else {
            self.wait_all_done(None)?;
        }

        result
    }

    pub fn send_and_wait<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        self.scope(|scope| {
            scope.send(encoder, signal, config)?;
            Ok(())
        })?;

        Ok(())
    }

    /// Transmits the signal using the specified encoder and config.
    ///
    /// It will not wait for the transmission to finish, but a [`TxHandle`] is returned
    /// which can be used for this.
    ///
    /// # Safety
    ///
    /// The encoder is passed to the [`rmt_transmit`] function which then calls the
    /// encoders functions to transform the signal into RMT symbols. You must ensure
    /// that the encoder is implemented correctly, for more details, see the [RMT Encoder]
    /// documentation.
    ///
    /// The passed signal must be valid and can not be modified, until the transmission is done.
    /// One can await the transmission end by calling [`Self::wait_all_done`] or registering a
    /// callback to be notified when the transmission is done.
    ///
    /// If you are looking for a safe alternative, consider using [`Self::scope`].
    ///
    /// [RMT Encoder]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html#rmt-encoder
    ///
    /// # Queue blocking behavior
    ///
    /// This function constructs a transaction descriptor then pushes to a queue. The transaction
    /// will not start immediately if there's another one under processing. Based on the setting
    /// of [`TransmitConfig::queue_non_blocking`], if there're too many transactions pending in the
    /// queue, this function can block until it has free slot, otherwise just return quickly.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Transmit failed because of invalid argument
    /// - `ESP_ERR_INVALID_STATE`: The channel is not enabled, make sure you called [`Self::enable`] before transmitting.
    /// - `ESP_ERR_NOT_SUPPORTED`: Some feature is not supported by hardware e.g. unsupported loop count
    /// - `ESP_FAIL`: Because of other errors
    pub unsafe fn start_send<'t, E: Encoder>(
        &'t self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<TxHandle<'t>, EspError> {
        let sys_config = rmt_transmit_config_t {
            loop_count: match config.loop_count {
                Loop::Count(value) => value as i32,
                Loop::Endless => -1,
                Loop::None => 0,
            },
            flags: rmt_transmit_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_transmit_config_t__bindgen_ty_1::new_bitfield_1(
                    config.eot_level as u32,
                    #[cfg(esp_idf_version_at_least_5_1_3)]
                    {
                        config.queue_non_blocking as u32
                    },
                ),
                ..Default::default()
            },
        };

        // This gets the id for the current transmission, which is
        // later used to check if the transmission is done.
        let id = self.user_data.next_tx_id.fetch_add(1, Ordering::SeqCst);

        let handle = TxHandle {
            notif: &self.user_data.notif,
            id,
            finished_count: &self.user_data.finished_count,
        };
        esp!(unsafe {
            rmt_transmit(
                self.handle,
                encoder.handle(),
                signal.as_ptr() as *const _,
                // size should be given in bytes:
                mem::size_of_val(signal),
                &sys_config,
            )
        })?;

        Ok(handle)
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
        super::assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't subscribe when the channel is enabled");
        }

        // TODO: allocate callback in IRAM?
        // See https://github.com/esp-rs/esp-idf-hal/issues/486
        let callback: alloc::boxed::Box<dyn Fn(rmt_tx_done_event_data_t) + Send + 'static> =
            alloc::boxed::Box::new(callback);

        // SAFETY: The interrupt handler is not called while the channel is disabled
        // -> it should be safe to change the callback.
        self.user_data.callback = Some(callback);
    }

    /// Remove the ISR handler for when a transmission is done.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        super::assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't unsubscribe when the channel is enabled");
        }

        // SAFETY: The interrupt handler is not called while the channel is disabled
        // -> it should be safe to change the callback.
        self.user_data.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        #[allow(unused_variables)] event_data: *const rmt_tx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        #[cfg(feature = "alloc")]
        let event_data = event_data.read();

        let user_data = &*(user_data as *const UserData);
        user_data.finished_count.fetch_add(1, Ordering::SeqCst);

        interrupt::with_isr_yield_signal(move || {
            #[cfg(feature = "alloc")]
            if let Some(handler) = user_data.callback.as_ref() {
                handler(event_data);
            }

            user_data.notif.notify_lsb();
        })
    }
}

impl<'d> RmtChannel for TxChannel<'d> {
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

impl<'d> Drop for TxChannel<'d> {
    fn drop(&mut self) {
        // Deleting the channel might fail if it is not disabled first.
        //
        // The result is ignored here, because there is nothing we can do about it.
        if self.is_enabled() {
            let _res = self.disable();
        }

        unsafe {
            rmt_tx_register_event_callbacks(
                self.handle,
                &Self::TX_EVENT_CALLBACKS_DISABLE,
                ptr::null_mut(),
            )
        };

        unsafe { rmt_del_channel(self.handle) };
    }
}
