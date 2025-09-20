use core::fmt;
use core::marker::PhantomData;
use core::mem;
use core::pin::pin;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};

use crate::gpio::OutputPin;

use esp_idf_sys::*;

use crate::interrupt::asynch::HalIsrNotification;
use crate::rmt::config::{Loop, TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::Encoder;
use crate::rmt::{assert_not_in_isr, RmtChannel};

#[cfg(feature = "alloc")]
type AnyPinned = core::pin::Pin<alloc::boxed::Box<dyn core::any::Any>>;

/// A handle to a sent transmission that can be used to wait for its completion.
pub struct TxHandle<'t> {
    id: usize,
    progress: &'t TransmissionProgress,
}

impl<'t> TxHandle<'t> {
    /// Waits for the transmission to complete.
    ///
    /// # Note
    ///
    /// If the transmission never ends (e.g. [`Loop::Endless`]) this function will never return.
    pub fn wait_blocking(&mut self) {
        #[cfg(feature = "alloc")]
        crate::task::block_on(self.wait());
        #[cfg(not(feature = "alloc"))]
        {
            // Calling self.notif.wait() is explicitly not allowed from an ISR context
            // (it should never block when called in an ISR context),
            // this makes sure that this is not called from an ISR:
            assert_not_in_isr();

            // The notif will notify every time a transmission is done, this might not
            // be the one we are waiting for, therefore the id is checked here:
            while !self.progress.is_finished(self.id) {
                crate::task::do_yield();
            }
        }
    }

    /// Waits for the transmission to complete.
    ///
    /// # Note
    ///
    /// If the transmission never ends (e.g. [`Loop::Endless`]) this function will never return.
    pub async fn wait(&mut self) {
        self.progress.wait_for(self.id).await
    }
}

/// A scope in which multiple transmissions can be sent.
pub struct Scope<'channel, 'env> {
    channel: &'channel AsyncTxChannelDriver<'channel>,
    is_canceled: bool,
    progress: &'channel TransmissionProgress,
    // The outstanding transmissions will still reference the encoder and signal,
    // it is stored here to ensure that they live long enough:
    #[cfg(feature = "alloc")]
    _data: alloc::vec::Vec<(usize, AnyPinned, AnyPinned)>,
    _p: PhantomData<&'env mut ()>,
}

impl<'channel, 'env> Scope<'channel, 'env> {
    /// # Safety
    ///
    /// The caller must ensure that the encoder and signal live long enough.
    unsafe fn start_send<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
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

        esp!(unsafe {
            rmt_transmit(
                self.channel.handle(),
                encoder.handle(),
                signal.as_ptr() as *const _,
                // size should be given in bytes:
                mem::size_of_val(signal),
                &sys_config,
            )
        })
    }

    // SAFETY:
    //
    // It must be guaranteed that the encoder and signal live for the 'env lifetime.
    //
    // Make sure that the send function uses an `&mut self` reference and not `&self`,
    // otherwise the compiler might shorten the lifetime of the parameters:
    // https://doc.rust-lang.org/nomicon/subtyping.html#variance
    //
    // resulting in use-after-free bugs.

    /// Transmits the signal using the specified encoder and config.
    ///
    /// It will **not** wait for the transmission to finish, but a [`TxHandle`] is returned
    /// which can be used for this.
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
    pub fn send_ref<E: Encoder>(
        &mut self,
        encoder: &'env mut E,
        signal: &'env [E::Item],
        config: &TransmitConfig,
    ) -> Result<TxHandle<'channel>, EspError> {
        assert_not_in_isr();

        unsafe { self.start_send(encoder, signal, config) }?;

        Ok(TxHandle {
            id: self.progress.next_id(),
            progress: self.progress,
        })
    }

    /// Transmits the signal using the specified encoder and config.
    ///
    /// This function will ensure that the encoder and signal live long enough by storing them
    /// in the scope.
    ///
    /// See also [`Self::send_ref`] for more details on the behavior of this function.
    #[cfg(feature = "alloc")]
    pub fn send<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        signal: impl Into<alloc::vec::Vec<E::Item>>,
        config: &TransmitConfig,
    ) -> Result<TxHandle<'channel>, EspError> {
        assert_not_in_isr();

        let signal = alloc::boxed::Box::pin(signal.into());
        let mut encoder = alloc::boxed::Box::pin(encoder);

        // SAFETY: Both encoder and signal are pinned, and are stored in the scope to ensure that they live long enough
        unsafe {
            self.start_send(
                encoder.as_mut().get_unchecked_mut(),
                signal.as_ref().get_ref(),
                config,
            )
        }?;

        // Store the pinned data to ensure it lives long enough:
        let id = self.progress.next_id();
        self.store_data(id, encoder, signal);

        Ok(TxHandle {
            id,
            progress: self.progress,
        })
    }

    /// Stores the data in the scope of the transmission to ensure that it lives long enough.
    ///
    /// While adding the transmission, it will check the progress to see if any old transmission
    /// data can be removed. (This helps to keep memory usage low when doing many transmissions.)
    #[cfg(feature = "alloc")]
    fn store_data(&mut self, id: usize, encoder: AnyPinned, signal: AnyPinned) {
        // Remove old data that is no longer needed:
        self._data
            .retain(|(stored_id, _, _)| !self.progress.is_finished(*stored_id));

        self._data.push((id, encoder, signal));
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

struct TransmissionProgress {
    next_tx_id: AtomicUsize,
    finished_count: AtomicUsize,
    notif: HalIsrNotification,
}

impl TransmissionProgress {
    /// Returns a new unique id for a transmission.
    #[must_use]
    fn next_id(&self) -> usize {
        self.next_tx_id.fetch_add(1, Ordering::SeqCst)
    }

    #[must_use]
    fn is_finished(&self, id: usize) -> bool {
        // Assuming id is 1, we have the transmissions [0, 1].
        // For id 1 to be finished, the count has to be at least 2.
        self.finished_count.load(Ordering::SeqCst) > id
    }

    /// Waits for the transmission with the given id to complete.
    async fn wait_for(&self, id: usize) {
        // Calling self.notif.wait() is explicitly not allowed from an ISR context
        // (it should never block when called in an ISR context),
        // this makes sure that this is not called from an ISR:
        assert_not_in_isr();

        // The notif will notify every time a transmission is done, this might not
        // be the one we are waiting for, therefore the id is checked here:
        while !self.is_finished(id) {
            #[cfg(feature = "alloc")]
            self.notif.wait().await;
            #[cfg(not(feature = "alloc"))]
            crate::task::yield_now().await;
        }
    }

    /// Signals that the next transmission has finished.
    fn signal_finished(&self) {
        self.finished_count.fetch_add(1, Ordering::SeqCst);
        self.notif.notify_lsb();
    }
}

struct UserData<'c> {
    callback: Option<&'c mut (dyn FnMut(rmt_tx_done_event_data_t) + Send + 'static)>,
    progress: &'c TransmissionProgress,
}

pub struct AsyncTxChannelDriver<'d> {
    is_enabled: bool,
    handle: rmt_channel_handle_t,
    #[cfg(feature = "alloc")]
    callback: Option<alloc::boxed::Box<dyn FnMut(rmt_tx_done_event_data_t) + Send + 'static>>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> AsyncTxChannelDriver<'d> {
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
        assert_not_in_isr();

        let sys_config: rmt_tx_channel_config_t = rmt_tx_channel_config_t {
            clk_src: config.clock_source.into(),
            resolution_hz: config.resolution.into(),
            mem_block_symbols: config.memory_access.symbols(),
            trans_queue_depth: config.transaction_queue_depth,
            #[cfg(esp_idf_version_at_least_5_1_2)]
            intr_priority: config.interrupt_priority,
            flags: rmt_tx_channel_config_t__bindgen_ty_1 {
                _bitfield_1: rmt_tx_channel_config_t__bindgen_ty_1::new_bitfield_1(
                    config.invert_out as u32,
                    config.memory_access.is_direct() as u32,
                    config.io_loop_back as u32,
                    config.io_od_mode as u32,
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
        esp!(unsafe { rmt_new_tx_channel(&sys_config, &mut handle) })?;

        Ok(Self {
            is_enabled: false,
            handle,
            #[cfg(feature = "alloc")]
            callback: None,
            _p: PhantomData,
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
    /// - `ESP_FAIL`: Flush transactions failed because of other error
    pub async fn wait_all_done(&self) -> Result<(), EspError> {
        loop {
            match esp!(unsafe { rmt_tx_wait_all_done(self.handle, 0) }) {
                Ok(()) => return Ok(()),
                Err(e) if e.code() == esp_idf_sys::ESP_ERR_TIMEOUT => {
                    // Yield to allow other tasks to run
                    crate::task::yield_now().await;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// A helper function that registers the ISR handler before calling the provided closure
    /// and always unregisters the ISR handler afterwards.
    async fn with_callback<R>(
        handle: rmt_channel_handle_t,
        callback: Option<&mut (dyn FnMut(rmt_tx_done_event_data_t) + Send + 'static)>,
        f: impl AsyncFnOnce(&TransmissionProgress) -> Result<R, EspError>,
    ) -> Result<R, EspError> {
        assert_not_in_isr();

        let progress = TransmissionProgress {
            next_tx_id: AtomicUsize::new(0),
            finished_count: AtomicUsize::new(0),
            notif: HalIsrNotification::new(),
        };

        // This struct contains the data that will be passed to the ISR handler.
        // The C code wants a mutable pointer, therefore the data must be pinned, to ensure
        // that rust does not invalidate the pointer through moves:
        let mut user_data = pin!(UserData {
            progress: &progress,
            callback,
        });

        // The callback is used to detect when a transmission is finished.
        esp!(unsafe {
            rmt_tx_register_event_callbacks(
                handle,
                &Self::TX_EVENT_CALLBACKS,
                // SAFETY: The referenced data will not be moved and this is the only reference to it
                user_data.as_mut().get_unchecked_mut() as *mut UserData as *mut core::ffi::c_void,
            )
        })?;

        let result = f(&progress).await;

        // Disable the callback to prevent use-after-free bugs:
        esp!(unsafe {
            rmt_tx_register_event_callbacks(
                handle,
                &Self::TX_EVENT_CALLBACKS_DISABLE,
                ptr::null_mut(),
            )
        })
        // SAFETY: This must always succeed, if it doesn't it might cause undefined behavior
        .expect("failed to disable TX callback, this is a bug in the esp-idf-hal library.");

        result
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
    pub async fn scope<'env, F, R>(&'env mut self, f: F) -> Result<R, EspError>
    where
        for<'s, 'channel> F: AsyncFnOnce(&'s mut Scope<'channel, 'env>) -> Result<R, EspError>,
    {
        assert_not_in_isr();

        let handle = self.handle();

        // Temporarily take the callback, to prevent multiple mutable references to self:
        #[cfg(feature = "alloc")]
        let mut callback = self.callback.take();

        let result = Self::with_callback(
            handle,
            {
                #[cfg(feature = "alloc")]
                {
                    callback.as_deref_mut()
                }
                #[cfg(not(feature = "alloc"))]
                {
                    None
                }
            },
            async |progress| {
                // If the channel is not enabled yet, enable it now:
                if !self.is_enabled() {
                    self.enable()?;
                }

                let (is_canceled, result) = {
                    let mut scope = Scope {
                        channel: self,
                        progress,
                        is_canceled: false,
                        _p: PhantomData,
                        #[cfg(feature = "alloc")]
                        _data: alloc::vec::Vec::new(),
                    };

                    let result = f(&mut scope).await;

                    (scope.is_canceled, result)
                };

                if is_canceled {
                    // If the scope was canceled, we don't wait for all done, instead disable the channel.
                    self.disable()?;
                } else {
                    self.wait_all_done().await?;
                }

                result
            },
        )
        .await;

        #[cfg(feature = "alloc")]
        {
            self.callback = callback;
        }

        result
    }

    /// Sends the given signal using the specified encoder and config,
    /// then waits for the transmission to finish before returning.
    pub async fn send_and_wait<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        self.scope(async |scope| {
            scope.send_ref(encoder, signal, config)?;
            Ok(())
        })
        .await
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
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't subscribe while the channel is enabled");
        }

        let callback: alloc::boxed::Box<dyn Fn(rmt_tx_done_event_data_t) + Send + 'static> =
            alloc::boxed::Box::new(callback);

        self.callback = Some(callback);
    }

    /// Remove the ISR handler for when a transmission is done.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't unsubscribe while the channel is enabled");
        }

        self.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        event_data: *const rmt_tx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let event_data = event_data.read();
        let user_data = &mut *(user_data as *mut UserData);

        crate::interrupt::with_isr_yield_signal(move || {
            if let Some(handler) = user_data.callback.as_mut() {
                handler(event_data);
            }

            user_data.progress.signal_finished();
        })
    }
}

impl<'d> RmtChannel for AsyncTxChannelDriver<'d> {
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

impl<'d> Drop for AsyncTxChannelDriver<'d> {
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

impl<'d> fmt::Debug for AsyncTxChannelDriver<'d> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AsyncTxChannelDriver")
            .field("is_enabled", &self.is_enabled)
            .field("handle", &self.handle)
            .finish()
    }
}
