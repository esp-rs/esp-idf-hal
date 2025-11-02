use core::fmt;
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::time::Duration;

use alloc::boxed::Box;
use esp_idf_sys::*;

use crate::gpio::OutputPin;
use crate::interrupt::asynch::HalIsrNotification;
use crate::rmt::config::{Loop, TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::{into_raw, Encoder, RawEncoder};
use crate::rmt::tx_queue::TxQueue;
use crate::rmt::TxDoneEventData;
use crate::rmt::{assert_not_in_isr, EncoderBuffer, RmtChannel};

struct UserData<'d> {
    callback: Option<Box<dyn FnMut(TxDoneEventData) + Send + 'd>>,
    queue_size: AtomicUsize,
    queue_has_progressed: HalIsrNotification,
}

pub struct TxChannelDriver<'d> {
    // SAFETY: The unsafe code relies on this field to accurately reflect the channel state.
    //         It relies on the fact that in disabled state, the ISR handler will not be called.
    is_enabled: bool,
    handle: rmt_channel_handle_t,
    on_transmit_data: Box<UserData<'d>>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TxChannelDriver<'d> {
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
                    #[cfg(any(
                        esp_idf_version_patch_at_least_5_4_3,
                        esp_idf_version_at_least_5_4_0
                    ))]
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

        #[cfg_attr(not(feature = "alloc"), allow(unused_mut))]
        let mut this = Self {
            is_enabled: false,
            handle,
            on_transmit_data: Box::new(UserData {
                callback: None,
                queue_size: AtomicUsize::new(0),
                queue_has_progressed: HalIsrNotification::new(),
            }),
            _p: PhantomData,
        };

        // The callback is used to detect when a transmission is finished.
        esp!(unsafe {
            rmt_tx_register_event_callbacks(
                handle,
                &Self::TX_EVENT_CALLBACKS,
                (&raw mut *this.on_transmit_data) as *mut core::ffi::c_void,
            )
        })?;

        Ok(this)
    }

    /// Wait for all pending TX transactions to finish.
    ///
    /// If `timeout` is `None`, it will wait indefinitely. If `timeout` is `Some(duration)`,
    /// it will wait for at most `duration`.
    ///
    /// # Note
    ///
    /// This function will block forever if the pending transaction can't
    /// be finished within a limited time (e.g. an infinite loop transaction).
    /// See also [`Self::disable`] for how to terminate a working channel.
    ///
    /// If the given `timeout` converted to milliseconds is larger than `i32::MAX`,
    /// it will be treated as `None` (wait indefinitely).
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Flush transactions failed because of invalid argument
    /// - `ESP_FAIL`: Flush transactions failed because of other error
    ///
    /// # Polling
    ///
    /// When polling this function (calling with a timeout duration of 0ms),
    /// esp-idf will log flush timeout errors to the console.
    /// This issue is tracked in <https://github.com/espressif/esp-idf/issues/17527>
    /// and should be fixed in future esp-idf versions.
    pub fn wait_all_done(&mut self, timeout: Option<Duration>) -> Result<(), EspError> {
        esp!(unsafe {
            rmt_tx_wait_all_done(
                self.handle,
                timeout.map_or(-1, |duration| duration.as_millis().try_into().unwrap_or(-1)),
            )
        })
    }

    /// Define the ISR handler for when a transmission is done.
    ///
    /// The callback will be called with the number of transmitted symbols, including one EOF symbol,
    /// which is appended by the driver to mark the end of the transmission. For a loop transmission,
    /// this value only counts for one round.
    ///
    /// There is only one callback possible, you can not subscribe multiple callbacks.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    ///
    /// # ISR Safety
    ///
    /// Care should be taken not to call std, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// You are not allowed to block, but you are allowed to call FreeRTOS APIs with the FromISR suffix.
    pub fn subscribe(&mut self, callback: impl FnMut(TxDoneEventData) + Send + 'static) {
        // SAFETY: because of 'static lifetime, it doesn't matter if mem::forget is called on the driver
        unsafe {
            self.subscribe_nonstatic(callback);
        }
    }

    /// Subscribe a non-'static callback for when a transmission is done.
    ///
    /// # Safety
    ///
    /// You must not forget the channel driver (for example through [`mem::forget`]),
    /// while the callback is still subscribed, otherwise this would lead to undefined behavior.
    pub unsafe fn subscribe_nonstatic(
        &mut self,
        callback: impl FnMut(TxDoneEventData) + Send + 'd,
    ) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't subscribe while the channel is enabled");
        }

        self.on_transmit_data.callback = Some(Box::new(callback));
    }

    /// Remove the ISR handler for when a transmission is done.
    ///
    /// # Panics
    ///
    /// This function will panic if called from an ISR context or while the channel is enabled.
    pub fn unsubscribe(&mut self) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't unsubscribe while the channel is enabled");
        }

        self.on_transmit_data.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        _channel: rmt_channel_handle_t,
        event_data: *const rmt_tx_done_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let event_data = TxDoneEventData::from(event_data.read());
        let user_data = &mut *(user_data as *mut UserData<'d>);

        user_data.queue_size.fetch_sub(1, Ordering::SeqCst);
        if let Some(handler) = user_data.callback.as_mut() {
            handler(event_data);
        }

        user_data.queue_has_progressed.notify_lsb()
    }

    /// Starts transmitting the signal using the specified encoder and config.
    ///
    /// # Safety
    ///
    /// This function is a thin wrapper around the `rmt_transmit` function, it assumes that
    /// - the encoder (the returned pointer of [`RawEncoder::handle`]) is valid until the transmission
    ///   is done, if not, it is guaranteed to crash
    /// - the signal is valid until the transmission is done
    /// - the encoder and signal are not modified during the transmission
    ///
    /// The caller must ensure that the encoder and signal **live long enough** and are **not moved**.
    pub unsafe fn start_send<E: RawEncoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        if !self.is_enabled() {
            self.enable()?;
        }

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
                self.handle(),
                encoder.handle(),
                signal.as_ptr() as *const core::ffi::c_void,
                // size should be given in bytes:
                mem::size_of_val::<[E::Item]>(signal),
                &sys_config,
            )
        })?;

        self.on_transmit_data
            .queue_size
            .fetch_add(1, Ordering::SeqCst);

        Ok(())
    }

    /// Transmits the signals provided by the iterator using the specified encoder and config.
    ///
    /// This is a convenience function that will create a [`TxQueue`], push all signals from the iterator
    /// to the queue, and then drop the queue, waiting for all transmissions to finish.
    ///
    /// # Non blocking behavior
    ///
    /// It is not recommended to use this function with [`TransmitConfig::queue_non_blocking`] set to true,
    /// because it will drop the queue if it would block, resulting in it blocking until all pending transmissions
    /// are done.
    ///
    /// Therefore, one should use [`TxChannelDriver::queue`] for a non-blocking use case.
    pub fn send_iter<E: Encoder, S: AsRef<[E::Item]>>(
        &mut self,
        encoders: impl IntoIterator<Item = E>,
        iter: impl Iterator<Item = S>,
        config: &TransmitConfig,
    ) -> Result<(), EspError>
    where
        E::Item: Clone,
    {
        let mut pending = TxQueue::new(
            encoders
                .into_iter()
                .map(|encoder| EncoderBuffer::new(into_raw(encoder)))
                .collect(),
            self,
        );

        for signal in iter {
            pending.push(signal.as_ref(), config)?;
        }

        // The remaining pending transmissions will be awaited in the drop of the queue.

        Ok(())
    }

    /// Creates a new queue for transmitting multiple signals with the given encoders.
    ///
    /// For more information, see [`TxQueue`].
    ///
    /// # Panics
    ///
    /// If no encoders are provided.
    #[must_use]
    pub fn queue<E: Encoder>(
        &mut self,
        encoders: impl IntoIterator<Item = E>,
    ) -> TxQueue<'_, 'd, E> {
        TxQueue::new(
            encoders
                .into_iter()
                .map(|encoder| EncoderBuffer::new(into_raw(encoder)))
                .collect(),
            self,
        )
    }

    /// Asynchronously waits until the next pending transmission has finished.
    ///
    /// If there are no pending transmissions, this function will wait indefinitely.
    pub async fn wait_for_progress(&self) {
        self.on_transmit_data.queue_has_progressed.wait().await;
    }

    /// Returns the number of currently pending transmissions.
    ///
    /// This will be updated when a transmission is started or finished.
    pub fn queue_size(&self) -> usize {
        self.on_transmit_data.queue_size.load(Ordering::SeqCst)
    }

    /// Transmits the signal and waits for the transmission to finish.
    ///
    /// If the channel is not enabled yet, it will be enabled automatically.
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
    /// - `ESP_ERR_NOT_SUPPORTED`: Some feature is not supported by hardware e.g. unsupported loop count
    /// - `ESP_FAIL`: Because of other errors
    pub fn send_and_wait<E: Encoder>(
        &mut self,
        encoder: E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError>
    where
        E::Item: Clone,
    {
        self.send_iter([encoder], core::iter::once(signal), config)
    }
}

// SAFETY: The C code doesn't seem to use any thread locals -> it should be safe to send the channel to another thread.
unsafe impl<'d> Send for TxChannelDriver<'d> {}

// SAFETY: All non-thread-safe methods require exclusive access to self
// -> it is safe to send &TxChannelDriver to another thread.
// -> it is safe to implement Sync
unsafe impl<'d> Sync for TxChannelDriver<'d> {}

impl<'d> RmtChannel for TxChannelDriver<'d> {
    fn handle(&self) -> rmt_channel_handle_t {
        self.handle
    }

    fn is_enabled(&self) -> bool {
        self.is_enabled
    }

    unsafe fn set_internal_enabled(&mut self, is_enabled: bool) {
        self.is_enabled = is_enabled;

        // If the channel was disabled, all pending transmissions are cancelled
        if !self.is_enabled {
            self.on_transmit_data.queue_size.store(0, Ordering::SeqCst);
            self.on_transmit_data.queue_has_progressed.reset();
        }
    }
}

impl<'d> Drop for TxChannelDriver<'d> {
    fn drop(&mut self) {
        // Deleting the channel might fail if it is not disabled first.
        //
        // The result is ignored here, because there is nothing we can do about it.
        if self.is_enabled() {
            let _res = self.disable();
        }

        // SAFETY: The disable will cancel all pending transmission -> the stored data can be freed

        // Remove the isr handler:
        let _res = unsafe {
            rmt_tx_register_event_callbacks(
                self.handle,
                &Self::TX_EVENT_CALLBACKS_DISABLE,
                ptr::null_mut(),
            )
        };

        unsafe { rmt_del_channel(self.handle) };
    }
}

impl<'d> fmt::Debug for TxChannelDriver<'d> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxChannelDriver")
            .field("is_enabled", &self.is_enabled)
            .field("handle", &self.handle)
            .finish()
    }
}
