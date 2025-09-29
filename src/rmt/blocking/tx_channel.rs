use core::fmt;
use core::marker::PhantomData;
use core::mem;
#[cfg(feature = "alloc")]
use core::pin::{pin, Pin};
use core::ptr;
#[cfg(feature = "alloc")]
use core::sync::atomic::{AtomicUsize, Ordering};
use core::time::Duration;

#[cfg(feature = "alloc")]
use alloc::boxed::Box;
use esp_idf_sys::*;

use crate::gpio::OutputPin;
#[cfg(feature = "alloc")]
use crate::interrupt::asynch::HalIsrNotification;
use crate::rmt::config::{Loop, TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::Encoder;
#[cfg(feature = "alloc")]
use crate::rmt::TxQueue;
use crate::rmt::{assert_not_in_isr, RmtChannel};

#[cfg(feature = "alloc")]
type AnyPinned = core::pin::Pin<Box<dyn core::any::Any>>;

/// A token is given out after starting a transmission.
///
/// It can be used to wait for the transmission to finish,
/// or to check if it is finished.
///
/// A token for a transmission that was started before another is
/// guaranteed to be "less" than the other token.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Token {
    // TODO: wrap on overflow? or make id counter larger?
    pub(crate) id: usize,
    // To prevent tokens from being used in different channels, the channel handle
    // is stored here for comparison. It is stored as an usize to avoid someone accidentally
    // treating it like a valid pointer (which it might not be, because a token can outlive the channel).
    channel: usize,
}

impl Token {
    pub(crate) fn new(id: usize, channel: rmt_channel_handle_t) -> Self {
        Self {
            id,
            channel: channel as usize,
        }
    }

    /// Checks if the token was created by the given channel.
    #[must_use]
    pub fn is_from(&self, channel: &impl RmtChannel) -> bool {
        self.channel == channel.handle() as usize
    }
}

// TODO: reset ids before they overflow
// Currently each transmission increments the counter -> it will eventually overflow

#[cfg(feature = "alloc")]
pub(crate) struct TransmissionProgress {
    finished_count: AtomicUsize,
    notif: HalIsrNotification,
}

#[cfg(feature = "alloc")]
impl TransmissionProgress {
    #[must_use]
    pub(crate) fn is_finished(&self, id: usize) -> bool {
        // Assuming id is 1, we have the transmissions [0, 1].
        // For id 1 to be finished, the count has to be at least 2.
        self.finished_count.load(Ordering::SeqCst) > id
    }

    /// Waits for the transmission with the given id to complete.
    pub(crate) async fn wait_for(&self, id: usize) {
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

    fn notify_all(&self, next_tx_id: usize) {
        // Mark all transmissions as finished:
        self.finished_count.store(next_tx_id, Ordering::SeqCst);

        // TODO: Is this correct?
        // Notify all waiting tasks:
        self.notif.notify_lsb();
    }
}

#[cfg(feature = "alloc")]
struct UserData {
    callback: Option<Box<dyn FnMut(rmt_tx_done_event_data_t) + Send + 'static>>,
    progress: TransmissionProgress,
}

pub struct TxChannelDriver<'d> {
    // SAFETY: The unsafe code relies on this field to accurately reflect the channel state.
    //         It relies on the fact that in disabled state, the ISR handler will not be called.
    is_enabled: bool,
    handle: rmt_channel_handle_t,
    #[cfg(feature = "alloc")]
    next_tx_id: usize,
    #[cfg(feature = "alloc")]
    on_transmit_data: Box<UserData>,
    #[cfg(feature = "alloc")]
    transmission_data: alloc::vec::Vec<(Token, AnyPinned, Option<AnyPinned>)>,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TxChannelDriver<'d> {
    #[cfg(feature = "alloc")]
    const TX_EVENT_CALLBACKS: rmt_tx_event_callbacks_t = rmt_tx_event_callbacks_t {
        on_trans_done: Some(Self::handle_isr),
    };
    #[cfg(feature = "alloc")]
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

        #[cfg_attr(not(feature = "alloc"), allow(unused_mut))]
        let mut this = Self {
            is_enabled: false,
            handle,
            #[cfg(feature = "alloc")]
            next_tx_id: 0,
            #[cfg(feature = "alloc")]
            on_transmit_data: Box::new(UserData {
                callback: None,
                progress: TransmissionProgress {
                    finished_count: AtomicUsize::new(0),
                    notif: HalIsrNotification::new(),
                },
            }),
            #[cfg(feature = "alloc")]
            transmission_data: alloc::vec::Vec::new(),
            _p: PhantomData,
        };

        // The callback is used to detect when a transmission is finished.
        #[cfg(feature = "alloc")]
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
        })?;

        // All pending transmissions are done -> all stored data can be removed:
        #[cfg(feature = "alloc")]
        self.transmission_data.clear();
        // The notification of waiting tasks happens through the regular isr handler.

        Ok(())
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
    /// # Safety
    ///
    /// Care should be taken not to call std, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// You are not allowed to block, but you are allowed to call FreeRTOS APIs with the FromISR suffix.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(
        &mut self,
        callback: impl FnMut(rmt_tx_done_event_data_t) + Send + 'static,
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
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&mut self) {
        assert_not_in_isr();
        if self.is_enabled() {
            panic!("Can't unsubscribe while the channel is enabled");
        }

        self.on_transmit_data.callback = None;
    }

    /// Handles the ISR event for when a transmission is done.
    #[cfg(feature = "alloc")]
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

    #[cfg(feature = "alloc")]
    #[must_use]
    pub(crate) fn progress(&self) -> &TransmissionProgress {
        &self.on_transmit_data.progress
    }

    /// Starts transmitting the signal using the specified encoder and config.
    ///
    /// # Safety
    ///
    /// This function is a thin wrapper around the `rmt_transmit` function, it assumes that
    /// - the encoder is valid until the transmission is done (if not it is guaranteed to crash)
    /// - the signal is valid until the transmission is done
    /// - the encoder and signal are not modified during the transmission
    ///
    /// The caller must ensure that the encoder and signal live long enough and are not moved.
    pub unsafe fn start_send<E: Encoder>(
        &mut self,
        encoder: &mut E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
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

        #[cfg(feature = "alloc")]
        {
            Ok(Token::new(
                {
                    let id = self.next_tx_id;
                    self.next_tx_id += 1;
                    id
                },
                self.handle(),
            ))
        }
        #[cfg(not(feature = "alloc"))]
        {
            Ok(Token::new(0, self.handle()))
        }
    }

    #[cfg(feature = "alloc")]
    pub(crate) fn store_data(&mut self, id: Token, encoder: AnyPinned, signal: AnyPinned) {
        self.cleanup_data();

        self.transmission_data.push((id, encoder, Some(signal)));
    }

    #[cfg(feature = "alloc")]
    pub(crate) fn cleanup_data(&mut self) {
        // If the channel is disabled, all transmissions are cancelled -> they should be done
        if !self.is_enabled() && !self.transmission_data.is_empty() {
            self.transmission_data.clear();
            self.progress().notify_all(self.next_tx_id);
            return;
        }

        let progress = &self.on_transmit_data.progress;
        self.transmission_data
            .retain(|(stored_id, _, _)| !progress.is_finished(stored_id.id));
    }

    /// Transmits the signals provided by the iterator using the specified encoder and config.
    #[cfg(feature = "alloc")]
    pub fn send_iter<'s, E: Encoder, S: AsRef<[E::Item]>, const N: usize>(
        &mut self,
        encoders: [E; N],
        iter: impl Iterator<Item = S> + 's,
        config: &TransmitConfig,
    ) -> Result<(), EspError>
    where
        E::Item: 's,
    {
        if N == 0 {
            // Impossible to encode anything, just return the encoder as is:
            return Ok(());
        }

        let mut pending: Pin<&mut TxQueue<'_, '_, E, S, N>> = pin!(TxQueue::new(encoders, self));

        for signal in iter {
            pending.as_mut().push(signal, config)?;
        }

        // The remaining pending transmissions will be awaited in the drop of the queue.

        Ok(())
    }

    /// Creates a transmission queue that manages the given encoders, and reuses them for new transmissions when possible.
    ///
    /// # Safety
    ///
    /// The pending transmissions are stored in the queue, which is why it must be pinned.
    ///
    /// The pinning ensures that the memory address of the signals and encoders does not change,
    /// before the transmission is done.
    ///
    /// To prevent invalid memory accesses, the queue will wait for all pending transmissions to finish
    /// in its drop implementation. If one were to [`mem::forget`] the queue with it on the stack,
    /// it could result in invalid memory accesses (undefined behaviour).
    ///
    /// Pinning the queue on the heap is one way to ensure that [`mem::forget`] does not result in
    /// undefined behaviour, for this reason, the safe wrapper [`Self::queue`] is provided.
    ///
    /// This function is provided to avoid the allocation. The caller has to pin it on the stack
    /// with [`pin!`].
    #[cfg(feature = "alloc")]
    pub unsafe fn raw_queue<E: Encoder, S, const N: usize>(
        &mut self,
        encoders: [E; N],
    ) -> TxQueue<'_, 'd, E, S, N> {
        TxQueue::new(encoders, self)
    }

    /// Creates a transmission queue that manages the given encoders, and reuses them for new transmissions when possible.
    #[cfg(feature = "alloc")]
    pub fn queue<E: Encoder, S, const N: usize>(
        &mut self,
        encoders: [E; N],
    ) -> Pin<Box<TxQueue<'_, 'd, E, S, N>>> {
        // SAFETY: The queue is pinned on the heap, if it is forgotten,
        //         the memory the pending transmissions reference won't be freed.
        unsafe { Box::pin(self.raw_queue(encoders)) }
    }

    /// Transmits the signal using the specified encoder and config.
    ///
    /// It will **not** wait for the transmission to finish, but a [`Token`] is returned
    /// which can be used for this.
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
    #[cfg(feature = "alloc")]
    pub fn send<E: Encoder + 'static>(
        &mut self,
        encoder: E,
        // TODO: Is this safe?
        //
        // This assumes that when the signal is stored,
        // a reference to it through AsRef will stay valid
        // until the signal itself is dropped
        signal: impl AsRef<[E::Item]> + 'static,
        config: &TransmitConfig,
    ) -> Result<Token, EspError> {
        let mut encoder = Box::pin(encoder);
        let signal = Box::pin(signal);

        let token = unsafe {
            self.start_send(
                encoder.as_mut().get_unchecked_mut(),
                signal.as_ref().get_ref().as_ref(),
                config,
            )
        }?;

        self.store_data(token, encoder, signal);

        Ok(token)
    }

    /// Transmits the signal and waits for the transmission to finish.
    #[cfg(feature = "alloc")]
    pub fn send_and_wait<E: Encoder>(
        &mut self,
        encoder: E,
        signal: &[E::Item],
        config: &TransmitConfig,
    ) -> Result<(), EspError> {
        self.send_iter([encoder], core::iter::once(signal), config)
    }

    /// Checks if the transmission associated with the given token is finished.
    #[cfg(feature = "alloc")]
    pub fn is_finished(&self, token: Token) -> bool {
        !self.is_enabled || self.progress().is_finished(token.id)
    }

    /// Waits for the transmission associated with the given token to finish.
    #[cfg(feature = "alloc")]
    pub fn wait_for(&mut self, token: Token) {
        if !token.is_from(self) {
            panic!("The given token {token:?} is not from this channel.");
        }

        // Only wait if the transmission is not already finished: (ensures that it does not wait when the channel is disabled)
        if !self.is_finished(token) {
            crate::task::block_on(self.progress().wait_for(token.id))
        }

        self.cleanup_data();
    }
}

// SAFETY: The C code doesn't seem to use any thread locals -> it should be safe to send the channel to another thread.
unsafe impl<'d> Send for TxChannelDriver<'d> {}

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
        #[cfg(feature = "alloc")]
        if !self.is_enabled {
            self.cleanup_data();
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
        #[cfg(feature = "alloc")]
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
