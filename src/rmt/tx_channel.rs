use core::marker::PhantomData;
use core::ptr;
use core::time::Duration;
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::gpio::OutputPin;
use crate::interrupt::asynch::HalIsrNotification;

use esp_idf_sys::*;

use crate::interrupt;
use crate::rmt::config::{CarrierConfig, Loop, TransmitConfig, TxChannelConfig};
use crate::rmt::encoder::{CopyEncoder, Encoder};
use crate::rmt::Signal;

#[must_use]
pub struct TxHandle<'t, E, S> {
    channel: &'t TxChannel<'t>,
    notif: &'t HalIsrNotification,
    signal: S,
    id: usize,
    encoder: E, // Ensures that the encoder lives as long as the TxHandle
    enabled: bool,
}

impl<'t, E, S> TxHandle<'t, E, S> {
    pub async fn wait(&mut self) {
        self.channel.check();

        // Wait until the transmission with this id is done
        while TRANSMISSION_COUNTER[self.channel.channel_id()]
            .sent
            .load(Ordering::SeqCst)
            <= self.id
        {
            self.notif.wait().await; // Will be notified from the ISR every time a transmission is done
        }
    }

    pub fn wait_blocking(&mut self) {
        // TODO: This might block forever if the transmission never ends; Loop::Endless
        crate::task::block_on(self.wait())
    }
}

// TODO: impl IntoFuture for TxHandle?

impl<'t, E, S> Drop for TxHandle<'t, E, S> {
    fn drop(&mut self) {
        if self.enabled {
            self.wait_blocking()
        }
    }
}

pub struct TxChannel<'d> {
    handle: rmt_channel_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> TxChannel<'d> {
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
    pub fn new(pin: impl OutputPin + 'd, config: &TxChannelConfig) -> Result<Self, EspError> {
        let sys_config: rmt_tx_channel_config_t = rmt_tx_channel_config_t {
            clk_src: config.source_clock.into(),
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
        let mut handle: rmt_channel_handle_t = core::ptr::null_mut();
        esp!(unsafe { rmt_new_tx_channel(&sys_config, &mut handle) })?;

        // The callback is used to detect when a transmission is finished.
        // Therefore it is always registered:
        esp!(unsafe {
            rmt_tx_register_event_callbacks(
                handle,
                &Self::TX_EVENT_CALLBACKS,
                core::ptr::null_mut(),
            )
        })?;

        Ok(Self {
            handle,
            _p: PhantomData,
        })
    }

    /// Returns the underlying `rmt_channel_handle_t`.
    pub fn handle(&self) -> rmt_channel_handle_t {
        self.handle
    }

    /// Must be called in advance before transmitting or receiving RMT symbols.
    /// For TX channels, enabling a channel enables a specific interrupt and
    /// prepares the hardware to dispatch transactions. For RX channels, enabling
    /// a channel enables an interrupt, but the receiver is not started during
    /// this time, as the characteristics of the incoming signal have yet to be
    /// specified.
    ///
    /// The receiver is started in rmt_receive().
    pub fn enable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_enable(self.handle) })?;
        Ok(())
    }

    /// Disables the interrupt and clearing any pending interrupts. The transmitter
    /// and receiver are disabled as well.
    pub fn disable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { rmt_disable(self.handle) })
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

    /// Apply modulation feature for the channel.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Apply carrier failed because of invalid argument
    /// - `ESP_FAIL`: Apply carrier failed because of other error
    pub fn apply_carrier(
        &mut self,
        carrier_config: Option<&CarrierConfig>,
    ) -> Result<(), EspError> {
        let mut sys_carrier = None;
        if let Some(CarrierConfig {
            frequency,
            duty_cycle,
            polarity_active_low,
            always_on,
        }) = carrier_config
        {
            sys_carrier = Some(rmt_carrier_config_t {
                frequency_hz: (*frequency).into(),
                duty_cycle: duty_cycle.0 as f32 / 100.0,
                flags: rmt_carrier_config_t__bindgen_ty_1 {
                    _bitfield_1: rmt_carrier_config_t__bindgen_ty_1::new_bitfield_1(
                        *polarity_active_low as u32,
                        *always_on as u32,
                    ),
                    ..Default::default()
                },
            })
        }

        esp!(unsafe {
            rmt_apply_carrier(
                self.handle,
                sys_carrier.as_ref().map_or(ptr::null(), |c| c as *const _),
            )
        })
    }

    /// Starts transmitting the given payloadk, it will not wait for the transmission to be done.
    ///
    /// # Safety
    ///
    /// You are not allowed to `mem::forget` the returned `TxHandle`, this might lead to undefined behavior.
    pub unsafe fn start_transmit<'t, S>(
        &'t self,
        payload: S,
        config: &TransmitConfig,
    ) -> Result<TxHandle<'t, CopyEncoder, S>, EspError>
    where
        S: Signal<rmt_symbol_word_t>,
    {
        let copy_encoder = CopyEncoder::new(&rmt_copy_encoder_config_t {})?;
        let handle = self.transmit_raw(copy_encoder, payload, config)?;

        Ok(handle)
    }

    /// Transmits the raw data using the specified encoder and config.
    ///
    /// # Safety
    ///
    /// The encoder is passed to the [`rmt_transmit`] function which then calls the
    /// encoders functions to transform the payload into RMT symbols. You must ensure
    /// that the encoder is implemented correctly, for more details, see the [RMT Encoder]
    /// documentation.
    ///
    /// The passed payload must be valid and can not be modified, until the transmission is done.
    /// One can await the transmission end by calling [`Self::wait_all_done`] or registering a
    /// callback to be notified when the transmission is done.
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
    pub unsafe fn transmit_raw<'t, E: Encoder, S>(
        &'t self,
        encoder: E,
        payload: S,
        config: &TransmitConfig,
    ) -> Result<TxHandle<'t, E, S>, EspError>
    where
        S: Signal<E::Item>,
    {
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

        // TODO: The rmt_transmit might block if the queue is full and queue_non_blocking is false.

        // SAFETY: The payload buffer must not be modified or dropped until the transmission is completed.
        // SAFETY: The encoder must live until the transmission is completed.
        //
        // Both of these conditions are guaranteed by the TxHandle that is returned. This functions signature
        // ensures that the channel, payload and encoder live at least as long as the TxHandle. When a TxHandle
        // is dropped, it will wait for the transmission to be done, ensuring that the payload and encoder will
        // not be dropped before the transmission is done.
        // TODO: couldn't one forget the TxHandle? This would prevent it from calling the drop function -> not waiting for the transmission to be done?
        // TODO: maybe define TxHandle such that it owns the data -> on forget the data will still be there?
        let id = TRANSMISSION_COUNTER[self.channel_id()]
            .queued
            .fetch_add(1, Ordering::SeqCst);

        let mut handle = TxHandle {
            channel: self,
            notif: &RMT_NOTIF[self.channel_id()],
            id,
            encoder,
            signal: payload,
            enabled: false,
        };
        // TODO: is it enough to keep ownership or do we have to keep the reference in TxHandle as well?
        let data = handle.signal.as_slice();
        esp!(unsafe {
            rmt_transmit(
                self.handle,
                handle.encoder.handle(),
                data.as_ptr() as *const _,
                // size should be given in bytes:
                data.len() * core::mem::size_of::<E::Item>(),
                &sys_config,
            )
        })?;
        // The above might error, in which case the transmission was not started/it should not be waited for.
        // This field solves that issue.
        handle.enabled = true;

        Ok(handle)
    }

    const TX_EVENT_CALLBACKS: rmt_tx_event_callbacks_t = rmt_tx_event_callbacks_t {
        on_trans_done: Some(Self::handle_isr),
    };
    const TX_EVENT_CALLBACKS_DISABLE: rmt_tx_event_callbacks_t = rmt_tx_event_callbacks_t {
        on_trans_done: None,
    };

    /// Add ISR handler for when a transmission is done.
    ///
    /// The callback will be called with the number of transmitted symbols, including one EOF symbol,
    /// which is appended by the driver to mark the end of the transmission. For a loop transmission,
    /// this value only counts for one round.
    ///
    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe(&mut self, callback: impl FnMut(usize) + Send + 'static) {
        self.check();

        // TODO: allocate callback in IRAM?
        // See https://github.com/esp-rs/esp-idf-hal/issues/486
        let callback: alloc::boxed::Box<dyn FnMut(usize) + Send + 'static> =
            alloc::boxed::Box::new(callback);

        unsafe {
            ISR_HANDLERS[self.channel_id()] = Some(callback);
        }
    }

    /// Remove the ISR handler for when a transmission is done.
    pub fn unsubscribe(&mut self) {
        self.check();

        unsafe {
            ISR_HANDLERS[self.channel_id()] = None;
        }
    }

    /// Handles the ISR event for when a transmission is done.
    unsafe extern "C" fn handle_isr(
        channel: rmt_channel_handle_t,
        event_data: *const rmt_tx_done_event_data_t,
        _user_data: *mut core::ffi::c_void,
    ) -> bool {
        // rmt_channel_handle_t is a `typedef struct rmt_channel_t *rmt_channel_handle_t;`
        // which should correspond to a *mut rmt_channel_t
        let id = (channel as *mut rmt_channel_t).read() as usize;
        let num_symbols = event_data.read().num_symbols;

        // TODO: This might not be safe in ISR?
        TRANSMISSION_COUNTER[id].sent.fetch_add(1, Ordering::SeqCst);

        interrupt::with_isr_yield_signal(move || {
            if let Some(handler) = ISR_HANDLERS[id].as_mut() {
                handler(num_symbols);
            }

            RMT_NOTIF[id].notify_lsb();
        })
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
}

impl<'d> Drop for TxChannel<'d> {
    fn drop(&mut self) {
        // Deleting the channel might fail if it is not disabled first.
        //
        // The result is ignored here, because there is nothing we can do about it.
        let _res = self.disable();

        unsafe {
            rmt_tx_register_event_callbacks(
                self.handle,
                &Self::TX_EVENT_CALLBACKS_DISABLE,
                core::ptr::null_mut(),
            )
        };

        unsafe {
            ISR_HANDLERS[self.channel_id()] = None;
        }
        RMT_NOTIF[self.channel_id()].reset();
        TRANSMISSION_COUNTER[self.channel_id()].reset();

        unsafe { rmt_del_channel(self.handle) };
    }
}

type IsrHandler = Option<alloc::boxed::Box<dyn FnMut(usize)>>;
static mut ISR_HANDLERS: [IsrHandler; rmt_channel_t_RMT_CHANNEL_MAX as usize] =
    [const { None }; rmt_channel_t_RMT_CHANNEL_MAX as usize];

static RMT_NOTIF: [interrupt::asynch::HalIsrNotification; rmt_channel_t_RMT_CHANNEL_MAX as usize] =
    [const { interrupt::asynch::HalIsrNotification::new() }; rmt_channel_t_RMT_CHANNEL_MAX as usize];

struct TransmissionCounter {
    // TODO: move this into TxChannel?
    queued: AtomicUsize,
    sent: AtomicUsize,
}

impl TransmissionCounter {
    const fn new() -> Self {
        Self {
            queued: AtomicUsize::new(0),
            sent: AtomicUsize::new(0),
        }
    }

    #[inline(always)]
    fn reset(&self) {
        self.queued.store(0, Ordering::SeqCst);
        self.sent.store(0, Ordering::SeqCst);
    }
}
static TRANSMISSION_COUNTER: [TransmissionCounter; rmt_channel_t_RMT_CHANNEL_MAX as usize] =
    [const { TransmissionCounter::new() }; rmt_channel_t_RMT_CHANNEL_MAX as usize];
