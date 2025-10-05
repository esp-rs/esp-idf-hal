use core::marker::PhantomData;
use core::pin::Pin;
use core::{mem, ptr};

#[cfg(feature = "alloc")]
use alloc::boxed::Box;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use esp_idf_sys::*;

use crate::gpio::InputPin;
use crate::interrupt;
use crate::new_pcnt::private::Sealed;

use config::*;

#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct WatchEventData {
    pub watch_point_value: i32,
    pub zero_cross_mode: ZeroCrossMode,
}

impl From<pcnt_watch_event_data_t> for WatchEventData {
    fn from(value: pcnt_watch_event_data_t) -> Self {
        Self {
            watch_point_value: value.watch_point_value,
            zero_cross_mode: value.zero_cross_mode.into(),
        }
    }
}

pub mod config {
    use core::time::Duration;

    use esp_idf_sys::*;

    #[derive(Debug, Clone, Copy)]
    pub struct UnitConfig {
        /// Specifies the lower limit for the internal hardware counter.
        ///
        /// The counter will reset to zero automatically when it crosses the low limit.
        pub low_limit: i32,
        /// Specifies the higher limit for the internal hardware counter.
        ///
        /// The counter will reset to zero automatically when it crosses the high limit.
        pub high_limit: i32,
        /// Sets the priority of the interrupt. If it is set to 0, the driver will
        /// allocate an interrupt with a default priority. Otherwise, the driver
        /// will use the given priority.
        ///
        /// Since all PCNT units share the same interrupt source, when installing
        /// multiple PCNT units make sure that the interrupt priority is the same for each unit.
        pub intr_priority: i32,
        /// If set to `true`, it will create an internal accumulator for the counter.
        /// This is helpful when you want to extend the counter's width, which by default
        /// is 16 bit at most, defined in the hardware.
        ///
        /// If enabled, the [`PcntUnitDriver::get_count`] will not only return the hardware's count
        /// value, but also the accumulated underflow/overflow.
        ///
        /// # Note
        ///
        /// You should add the high/low limits as the watch points.
        ///
        /// When enabling the count overflow compensation, it is recommended to use
        /// as large a high/low count limit as possible, as it can avoid frequent
        /// interrupt triggering, improve system performance, and avoid compensation
        /// failure due to multiple overflows.
        pub accum_count: bool,
    }

    impl Default for UnitConfig {
        fn default() -> Self {
            Self {
                low_limit: -1024,
                high_limit: 1024,
                intr_priority: 0,
                accum_count: false,
            }
        }
    }

    impl From<UnitConfig> for pcnt_unit_config_t {
        fn from(value: UnitConfig) -> Self {
            pcnt_unit_config_t {
                low_limit: value.low_limit,
                high_limit: value.high_limit,
                intr_priority: value.intr_priority,
                flags: pcnt_unit_config_t__bindgen_ty_1 {
                    _bitfield_1: pcnt_unit_config_t__bindgen_ty_1::new_bitfield_1(
                        value.accum_count as u32,
                    ),
                    ..Default::default()
                },
            }
        }
    }

    /// Configuration for the PCNT glitch filter.
    #[derive(Debug, Clone)]
    pub struct GlitchFilterConfig {
        /// The maximum glitch width (will be converted to nanoseconds).
        ///
        /// If a signal pulse's width is smaller than this value, then it
        /// will be treated as noise and will not increase/decrease the
        /// internal counter.
        pub max_glitch: Duration,
    }

    impl From<&GlitchFilterConfig> for pcnt_glitch_filter_config_t {
        fn from(value: &GlitchFilterConfig) -> Self {
            pcnt_glitch_filter_config_t {
                max_glitch_ns: value.max_glitch.as_nanos().try_into().unwrap_or(u32::MAX),
            }
        }
    }

    #[non_exhaustive]
    pub struct ChannelConfig {
        pub invert_edge_input: bool,
        pub invert_level_input: bool,
        pub virt_edge_io_level: bool,
        pub virt_level_io_level: bool,
    }

    /// PCNT channel action on control level.
    #[non_exhaustive]
    pub enum ChannelLevelAction {
        /// Keep current count mode
        Keep,
        /// Invert current count mode (increase -> decrease, decrease -> increase).
        Inverse,
        /// Hold current count value.
        Hold,
    }

    impl From<ChannelLevelAction> for pcnt_channel_level_action_t {
        fn from(value: ChannelLevelAction) -> Self {
            match value {
                ChannelLevelAction::Keep => {
                    pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_KEEP
                }
                ChannelLevelAction::Inverse => {
                    pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_INVERSE
                }
                ChannelLevelAction::Hold => {
                    pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_HOLD
                }
            }
        }
    }

    /// PCNT channel action on signal edge.
    #[non_exhaustive]
    pub enum ChannelEdgeAction {
        /// Hold current count value
        Hold,
        /// Increase count value
        Increase,
        /// Decrease count value
        Decrease,
    }

    impl From<ChannelEdgeAction> for pcnt_channel_edge_action_t {
        fn from(value: ChannelEdgeAction) -> Self {
            match value {
                ChannelEdgeAction::Hold => pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD,
                ChannelEdgeAction::Increase => {
                    pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE
                }
                ChannelEdgeAction::Decrease => {
                    pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE
                }
            }
        }
    }

    /// PCNT unit zero cross mode.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[non_exhaustive]
    pub enum ZeroCrossMode {
        /// start from positive value, end to zero, i.e. +N->0
        PositionZero,
        /// start from negative value, end to zero, i.e. -N->0
        NegativeZero,
        /// start from negative value, end to positive value, i.e. -N->+M
        NegativePosition,
        /// start from positive value, end to negative value, i.e. +N->-M
        PositiveNegative,
        /// invalid zero cross mode
        Invalid,
    }

    impl From<pcnt_unit_zero_cross_mode_t> for ZeroCrossMode {
        fn from(value: pcnt_unit_zero_cross_mode_t) -> Self {
            #[allow(non_upper_case_globals)]
            match value {
                pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_POS_ZERO => Self::PositionZero,
                pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_NEG_ZERO => Self::NegativeZero,
                pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_NEG_POS => Self::NegativePosition,
                pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_POS_NEG => Self::PositiveNegative,
                pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_INVALID => Self::Invalid,
                _ => unreachable!("unknown zero cross mode: {value}"),
            }
        }
    }
}

#[derive(Debug)]
pub struct PcntChannel<'d> {
    handle: pcnt_channel_handle_t,
    _p: PhantomData<&'d mut ()>,
}

impl<'d> PcntChannel<'d> {
    unsafe fn new(
        pcnt_unit: pcnt_unit_handle_t,
        edge_pin: Option<impl InputPin + 'd>,
        level_pin: Option<impl InputPin + 'd>,
        config: &ChannelConfig,
    ) -> Result<Self, EspError> {
        let sys_config = pcnt_chan_config_t {
            edge_gpio_num: edge_pin.map_or(-1, |p| p.pin().into()),
            level_gpio_num: level_pin.map_or(-1, |p| p.pin().into()),
            flags: pcnt_chan_config_t__bindgen_ty_1 {
                _bitfield_1: pcnt_chan_config_t__bindgen_ty_1::new_bitfield_1(
                    config.invert_edge_input as u32,
                    config.invert_level_input as u32,
                    config.virt_edge_io_level as u32,
                    config.virt_level_io_level as u32,
                    // TODO: this will be removed with 6.0
                    false as u32,
                ),
                ..Default::default()
            },
        };

        let mut handle = ptr::null_mut();
        esp!(unsafe { pcnt_new_channel(pcnt_unit, &sys_config, &mut handle) })?;

        Ok(Self {
            handle,
            _p: PhantomData,
        })
    }

    /// Set channel actions when edge signal changes (e.g. falling or rising edge occurred).
    ///
    /// The edge signal is input from the edge pin this channel was configured with.
    /// We use these actions to control when and how to change the counter value.
    pub fn set_edge_action(
        &mut self,
        pos_act: ChannelEdgeAction,
        neg_act: ChannelEdgeAction,
    ) -> Result<&mut Self, EspError> {
        esp!(unsafe { pcnt_channel_set_edge_action(self.handle, pos_act.into(), neg_act.into()) })?;

        Ok(self)
    }

    /// Set channel actions when level signal changes (e.g. signal level goes from high to low).
    ///
    /// The level signal is input from the `level_pin` configured in [`Self::new`].
    /// We use these actions to control when and how to change the counting mode.
    ///
    /// # Note
    ///
    /// If both `high_act` and `low_act` are set to `Hold`, it will not count anything.
    pub fn set_level_action(
        &mut self,
        high_act: ChannelLevelAction,
        low_act: ChannelLevelAction,
    ) -> Result<&mut Self, EspError> {
        esp!(unsafe {
            pcnt_channel_set_level_action(self.handle, high_act.into(), low_act.into())
        })?;

        Ok(self)
    }
}

impl<'d> Drop for PcntChannel<'d> {
    fn drop(&mut self) {
        unsafe {
            pcnt_del_channel(self.handle);
        }
    }
}

mod private {
    pub trait Sealed {}
}

pub trait State: Sealed {}

pub enum Enabled {}
impl State for Enabled {}
impl private::Sealed for Enabled {}

pub enum Disabled {}
impl State for Disabled {}
impl private::Sealed for Disabled {}

struct DelegateUserData {
    on_reach: Box<dyn FnMut(WatchEventData) + Send + 'static>,
}

pub struct PcntUnitDriver<'d, S: State> {
    handle: pcnt_unit_handle_t,
    channels: Vec<PcntChannel<'d>>,
    user_data: Option<Pin<Box<DelegateUserData>>>,
    _p: PhantomData<(&'d mut (), S)>,
}

impl<'d> PcntUnitDriver<'d, Disabled> {
    /// Create a new PCNT unit driver instance, with the given configuration.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Create PCNT unit failed because of invalid argument (e.g. high/low limit value out of the range)
    /// - `ESP_ERR_NO_MEM`: Create PCNT unit failed because out of memory
    /// - `ESP_ERR_NOT_FOUND`: Create PCNT unit failed because all PCNT units are used up and no more free one
    /// - `ESP_FAIL`: Create PCNT unit failed because of other error
    pub fn new(config: &UnitConfig) -> Result<Self, EspError> {
        let sys_config = (*config).into();
        let mut handle = ptr::null_mut();
        esp!(unsafe { pcnt_new_unit(&sys_config, &mut handle) })?;

        Ok(Self {
            handle,
            channels: Vec::new(),
            user_data: None,
            _p: PhantomData,
        })
    }

    /// The PCNT unit features filters to ignore possible short glitches in the signals.
    ///
    /// You can enable the glitch filter for the pcnt unit, by calling this method with
    /// a filter configuration. If you want to disable the filter, call this method with
    /// `None`.
    ///
    /// # Note
    ///
    /// The glitch filter operates using the APB clock. To ensure the counter does not miss
    /// any pulses, the maximum glitch width should be longer than one APB_CLK cycle
    /// (typically 12.5 ns if APB is 80 MHz). Since the APB frequency can change with Dynamic
    /// Frequency Scaling (DFS), the filter may not function as expected in such cases.
    ///
    /// Therefore, the driver installs a power management lock for each PCNT unit. For more
    /// details on the power management strategy used in the PCNT driver, please refer
    /// to [Power Management].
    ///
    /// [Power Management]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html#pcnt-power-management
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Set glitch filter failed because of invalid argument (e.g. glitch width is too big)
    /// - `ESP_FAIL`: Set glitch filter failed because of other error
    pub fn set_glitch_filter(
        &mut self,
        config: Option<&GlitchFilterConfig>,
    ) -> Result<&mut Self, EspError> {
        let sys_config = config.map(|config| config.into());
        esp!(unsafe {
            pcnt_unit_set_glitch_filter(self.handle, sys_config.as_ref().map_or(ptr::null(), |c| c))
        })?;
        Ok(self)
    }

    /// Adds a new channel to the PCNT unit, with the given edge and level pins.
    pub fn add_channel(
        &mut self,
        edge_pin: Option<impl InputPin + 'd>,
        level_pin: Option<impl InputPin + 'd>,
        config: &ChannelConfig,
    ) -> Result<&mut PcntChannel<'d>, EspError> {
        self.channels
            .push(unsafe { PcntChannel::new(self.handle, edge_pin, level_pin, config)? });

        Ok(self.channels.last_mut().unwrap())
    }

    /// Set event callback for the PCNT unit.
    ///
    /// # Safety
    ///
    /// Care should be taken not to call std, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// You are not allowed to block, but you are allowed to call FreeRTOS APIs with the FromISR suffix.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Set event callbacks failed because of invalid argument
    /// - `ESP_FAIL`: Set event callbacks failed because of other error
    pub unsafe fn subscribe(
        &mut self,
        on_reach: impl FnMut(WatchEventData) + Send + 'static,
    ) -> Result<&mut Self, EspError> {
        let mut pinned: Pin<Box<DelegateUserData>> = Box::pin(DelegateUserData {
            on_reach: Box::new(on_reach),
        });

        esp!(pcnt_unit_register_event_callbacks(
            self.handle,
            &Self::ENABLE_EVENT_CALLBACKS,
            // SAFETY: The referenced data will not be moved out and this is the only reference to it
            (pinned.as_mut().get_unchecked_mut()) as *mut DelegateUserData
                as *mut core::ffi::c_void,
        ))?;

        self.user_data = Some(pinned);

        Ok(self)
    }

    /// Unregister event callback for the PCNT unit.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Failed because of other error
    pub fn unsubscribe(&mut self) -> Result<&mut Self, EspError> {
        esp!(unsafe {
            pcnt_unit_register_event_callbacks(
                self.handle,
                &Self::DISABLE_EVENT_CALLBACKS,
                ptr::null_mut(),
            )
        })?;
        self.user_data = None;
        Ok(self)
    }

    const ENABLE_EVENT_CALLBACKS: pcnt_event_callbacks_t = pcnt_event_callbacks_t {
        on_reach: Some(Self::delegate_on_reach),
    };
    const DISABLE_EVENT_CALLBACKS: pcnt_event_callbacks_t =
        pcnt_event_callbacks_t { on_reach: None };

    /// Enable the PCNT unit.
    ///
    /// # Note
    ///
    /// This function will enable the interrupt service if it was subscribed to with [`Self::subscribe`].
    /// This function will acquire a power management lock if the glitch filter was enabled with [`Self::set_glitch_filter`].
    ///
    /// Enabling the PCNT unit will not start the counting. You must call [`PcntUnitDriver::start`] to start counting.
    ///
    /// # Errors
    ///
    /// - `ESP_FAIL`: Enable PCNT unit failed because of other error
    pub fn enable(self) -> Result<PcntUnitDriver<'d, Enabled>, EspError> {
        esp!(unsafe { pcnt_unit_enable(self.handle) })?;

        // SAFETY: The generic parameter is only a marker that is not present in the memory layout, other than that they are identical
        Ok(unsafe {
            mem::transmute::<PcntUnitDriver<'d, Disabled>, PcntUnitDriver<'d, Enabled>>(self)
        })
    }

    unsafe extern "C" fn delegate_on_reach(
        _handle: pcnt_unit_handle_t,
        event_data: *const pcnt_watch_event_data_t,
        user_data: *mut core::ffi::c_void,
    ) -> bool {
        let user_data = &mut *(user_data as *mut DelegateUserData);
        let event_data = WatchEventData::from(*event_data);

        interrupt::with_isr_yield_signal(move || (user_data.on_reach)(event_data))
    }
}

impl<'d> PcntUnitDriver<'d, Enabled> {
    /// Returns a handle to the underlying PCNT unit.
    ///
    /// # Note
    ///
    /// This function is safe to call, but it is unsafe to use the returned handle in a way that would break
    /// assumptions made by this driver. For example disabling the unit while the type is `Enabled`.
    #[must_use]
    pub fn handle(&self) -> pcnt_unit_handle_t {
        self.handle
    }

    /// Disable the PCNT unit.
    ///
    /// # Note
    ///
    /// Disable a PCNT unit doesn't mean to stop it. See also [`Self::stop`] for how to stop the PCNT counter.
    pub fn disable(self) -> Result<PcntUnitDriver<'d, Disabled>, EspError> {
        esp!(unsafe { pcnt_unit_disable(self.handle) })?;

        // SAFETY: The generic parameter is only a marker that is not present in the memory layout, other than that they are identical
        Ok(unsafe {
            mem::transmute::<PcntUnitDriver<'d, Enabled>, PcntUnitDriver<'d, Disabled>>(self)
        })
    }

    /// Start the PCNT unit, the counter will start to count according to the edge and/or level input signals.
    ///
    /// # ISR Safety
    ///
    /// This function is safe to be called from an ISR context.
    ///
    /// This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`
    /// is on, so that it's allowed to be executed when cache is disabled.
    ///
    /// # Errors
    ///
    /// - `ESP_FAIL`: Start PCNT unit failed because of other error
    pub fn start(&self) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_start(self.handle) })
    }

    /// Stop PCNT from counting.
    ///
    /// # ISR Safety
    ///
    /// This function is safe to be called from an ISR context.
    ///
    /// This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`
    /// is on, so that it's allowed to be executed when cache is disabled.
    ///
    /// # Note
    ///
    /// The stop operation won't clear the counter. Also see [`Self::clear_count`]
    /// for how to clear pulse count value.
    ///
    /// # Errors
    ///
    /// - `ESP_FAIL`:Failed because of other error
    pub fn stop(&self) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_stop(self.handle) })
    }

    /// Clear PCNT pulse count value to zero.
    ///
    /// # ISR Safety
    ///
    /// This function is safe to be called from an ISR context.
    ///
    /// This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`
    /// is on, so that it's allowed to be executed when cache is disabled.
    ///
    /// # Errors
    ///
    /// - `ESP_FAIL`: Failed because of other error
    pub fn clear_count(&self) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_clear_count(self.handle) })
    }

    /// Get the current pulse count value.
    ///
    /// If the accumulation mode was enabled in the unit configuration,
    /// this function will return the accumulated count value, which
    /// includes the hardware counter value and the accumulated underflow/overflow.
    ///
    /// # ISR Safety
    ///
    /// This function is safe to be called from an ISR context.
    ///
    /// This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`
    /// is on, so that it's allowed to be executed when cache is disabled.
    ///
    /// # Errors
    ///
    /// - `ESP_FAIL`: Failed because of other error
    pub fn get_count(&self) -> Result<i32, EspError> {
        let mut count = 0;
        esp!(unsafe { pcnt_unit_get_count(self.handle, &mut count) })?;
        Ok(count)
    }

    /// Add a watch point to the PCNT unit.
    ///
    /// PCNT will generate an event when the counter value reaches the watch point value.
    ///
    /// It is recommended to call [`Self::clear_count`] after adding a watch point,
    /// so that the newly added watch point can take effect immediately.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Add watch point failed because of invalid argument, like the value is out of range
    /// - `ESP_ERR_INVALID_STATE`: Add watch point failed because the same watch point has already been added
    /// - `ESP_ERR_NOT_FOUND`: Add watch point failed because no more hardware watch point can be configured
    /// - `ESP_FAIL`: Add watch point failed because of other error
    pub fn add_watch_point(&mut self, watch_point: i32) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_add_watch_point(self.handle, watch_point) })
    }

    /// Remove a watch point for PCNT unit.
    ///
    /// # Errors
    ///
    /// - `ESP_ERR_INVALID_ARG`: Remove watch point failed because of invalid argument
    /// - `ESP_ERR_INVALID_STATE`: Remove watch point failed because the watch point was not added by [`Self::add_watch_point`] yet
    /// - `ESP_FAIL`: Remove watch point failed because of other error
    pub fn remove_watch_point(&mut self, watch_point: i32) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_remove_watch_point(self.handle, watch_point) })
    }

    /* // TODO: these are only available in some esp-idf versions
    pub fn add_watch_step(&mut self, step_interval: i32) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_add_watch_step(self.handle, step_interval) })
    }*/

    /*pub fn remove_all_watch_step(&mut self) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_remove_all_watch_step(self.handle) })
    }*/

    /*pub fn remove_single_watch_step(&mut self, step_interval: i32) -> Result<(), EspError> {
        esp!(unsafe { pcnt_unit_remove_single_watch_step(self.handle, step_interval) })
    }*/
}

// SAFETY: No thread-locals are used
unsafe impl<'d, S: State> Send for PcntUnitDriver<'d, S> {}
// SAFETY: All the functions taking &self are safe to be called from other threads
unsafe impl<'d, S: State> Sync for PcntUnitDriver<'d, S> {}

impl<'d, S: State> Drop for PcntUnitDriver<'d, S> {
    fn drop(&mut self) {
        // Disable the unit before deleting it (this is necessary). If it is already disabled,
        // this call would error, but that is ignored here (there is nothing we can do about it).
        unsafe { pcnt_unit_disable(self.handle) };
        unsafe {
            pcnt_unit_register_event_callbacks(
                self.handle,
                &PcntUnitDriver::DISABLE_EVENT_CALLBACKS,
                ptr::null_mut(),
            )
        };

        unsafe {
            pcnt_del_unit(self.handle);
        }
    }
}
