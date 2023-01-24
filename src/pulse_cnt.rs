
use esp_idf_sys::*;
use crate::gpio::InputPin;
use crate::peripheral::Peripheral;

/// @brief PCNT glitch filter configuration
#[derive(Debug, Default, Copy, Clone)]
pub struct PcntFilterConfig {
    ///< Pulse width smaller than this threshold will be treated as glitch and ignored, in the unit of ns
    pub max_glitch_ns: u32,
}

impl From<&PcntFilterConfig> for pcnt_glitch_filter_config_t {
    fn from(value: &PcntFilterConfig) -> Self {
        pcnt_glitch_filter_config_t {
            max_glitch_ns: value.max_glitch_ns
        }
    }
}

/// @brief PCNT unit zero cross mode
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PcntZeroCrossMode {
    ///< start from positive value, end to zero, i.e. +N->0
    PosZero,
    ///< start from negative value, end to zero, i.e. -N->0
    NegZero,
    ///< start from negative value, end to positive value, i.e. -N->+M
    NegPos,
    ///< start from positive value, end to negative value, i.e. +N->-M
    PosNeg,
}

impl From<pcnt_unit_zero_cross_mode_t> for PcntZeroCrossMode {
    fn from(x: pcnt_unit_zero_cross_mode_t) -> Self {
        match x {
            esp_idf_sys::pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_POS_ZERO => PcntZeroCrossMode::PosZero,
            esp_idf_sys::pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_NEG_ZERO => PcntZeroCrossMode::NegZero,
            esp_idf_sys::pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_NEG_POS => PcntZeroCrossMode::NegPos,
            esp_idf_sys::pcnt_unit_zero_cross_mode_t_PCNT_UNIT_ZERO_CROSS_POS_NEG => PcntZeroCrossMode::PosNeg,
            _ => panic!("unknown pcnt_unit_zero_cross_mode_t value {}", x),
        }
    }
}

/// @brief PCNT watch event data
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub struct PcntWatchEventData {
    ///< Watch point value that triggered the event
    pub watch_point_value: i32,
    ///< Zero cross mode
    pub zero_cross_mode: PcntZeroCrossMode,
}

impl From<*const pcnt_watch_event_data_t> for PcntWatchEventData {
    fn from(x: *const pcnt_watch_event_data_t) -> Self {
        unsafe {
            Self {
                watch_point_value: (*x).watch_point_value,
                zero_cross_mode: (*x).zero_cross_mode.into(),
            }
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PcntEdgeAction {
    ///< Hold current count value
    Hold,
    ///< Increase count value
    Increase,
    ///< Decrease count value
    Decrease,
}

impl From<PcntEdgeAction> for pcnt_channel_edge_action_t {
    fn from(value: PcntEdgeAction) -> Self {
        match value {
            PcntEdgeAction::Hold => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PcntEdgeAction::Increase => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PcntEdgeAction::Decrease => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PcntLevelAction {
    ///< Keep current count mode
    Keep,
    ///< Invert current count mode (increase -> decrease, decrease -> increase)
    Inverse,
    ///< Hold current count value
    Hold,
}

impl From<PcntLevelAction> for pcnt_channel_level_action_t {
    fn from(value: PcntLevelAction) -> Self {
        match value {
            PcntLevelAction::Keep => esp_idf_sys::pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PcntLevelAction::Inverse => esp_idf_sys::pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PcntLevelAction::Hold => esp_idf_sys::pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_HOLD,
        }
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct PcntChanFlags(pcnt_chan_config_t__bindgen_ty_1);

impl PcntChanFlags {
    #[inline]
    pub fn invert_edge_input(&self) -> u32 {
        self.0.invert_edge_input()
    }

    #[inline]
    pub fn set_invert_edge_input(&mut self, val: u32) {
        self.0.set_invert_edge_input(val)
    }

    #[inline]
    pub fn invert_level_input(&self) -> u32 {
        self.0.invert_level_input()
    }

    #[inline]
    pub fn set_invert_level_input(&mut self, val: u32) {
        self.0.set_invert_level_input(val)
    }

    #[inline]
    pub fn virt_edge_io_level(&self) -> u32 {
        self.0.virt_edge_io_level()
    }

    #[inline]
    pub fn set_virt_edge_io_level(&mut self, val: u32) {
        self.0.set_virt_edge_io_level(val)
    }

    #[inline]
    pub fn virt_level_io_level(&self) -> u32 {
        self.0.virt_level_io_level()
    }

    #[inline]
    pub fn set_virt_level_io_level(&mut self, val: u32) {
        self.0.set_virt_level_io_level(val)
    }

    #[inline]
    pub fn io_loop_back(&self) -> u32 {
        self.0.io_loop_back()
    }

    #[inline]
    pub fn set_io_loop_back(&mut self, val: u32) {
        self.0.set_io_loop_back(val)
    }
}

/// @brief PCNT channel
#[derive(Debug)]
pub struct PcntChannel(pcnt_channel_handle_t);

impl PcntChannel {
    /// @brief Set channel actions when edge signal changes (e.g. falling or rising edge occurred).
    ///        The edge signal is input from the `edge_pin` configured in `PcntChanConfig`.
    ///        We use these actions to control when and how to change the counter value.
    ///
    /// @param[in] pos_act Action on posedge signal
    /// @param[in] neg_act Action on negedge signal
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn set_edge_action(&self,pos_act: PcntEdgeAction, neg_act: PcntEdgeAction) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_channel_set_edge_action(self.0, pos_act.into(), neg_act.into()))
        }
    }

    /// @brief Set channel actions when level signal changes (e.g. signal level goes from high to low).
    ///        The level signal is input from the `level_gpio_num` configured in `pcnt_chan_config_t`.
    ///        We use these actions to control when and how to change the counting mode.
    ///
    /// @param[in] chan PCNT channel handle created by `pcnt_new_channel()`
    /// @param[in] high_act Action on high level signal
    /// @param[in] low_act Action on low level signal
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn set_level_action(&self, high_act: PcntLevelAction, low_act: PcntLevelAction) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_channel_set_level_action(self.0, high_act.into(), low_act.into()))
        }
    }
}

impl Drop for PcntChannel {
    fn drop(&mut self) {
        unsafe {
            esp!(pcnt_del_channel(self.0)).unwrap();
        }
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct PcntUnitFlags(pcnt_unit_config_t__bindgen_ty_1);
impl PcntUnitFlags {
    #[inline]
    pub fn accum_count(&self) -> u32 {
        self.0.accum_count()
    }

    #[inline]
    pub fn set_accum_count(&mut self, val: u32) -> Self {
        self.0.set_accum_count(val);
        *self
    }
}

/// @brief PCNT unit configuration
#[derive(Debug, Default, Clone)]
pub struct PcntUnitConfig {
    /// @brief the low limit value, should be < 0
    pub low_limit: i32,
    /// @brief the high limit value, should be > 0
    pub high_limit: i32,
    pub flags: PcntUnitFlags,
}

/// @brief PCNT unit
#[derive(Debug)]
pub struct PcntUnit {
    unit: pcnt_unit_handle_t,
    isr_id: Option<usize>,
}

impl PcntUnit {
    /// @brief Create a new PCNT unit
    ///
    /// @note The newly created PCNT unit is put in the init state.
    ///
    /// @param[in] config PCNT unit configuration
    /// @return
    ///      - PcntUnit: on success
    ///      - EspError: on failure
    pub fn new(config: &PcntUnitConfig) -> Result<Self, EspError> {
        let config = pcnt_unit_config_t {
            low_limit: config.low_limit,
            high_limit: config.high_limit,
            flags: config.flags.0,
        };
        let mut unit: pcnt_unit_handle_t = std::ptr::null_mut();
        unsafe {
            esp!(pcnt_new_unit(&config, &mut unit))?;
        }
        Ok(Self {
            unit,
            isr_id: None,
        })
    }

    /// @brief Set glitch filter for PCNT unit
    ///
    /// @note The glitch filter module is clocked from APB, and APB frequency can be changed during DFS, which in return make the filter out of action.
    ///       So this function will lazy-install a PM lock internally when the power management is enabled. With this lock, the APB frequency won't be changed.
    ///       The PM lock can be uninstalled when the unit is dropped.
    /// @note This function should be called when the PCNT unit is in the init state (i.e. before calling `enable()`)
    ///
    /// @param[in] config PCNT filter configuration, set config to None means disabling the filter function
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn set_filter(&self, config: Option<&PcntFilterConfig>) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_set_glitch_filter(self.unit, match config {
                Some(x) => &x.into(),
                None => std::ptr::null(),
            }))
        }
    }

    /// @brief Enable the PCNT unit
    ///
    /// @note This function will transit the unit state from init to enable.
    /// @note This function will enable the interrupt service, if it's lazy installed in `subscribe()`.
    /// @note This function will acquire the PM lock if it's lazy installed in `set_glitch_filter()`.
    /// @note Enable a PCNT unit doesn't mean to start it. See also `start()` for how to start the PCNT counter.
    ///
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn enable(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_enable(self.unit))
        }
    }

    /// @brief Disable the PCNT unit
    ///
    /// @note This function will do the opposite work to the `enable()`
    /// @note Disable a PCNT unit doesn't mean to stop it. See also `stop()` for how to stop the PCNT counter.
    ///
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn disable(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_disable(self.unit))
        }
    }

    /// @brief Start the PCNT unit, the counter will start to count according to the edge and/or level input signals
    ///
    /// @note This function should be called when the unit is in the enable state (i.e. after calling `pcnt_unit_enable()`)
    /// @note This function is allowed to run within ISR context
    /// @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM` is on, so that it's allowed to be executed when Cache is disabled
    ///
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn start(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_start(self.unit))
        }
    }

    /// @brief Stop PCNT from counting
    ///
    /// @note This function should be called when the unit is in the enable state (i.e. after calling `pcnt_unit_enable()`)
    /// @note The stop operation won't clear the counter. Also see `pcnt_unit_clear_count()` for how to clear pulse count value.
    /// @note This function is allowed to run within ISR context
    /// @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it is allowed to be executed when Cache is disabled
    ///
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn stop(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_stop(self.unit))
        }
    }

    /// @brief Clear PCNT pulse count value to zero
    ///
    /// @note It's recommended to call this function after adding a watch point by `pcnt_unit_add_watch_point()`, so that the newly added watch point is effective immediately.
    /// @note This function is allowed to run within ISR context
    /// @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it's allowed to be executed when Cache is disabled
    ///
    /// @return
    ///      - (): on success
    ///      - EspError: on failure
    pub fn clear_count(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_clear_count(self.unit))
        }
    }

    /// @brief Get PCNT count value
    ///
    /// @note This function is allowed to run within ISR context
    /// @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it's allowed to be executed when Cache is disabled
    ///
    /// @param[out] value Returned count value
    /// @return
    ///      - value: on success
    ///      - EspError: on failure
    pub fn get_count(&self) -> Result<i32, EspError> {
        let mut value: core::ffi::c_int = 0;
        unsafe { esp!(pcnt_unit_get_count(self.unit, &mut value))?; }
        Ok(value)
    }

    /// @brief Add a watch point for PCNT unit, PCNT will generate an event when the counter value reaches the watch point value
    ///
    /// @param[in] watch_point Value to be watched
    /// @return
    ///      - value: on success
    ///      - EspError: on failure
    pub fn add_watch_point(&self, watch_point: i32) -> Result<(), EspError> {
        unsafe {esp!(pcnt_unit_add_watch_point(self.unit, watch_point))}
    }

    /// @brief Remove a watch point for PCNT unit
    ///
    /// @param[in] watch_point Watch point value
    /// @return
    ///      - value: on success
    ///      - EspError: on failure
    pub fn remove_watch_point(&self, watch_point: i32) -> Result<(), EspError> {
        unsafe {esp!(pcnt_unit_remove_watch_point(self.unit, watch_point))}
    }

    /// @brief subscribe to PCNT events
    ///
    /// @note User registered callbacks are expected to be runnable within ISR context
    /// @note The first call to this function needs to be before the call to `pcnt_unit_enable`
    /// @note User can deregister a previously registered callback by calling this function and passing None.
    ///
    /// @return
    ///      - value: on success
    ///      - EspError: on failure
    pub fn subscribe<F>(&mut self, callback: F ) -> Result<(), EspError>
    where
        F: FnMut(PcntWatchEventData)->bool + Send + 'static
    {
        let id = match self.isr_id {
            Some(x) => x,
            None => {
                let x = allocate_isr_id(callback);
                self.isr_id = Some(x);
                x
            }
        };
        let cbs = pcnt_event_callbacks_t {
            on_reach: Some(Self::handle_isr),
        };
        let data = id as *mut core::ffi::c_void;
        unsafe {esp!(pcnt_unit_register_event_callbacks(self.unit, &cbs, data as *mut core::ffi::c_void))}
    }

    unsafe extern "C" fn handle_isr(_unit: pcnt_unit_handle_t, edata: *const pcnt_watch_event_data_t, data: *mut core::ffi::c_void) -> bool {
        let id = data as usize;
        match &mut ISR_HANDLERS[id] {
            Some(f) => f(edata.into()),
            None => panic!("isr handler called with no ISR!"),
        }
    }

    /// @brief Unsubscribe to PCNT events
    ///
    /// @param unit PCNT unit number
    ///
    /// @return
    ///      - value: on success
    ///      - EspError: on failure
    pub fn unsubscribe(&mut self) -> Result<(), EspError> {
        if let Some(id) = self.isr_id.take() {
            unsafe {
                esp!(pcnt_unit_register_event_callbacks(self.unit, &pcnt_event_callbacks_t {
                    on_reach: None,
                }, 0 as *mut core::ffi::c_void))?;
            }
            free_isr_id(id);
        }
        Ok(())
    }

    /// @brief Create PCNT channel for specific unit, each PCNT has several channels associated with it
    ///
    /// @note This function should be called when the unit is in init state (i.e. before calling `pcnt_unit_enable()`)
    ///
    /// @param[in] config PCNT channel configuration
    /// @return
    ///      - PcntChannel: on success
    ///      - EspError: on failure
    pub fn channel<'a>(
        &self,
        edge_pin: Option<impl Peripheral<P = impl InputPin>+'a>,
        level_pin: Option<impl Peripheral<P = impl InputPin>+'a>,
        flags: PcntChanFlags
    ) -> Result<PcntChannel, EspError> {
        let config = pcnt_chan_config_t {
            edge_gpio_num: match edge_pin {
                Some(pin) => {
                    crate::into_ref!(pin);
                    pin.pin()
                },
                None => -1,
            },
            level_gpio_num: match level_pin {
                Some(pin) => {
                    crate::into_ref!(pin);
                    pin.pin()
                },
                None => -1,
            },
            flags: flags.0,
        };
        let mut channel: pcnt_channel_handle_t = std::ptr::null_mut();
        unsafe {
            esp!(pcnt_new_channel(self.unit, &config, &mut channel))?;
        }
        Ok(PcntChannel(channel))
    }
}

impl Drop for PcntUnit {
    fn drop(&mut self) {
        unsafe {
            esp!(pcnt_del_unit(self.unit)).unwrap();
        }
    }
}

static PCNT_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

#[cfg(esp32s3)]
const PCNT_UNIT_MAX: usize = 4;
#[cfg(not(esp32s3))]
const PCNT_UNIT_MAX: usize = 8;

static mut ISR_HANDLERS: [Option<Box<dyn FnMut(PcntWatchEventData)->bool>>; PCNT_UNIT_MAX] = [
    None, None, None, None, 
    #[cfg(not(esp32s3))]
    None,
    #[cfg(not(esp32s3))]
    None,
    #[cfg(not(esp32s3))]
    None,
    #[cfg(not(esp32s3))]
    None, 
];

fn allocate_isr_id(callback: impl FnMut(PcntWatchEventData)->bool + 'static ) -> usize {
    let _cs = PCNT_CS.enter();
    for i in 0..PCNT_UNIT_MAX {
        unsafe {
            if ISR_HANDLERS[i].is_none() {
                ISR_HANDLERS[i] = Some(Box::new(callback));
                return i;
            }
        }
    }
    panic!("all ISR slots are full!");
}

fn free_isr_id(id: usize) {
    let _cs = PCNT_CS.enter();
    unsafe {
        ISR_HANDLERS[id] = None;
    }
}
