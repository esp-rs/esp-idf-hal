


use esp_idf_sys::esp;
use esp_idf_sys::pcnt_chan_config_t;
// channel flags
use esp_idf_sys::pcnt_chan_config_t__bindgen_ty_1;
use esp_idf_sys::pcnt_channel_edge_action_t;
use esp_idf_sys::pcnt_channel_handle_t;
use esp_idf_sys::pcnt_channel_level_action_t;
use esp_idf_sys::pcnt_channel_set_edge_action;
use esp_idf_sys::pcnt_channel_set_level_action;
use esp_idf_sys::pcnt_del_channel;
use esp_idf_sys::pcnt_del_unit;
use esp_idf_sys::pcnt_event_callbacks_t;
use esp_idf_sys::pcnt_glitch_filter_config_t;
use esp_idf_sys::pcnt_new_channel;
use esp_idf_sys::pcnt_new_unit;
use esp_idf_sys::pcnt_unit_add_watch_point;
use esp_idf_sys::pcnt_unit_clear_count;
use esp_idf_sys::pcnt_unit_config_t;
// unit flags
use esp_idf_sys::pcnt_unit_config_t__bindgen_ty_1;
use esp_idf_sys::pcnt_unit_disable;
use esp_idf_sys::pcnt_unit_enable;
use esp_idf_sys::pcnt_unit_get_count;
use esp_idf_sys::pcnt_unit_handle_t;
use esp_idf_sys::EspError;
use esp_idf_sys::pcnt_unit_register_event_callbacks;
use esp_idf_sys::pcnt_unit_remove_watch_point;
use esp_idf_sys::pcnt_unit_set_glitch_filter;
use esp_idf_sys::pcnt_unit_start;
use esp_idf_sys::pcnt_unit_stop;
use esp_idf_sys::pcnt_unit_zero_cross_mode_t;
use esp_idf_sys::pcnt_watch_event_data_t;

use crate::gpio::AnyInputPin;
use crate::gpio::Pin;

#[doc = " @brief PCNT glitch filter configuration"]
#[derive(Debug, Default, Copy, Clone)]
pub struct PcntFilterConfig {
    #[doc = "< Pulse width smaller than this threshold will be treated as glitch and ignored, in the unit of ns"]
    pub max_glitch_ns: u32,
}

impl Into<pcnt_glitch_filter_config_t> for &PcntFilterConfig {
    fn into(self) -> pcnt_glitch_filter_config_t {
        pcnt_glitch_filter_config_t {
            max_glitch_ns: self.max_glitch_ns
        }
    }
}

#[doc = " @brief PCNT unit zero cross mode"]
#[derive(Debug, Copy, Clone)]
pub enum PcntZeroCrossMode {
    #[doc = "< start from positive value, end to zero, i.e. +N->0"]
    PosZero,
    #[doc = "< start from negative value, end to zero, i.e. -N->0"]
    NegZero,
    #[doc = "< start from negative value, end to positive value, i.e. -N->+M"]
    NegPos,
    #[doc = "< start from positive value, end to negative value, i.e. +N->-M"]
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

#[doc = " @brief PCNT watch event data"]
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
pub struct PcntWatchEventData {
    #[doc = "< Watch point value that triggered the event"]
    pub watch_point_value: i32,
    #[doc = "< Zero cross mode"]
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

#[derive(Debug, Copy, Clone)]
pub enum PcntEdgeAction {
    #[doc = "< Hold current count value"]
    Hold,
    #[doc = "< Increase count value"]
    Increase,
    #[doc = "< Decrease count value"]
    Decrease,
}

impl Into<pcnt_channel_edge_action_t> for PcntEdgeAction {
    fn into(self) -> pcnt_channel_edge_action_t {
        match self {
            PcntEdgeAction::Hold => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PcntEdgeAction::Increase => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PcntEdgeAction::Decrease => esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PcntLevelAction {
    #[doc = "< Keep current count mode"]
    Keep,
    #[doc = "< Invert current count mode (increase -> decrease, decrease -> increase)"]
    Inverse,
    #[doc = "< Hold current count value"]
    Hold,
}

impl Into<pcnt_channel_level_action_t> for PcntLevelAction {
    fn into(self) -> pcnt_channel_edge_action_t {
        match self {
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

#[doc = " @brief PCNT channel configuration"]
#[derive(Default)]
pub struct PcntChanConfig<'a> {
    #[doc = "< GPIO number used by the edge signal, input mode with pull up enabled. Set to -1 if unused"]
    pub edge_pin: Option<&'a AnyInputPin>,
    #[doc = "< GPIO number used by the level signal, input mode with pull up enabled. Set to -1 if unused"]
    pub level_pin: Option<&'a AnyInputPin>,
    #[doc = "< Channel config flags"]
    pub flags: PcntChanFlags,
}


#[doc = " @brief PCNT channel"]
#[derive(Debug)]
pub struct PcntChannel(pcnt_channel_handle_t);

impl PcntChannel {
    #[doc = " @brief Set channel actions when edge signal changes (e.g. falling or rising edge occurred)."]
    #[doc = "        The edge signal is input from the `edge_pin` configured in `PcntChanConfig`."]
    #[doc = "        We use these actions to control when and how to change the counter value."]
    #[doc = ""]
    #[doc = " @param[in] pos_act Action on posedge signal"]
    #[doc = " @param[in] neg_act Action on negedge signal"]
    #[doc = " @return"]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn set_edge_action(&self,pos_act: PcntEdgeAction, neg_act: PcntEdgeAction) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_channel_set_edge_action(self.0, pos_act.into(), neg_act.into()))
        }
    }

    #[doc = " @brief Set channel actions when level signal changes (e.g. signal level goes from high to low)."]
    #[doc = "        The level signal is input from the `level_gpio_num` configured in `pcnt_chan_config_t`."]
    #[doc = "        We use these actions to control when and how to change the counting mode."]
    #[doc = ""]
    #[doc = " @param[in] chan PCNT channel handle created by `pcnt_new_channel()`"]
    #[doc = " @param[in] high_act Action on high level signal"]
    #[doc = " @param[in] low_act Action on low level signal"]
    #[doc = " @return"]
    #[doc = "      - ESP_OK: Set level action for PCNT channel successfully"]
    #[doc = "      - ESP_ERR_INVALID_ARG: Set level action for PCNT channel failed because of invalid argument"]
    #[doc = "      - ESP_FAIL: Set level action for PCNT channel failed because of other error"]
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

#[doc = " @brief PCNT unit configuration"]
#[derive(Debug, Default, Copy, Clone)]
pub struct PcntUnitConfig {
    #[doc = " @brief the low limit value, should be < 0"]
    pub low_limit: i32,
    #[doc = " @brief the high limit value, should be > 0"]
    pub high_limit: i32,
    pub flags: PcntUnitFlags,
}

#[doc = " @brief PCNT unit"]
#[derive(Debug)]
pub struct PcntUnit {
    unit: pcnt_unit_handle_t,
}

impl PcntUnit {
    #[doc = " @brief Create a new PCNT unit"]
    #[doc = ""]
    #[doc = " @note The newly created PCNT unit is put in the init state."]
    #[doc = ""]
    #[doc = " @param[in] config PCNT unit configuration"]
    #[doc = " @return"]
    #[doc = "      - PcntUnit: on success"]
    #[doc = "      - EspError: on failure"]
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
        })
    }

    #[doc = " @brief Set glitch filter for PCNT unit"]
    #[doc = ""]
    #[doc = " @note The glitch filter module is clocked from APB, and APB frequency can be changed during DFS, which in return make the filter out of action."]
    #[doc = "       So this function will lazy-install a PM lock internally when the power management is enabled. With this lock, the APB frequency won't be changed."]
    #[doc = "       The PM lock can be uninstalled when the unit is dropped."]
    #[doc = " @note This function should be called when the PCNT unit is in the init state (i.e. before calling `enable()`)"]
    #[doc = ""]
    #[doc = " @param[in] config PCNT filter configuration, set config to None means disabling the filter function"]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn set_filter(&self, config: Option<&PcntFilterConfig>) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_set_glitch_filter(self.unit, match config {
                Some(x) => &x.into(),
                None => std::ptr::null(),
            }))
        }
    }

    #[doc = " @brief Enable the PCNT unit"]
    #[doc = ""]
    #[doc = " @note This function will transit the unit state from init to enable."]
    #[doc = " @note This function will enable the interrupt service, if it's lazy installed in `subscribe()`."]
    #[doc = " @note This function will acquire the PM lock if it's lazy installed in `set_glitch_filter()`."]
    #[doc = " @note Enable a PCNT unit doesn't mean to start it. See also `start()` for how to start the PCNT counter."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn enable(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_enable(self.unit))
        }
    }

    #[doc = " @brief Disable the PCNT unit"]
    #[doc = ""]
    #[doc = " @note This function will do the opposite work to the `enable()`"]
    #[doc = " @note Disable a PCNT unit doesn't mean to stop it. See also `stop()` for how to stop the PCNT counter."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn disable(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_disable(self.unit))
        }
    }

    #[doc = " @brief Start the PCNT unit, the counter will start to count according to the edge and/or level input signals"]
    #[doc = ""]
    #[doc = " @note This function should be called when the unit is in the enable state (i.e. after calling `pcnt_unit_enable()`)"]
    #[doc = " @note This function is allowed to run within ISR context"]
    #[doc = " @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM` is on, so that it's allowed to be executed when Cache is disabled"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn start(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_start(self.unit))
        }
    }

    #[doc = " @brief Stop PCNT from counting"]
    #[doc = ""]
    #[doc = " @note This function should be called when the unit is in the enable state (i.e. after calling `pcnt_unit_enable()`)"]
    #[doc = " @note The stop operation won't clear the counter. Also see `pcnt_unit_clear_count()` for how to clear pulse count value."]
    #[doc = " @note This function is allowed to run within ISR context"]
    #[doc = " @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it is allowed to be executed when Cache is disabled"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn stop(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_stop(self.unit))
        }
    }

    #[doc = " @brief Clear PCNT pulse count value to zero"]
    #[doc = ""]
    #[doc = " @note It's recommended to call this function after adding a watch point by `pcnt_unit_add_watch_point()`, so that the newly added watch point is effective immediately."]
    #[doc = " @note This function is allowed to run within ISR context"]
    #[doc = " @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it's allowed to be executed when Cache is disabled"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - (): on success"]
    #[doc = "      - EspError: on failure"]
    pub fn clear_count(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_unit_clear_count(self.unit))
        }
    }

    #[doc = " @brief Get PCNT count value"]
    #[doc = ""]
    #[doc = " @note This function is allowed to run within ISR context"]
    #[doc = " @note This function will be placed into IRAM if `CONFIG_PCNT_CTRL_FUNC_IN_IRAM`, so that it's allowed to be executed when Cache is disabled"]
    #[doc = ""]
    #[doc = " @param[out] value Returned count value"]
    #[doc = " @return"]
    #[doc = "      - value: on success"]
    #[doc = "      - EspError: on failure"]
    pub fn get_count(&self) -> Result<i32, EspError> {
        let mut value: esp_idf_sys::c_types::c_int = 0;
        unsafe { esp!(pcnt_unit_get_count(self.unit, &mut value))?; }
        Ok(value)
    }

    #[doc = " @brief Add a watch point for PCNT unit, PCNT will generate an event when the counter value reaches the watch point value"]
    #[doc = ""]
    #[doc = " @param[in] watch_point Value to be watched"]
    #[doc = " @return"]
    #[doc = "      - value: on success"]
    #[doc = "      - EspError: on failure"]
    pub fn add_watch_point(&self, watch_point: i32) -> Result<(), EspError> {
        unsafe {esp!(pcnt_unit_add_watch_point(self.unit, watch_point))}
    }

    #[doc = " @brief Remove a watch point for PCNT unit"]
    #[doc = ""]
    #[doc = " @param[in] watch_point Watch point value"]
    #[doc = " @return"]
    #[doc = "      - value: on success"]
    #[doc = "      - EspError: on failure"]
    pub fn remove_watch_point(&self, watch_point: i32) -> Result<(), EspError> {
        unsafe {esp!(pcnt_unit_remove_watch_point(self.unit, watch_point))}
    }

    #[doc = " @brief subscribe to PCNT events"]
    #[doc = ""]
    #[doc = " @note User registered callbacks are expected to be runnable within ISR context"]
    #[doc = " @note The first call to this function needs to be before the call to `pcnt_unit_enable`"]
    #[doc = " @note User can deregister a previously registered callback by calling this function and passing None."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "      - value: on success"]
    #[doc = "      - EspError: on failure"]
    pub fn subscribe<F>(&self, callback: F ) -> Result<(), EspError>
    where
        F: FnMut(PcntWatchEventData)->bool + Send + 'static
    {
        let mut closure = callback;
        let cbs = pcnt_event_callbacks_t {
            on_reach: Self::get_trampoline(&closure),
        };
        let data = &mut closure as *mut _ as *mut esp_idf_sys::c_types::c_void;
        unsafe {esp!(pcnt_unit_register_event_callbacks(self.unit, &cbs, data as *mut esp_idf_sys::c_types::c_void))}
    }

    pub fn get_trampoline<F>(_closure: &F) -> esp_idf_sys::pcnt_watch_cb_t
        where
            F: FnMut(PcntWatchEventData)->bool
    {
        Some(Self::trampoline::<F>)
    }
        
    unsafe extern "C" fn trampoline<F>(_unit: pcnt_unit_handle_t, edata: *const pcnt_watch_event_data_t, data: *mut esp_idf_sys::c_types::c_void) -> bool
    where
        F: FnMut(PcntWatchEventData)->bool
    {
        let f = &mut *(data as *mut F);
        f(edata.into())
    }
    #[doc = " @brief Create PCNT channel for specific unit, each PCNT has several channels associated with it"]
    #[doc = ""]
    #[doc = " @note This function should be called when the unit is in init state (i.e. before calling `pcnt_unit_enable()`)"]
    #[doc = ""]
    #[doc = " @param[in] config PCNT channel configuration"]
    #[doc = " @return"]
    #[doc = "      - PcntChannel: on success"]
    #[doc = "      - EspError: on failure"]
    pub fn channel(&self, config: &PcntChanConfig) -> Result<PcntChannel, EspError> {
        let config = pcnt_chan_config_t {
            edge_gpio_num: match config.edge_pin {
                Some(pin) => pin.pin(),
                None => -1,
            },
            level_gpio_num: match config.level_pin {
                Some(pin) => pin.pin(),
                None => -1,
            },
            flags: config.flags.0,
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
