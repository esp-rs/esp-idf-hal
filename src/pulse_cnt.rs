

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
use esp_idf_sys::pcnt_new_channel;
use esp_idf_sys::pcnt_new_unit;
use esp_idf_sys::pcnt_unit_config_t;
use esp_idf_sys::pcnt_unit_config_t__bindgen_ty_1;
use esp_idf_sys::pcnt_unit_disable;
use esp_idf_sys::pcnt_unit_enable;
use esp_idf_sys::pcnt_unit_get_count;
use esp_idf_sys::pcnt_unit_handle_t;
use esp_idf_sys::EspError;
use esp_idf_sys::pcnt_unit_start;
use esp_idf_sys::pcnt_unit_stop;

use crate::gpio::AnyInputPin;
use crate::gpio::Pin;

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
pub struct PcntUnit(pcnt_unit_handle_t);

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
        Ok(Self(unit))
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
            esp!(pcnt_unit_enable(self.0))
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
            esp!(pcnt_unit_disable(self.0))
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
            esp!(pcnt_unit_start(self.0))
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
            esp!(pcnt_unit_stop(self.0))
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
        unsafe {
            esp!(pcnt_unit_get_count(self.0, &mut value))?;
        }
        Ok(value)
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
            esp!(pcnt_new_channel(self.0, &config, &mut channel))?;
        }
        Ok(PcntChannel(channel))
    }
}

impl Drop for PcntUnit {
    fn drop(&mut self) {
        unsafe {
            esp!(pcnt_del_unit(self.0)).unwrap();
        }
    }
}
