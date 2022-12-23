use core::fmt::Debug;


use esp_idf_sys::*;

use enumset::EnumSetType;

use crate::gpio::AnyInputPin;
use crate::gpio::Pin;

type UnitHandle = pcnt_unit_t;

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum PcntChannel {
    Channel0,
    Channel1,
}

impl From<PcntChannel> for pcnt_channel_t {
    fn from(value: PcntChannel) -> Self {
        match value {
            PcntChannel::Channel0 => pcnt_channel_t_PCNT_CHANNEL_0,
            PcntChannel::Channel1 => pcnt_channel_t_PCNT_CHANNEL_1,
        }
    }
}

#[doc = " @brief PCNT channel action on signal edge"]
#[derive(Debug, Copy, Clone)]
pub enum PcntCountMode {
    #[doc = "< Hold current count value"]
    Hold,
    #[doc = "< Increase count value"]
    Increment,
    #[doc = "< Decrease count value"]
    Decrement,
}

impl From<PcntCountMode> for pcnt_count_mode_t {
    fn from(value: PcntCountMode) -> Self {
        match value {
            PcntCountMode::Hold => pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PcntCountMode::Increment => pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PcntCountMode::Decrement => pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        }
    }
}

#[doc = " @brief PCNT channel action on control level"]
#[derive(Debug, Copy, Clone)]
pub enum PcntControlMode {
    #[doc = "< Keep current count mode"]
    Keep,
    #[doc = "< Invert current count mode (increase -> decrease, decrease -> increase)"]
    Reverse,
    #[doc = "< Hold current count value"]
    Disable,
}

impl From<PcntControlMode> for pcnt_ctrl_mode_t {
    fn from(value: PcntControlMode) -> Self {
        match value {
            PcntControlMode::Keep => pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PcntControlMode::Reverse => pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
            PcntControlMode::Disable => pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_HOLD,
        }
    }
}

#[derive(Debug, EnumSetType)]
#[enumset(repr = "u32")]
pub enum PcntEvent {
    /// PCNT watch point event: threshold1 value event
    Threshold1 = 2, // pcnt_evt_type_t_PCNT_EVT_THRES_1 = 0x04,
    /// PCNT watch point event: threshold0 value event
    Threshold0 = 3, // pcnt_evt_type_t_PCNT_EVT_THRES_0 = 0x08,
    /// PCNT watch point event: Minimum counter value
    LowLimit = 4, // pcnt_evt_type_t_PCNT_EVT_L_LIM = 0x10,
    /// PCNT watch point event: Maximum counter value
    HighLimit = 5, // pcnt_evt_type_t_PCNT_EVT_H_LIM = 0x20,
    /// PCNT watch point event: counter value zero event
    Zero = 6, // pcnt_evt_type_t_PCNT_EVT_ZERO = 0x40,
}

pub type PcntEventType = enumset::EnumSet<PcntEvent>;

#[doc = " @brief Pulse Counter configuration for a single channel"]
pub struct PcntConfig<'d> {
    #[doc = "< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored"]
    pub pulse_pin: Option<&'d AnyInputPin>,
    #[doc = "< Control signal input GPIO number, a negative value will be ignored"]
    pub ctrl_pin: Option<&'d AnyInputPin>,
    #[doc = "< PCNT low control mode"]
    pub lctrl_mode: PcntControlMode,
    #[doc = "< PCNT high control mode"]
    pub hctrl_mode: PcntControlMode,
    #[doc = "< PCNT positive edge count mode"]
    pub pos_mode: PcntCountMode,
    #[doc = "< PCNT negative edge count mode"]
    pub neg_mode: PcntCountMode,
    #[doc = "< Maximum counter value"]
    pub counter_h_lim: i16,
    #[doc = "< Minimum counter value"]
    pub counter_l_lim: i16,
    #[doc = "< the PCNT channel"]
    pub channel: PcntChannel,
}

#[derive(Debug)]
pub struct Pcnt {
    unit: pcnt_unit_t,
}

impl<'d> Pcnt {
    pub fn new() -> Result<Self, EspError> {
        Ok(Pcnt {
            unit: unit_allocate()?,
        })
    }

    #[doc = " @brief Configure Pulse Counter unit"]
    #[doc = "        @note"]
    #[doc = "        This function will disable three events: PCNT_EVT_L_LIM, PCNT_EVT_H_LIM, PCNT_EVT_ZERO."]
    #[doc = ""]
    #[doc = " @param pconfig Reference of PcntConfig"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn config(&mut self, pconfig: & PcntConfig) -> Result<(), EspError> {
        let config = pcnt_config_t {
            pulse_gpio_num: match pconfig.pulse_pin {
                Some(pin) => pin.pin(),
                None => PCNT_PIN_NOT_USED,
            },
            ctrl_gpio_num: match pconfig.ctrl_pin {
                Some(pin) => pin.pin(),
                None => PCNT_PIN_NOT_USED,
            },
            lctrl_mode: pconfig.lctrl_mode.into(),
            hctrl_mode: pconfig.hctrl_mode.into(),
            pos_mode: pconfig.pos_mode.into(),
            neg_mode: pconfig.neg_mode.into(),
            counter_h_lim: pconfig.counter_h_lim,
            counter_l_lim: pconfig.counter_l_lim,
            channel: pconfig.channel.into(),
            unit: self.unit,
        };

        unsafe {
            esp!(pcnt_unit_config(
                &config as *const pcnt_config_t
            ))
        }
    }

    #[doc = " @brief Get pulse counter value"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - i16"]
    #[doc = "     - EspError"]
    pub fn get_counter_value(&self) -> Result<i16, EspError> {
        let mut value = 0i16;
        unsafe {
            esp!(pcnt_get_counter_value(
                self.unit,
                &mut value as *mut i16
            ))?;
        }
        Ok(value)
    }

    #[doc = " @brief Pause PCNT counter of PCNT unit"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn counter_pause(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_pause(self.unit)) }
    }

    #[doc = " @brief Resume counting for PCNT counter"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn counter_resume(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_resume(self.unit)) }
    }

    #[doc = " @brief Clear and reset PCNT counter value to zero"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn counter_clear(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_clear(self.unit)) }
    }

    #[doc = " @brief Enable PCNT interrupt for PCNT unit"]
    #[doc = "        @note"]
    #[doc = "        Each Pulse counter unit has five watch point events that share the same interrupt."]
    #[doc = "        Configure events with pcnt_event_enable() and pcnt_event_disable()"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn intr_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_intr_enable(self.unit)) }
    }

    #[doc = " @brief Disable PCNT interrupt for PCNT unit"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn intr_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_intr_disable(self.unit)) }
    }

    #[doc = " @brief Enable PCNT event of PCNT unit"]
    #[doc = ""]
    #[doc = " @param evt_type Watch point event type."]
    #[doc = "                 All enabled events share the same interrupt (one interrupt per pulse counter unit)."]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn event_enable(&self, evt_type: PcntEvent) -> Result<(), EspError> {
        let evt_type: pcnt_evt_type_t = PcntEventType::only(evt_type).as_repr();
        unsafe { esp!(pcnt_event_enable(self.unit, evt_type)) }
    }

    #[doc = " @brief Disable PCNT event of PCNT unit"]
    #[doc = ""]
    #[doc = " @param evt_type Watch point event type."]
    #[doc = "                 All enabled events share the same interrupt (one interrupt per pulse counter unit)."]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn event_disable(&self, evt_type: PcntEvent) -> Result<(), EspError> {
        let evt_type: pcnt_evt_type_t = PcntEventType::only(evt_type).as_repr();
        unsafe { esp!(pcnt_event_disable(self.unit, evt_type)) }
    }

    fn only_one_event_type(evt_type: PcntEventType) -> Result<pcnt_evt_type_t, EspError> {
        match evt_type.iter().count() {
            1 => Ok(evt_type.as_repr()),
            _ =>Err(EspError::from(ESP_ERR_INVALID_ARG as esp_err_t).unwrap()),
        }
    }

    #[doc = " @brief Set PCNT event value of PCNT unit"]
    #[doc = ""]
    #[doc = " @param evt_type Watch point event type."]
    #[doc = "                 All enabled events share the same interrupt (one interrupt per pulse counter unit)."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn set_event_value(&self, evt_type: PcntEventType, value: i16) -> Result<(), EspError> {
        let evt_type = Self::only_one_event_type(evt_type)?;
        unsafe {
            esp!(pcnt_set_event_value(
                self.unit, evt_type, value
            ))
        }
    }

    #[doc = " @brief Get PCNT event value of PCNT unit"]
    #[doc = ""]
    #[doc = " @param evt_type Watch point event type."]
    #[doc = "                 All enabled events share the same interrupt (one interrupt per pulse counter unit)."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - i16"]
    #[doc = "     - EspError"]
    pub fn get_event_value(&self, evt_type: PcntEventType) -> Result<i16, EspError> {
        let evt_type = Self::only_one_event_type(evt_type)?;
        let mut value = 0i16;
        unsafe {
            esp!(pcnt_get_event_value(
                self.unit,
                evt_type,
                &mut value as *mut i16
            ))?;
        }
        Ok(value)
    }

    #[doc = " @brief Get PCNT event status of PCNT unit"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - i32"]
    #[doc = "     - EspError"]
    // TODO: status is a bit field!
    pub fn get_event_status(&self) -> Result<u32, EspError> {
        let mut value = 0u32;
        unsafe {
            esp!(pcnt_get_event_status(
                self.unit,
                &mut value as *mut u32
            ))?;
        }
        Ok(value)
    }

    #[doc = " @brief Configure PCNT pulse signal input pin and control input pin"]
    #[doc = ""]
    #[doc = " @param channel PcntChannel"]
    #[doc = " @param pulse_io Pulse signal input pin"]
    #[doc = " @param ctrl_io Control signal input pin"]
    #[doc = ""]
    #[doc = " @note  Set the signal input to PCNT_PIN_NOT_USED if unused."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn set_pin(
        &mut self,
        channel: PcntChannel,
        pulse_pin: Option<&AnyInputPin>,
        ctrl_pin: Option<&AnyInputPin>,
    ) -> Result<(), EspError> {
        let pulse_io_num = match pulse_pin {
            Some(pin) => pin.pin(),
            None => PCNT_PIN_NOT_USED,
        };
        let ctrl_io_num = match ctrl_pin {
            Some(pin) => pin.pin(),
            None => PCNT_PIN_NOT_USED,
        };
        unsafe {
            esp!(pcnt_set_pin(
                self.unit,
                channel.into(),
                pulse_io_num,
                ctrl_io_num
            ))
        }
    }

    #[doc = " @brief Enable PCNT input filter"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn filter_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_filter_enable(self.unit)) }
    }

    #[doc = " @brief Disable PCNT input filter"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn filter_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_filter_disable(self.unit)) }
    }

    #[doc = " @brief Set PCNT filter value"]
    #[doc = ""]
    #[doc = " @param filter_val PCNT signal filter value, counter in APB_CLK cycles."]
    #[doc = "        Any pulses lasting shorter than this will be ignored when the filter is enabled."]
    #[doc = "        @note"]
    #[doc = "        filter_val is a 10-bit value, so the maximum filter_val should be limited to 1023."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn set_filter_value(&self, value: u16) -> Result<(), EspError> {
        unsafe { esp!(pcnt_set_filter_value(self.unit, value)) }
    }

    #[doc = " @brief Get PCNT filter value"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - i16"]
    #[doc = "     - EspError"]
    pub fn get_filter_value(&self) -> Result<u16, EspError> {
        let mut value = 0u16;
        unsafe {
            esp!(pcnt_get_filter_value(
                self.unit,
                &mut value as *mut u16
            ))?;
        }
        Ok(value)
    }

    #[doc = " @brief Set PCNT counter mode"]
    #[doc = ""]
    #[doc = " @param channel PCNT channel number"]
    #[doc = " @param pos_mode Counter mode when detecting positive edge"]
    #[doc = " @param neg_mode Counter mode when detecting negative edge"]
    #[doc = " @param hctrl_mode Counter mode when control signal is high level"]
    #[doc = " @param lctrl_mode Counter mode when control signal is low level"]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn set_mode(&self,
        channel: PcntChannel,
        pos_mode: PcntCountMode,
        neg_mode: PcntCountMode,
        hctrl_mode: PcntControlMode,
        lctrl_mode: PcntControlMode) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_set_mode(self.unit, channel.into(), pos_mode.into(), neg_mode.into(), hctrl_mode.into(), lctrl_mode.into()))
        }
    }

    #[doc = " @brief Add ISR handler for specified unit."]
    #[doc = ""]
    #[doc = " This ISR handler will be called from an ISR. So there is a stack"]
    #[doc = " size limit (configurable as \"ISR stack size\" in menuconfig). This"]
    #[doc = " limit is smaller compared to a global PCNT interrupt handler due"]
    #[doc = " to the additional level of indirection."]
    #[doc = ""]
    #[doc = " @param callback Interrupt handler function."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub unsafe fn subscribe(&self, callback: impl FnMut(u32) + Send + 'static) -> Result<(), EspError> {
        enable_isr_service()?;

        //self.unsubscribe();
        let callback: Box<dyn FnMut(u32) + 'static> = Box::new(callback);
        ISR_HANDLERS[self.unit as usize] = Some(callback);
        esp!(pcnt_isr_handler_add(
            self.unit,
            Some(Self::handle_isr),
            self.unit as *mut core::ffi::c_void,
        ))?;
        Ok(())
    }

    #[doc = " @brief Remove ISR handler for specified unit."]
    #[doc = ""]
    #[doc = " @return"]
    #[doc = "     - ()"]
    #[doc = "     - EspError"]
    pub fn unsubscribe(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_isr_handler_remove(self.unit))?;
            ISR_HANDLERS[self.unit as usize] = None;
        }
        Ok(())
    }

    unsafe extern "C" fn handle_isr(data: *mut core::ffi::c_void) {
        let unit = data as pcnt_unit_t;
        if let Some(f) = &mut ISR_HANDLERS[unit as usize] {
            let mut value = 0u32;
            esp!(pcnt_get_event_status(
                unit,
                &mut value as *mut u32
            )).expect("failed to fetch event status!");
            f(value);
        }
    }
}

impl Drop for Pcnt {
    fn drop(&mut self) {
        let _ = self.counter_pause();
        let _ = self.intr_disable();
        unsafe {ISR_HANDLERS[self.unit as usize] = None};
        unit_deallocate(self.unit)
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static ISR_SERVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static PCNT_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

fn enable_isr_service() -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        let _ = PCNT_CS.enter();

        if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { pcnt_isr_service_install(0) })?;

            ISR_SERVICE_ENABLED.store(true, Ordering::SeqCst);
        }
    }

    Ok(())
}

static mut ISR_HANDLERS: [Option<Box<dyn FnMut(u32)>>; pcnt_unit_t_PCNT_UNIT_MAX as usize] = [
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

static mut PCNT_UNITS: [Option<UnitHandle>; pcnt_unit_t_PCNT_UNIT_MAX as usize] = [
    Some(pcnt_unit_t_PCNT_UNIT_0),
    Some(pcnt_unit_t_PCNT_UNIT_1),
    Some(pcnt_unit_t_PCNT_UNIT_2),
    Some(pcnt_unit_t_PCNT_UNIT_3),
    #[cfg(not(esp32s3))]
    Some(pcnt_unit_t_PCNT_UNIT_4),
    #[cfg(not(esp32s3))]
    Some(pcnt_unit_t_PCNT_UNIT_5),
    #[cfg(not(esp32s3))]
    Some(pcnt_unit_t_PCNT_UNIT_6),
    #[cfg(not(esp32s3))]
    Some(pcnt_unit_t_PCNT_UNIT_7),
];

fn unit_allocate() -> Result<pcnt_unit_t, EspError> {
    let _ = PCNT_CS.enter();
    for i in 0..pcnt_unit_t_PCNT_UNIT_MAX {
        if let Some(unit) = unsafe { PCNT_UNITS[i as usize].take() } {
            return Ok(unit);
        }
    }
    Err(EspError::from(ESP_ERR_NO_MEM as esp_err_t).unwrap())
}

fn unit_deallocate(unit: UnitHandle) {
    let _ = PCNT_CS.enter();
    unsafe {
        PCNT_UNITS[unit as usize] = Some(unit);
    }
}
