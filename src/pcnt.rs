use core::borrow::Borrow;
use core::fmt;
use core::fmt::Debug;
use core::marker::PhantomData;

use esp_idf_sys::esp;
use esp_idf_sys::pcnt_config_t;
use esp_idf_sys::pcnt_unit_t;
use esp_idf_sys::EspError;

use bitflags::bitflags;

use crate::gpio::AnyIOPin;
use crate::gpio::IOPin;
use crate::gpio::Input;
use crate::gpio::PinDriver;
use crate::gpio::Pull;
use crate::peripheral::Peripheral;
use crate::peripheral::PeripheralRef;

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum PcntChannel {
    Channel0,
    Channel1,
}

impl Into<esp_idf_sys::pcnt_channel_t> for PcntChannel {
    fn into(self) -> esp_idf_sys::pcnt_channel_t {
        match self {
            PcntChannel::Channel0 => esp_idf_sys::pcnt_channel_t_PCNT_CHANNEL_0,
            PcntChannel::Channel1 => esp_idf_sys::pcnt_channel_t_PCNT_CHANNEL_1,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub enum PcntUnit {
    Unit0,
    Unit1,
    Unit2,
    Unit3,
    #[cfg(not(esp32s3))]
    Unit4,
    #[cfg(not(esp32s3))]
    Unit5,
    #[cfg(not(esp32s3))]
    Unit6,
    #[cfg(not(esp32s3))]
    Unit7,
}

impl Into<esp_idf_sys::pcnt_unit_t> for PcntUnit {
    fn into(self) -> esp_idf_sys::pcnt_unit_t {
        match self {
            PcntUnit::Unit0 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_0,
            PcntUnit::Unit1 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_1,
            PcntUnit::Unit2 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_2,
            PcntUnit::Unit3 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_3,
            #[cfg(not(esp32s3))]
            PcntUnit::Unit4 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_4,
            #[cfg(not(esp32s3))]
            PcntUnit::Unit5 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_5,
            #[cfg(not(esp32s3))]
            PcntUnit::Unit6 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_6,
            #[cfg(not(esp32s3))]
            PcntUnit::Unit7 => esp_idf_sys::pcnt_unit_t_PCNT_UNIT_7,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum PcntCountMode {
    Hold,
    Increment,
    Decrement,
}

impl Into<esp_idf_sys::pcnt_count_mode_t> for PcntCountMode {
    fn into(self) -> esp_idf_sys::pcnt_count_mode_t {
        match self {
            PcntCountMode::Hold => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD
            }
            PcntCountMode::Increment => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE
            }
            PcntCountMode::Decrement => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE
            }
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum PcntControlMode {
    Keep,
    Reverse,
    Disable,
}

impl Into<esp_idf_sys::pcnt_ctrl_mode_t> for PcntControlMode {
    fn into(self) -> esp_idf_sys::pcnt_ctrl_mode_t {
        match self {
            PcntControlMode::Keep => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD
            }
            PcntControlMode::Reverse => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE
            }
            PcntControlMode::Disable => {
                esp_idf_sys::pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE
            }
        }
    }
}

bitflags! {
    #[allow(dead_code)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct PcntEventType: u32 {
        const THRES_1 = esp_idf_sys::pcnt_evt_type_t_PCNT_EVT_THRES_1;
        const THRES_0 = esp_idf_sys::pcnt_evt_type_t_PCNT_EVT_THRES_0;
        const L_LIM = esp_idf_sys::pcnt_evt_type_t_PCNT_EVT_L_LIM;
        const H_LIM = esp_idf_sys::pcnt_evt_type_t_PCNT_EVT_H_LIM;
        const ZERO = esp_idf_sys::pcnt_evt_type_t_PCNT_EVT_ZERO;
    }
}

#[doc = " @brief Pulse Counter configuration for a single channel"]
#[derive(Debug)]
pub struct PcntConfig<'d, T: Borrow<PcntPin<'d>>> {
    #[doc = "< Pulse input GPIO number, if you want to use GPIO16, enter pulse_gpio_num = 16, a negative value will be ignored"]
    pub pulse_pin: Option<T>,
    #[doc = "< Control signal input GPIO number, a negative value will be ignored"]
    pub ctrl_pin: Option<T>,
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
    pub _p: PhantomData<&'d ()>,
}

pub struct PcntPin<'d> {
    pin: PinDriver<'d, AnyIOPin, Input>
}

impl<'d> PcntPin<'d> {
    pub fn new(pin: impl Peripheral<P = impl IOPin> + 'd, pull_mode: Pull) -> Result<Self, EspError>
    {
        let pin: PeripheralRef<AnyIOPin> = pin.into_ref().map_into();
        let mut pin = PinDriver::input(pin)?;
        pin.set_pull(pull_mode)?;
        Ok(Self{pin})
    
    }

    fn pin_num(&self) -> i32 {
        self.pin.pin()
    }
}

impl<'d> Debug for PcntPin<'d> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "PcntPin: {{pin: {}}}", self.pin_num())
    }
}

#[derive(Debug)]
struct PcntChannelPins <'d, T: Borrow<PcntPin<'d>>>
{
    pulse: Option<T>,
    ctrl: Option<T>,
    _p: PhantomData<&'d ()>,
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct Pcnt<'d, T: Borrow<PcntPin<'d>>> {
    unit: pcnt_unit_t,
    channel_pins: [PcntChannelPins<'d, T>; esp_idf_sys::pcnt_channel_t_PCNT_CHANNEL_MAX as usize]
}

#[allow(dead_code)]
impl<'d, T: Borrow<PcntPin<'d>>> Pcnt<'d, T> {
    pub fn new(unit: PcntUnit) -> Pcnt<'d, T> {
        Pcnt {
            unit: unit.into(),
            channel_pins: [PcntChannelPins { pulse: None, ctrl: None, _p: PhantomData::default() }, PcntChannelPins { pulse: None, ctrl: None, _p: PhantomData::default() }],
        }
    }

    pub fn config(&mut self, pconfig: &mut PcntConfig<'d, T>) -> Result<(), EspError> {
        let mut config =         pcnt_config_t {
            pulse_gpio_num: match &pconfig.pulse_pin {
                Some(pin) => pin.borrow().pin_num(),
                None => esp_idf_sys::PCNT_PIN_NOT_USED,
            },
            ctrl_gpio_num: match &pconfig.ctrl_pin {
                Some(pin) => pin.borrow().pin_num(),
                None => esp_idf_sys::PCNT_PIN_NOT_USED,
            },
            lctrl_mode: pconfig.lctrl_mode.into(),
            hctrl_mode: pconfig.hctrl_mode.into(),
            pos_mode: pconfig.pos_mode.into(),
            neg_mode: pconfig.neg_mode.into(),
            counter_h_lim: pconfig.counter_h_lim,
            counter_l_lim: pconfig.counter_l_lim,
            channel: pconfig.channel.into(),
            ..Default::default()
        };
        config.unit = self.unit.into();
        self.channel_pins[config.channel as usize].pulse = pconfig.pulse_pin.take();
        self.channel_pins[config.channel as usize].ctrl = pconfig.pulse_pin.take();
        unsafe {
            esp!(esp_idf_sys::pcnt_unit_config(
                &config as *const pcnt_config_t
            ))
        }
    }

    pub fn get_counter_value(&self) -> Result<i16, EspError> {
        let mut value = 0i16;
        unsafe {
            esp!(esp_idf_sys::pcnt_get_counter_value(
                self.unit,
                &mut value as *mut i16
            ))?;
        }
        Ok(value)
    }

    pub fn counter_pause(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_counter_pause(self.unit)) }
    }

    pub fn counter_resume(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_counter_resume(self.unit)) }
    }

    pub fn counter_clear(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_counter_clear(self.unit)) }
    }

    pub fn intr_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_intr_enable(self.unit)) }
    }

    pub fn intr_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_intr_disable(self.unit)) }
    }

    pub fn event_enable(&self, evt_type: PcntEventType) -> Result<(), EspError> {
        let evt_type: esp_idf_sys::pcnt_evt_type_t = evt_type.bits();
        unsafe { esp!(esp_idf_sys::pcnt_event_enable(self.unit, evt_type)) }
    }

    pub fn event_disable(&self, evt_type: PcntEventType) -> Result<(), EspError> {
        let evt_type: esp_idf_sys::pcnt_evt_type_t = evt_type.bits();
        unsafe { esp!(esp_idf_sys::pcnt_event_disable(self.unit, evt_type)) }
    }

    fn only_one_event_type(evt_type: PcntEventType) -> Result<esp_idf_sys::pcnt_evt_type_t, EspError> {
        match evt_type.iter().count() {
            1 => Ok(evt_type.bits()),
            _ =>Err(EspError::from(esp_idf_sys::ESP_ERR_INVALID_ARG as esp_idf_sys::esp_err_t).unwrap()),
        }
    }

    pub fn set_event_value(&self, evt_type: PcntEventType, value: i16) -> Result<(), EspError> {
        let evt_type = Self::only_one_event_type(evt_type)?;
        unsafe {
            esp!(esp_idf_sys::pcnt_set_event_value(
                self.unit, evt_type, value
            ))
        }
    }

    pub fn get_event_value(&self, evt_type: PcntEventType) -> Result<i16, EspError> {
        let evt_type = Self::only_one_event_type(evt_type)?;
        let mut value = 0i16;
        unsafe {
            esp!(esp_idf_sys::pcnt_get_event_value(
                self.unit,
                evt_type,
                &mut value as *mut i16
            ))?;
        }
        Ok(value)
    }

    // TODO: status is a bit field!
    pub fn get_event_status(&self) -> Result<u32, EspError> {
        let mut value = 0u32;
        unsafe {
            esp!(esp_idf_sys::pcnt_get_event_status(
                self.unit,
                &mut value as *mut u32
            ))?;
        }
        Ok(value)
    }

    pub fn set_pin(
        &mut self,
        channel: PcntChannel,
        pulse_pin: Option<T>,
        ctrl_pin: Option<T>,
    ) -> Result<(), EspError> {
        let pulse_io_num = match &pulse_pin {
            Some(pin) => pin.borrow().pin_num(),
            None => esp_idf_sys::PCNT_PIN_NOT_USED,
        };
        let ctrl_io_num = match &ctrl_pin {
            Some(pin) => pin.borrow().pin_num(),
            None => esp_idf_sys::PCNT_PIN_NOT_USED,
        };
        self.channel_pins[channel as usize].pulse = pulse_pin;
        self.channel_pins[channel as usize].ctrl = ctrl_pin;
        unsafe {
            esp!(esp_idf_sys::pcnt_set_pin(
                self.unit,
                channel.into(),
                pulse_io_num,
                ctrl_io_num
            ))
        }
    }

    pub fn filter_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_filter_enable(self.unit)) }
    }

    pub fn filter_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_filter_disable(self.unit)) }
    }

    pub fn set_filter_value(&self, value: u16) -> Result<(), EspError> {
        unsafe { esp!(esp_idf_sys::pcnt_set_filter_value(self.unit, value)) }
    }

    pub fn get_filter_value(&self) -> Result<u16, EspError> {
        let mut value = 0u16;
        unsafe {
            esp!(esp_idf_sys::pcnt_get_filter_value(
                self.unit,
                &mut value as *mut u16
            ))?;
        }
        Ok(value)
    }

    pub unsafe fn subscribe(&self, callback: impl FnMut(u32) + Send + 'static) -> Result<(), EspError> {
        enable_isr_service()?;

        //self.unsubscribe();
        let callback: Box<dyn FnMut(u32) + 'static> = Box::new(callback);
        ISR_HANDLERS[self.unit as usize] = Some(callback);
        esp!(esp_idf_sys::pcnt_isr_handler_add(
            self.unit,
            Some(Self::handle_isr),
            self.unit as *mut esp_idf_sys::c_types::c_void,
        ))?;
        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::pcnt_isr_handler_remove(self.unit))?;
            ISR_HANDLERS[self.unit as usize] = None;
        }
        Ok(())
    }

    unsafe extern "C" fn handle_isr(data: *mut esp_idf_sys::c_types::c_void) {
        let unit = data as pcnt_unit_t;
        if let Some(f) = &mut ISR_HANDLERS[unit as usize] {
            let mut value = 0u32;
            esp!(esp_idf_sys::pcnt_get_event_status(
                unit,
                &mut value as *mut u32
            )).expect("failed to fetch event status!");
            f(value);
        }
    }
}

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static ISR_SERVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

#[cfg(all(not(feature = "riscv-ulp-hal"), feature = "alloc"))]
static ISR_SERVICE_ENABLED_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

#[allow(dead_code)]
fn enable_isr_service() -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        let _ = ISR_SERVICE_ENABLED_CS.enter();

        if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { esp_idf_sys::pcnt_isr_service_install(0) })?;

            ISR_SERVICE_ENABLED.store(true, Ordering::SeqCst);
        }
    }

    Ok(())
}

#[allow(dead_code)]
static mut ISR_HANDLERS: [Option<Box<dyn FnMut(u32)>>; esp_idf_sys::pcnt_unit_t_PCNT_UNIT_MAX as usize] = [
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

impl<'d, T: Borrow<PcntPin<'d>>> Drop for Pcnt<'d, T> {
    fn drop(&mut self) {
        let _ = self.intr_disable();
        unsafe {ISR_HANDLERS[self.unit as usize] = None};
    }
}
