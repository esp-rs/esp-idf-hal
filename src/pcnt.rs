use core::fmt::Debug;
use core::marker::PhantomData;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
use alloc::boxed::Box;

use esp_idf_sys::*;

use enumset::EnumSetType;

use crate::gpio::InputPin;
use crate::peripheral::Peripheral;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
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

/// PCNT channel action on signal edge
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq)]
pub enum PcntCountMode {
    /// Hold current count value
    Hold,
    /// Increase count value
    #[default]
    Increment,
    /// Decrease count value
    Decrement,
}

impl From<PcntCountMode> for pcnt_count_mode_t {
    fn from(value: PcntCountMode) -> Self {
        match value {
            PcntCountMode::Hold => pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_HOLD,
            PcntCountMode::Increment => {
                pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_INCREASE
            }
            PcntCountMode::Decrement => {
                pcnt_channel_edge_action_t_PCNT_CHANNEL_EDGE_ACTION_DECREASE
            }
        }
    }
}

/// PCNT channel action on control level
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq)]
pub enum PcntControlMode {
    /// Keep current count mode
    Keep,
    /// Invert current count mode (increase -> decrease, decrease -> increase)
    #[default]
    Reverse,
    /// Hold current count value
    Disable,
}

impl From<PcntControlMode> for pcnt_ctrl_mode_t {
    fn from(value: PcntControlMode) -> Self {
        match value {
            PcntControlMode::Keep => pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PcntControlMode::Reverse => {
                pcnt_channel_level_action_t_PCNT_CHANNEL_LEVEL_ACTION_INVERSE
            }
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

/// Pulse Counter configuration for a single channel
#[derive(Debug, Copy, Clone, Default)]
pub struct PcntChannelConfig {
    /// PCNT low control mode
    pub lctrl_mode: PcntControlMode,
    /// PCNT high control mode
    pub hctrl_mode: PcntControlMode,
    /// PCNT positive edge count mode
    pub pos_mode: PcntCountMode,
    /// PCNT negative edge count mode
    pub neg_mode: PcntCountMode,
    /// Maximum counter value
    pub counter_h_lim: i16,
    /// Minimum counter value
    pub counter_l_lim: i16,
}

impl PcntChannelConfig {
    pub fn new() -> Self {
        Default::default()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum PinIndex {
    Pin0 = 0,
    Pin1 = 1,
    Pin2 = 2,
    Pin3 = 3,
}

pub struct PcntDriver<'d> {
    unit: pcnt_unit_t,
    pins: [i32; 4],
    _p: PhantomData<&'d mut ()>,
}

macro_rules! pin_to_number {
    ($pin:ident) => {
        match $pin {
            Some(pin) => {
                crate::into_ref!(pin);
                pin.pin()
            }
            None => PCNT_PIN_NOT_USED,
        }
    };
}

impl<'d> PcntDriver<'d> {
    pub fn new<PCNT: Pcnt>(
        _pcnt: impl Peripheral<P = PCNT> + 'd,
        pin0: Option<impl Peripheral<P = impl InputPin> + 'd>,
        pin1: Option<impl Peripheral<P = impl InputPin> + 'd>,
        pin2: Option<impl Peripheral<P = impl InputPin> + 'd>,
        pin3: Option<impl Peripheral<P = impl InputPin> + 'd>,
    ) -> Result<Self, EspError> {
        // consume the pins and keep only the pin number.
        let pins = [
            pin_to_number!(pin0),
            pin_to_number!(pin1),
            pin_to_number!(pin2),
            pin_to_number!(pin3),
        ];
        Ok(Self {
            unit: PCNT::unit(),
            pins,
            _p: PhantomData,
        })
    }

    /// Configure Pulse Counter chanel
    ///       @note
    ///       This function will disable three events: PCNT_EVT_L_LIM, PCNT_EVT_H_LIM, PCNT_EVT_ZERO.
    ///
    /// @param channel Channel to configure
    /// @param pulse_pin Pulse signal input pin
    /// @param ctrl_pin Control signal input pin
    /// @param pconfig Reference of PcntConfig
    ///
    /// @note  Set the signal input to PCNT_PIN_NOT_USED if unused.
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn channel_config(
        &mut self,
        channel: PcntChannel,
        pulse_pin: PinIndex,
        ctrl_pin: PinIndex,
        pconfig: &PcntChannelConfig,
    ) -> Result<(), EspError> {
        let config = pcnt_config_t {
            pulse_gpio_num: self.pins[pulse_pin as usize],
            ctrl_gpio_num: self.pins[ctrl_pin as usize],
            lctrl_mode: pconfig.lctrl_mode.into(),
            hctrl_mode: pconfig.hctrl_mode.into(),
            pos_mode: pconfig.pos_mode.into(),
            neg_mode: pconfig.neg_mode.into(),
            counter_h_lim: pconfig.counter_h_lim,
            counter_l_lim: pconfig.counter_l_lim,
            channel: channel.into(),
            unit: self.unit,
        };

        unsafe { esp!(pcnt_unit_config(&config as *const pcnt_config_t)) }
    }

    /// Get pulse counter value
    ///
    /// returns
    /// - i16
    /// - EspError
    pub fn get_counter_value(&self) -> Result<i16, EspError> {
        let mut value = 0i16;
        unsafe {
            esp!(pcnt_get_counter_value(self.unit, &mut value as *mut i16))?;
        }
        Ok(value)
    }

    /// Pause PCNT counter of PCNT unit
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn counter_pause(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_pause(self.unit)) }
    }

    /// Resume counting for PCNT counter
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn counter_resume(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_resume(self.unit)) }
    }

    /// Clear and reset PCNT counter value to zero
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn counter_clear(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_counter_clear(self.unit)) }
    }

    /// Enable PCNT interrupt for PCNT unit
    ///       @note
    ///       Each Pulse counter unit has five watch point events that share the same interrupt.
    ///       Configure events with pcnt_event_enable() and pcnt_event_disable()
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn intr_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_intr_enable(self.unit)) }
    }

    /// Disable PCNT interrupt for PCNT unit
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn intr_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_intr_disable(self.unit)) }
    }

    /// Enable PCNT event of PCNT unit
    ///
    /// @param evt_type Watch point event type.
    ///                All enabled events share the same interrupt (one interrupt per pulse counter unit).
    /// returns
    /// - ()
    /// - EspError
    pub fn event_enable(&self, evt_type: PcntEvent) -> Result<(), EspError> {
        let evt_type: pcnt_evt_type_t = PcntEventType::only(evt_type).as_repr();
        unsafe { esp!(pcnt_event_enable(self.unit, evt_type)) }
    }

    /// Disable PCNT event of PCNT unit
    ///
    /// @param evt_type Watch point event type.
    ///                All enabled events share the same interrupt (one interrupt per pulse counter unit).
    /// returns
    /// - ()
    /// - EspError
    pub fn event_disable(&self, evt_type: PcntEvent) -> Result<(), EspError> {
        let evt_type: pcnt_evt_type_t = PcntEventType::only(evt_type).as_repr();
        unsafe { esp!(pcnt_event_disable(self.unit, evt_type)) }
    }

    fn only_one_event_type(evt_type: PcntEventType) -> Result<pcnt_evt_type_t, EspError> {
        match evt_type.iter().count() {
            1 => Ok(evt_type.as_repr()),
            _ => Err(EspError::from(ESP_ERR_INVALID_ARG as esp_err_t).unwrap()),
        }
    }

    /// Set PCNT event value of PCNT unit
    ///
    /// @param evt_type Watch point event type.
    ///                All enabled events share the same interrupt (one interrupt per pulse counter unit).
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn set_event_value(&self, evt_type: PcntEventType, value: i16) -> Result<(), EspError> {
        let evt_type = Self::only_one_event_type(evt_type)?;
        unsafe { esp!(pcnt_set_event_value(self.unit, evt_type, value)) }
    }

    /// Get PCNT event value of PCNT unit
    ///
    /// @param evt_type Watch point event type.
    ///                All enabled events share the same interrupt (one interrupt per pulse counter unit).
    ///
    /// returns
    /// - i16
    /// - EspError
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

    /// Get PCNT event status of PCNT unit
    ///
    /// returns
    /// - i32
    /// - EspError
    // TODO: status is a bit field!
    pub fn get_event_status(&self) -> Result<u32, EspError> {
        let mut value = 0u32;
        unsafe {
            esp!(pcnt_get_event_status(self.unit, &mut value as *mut u32))?;
        }
        Ok(value)
    }

    // TODO: not implementing until we can do it safely! Will need to reconfigure channels?
    //
    // /// Configure PCNT pulse signal input pin and control input pin
    // ///
    // /// @param channel PcntChannel
    // /// @param pulse_io Pulse signal input pin
    // /// @param ctrl_io Control signal input pin
    // ///
    // /// @note  Set the signal input to PCNT_PIN_NOT_USED if unused.
    // ///
    // /// returns
    // /// - ()
    // /// - EspError
    // pub fn set_pin<'a>(
    //     &mut self,
    //     channel: PcntChannel,
    //     pulse_pin: Option<impl Peripheral<P = impl InputPin> + 'a>,
    //     ctrl_pin: Option<impl Peripheral<P = impl InputPin> + 'a>,
    // ) -> Result<(), EspError> {
    // }

    /// Enable PCNT input filter
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn filter_enable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_filter_enable(self.unit)) }
    }

    /// Disable PCNT input filter
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn filter_disable(&self) -> Result<(), EspError> {
        unsafe { esp!(pcnt_filter_disable(self.unit)) }
    }

    /// Set PCNT filter value
    ///
    /// @param filter_val PCNT signal filter value, counter in APB_CLK cycles.
    ///       Any pulses lasting shorter than this will be ignored when the filter is enabled.
    ///       @note
    ///       filter_val is a 10-bit value, so the maximum filter_val should be limited to 1023.
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn set_filter_value(&self, value: u16) -> Result<(), EspError> {
        unsafe { esp!(pcnt_set_filter_value(self.unit, value)) }
    }

    /// Get PCNT filter value
    ///
    /// returns
    /// - i16
    /// - EspError
    pub fn get_filter_value(&self) -> Result<u16, EspError> {
        let mut value = 0u16;
        unsafe {
            esp!(pcnt_get_filter_value(self.unit, &mut value as *mut u16))?;
        }
        Ok(value)
    }

    /// Set PCNT counter mode
    ///
    /// @param channel PCNT channel number
    /// @param pos_mode Counter mode when detecting positive edge
    /// @param neg_mode Counter mode when detecting negative edge
    /// @param hctrl_mode Counter mode when control signal is high level
    /// @param lctrl_mode Counter mode when control signal is low level
    ///
    /// returns
    /// - ()
    /// - EspError
    pub fn set_mode(
        &self,
        channel: PcntChannel,
        pos_mode: PcntCountMode,
        neg_mode: PcntCountMode,
        hctrl_mode: PcntControlMode,
        lctrl_mode: PcntControlMode,
    ) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_set_mode(
                self.unit,
                channel.into(),
                pos_mode.into(),
                neg_mode.into(),
                hctrl_mode.into(),
                lctrl_mode.into()
            ))
        }
    }

    /// Add ISR handler for specified unit.
    ///
    /// This ISR handler will be called from an ISR. So there is a stack
    /// size limit (configurable as \"ISR stack size\" in menuconfig). This
    /// limit is smaller compared to a global PCNT interrupt handler due
    /// to the additional level of indirection.
    ///
    /// # Safety
    ///
    /// Care should be taken not to call STD, libc or FreeRTOS APIs (except for a few allowed ones)
    /// in the callback passed to this function, as it is executed in an ISR context.
    ///
    /// @param callback Interrupt handler function.
    ///
    /// returns
    /// - ()
    /// - EspError
    #[cfg(feature = "alloc")]
    pub unsafe fn subscribe<C>(&self, callback: C) -> Result<(), EspError>
    where
        C: FnMut(u32) + Send + 'static,
    {
        enable_isr_service()?;

        self.unsubscribe()?;
        let callback: alloc::boxed::Box<dyn FnMut(u32) + 'static> =
            alloc::boxed::Box::new(callback);
        ISR_HANDLERS[self.unit as usize] = Some(callback);
        esp!(pcnt_isr_handler_add(
            self.unit,
            Some(Self::handle_isr),
            self.unit as *mut core::ffi::c_void,
        ))?;
        Ok(())
    }

    /// Remove ISR handler for specified unit.
    ///
    /// returns
    /// - ()
    /// - EspError
    #[cfg(feature = "alloc")]
    pub fn unsubscribe(&self) -> Result<(), EspError> {
        unsafe {
            esp!(pcnt_isr_handler_remove(self.unit))?;
            ISR_HANDLERS[self.unit as usize] = None;
        }
        Ok(())
    }

    #[cfg(feature = "alloc")]
    unsafe extern "C" fn handle_isr(data: *mut core::ffi::c_void) {
        let unit = data as pcnt_unit_t;
        if let Some(f) = &mut ISR_HANDLERS[unit as usize] {
            let mut value = 0u32;
            esp!(pcnt_get_event_status(unit, &mut value as *mut u32))
                .expect("failed to fetch event status!");
            f(value);
        }
    }
}

impl Drop for PcntDriver<'_> {
    fn drop(&mut self) {
        let _ = self.counter_pause();
        let _ = self.intr_disable();
        #[cfg(feature = "alloc")]
        unsafe {
            pcnt_isr_handler_remove(self.unit);
            ISR_HANDLERS[self.unit as usize] = None
        };
    }
}

static ISR_ALLOC_FLAGS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[cfg(feature = "alloc")]
static ISR_SERVICE_ENABLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

#[cfg(feature = "alloc")]
static PCNT_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

pub fn init_isr_alloc_flags(flags: enumset::EnumSet<crate::interrupt::IntrFlags>) {
    ISR_ALLOC_FLAGS.store(
        crate::interrupt::IntrFlags::to_native(flags),
        core::sync::atomic::Ordering::SeqCst,
    );
}

#[cfg(feature = "alloc")]
fn enable_isr_service() -> Result<(), EspError> {
    use core::sync::atomic::Ordering;

    if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
        let _cs = PCNT_CS.enter();

        if !ISR_SERVICE_ENABLED.load(Ordering::SeqCst) {
            esp!(unsafe { pcnt_isr_service_install(ISR_ALLOC_FLAGS.load(Ordering::SeqCst) as _) })?;

            ISR_SERVICE_ENABLED.store(true, Ordering::SeqCst);
        }
    }

    Ok(())
}

#[cfg(feature = "alloc")]
type IsrHandler = Option<Box<dyn FnMut(u32)>>;
#[cfg(feature = "alloc")]
static mut ISR_HANDLERS: [IsrHandler; pcnt_unit_t_PCNT_UNIT_MAX as usize] = [
    None,
    None,
    None,
    None,
    #[cfg(esp32)]
    None,
    #[cfg(esp32)]
    None,
    #[cfg(esp32)]
    None,
    #[cfg(esp32)]
    None,
];

pub trait Pcnt {
    fn unit() -> pcnt_unit_t;
}

macro_rules! impl_pcnt {
    ($pcnt:ident: $unit:expr) => {
        crate::impl_peripheral!($pcnt);

        impl Pcnt for $pcnt {
            #[inline(always)]
            fn unit() -> pcnt_unit_t {
                $unit
            }
        }
    };
}

impl_pcnt!(PCNT0: pcnt_unit_t_PCNT_UNIT_0);
impl_pcnt!(PCNT1: pcnt_unit_t_PCNT_UNIT_1);
impl_pcnt!(PCNT2: pcnt_unit_t_PCNT_UNIT_2);
impl_pcnt!(PCNT3: pcnt_unit_t_PCNT_UNIT_3);
#[cfg(esp32)]
impl_pcnt!(PCNT4: pcnt_unit_t_PCNT_UNIT_4);
#[cfg(esp32)]
impl_pcnt!(PCNT5: pcnt_unit_t_PCNT_UNIT_5);
#[cfg(esp32)]
impl_pcnt!(PCNT6: pcnt_unit_t_PCNT_UNIT_6);
#[cfg(esp32)]
impl_pcnt!(PCNT7: pcnt_unit_t_PCNT_UNIT_7);
