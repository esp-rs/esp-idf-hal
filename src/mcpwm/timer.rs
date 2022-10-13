use core::convert::TryInto;
use core::marker::PhantomData;
use std::ptr;

use esp_idf_sys::{
    esp, mcpwm_del_timer, mcpwm_new_timer, mcpwm_timer_config_t, mcpwm_timer_enable,
    mcpwm_timer_handle_t, EspError,
};

use crate::mcpwm::Group;
use crate::prelude::FromValueType;
use crate::units::Hertz;

use super::operator::NoOperator;
use super::timer_connection::TimerConnection;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimerConfig {
    frequency: Hertz,
    //resolution: Hertz,
    count_mode: CountMode,
    // TODO
    // on_full: FF,
    // on_empty: FE,
    // on_stop: FS,
}

impl Default for TimerConfig {
    fn default() -> Self {
        Self {
            frequency: 1_u32.kHz().into(),
            //resolution: Default::default(),
            count_mode: CountMode::Up,
        }
    }
}

impl TimerConfig {
    /*#[must_use]
    pub fn resolution(mut self, resolution: impl Into<Hertz>) -> Self {
        self.resolution = resolution.into();
        self
    }*/

    /// Frequency which the operator will run at, can also be changed live later
    #[must_use]
    pub fn frequency(mut self, frequency: impl Into<Hertz>) -> Self {
        self.frequency = frequency.into();
        self
    }

    #[must_use]
    pub fn counter_mode(mut self, counter_mode: CountMode) -> Self {
        self.count_mode = counter_mode;
        self
    }
}

//TODO
//impl<G, H> TimerConfig<NoCb, G, H> {
//#[must_use]
//pub fn on_full(mut self, on_full: CB) -> Self<CB, _, _> {
//    self.on_full = on_full;
//    self
//}
//}

pub struct Timer<const N: u8, G: Group> {
    _group: G,
    handle: mcpwm_timer_handle_t,
    _timer: TIMER<N, G>,
    /// Number of ticks within a period
    /// 
    /// See `Self::get_period_ticks` for more info
    period_ticks: u32,

    /// This is the maximum value that the comparator will see
    /// 
    /// See `Self::get_period_peak` for more info
    period_peak: u16,
}

impl<const N: u8, G: Group> Timer<N, G> {
    pub fn new(timer: TIMER<N, G>, config: TimerConfig) -> Self {
        let cfg = mcpwm_timer_config_t {
            group_id: todo!(),
            clk_src: todo!(),
            resolution_hz: todo!(),
            count_mode: todo!(),
            period_ticks: todo!(),
            flags: todo!(),
        };
        let mut handle: mcpwm_timer_handle_t = ptr::null_mut();
        unsafe {
            esp!(mcpwm_new_timer(&cfg, &mut handle));
        }
        // TODO: note that this has to be called before mcpwm_timer_enable
        // mcpwm_timer_register_event_callbacks()
        unsafe {
            esp!(mcpwm_timer_enable(handle)).unwrap();
        }

        let period_peak = if config.count_mode == CountMode::UpDown {
            (cfg.period_ticks / 2).try_into().unwrap()
        } else {
            cfg.period_ticks.try_into().unwrap()
        };

        Self {
            _group: G::default(),
            handle,
            _timer: timer,
            period_ticks: cfg.period_ticks,
            period_peak
        }
    }

    // TODO: make sure this description is accurate
    /// Get number of ticks per period
    /// 
    /// Use this when working with the frequency or the period
    /// 
    /// NOTE: This will be the same as `Self::get_period_peak` for all `CounterMode` except for
    /// `CounterMode::UpDown` where the period will be twice as large as the peak value since
    /// the timer will count from zero to peak and then down to zero again
    pub fn get_period_ticks(&self) -> u32 {
        self.period_ticks
    }

    // TODO: make sure this description is accurate
    /// This is the maximum value that the comparator will see
    /// 
    /// Use this working with the duty
    /// 
    /// NOTE: This will not be the same as `Self::get_period_ticks` when using `CounterMode::UpDown`
    /// See `Self::get_period_ticks` for more info
    pub fn get_period_peak(&self) -> u16 {
        self.period_peak
    }

    pub fn timer(&self) -> mcpwm_timer_handle_t {
        self.handle
    }

    // TODO: It seems that we can't have both at the same time:
    // a method for releasing its hardware resources
    // and implementing Drop.
    /*
    pub fn release(self) -> TIMER<N, G> {
        let Self {
            _group,
            _timer,
            handle
        } = self;
        unsafe {
            esp!(mcpwm_del_timer(handle)).unwrap();
        }
        _timer
    }*/

    pub fn into_connection(self) -> TimerConnection<N, G, NoOperator, NoOperator, NoOperator> {
        TimerConnection::new(self)
    }
}

impl<const N: u8, G: Group> Drop for Timer<N, G> {
    fn drop(&mut self) {
        unsafe {
            esp!(mcpwm_del_timer(self.handle)).unwrap();
        }
    }
}

/// Counter mode for operator's timer for generating PWM signal
// TODO: For UpDown, frequency is half of MCPWM frequency set
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CountMode {
    /// Timer is frozen or paused
    //#[cfg(not(esp_idf_version = "4.3"))]
    //Frozen,
    /// Edge aligned. The counter will start from its lowest value and increment every clock cycle until the period is reached.
    ///
    /// The wave form will end up looking something like the following:
    /// ```
    ///       start, counter = 0                     reset, counter = period
    ///         |                                       |
    ///         |                                       |*--- start, counter = 0
    ///         v <----  duty  ----> .                  v|
    ///         .                    .                  .v
    ///         .--------------------.                  ..----
    ///         |       Active       |                  .|
    ///         |                    |                  .|
    ///         |                    |     Not active   .|
    ///         -                    ---------------------
    /// ```
    Up,

    /// Edge aligned. The counter will start from its highest value, period and decrement every clock cycle until the zero is reached
    ///
    /// The wave form will end up looking something like the following:
    /// ```
    ///       start, counter = period                   reset, counter = 0
    ///         |                                         |
    ///         |                                         |*--- start, counter = period
    ///         v                    .                    v|
    ///         .                    . <----  duty  ----> .v
    ///         .                    .--------------------..
    ///         .       Active       |                    |.
    ///         .                    |                    |.
    ///         .     Not active     |      Active        |.
    ///         ----------------------                    ----
    /// ```
    Down,

    /// Symmetric mode. The counter will start from its lowest value and increment every clock cycle until the period is reached
    ///
    /// The wave form will end up looking something like the following:
    /// ```
    ///                                             change count dir to decrement, counter = period
    ///       start, counter = 0, incrementing          |                                     change count dir to increment, counter = 0
    ///         |                                       |                                        |
    ///         |                                       |*--- counter = period                   |*----- start, counter = 0, incrementing
    ///         v <----  duty  ----> .                  v|                  . <----  duty  ----> ||
    ///         .                    .                  .v                  .                    vv
    ///         ---------------------.                  ..                  .-------------------------------------------.                  ..                  .--
    ///                 Active       |                  ..                  |        Active                Active       |                  ..                  |
    ///                              |                  ..                  |                                           |                  ..                  |
    ///                              |     Not active   ..    Not active    |                                           |     Not active   ..    Not active    |
    ///                              ----------------------------------------                                           ----------------------------------------
    /// ```
    /// NOTE: That in this mode, the frequency will be half of that specified
    UpDown,
}

/*
impl From<CounterMode> for mcpwm_counter_type_t {
    fn from(val: CounterMode) -> Self {
        match val {
            //CounterMode::Frozen => mcpwm_counter_type_t_MCPWM_FREEZE_COUNTER,
            CounterMode::Up => mcpwm_counter_type_t_MCPWM_UP_COUNTER,
            CounterMode::Down => mcpwm_counter_type_t_MCPWM_DOWN_COUNTER,
            CounterMode::UpDown => mcpwm_counter_type_t_MCPWM_UP_DOWN_COUNTER,
        }
    }
}*/

pub struct TIMER<const N: u8, G: Group> {
    _ptr: PhantomData<*const ()>,
    _group: PhantomData<G>,
}

impl<const N: u8, G: Group> TIMER<N, G> {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this peripheralinstance, if it is already instantiated and used elsewhere
    #[inline(always)]
    pub unsafe fn new() -> Self {
        Self {
            _ptr: PhantomData,
            _group: PhantomData,
        }
    }
}

unsafe impl<const N: u8, G: Group> Send for TIMER<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::sealed::Sealed for TIMER<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::Peripheral for TIMER<N, G> {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self { ..*self }
    }
}
