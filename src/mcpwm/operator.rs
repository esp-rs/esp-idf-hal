use esp_idf_sys::{EspError, mcpwm_oper_handle_t};

use crate::{mcpwm::{Group, Group0, Group1}, gpio::OutputPin};

use super::{Duty, timer_connection::OptionalOutputPin};

use core::{ffi, marker::PhantomData};

pub struct OPERATOR<const N: u8, G: Group>{
    _ptr: PhantomData<*const ()>,
    _group: PhantomData<G>
}

impl<const N: u8, G: Group> OPERATOR<N, G> {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this peripheralinstance, if it is already instantiated and used elsewhere
    #[inline(always)]
    pub unsafe fn new() -> Self {
        Self {
            _ptr: PhantomData,
            _group: PhantomData
        }
    }
}

unsafe impl<const N: u8, G: Group> Send for OPERATOR<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::sealed::Sealed for OPERATOR<N, G> {}

impl<const N: u8, G: Group> crate::peripheral::Peripheral for OPERATOR<N, G> {
    type P = Self;

    #[inline]
    unsafe fn clone_unchecked(&mut self) -> Self::P {
        Self { ..*self }
    }
}

// TODO: How do we want syncing to fit in to this?
// TODO: How do we want carrier to fit into this?
// TODO: How do we want capture to fit into this?

/// Motor Control operator abstraction
///
/// Every Motor Control module has three operators. Every operator can generate two output signals called A and B.
/// A and B share the same timer and thus frequency and phase but can have induvidual duty set.
pub struct Operator<const N: u8, G: Group, PA: OptionalOutputPin, PB: OptionalOutputPin> {
    _group: PhantomData<G>,
    handle: mcpwm_oper_handle_t,
    _instance: OPERATOR<N, G>,

    _pin_a: PA,
    _pin_b: PB,

    //deadtime: D
}

impl<const N: u8, G, PA, PB> Operator<N, G, PA, PB>
where
    G: Group,
    PA: OutputPin,
    PB: OptionalOutputPin,
{
    /// Get duty as percentage between 0.0 and 100.0 for output A
    pub fn get_duty_a(&self) -> Duty {
        todo!()
    }

    /// Set duty as percentage between 0.0 and 100.0 for output A
    pub fn set_duty_a(&mut self, duty: Duty) -> Result<(), EspError> {
        todo!()
    }
}

impl<const N: u8, G, PA, PB> Operator<N, G, PA, PB>
where
    G: Group,
    PA: OptionalOutputPin,
    PB: OutputPin,
{
    /// Get duty as percentage between 0.0 and 100.0 for output B
    pub fn get_duty_b(&self) -> Duty {
        todo!()
    }

    /// Set duty as percentage between 0.0 and 100.0 for output B
    pub fn set_duty_b(&mut self, duty: Duty) -> Result<(), EspError> {
        todo!()
    }
}

/// Operator configuration
pub struct OperatorConfig {
    duty_a: Duty,
    duty_b: Duty,

    duty_mode: DutyMode,

    //deadtime: Option<DeadtimeConfig>,
}

impl OperatorConfig {
    /// Set initual duty cycle for output A as percentage
    #[must_use]
    pub fn duty_a(mut self, duty: Duty) -> Self {
        self.duty_a = duty;
        self
    }

    /// Set initual duty cycle for output B as percentage
    #[must_use]
    pub fn duty_b(mut self, duty: Duty) -> Self {
        self.duty_b = duty;
        self
    }

    /// Specify what duty mode to use for the operator.
    #[must_use]
    pub fn duty_mode(mut self, duty_mode: DutyMode) -> Self {
        self.duty_mode = duty_mode;
        self
    }

    /*#[must_use]
    pub fn deadtime(mut self, deadtime: impl Into<Option<DeadtimeConfig>>) -> Self {
        self.deadtime = deadtime.into();
        self
    }*/
}

impl Default for OperatorConfig {
    fn default() -> Self {
        Self {
            duty_a: 0,
            duty_b: 0,

            duty_mode: DutyMode::ActiveHigh,

            //deadtime: None,
        }
    }
}

/// Duty modes for operator
#[derive(Clone, Copy, Debug)]
pub enum DutyMode {
    /// Active high
    ///
    /// Setting duty = 100% will result in a constant high output
    /// Setting duty =   0% will result in a constant low output
    ActiveHigh,

    /// Active low
    ///
    /// Setting duty = 100% will result in a constant low output
    /// Setting duty =   0% will result in a constant high output
    ActiveLow,
}

pub trait OptionalOperator<const N: u8, G: Group> {}
impl<const N: u8, G: Group> OptionalOperator<N, G> for OPERATOR<N, G> {}

pub struct NoOperator;
impl<const N: u8, G: Group> OptionalOperator<N, G> for NoOperator {}


/*

#[derive(Default, Clone)]
struct DutyConfig {
    on_matches_cmp_a: CountingDirection,
    on_matches_cmp_b: CountingDirection,
    on_is_empty: CountingDirection,
    on_is_full: CountingDirection
}

#[derive(Default, Clone, Copy)]
struct CountingDirection {
    counting_up: GeneratorAction,
    counting_down: GeneratorAction
}

enum GeneratorAction {
    Nothing,
    SetLow,
    SetHigh,
    Toggle
}

impl Default for GeneratorAction {
    fn default() -> Self {
        GeneratorAction::Nothing
    }
}

impl DutyMode {
    fn into_duty_cfg<G: Generator>(self) -> DutyConfig<G> {
        match val {
            DutyMode::ActiveHigh => {
                let mut duty_config: DutyConfig = Default::default();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetHigh;
                if G::IS_A {
                    duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetLow;
                } else {
                    duty_config.on_matches_cmp_b.counting_up = GeneratorAction::SetLow;
                }
                duty_config
            },
            DutyMode::ActiveLow => {
                let mut duty_config: DutyConfig = Default::default();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetLow;
                if G::IS_A {
                    duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetHigh;
                } else {
                    duty_config.on_matches_cmp_b.counting_up = GeneratorAction::SetHigh;
                }
                
                duty_config
            },
        }
    }
} */