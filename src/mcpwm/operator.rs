use esp_idf_sys::{EspError, mcpwm_oper_handle_t};

use crate::{mcpwm::{Group, Group0, Group1}, gpio::OutputPin};

use super::{Duty, timer_connection::OptionalOutputPin};

use core::ffi;

// The hardware for ESP32 and ESP32-S3 can associate any operator(within the mcpwm module) with any
// timer(within the mcpwm module) for example allowing using the same timer for all three operators.
pub trait HwOperator<U: Group> {
    const GROUP_ID: ffi::c_int = U::ID;
}

macro_rules! impl_operator {
    ($t:ident, $g:ty) => {
        crate::impl_peripheral!($t);

        impl HwOperator<$g> for $t {}
    };
}

pub trait HwOperator0<G: Group>: HwOperator<G> {}
pub trait HwOperator1<G: Group>: HwOperator<G> {}
pub trait HwOperator2<G: Group>: HwOperator<G> {}

// Group 0
impl_operator!(OPERATOR00, Group0);
impl_operator!(OPERATOR01, Group0);
impl_operator!(OPERATOR02, Group0);

// Group 1
impl_operator!(OPERATOR10, Group1);
impl_operator!(OPERATOR11, Group1);
impl_operator!(OPERATOR12, Group1);

impl HwOperator0<Group0> for OPERATOR00 {}
impl HwOperator0<Group1> for OPERATOR10 {}

impl HwOperator1<Group0> for OPERATOR01 {}
impl HwOperator1<Group1> for OPERATOR11 {}

impl HwOperator2<Group0> for OPERATOR02 {}
impl HwOperator2<Group1> for OPERATOR12 {}

// TODO: How do we want syncing to fit in to this?
// TODO: How do we want carrier to fit into this?
// TODO: How do we want capture to fit into this?

/// Motor Control operator abstraction
///
/// Every Motor Control module has three operators. Every operator can generate two output signals called A and B.
/// A and B share the same timer and thus frequency and phase but can have induvidual duty set.
pub struct Operator<U: Group, O: HwOperator<U>, PA: OptionalOutputPin, PB: OptionalOutputPin> {
    handle: mcpwm_oper_handle_t,
    _instance: O,

    _pin_a: PA,
    _pin_b: PB,

    //deadtime: D
}

impl<U, O, PA, PB> Operator<U, O, PA, PB>
where
    U: Group,
    O: HwOperator<U>,
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

impl<U, O, PA, PB> Operator<U, O, PA, PB>
where
    U: Group,
    O: HwOperator<U>,
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
            duty_a: 0.0,
            duty_b: 0.0,

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

pub trait OptionalOperator<U: Group, O: HwOperator<U>> {}

pub struct NoOperator;
impl<U: Group, O: HwOperator<U>> OptionalOperator<U, O> for NoOperator {}

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