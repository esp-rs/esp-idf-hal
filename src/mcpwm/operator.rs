// The hardware for ESP32 and ESP32-S3 can associate any operator(within the mcpwm module) with any
// timer(within the mcpwm module) for example allowing using the same timer for all three operators.
// However at least as of IDF v4.4 timer0 is hardcoded to operator0 and timer1 to operator1 and so on...
pub trait HwOperator<U: Unit>: Into<Operator<U, Self>> {
    fn signal_a() -> mcpwm_io_signals_t;
    fn signal_b() -> mcpwm_io_signals_t;
    fn unit() -> mcpwm_unit_t {
        U::unit()
    }
}

macro_rules! impl_operator_helper {
    ($instance:ident: $timer:expr, $signal_a:expr, $signal_b:expr, $unit:ty) => {
        impl HwOperator<$unit> for $instance<$unit> {
            fn signal_a() -> mcpwm_io_signals_t {
                $signal_a
            }

            fn signal_b() -> mcpwm_io_signals_t {
                $signal_b
            }
        }
    };
}

macro_rules! impl_operator {
    ($instance:ident: $timer:expr, $signal_a:expr, $signal_b:expr) => {
        pub struct $instance<U: Unit> {
            _unit: U,
        }

        impl<U: Unit> $instance<U> {
            /// # Safety
            ///
            /// It is safe to instantiate this operator exactly one time per Unit.
            pub unsafe fn new() -> Self {
                $instance {
                    _unit: U::default(),
                }
            }
        }

        impl<U: Unit> Into<Operator<U>> for $instance<U> {
            fn into(self) -> Operator<U> {
                Operator {
                    _instance: self,
                    pin_a: NoPin,
                    pin_b: NoPin,
                }
            }
        }

        impl_operator_helper!($instance: $timer, $signal_a, $signal_b, UnitZero);
        impl_operator_helper!($instance: $timer, $signal_a, $signal_b, UnitOne);
    };
}

impl_operator!(
    OPERATOR0: mcpwm_timer_t_MCPWM_TIMER_0,
    mcpwm_io_signals_t_MCPWM0A,
    mcpwm_io_signals_t_MCPWM0B
);
impl_operator!(
    OPERATOR1: mcpwm_timer_t_MCPWM_TIMER_1,
    mcpwm_io_signals_t_MCPWM1A,
    mcpwm_io_signals_t_MCPWM1B
);
impl_operator!(
    OPERATOR2: mcpwm_timer_t_MCPWM_TIMER_2,
    mcpwm_io_signals_t_MCPWM2A,
    mcpwm_io_signals_t_MCPWM2B
);

// TODO: How do we want syncing to fit in to this?
// TODO: How do we want carrier to fit into this?
// TODO: How do we want capture to fit into this?

/// Motor Control operator abstraction
///
/// Every Motor Control module has three operators. Every operator can generate two output signals called A and B.
/// A and B share the same timer and thus frequency and phase but can have induvidual duty set.
pub struct Operator<U: Unit, O: HwOperator<U>, M: Borrow<Mcpwm<U>>, PA: OptionalPin, PB: OptionalPin, D> {
    handle: mcpwm_operator_t,
    _instance: O,

    _pin_a: PA,
    _pin_b: PB,

    deadtime: D
}

impl<U, O, M, PA, PB> Operator<U, O, M, PA, PB>
where
    U: Unit,
    O: HwOperator<U>,
    M: Borrow<Mcpwm<U>>,
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

impl<U, O, M, PA, PB> Operator<U, O, M, PA, PB>
where
    U: Unit,
    O: HwOperator<U>,
    M: Borrow<Mcpwm<U>>,
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

    deadtime: Option<DeadtimeConfig>,
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

    #[must_use]
    pub fn deadtime(mut self, deadtime: impl Into<Option<DeadtimeConfig>>) -> Self {
        self.deadtime = deadtime.into();
        self
    }
}

impl Default for OperatorConfig {
    fn default() -> Self {
        Self {
            frequency: 1000.Hz(),
            duty_a: 50.0,
            duty_b: 50.0,

            #[cfg(not(esp_idf_version = "4.3"))]
            lowest_frequency: 16.Hz(),

            duty_mode: DutyMode::ActiveHigh,

            deadtime: None,
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

impl From<DutyMode> for DutyConfig {
    fn from(val: DutyMode) -> Self {
        match val {
            DutyMode::ActiveHigh => {
                let mut duty_config: DutyConfig = Default::default();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetHigh;
                duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetLow;
                
                duty_config
            },
            DutyMode::ActiveLow => {
                let mut duty_config: DutyConfig = Default::default();
                duty_config.on_is_empty.counting_up = GeneratorAction::SetLow;
                duty_config.on_matches_cmp_a.counting_up = GeneratorAction::SetHigh;
                
                duty_config
            },
        }
    }
}