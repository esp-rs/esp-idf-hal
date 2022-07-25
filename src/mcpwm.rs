//! Motor Control Pulse Width Modulator peripheral
//!
//! Interface to the [Motor Control Pulse Width Modulator peripheral (MCPWM)
//! peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)
//!
//! ```
//!   ---------------                     ---------------
//!  | MCPWM Unit 0  |                   | MCPWM Unit 1  |
//!  | ------------  |                   | ------------  |
//!  |               |                   |               |
//!  | OPERATOR  0   |--> A              | OPERATOR  0   |--> A
//!  |               |--> B              |               |--> B
//!  |               |                   |               |
//!  | OPERATOR  1   |--> A              | OPERATOR  1   |--> A
//!  |               |--> B              |               |--> B
//!  |               |                   |               |
//!  | OPERATOR  2   |--> A              | OPERATOR  2   |--> A
//!  |               |--> B              |               |--> B
//!   ---------------                     ---------------
//! ```
//!
//! Create a pair of PWM signals on pin 4 and 5. The duty on pin 4 will ramp from 0% to 100%
//! while pin 5 will ramp from 100% down to 0%.
//! ```
//! let peripherals = Peripherals::take().unwrap();
//! let config = OperatorConfig::default().frequency(25.kHz().into());
//! let mcpwm = Mcpwm::new(peripherals.mcpwm0.mcpwm)?;
//! let mut operator = Operator::new(
//!     peripherals.mcpwm0.operator0,
//!     &mcpwm,
//!     &config,
//!     peripherals.pins.gpio4,
//!     peripherals.pins.gpio5,
//! )?;
//!
//! println!("Starting duty-cycle loop");
//!
//! for &duty in [0.0, 20.0, 40.0, 60.0, 80.0, 100.0].iter() {
//!     println!("Duty {}%", duty);
//!     operator.set_duty_a(duty)?;
//!     operator.set_duty_b(100.0 - duty)?;
//!     FreeRtos.delay_ms(2000)?;
//! }
//! ```
//!
//! See the `examples/` folder of this repository for more.

use core::borrow::Borrow;

use crate::gpio::OutputPin;
use crate::units::{FromValueType, Hertz};
use esp_idf_sys::*;

// MCPWM clock source frequency for ESP32 and ESP32-s3
const MCPWM_CLOCK_SOURCE_FREQUENCY: u32 = 160_000_000;

// Max PWM timer prescaler
const MAX_PWM_TIMER_PRESCALE: u32 = 0x1_00;

// Max PWM timer period
const MAX_PWM_TIMER_PERIOD: u32 = 0x1_00_00;

/// The Motor Control Pulse Width Modulator peripheral
pub struct Peripheral<U: Unit> {
    pub mcpwm: MCPWM<U>,
    pub operator0: OPERATOR0<U>,
    pub operator1: OPERATOR1<U>,
    pub operator2: OPERATOR2<U>,
}

impl<U: Unit> Peripheral<U> {
    /// # Safety
    ///
    /// It is safe to instantiate this exactly one time per `Unit`.
    pub unsafe fn new() -> Self {
        Self {
            mcpwm: MCPWM::new(),
            operator0: OPERATOR0::new(),
            operator1: OPERATOR1::new(),
            operator2: OPERATOR2::new(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum DutyMode {
    ActiveHigh,
    ActiveLow,
}

impl From<DutyMode> for mcpwm_duty_type_t {
    fn from(val: DutyMode) -> Self {
        match val {
            DutyMode::ActiveHigh => mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
            DutyMode::ActiveLow => mcpwm_duty_type_t_MCPWM_DUTY_MODE_1,
        }
    }
}

// TODO: For UpDown, frequency is half of MCPWM frequency set
#[derive(Clone, Copy, Debug)]
pub enum CounterMode {
    #[cfg(not(esp_idf_version = "4.3"))]
    Frozen,
    Up,
    Down,
    UpDown,
}

impl From<CounterMode> for mcpwm_counter_type_t {
    fn from(val: CounterMode) -> Self {
        match val {
            #[cfg(not(esp_idf_version = "4.3"))]
            CounterMode::Frozen => mcpwm_counter_type_t_MCPWM_FREEZE_COUNTER,
            CounterMode::Up => mcpwm_counter_type_t_MCPWM_UP_COUNTER,
            CounterMode::Down => mcpwm_counter_type_t_MCPWM_DOWN_COUNTER,
            CounterMode::UpDown => mcpwm_counter_type_t_MCPWM_UP_DOWN_COUNTER,
        }
    }
}

/// Dead time config for MCPWM operator
///
/// `red` and `fed` is time as in number of clock cycles after the MCPWM modules group prescaler.
///
/// Note that the dead times are calculated from MCPWMXA's flanks unless explicitly stated otherwise
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum DeadtimeConfig {
    // TODO: Figure out what all of those options do and give them nice descriptions
    /// MCPWM_BYPASS_RED
    ///
    ///               .                    .   .
    ///               .                    .   .
    ///               .--------------------.   .
    ///               |                    |   .
    /// MCPWMXA in    |                    |   .
    ///               |                    |   .
    /// ---------------                    ---------------------
    ///               .                    .   .
    ///               .                    .   .
    ///               .--------------------.   .
    ///               |                    |   .
    /// MCPWMXA out   |                    |   .
    ///               |                    |   .
    /// ---------------                    ---------------------
    ///               .                    .   .
    ///               .                    .   .
    ///               .------------------------.
    ///               |                   >.   |< fed
    /// MCPWMXB out   |                    .   |
    ///               |                    .   |
    /// --------------.                    .   -----------------
    ///               .                    .   .
    BypassRisingEdge { fed: u16 },

    /// MCPWM_BYPASS_FED
    ///
    ///               .   .                .
    ///               .   .                .
    ///               .--------------------.
    ///               |   .                |
    /// MCPWMXA in    |   .                |
    ///               |   .                |
    /// ---------------   .                ---------------------
    ///               .   .                .
    ///               .   .                .
    ///               .   .----------------.
    ///          red >.   |<               |
    /// MCPWMXA out   .   |                |
    ///               .   |                |
    /// -------------------                ---------------------
    ///               .   .                .
    ///               .   .                .
    ///               .--------------------.
    ///               |   .                |
    /// MCPWMXB out   |   .                |
    ///               |   .                |
    /// ---------------   .                ---------------------
    ///               .   .                .
    BypassFallingEdge { red: u16 },

    /// MCPWM_ACTIVE_HIGH_MODE
    ///
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .----------------.   .
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    /// -------------------                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .------------------------.
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    /// --------------.   .                .   -----------------
    ///               .   .                .   .
    ActiveHigh { red: u16, fed: u16 },

    /// MCPWM_ACTIVE_LOW_MODE
    ///
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ------------------.                .--------------------
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    ///               .   ------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// --------------.   .                .   .----------------
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    ///               --------------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ActiveLow { red: u16, fed: u16 },

    /// MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
    ///
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .----------------.   .
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    /// -------------------                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// --------------.   .                .   .----------------
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    ///               --------------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ActiveHighComplement { red: u16, fed: u16 },

    /// MCPWM_ACTIVE_LOW_COMPLIMENT_MODE
    ///
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ------------------.                .--------------------
    ///          red >.   |<               |   .
    /// MCPWMXA out   .   |                |   .
    ///               .   |                |   .
    ///               .   ------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .------------------------.
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    /// ---------------   .                .   -----------------
    ///               .   .                .   .
    ///               .   .                .   .
    ActiveLowComplement { red: u16, fed: u16 },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXA
    ActiveRedFedFromPwmxa { red: u16, fed: u16 },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXB
    ActiveRedFedFromPwmxb { red: u16, fed: u16 },
}

struct DeadtimeArgs {
    red: u16,
    fed: u16,
    mode: mcpwm_deadtime_type_t,
}

impl DeadtimeConfig {
    fn as_args(&self) -> DeadtimeArgs {
        match *self {
            DeadtimeConfig::BypassRisingEdge { fed } => DeadtimeArgs {
                red: 0,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_RED,
            },

            DeadtimeConfig::BypassFallingEdge { red } => DeadtimeArgs {
                red,
                fed: 0,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_FED,
            },

            DeadtimeConfig::ActiveHigh { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_MODE,
            },

            DeadtimeConfig::ActiveLow { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_MODE,
            },

            DeadtimeConfig::ActiveHighComplement { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveLowComplement { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxa { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXA,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxb { red, fed } => DeadtimeArgs {
                red,
                fed,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXB,
            },
        }
    }
}

pub struct OperatorConfig {
    frequency: Hertz,
    duty_a: Duty,
    duty_b: Duty,

    #[cfg(not(esp_idf_version = "4.3"))]
    lowest_frequency: Hertz,
    duty_mode: DutyMode,
    counter_mode: CounterMode,

    deadtime: Option<DeadtimeConfig>,
}

impl OperatorConfig {
    #[must_use]
    pub fn frequency(mut self, frequency: Hertz) -> Self {
        self.frequency = frequency;
        self
    }

    #[cfg(not(esp_idf_version = "4.3"))]
    #[must_use]
    pub fn lowest_frequency(mut self, lowest_frequency: Hertz) -> Self {
        self.lowest_frequency = lowest_frequency;
        self
    }

    #[must_use]
    pub fn duty_mode(mut self, duty_mode: DutyMode) -> Self {
        self.duty_mode = duty_mode;
        self
    }

    #[must_use]
    pub fn counter_mode(mut self, counter_mode: CounterMode) -> Self {
        self.counter_mode = counter_mode;
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
            counter_mode: CounterMode::Up,

            deadtime: None,
        }
    }
}

#[derive(Default)]
pub struct UnitZero;

#[derive(Default)]
pub struct UnitOne;

pub type Duty = f32;

pub struct MCPWM<U: Unit> {
    _unit: U,
}

impl<U: Unit> MCPWM<U> {
    /// # Safety
    ///
    /// It is safe to instantiate this exactly one time per `Unit`.
    unsafe fn new() -> Self {
        Self {
            _unit: U::default(),
        }
    }
}

pub trait Unit: Default {
    fn unit() -> mcpwm_unit_t;
}

impl Unit for UnitZero {
    fn unit() -> mcpwm_unit_t {
        mcpwm_unit_t_MCPWM_UNIT_0
    }
}

impl Unit for UnitOne {
    fn unit() -> mcpwm_unit_t {
        mcpwm_unit_t_MCPWM_UNIT_1
    }
}

// TODO: How do we want fault module to fit into this?

pub struct Mcpwm<U: Unit> {
    #[cfg(not(esp_idf_version = "4.3"))]
    /// This is the frequency of the clock signal passed on as clock source for the operators timers
    /// Those timers in turn have their own prescalers to scale this down even further
    ///
    /// NOTE: This is only to be set by calling Self::lowest_frequency
    operator_source_frequency: u32,
    _instance: MCPWM<U>,
}

impl<U: Unit> Mcpwm<U> {
    pub fn new(instance: MCPWM<U>) -> Result<Self, EspError> {
        let res = Self {
            #[cfg(not(esp_idf_version = "4.3"))]
            operator_source_frequency: 0,
            _instance: instance,
        };

        #[cfg(not(esp_idf_version = "4.3"))]
        {
            // TODO: Do we want to make this into something more builder-pattern like to
            // avoid this potentially redundant function call?
            res.operator_source_frequency(10.MHz())
        }

        #[cfg(esp_idf_version = "4.3")]
        {
            Ok(res)
        }
    }

    pub fn release(self) -> MCPWM<U> {
        // TODO: Do we need to reset any state here such as group_prescaler?
        self._instance
    }

    /// Specify lowest reachable frequency
    ///
    /// The lower this is set, the lower frequencies will be reachable. However, this is at the cost of worse
    /// resolution at higher frequencies.
    ///
    /// Same thing goes for the other way. The higher value set here, the more resolution and so on.
    #[cfg(not(esp_idf_version = "4.3"))]
    pub fn lowest_frequency(mut self, lowest_frequency: Hertz) -> Result<Self, EspError> {
        // TODO: Do we care about frequency < 1Hz?
        let operator_source_frequency =
            MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * u32::from(lowest_frequency);
        let group_pre_scale = MCPWM_CLOCK_SOURCE_FREQUENCY / operator_source_frequency;
        if !(1..=256).contains(&group_pre_scale) {
            return Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap());
        }

        esp!(unsafe { mcpwm_group_set_resolution(U::unit(), operator_source_frequency) })?;
        self.operator_source_frequency = operator_source_frequency;

        Ok(self)
    }

    /// Specify frequency passed to operators timers as clock source
    ///
    /// The timers of the operators can then in turn scale this frequency down further.
    ///
    /// The lower this is set, the lower frequencies will be reachable. However, this is
    /// at the cost of worse resolution at higher frequencies. Same thing goes for the
    /// other way. The higher value set here, the more resolution and so on.
    #[cfg(not(esp_idf_version = "4.3"))]
    pub fn operator_source_frequency(
        mut self,
        frequency: impl Into<Hertz>,
    ) -> Result<Self, EspError> {
        let frequency: Hertz = frequency.into();
        let frequency: u32 = frequency.into();

        // TODO: Do we care about frequency < 1Hz?
        let group_pre_scale = MCPWM_CLOCK_SOURCE_FREQUENCY / frequency;
        if !(1..=256).contains(&group_pre_scale) {
            return Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap());
        }

        esp!(unsafe { mcpwm_group_set_resolution(U::unit(), frequency) })?;
        self.operator_source_frequency = frequency;

        Ok(self)
    }
}

// The hardware for ESP32 and ESP32-S3 can associate any operator(within the mcpwm module) with any
// timer(within the mcpwm module) for example allowing using the same timer for all three operators.
// However at least as of IDF v4.4 timer0 is hardcoded to operator0 and timer1 to operator1 and so on...
pub trait HwOperator<U: Unit> {
    fn timer() -> mcpwm_timer_t;
    fn signal_a() -> mcpwm_io_signals_t;
    fn signal_b() -> mcpwm_io_signals_t;
    fn unit() -> mcpwm_unit_t {
        U::unit()
    }
}

macro_rules! impl_operator_helper {
    ($instance:ident: $timer:expr, $signal_a:expr, $signal_b:expr, $unit:ty) => {
        impl HwOperator<$unit> for $instance<$unit> {
            fn timer() -> mcpwm_timer_t {
                $timer
            }

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
// TODO: How do we want dead time to fit into this?
// TODO: How do we want carrier to fit into this?
// TODO: How do we want capture to fit into this?

pub struct Operator<U: Unit, O: HwOperator<U>, M: Borrow<Mcpwm<U>>, PA: OutputPin, PB: OutputPin> {
    _instance: O,
    _unit: U,
    _mcpwm_module: M,

    _pin_a: Option<PA>,
    _pin_b: Option<PB>,
}

impl<U, O, M, PA, PB> Operator<U, O, M, PA, PB>
where
    U: Unit,
    O: HwOperator<U>,
    M: Borrow<Mcpwm<U>>,
    PA: OutputPin,
    PB: OutputPin,
{
    pub fn new<A: Into<Option<PA>>, B: Into<Option<PB>>>(
        operator: O,
        mcpwm_module: M,
        config: &OperatorConfig,
        pin_a: A,
        pin_b: B,
    ) -> Result<Self, EspError> {
        #[cfg(not(esp_idf_version = "4.3"))]
        {
            if config.frequency < config.lowest_frequency {
                // Can not specify a clock frequency lower then what has
                // been configured as the lowest clock frequency
                // Use `OperatorConfig::lowest_frequency` to enable lower frequencies
                return Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap());
            }

            if config.lowest_frequency > mcpwm_module.borrow().operator_source_frequency.Hz() {
                // Can not specify a lowest_frequency larger than the corresponding value for
                // the parent MCPWM module. Use `Mcpwm::lowest_frequency` to enable higher frequencies
                return Err(EspError::from(ESP_ERR_INVALID_ARG).unwrap());
            }

            let resolution = u32::from(config.lowest_frequency) * MAX_PWM_TIMER_PERIOD;
            unsafe {
                esp_idf_sys::mcpwm_timer_set_resolution(U::unit(), O::timer(), resolution);
            }
        }

        // TODO: Handle/document half pwm frequency when counter_mode = UpDown?

        esp!(unsafe {
            esp_idf_sys::mcpwm_init(
                U::unit(),
                O::timer(),
                &mcpwm_config_t {
                    frequency: config.frequency.into(),
                    cmpr_a: config.duty_a,
                    cmpr_b: config.duty_b,
                    duty_mode: config.duty_mode.into(),
                    counter_mode: config.counter_mode.into(),
                },
            )
        })?;

        match config.deadtime {
            None => unsafe {
                // Only way this can faild is if an invalid timer or unit is specified
                // which we know can not happen. So we don't have to check for errors
                mcpwm_deadtime_disable(U::unit(), O::timer());
            },
            Some(config) => {
                let DeadtimeArgs { red, fed, mode } = config.as_args();
                unsafe {
                    mcpwm_deadtime_enable(U::unit(), O::timer(), mode, red.into(), fed.into());
                }
            }
        }

        let pin_a: Option<PA> = pin_a.into();
        let pin_b: Option<PB> = pin_b.into();

        if let Some(pin_a) = &pin_a {
            let io_signal = O::signal_a();
            esp!(unsafe { esp_idf_sys::mcpwm_gpio_init(U::unit(), io_signal, pin_a.pin()) })?;
        }

        if let Some(pin_b) = &pin_b {
            let io_signal = O::signal_b();
            esp!(unsafe { esp_idf_sys::mcpwm_gpio_init(U::unit(), io_signal, pin_b.pin()) })?;
        }

        Ok(Self {
            _instance: operator,
            _unit: U::default(),
            _mcpwm_module: mcpwm_module,
            _pin_a: pin_a,
            _pin_b: pin_b,
        })
    }

    pub fn release(self) -> (O, Option<PA>, Option<PB>) {
        // mcpwm_stop will only fail when invalid args are given
        esp!(unsafe { mcpwm_stop(U::unit(), O::timer()) }).unwrap();

        // TODO: Test and verify if this is the right way
        if self._pin_a.is_some() {
            // TODO: How to unset pin?
            // let io_signal = O::signal_a();
            // mcpwm_gpio_init(U::unit(), io_signal, -1)
            // does not seem to be it...
            todo!();
        }

        if self._pin_b.is_some() {
            // TODO: How to unset pin?
            // let io_signal = O::signal_b();
            // mcpwm_gpio_init(U::unit(), io_signal, -1)
            // does not seem to be it...

            todo!();
        }
        // TODO: Do we need to reset any more state here such as dead time config?
        (self._instance, self._pin_a, self._pin_b)
    }

    /// Get duty as percentage between 0.0 and 100.0
    pub fn get_duty_a(&self) -> Duty {
        unsafe {
            esp_idf_sys::mcpwm_get_duty(
                U::unit(),
                O::timer(),
                esp_idf_sys::mcpwm_generator_t_MCPWM_GEN_A,
            )
        }
    }

    /// Get duty as percentage between 0.0 and 100.0
    pub fn get_duty_b(&self) -> Duty {
        unsafe {
            esp_idf_sys::mcpwm_get_duty(
                U::unit(),
                O::timer(),
                esp_idf_sys::mcpwm_generator_t_MCPWM_GEN_B,
            )
        }
    }

    /// Set duty as percentage between 0.0 and 100.0
    pub fn set_duty_a(&mut self, duty: Duty) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::mcpwm_set_duty(
                U::unit(),
                O::timer(),
                esp_idf_sys::mcpwm_generator_t_MCPWM_GEN_A,
                duty
            ))
        }
    }

    /// Set duty as percentage between 0.0 and 100.0
    pub fn set_duty_b(&mut self, duty: Duty) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::mcpwm_set_duty(
                U::unit(),
                O::timer(),
                esp_idf_sys::mcpwm_generator_t_MCPWM_GEN_B,
                duty
            ))
        }
    }

    /// Set PWM frequency
    pub fn set_frequency(frequency: Hertz) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::mcpwm_set_frequency(
                U::unit(),
                O::timer(),
                frequency.into()
            ))
        }
    }

    pub fn get_frequency() -> Hertz {
        Hertz::from(unsafe { esp_idf_sys::mcpwm_get_frequency(U::unit(), O::timer()) })
    }
}
