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
//! # Example
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

impl From<DutyMode> for mcpwm_duty_type_t {
    fn from(val: DutyMode) -> Self {
        match val {
            DutyMode::ActiveHigh => mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
            DutyMode::ActiveLow => mcpwm_duty_type_t_MCPWM_DUTY_MODE_1,
        }
    }
}

/// Counter mode for operator's timer for generating PWM signal
// TODO: For UpDown, frequency is half of MCPWM frequency set
#[derive(Clone, Copy, Debug)]
pub enum CounterMode {
    /// Timer is frozen or paused
    #[cfg(not(esp_idf_version = "4.3"))]
    Frozen,
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
/// `rising_edge_delay` and `falling_edge_delay` is time as in number of clock cycles after the MCPWM modules group prescaler.
///
/// Note that the dead times are calculated from MCPWMXA's flanks unless explicitly stated otherwise
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DeadtimeConfig {
    // TODO: Figure out what all of those options do and give them nice descriptions
    /// MCPWM_BYPASS_RED
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    ///               .                    .   .
    /// ```
    BypassRisingEdge { falling_edge_delay: u16 },

    /// MCPWM_BYPASS_FED
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    ///               .   .                .
    /// ```
    BypassFallingEdge { rising_edge_delay: u16 },

    /// MCPWM_ACTIVE_HIGH_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    ///               .   .                .   .
    /// ```
    ActiveHigh {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_LOW_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    /// ```
    ActiveLow {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    /// ```
    ActiveHighComplement {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_LOW_COMPLIMENT_MODE
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
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
    ///               .   ------------------   .
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .------------------------.
    ///               |   .               >.   |< fed
    /// MCPWMXB out   |   .                .   |
    ///               |   .                .   |
    /// ---------------   .                .   -----------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveLowComplement {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXA
    ///
    /// Note that `MCPWMXB in` will be completely ignored. This means `Operator::set_duty_b` will
    /// have no effect with this dead time mode
    ///
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXA in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXA out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXB out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveRedFedFromPwmxa {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },

    /// MCPWM_ACTIVE_RED_FED_FROM_PWMXB
    ///
    /// Note that `MCPWMXA in` will be completely ignored. This means `Operator::set_duty_a` will
    /// have no effect with this dead time mode
    /// ```
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .--------------------.   .
    ///               |   .                |   .
    /// MCPWMXB in    |   .                |   .
    ///               |   .                |   .
    /// ---------------   .                ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXA out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    ///               .   .--------------------.
    ///          red >.   |<               .   |
    /// MCPWMXB out   .   |                .   |
    ///               .   |                .   |
    /// -------------------                .   ---------------------
    ///               .   .                .   .
    ///               .   .                .   .
    /// ```
    ActiveRedFedFromPwmxb {
        rising_edge_delay: u16,
        falling_edge_delay: u16,
    },
}

impl DeadtimeConfig {
    fn as_args(&self) -> DeadtimeArgs {
        match *self {
            DeadtimeConfig::BypassRisingEdge { falling_edge_delay } => DeadtimeArgs {
                rising_edge_delay: 0,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_RED,
            },

            DeadtimeConfig::BypassFallingEdge { rising_edge_delay } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay: 0,
                mode: mcpwm_deadtime_type_t_MCPWM_BYPASS_FED,
            },

            DeadtimeConfig::ActiveHigh {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_MODE,
            },

            DeadtimeConfig::ActiveLow {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_MODE,
            },

            DeadtimeConfig::ActiveHighComplement {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveLowComplement {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_LOW_COMPLIMENT_MODE,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxa {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXA,
            },

            DeadtimeConfig::ActiveRedFedFromPwmxb {
                rising_edge_delay,
                falling_edge_delay,
            } => DeadtimeArgs {
                rising_edge_delay,
                falling_edge_delay,
                mode: mcpwm_deadtime_type_t_MCPWM_ACTIVE_RED_FED_FROM_PWMXB,
            },
        }
    }
}

struct DeadtimeArgs {
    rising_edge_delay: u16,
    falling_edge_delay: u16,
    mode: mcpwm_deadtime_type_t,
}

/// Operator configuration
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
    /// Frequency which the operator will run at, can also be changed live later
    #[must_use]
    pub fn frequency(mut self, frequency: impl Into<Hertz>) -> Self {
        self.frequency = frequency.into();
        self
    }

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

    /// Lowest frequency which the operator needs to be able to reach
    ///
    /// This setting will limit what frequency range the operator can reach.
    /// Setting a low value here will lead to worse resolution at higher
    /// frequencies. A high value on the other hand will prevent any frequencies
    /// lower than that.
    ///
    /// Thus, for maximum resolution set lowest_frequency to the lowest expected
    /// frequency which will be used.
    ///
    /// NOTE: This value can currently not be changed live
    #[cfg(not(esp_idf_version = "4.3"))]
    #[must_use]
    pub fn lowest_frequency(mut self, lowest_frequency: impl Into<Hertz>) -> Self {
        self.lowest_frequency = lowest_frequency.into();
        self
    }

    /// Specify what duty mode to use for the operator.
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

struct Timer<U: Unit, T: HwTimer> {

}

impl Timer {
    /// Set PWM frequency
    pub fn set_frequency(&mut self, frequency: Hertz) -> Result<(), EspError> {
        todo!()
    }

    /// Get PWM frequency
    pub fn get_frequency(&self) -> Hertz {
        todo!()
    }

    pub fn timer(&self) -> mcpwm_timer_t {
        T::timer()
    }
}

trait OptionalOperator<U: Unit, O: HwOperator<U>> {

}
struct NoOperator;
impl<U: Unit, O: HwOperator> OptionalOperator<U, O> for NoOperator {}

// TODO: How do we want fault module to fit into this?
/// Motor Control PWM module abstraction
pub struct TimerConnection<U: Unit, T: HwTimer<U>, O0, O1, O2>
    where
        O0: OptionalOperator<U, OPERATOR0>,
        O1: OptionalOperator<U, OPERATOR1>,
        O2: OptionalOperator<U, OPERATOR2>,
{
    timer: Timer<T>,
    operator0: O0<U, OPERATOR0>,
    operator1: O1<U, OPERATOR1>,
    operator2: O2<U, OPERATOR2>
}

// Since there can only ever by one instance of every operator type (except NoOperator)
// we know that there can be no mem::swap or similar to cause any problems.
//
// Thus we know that after split is called nothing can be added/removed while still having access to
// the individual objects. We also garantuee that the operators wont live longer than the timer
impl<U, T, O0: OptionalOperator<U, OPERATOR0>, O1: OptionalOperator<U, OPERATOR1>, O2: OptionalOperator<U, OPERATOR2>> TimerConnection<U, T, O0, O1, O2> {
    fn split(&mut self) -> (&mut timer, &mut O0, &mut O1, &mut O2) {
        (
            &mut self.timer,
            &mut self.operator0,
            &mut self.operator1,
            &mut self.operator2,
        )
    }
}

impl<U, T> TimerConnection<U, T, NoOperator, NoOperator, NoOperator> {
    fn new(timer: T) -> Self {
        Self {
            timer,
            operator0: NoOperator,
            operator1: NoOperator,
            operator2: NoOperator
        }
    }
}
impl<U, T, O1, O2> TimerConnection<U, T, NoOperator, O1, O2> {
    fn attatch_operator0<O: OperatorConfig<U, T>>(mut self, operator_cfg: O) -> TimerConnection<U, T, O, O1, O2> {
        let operator = self.init_and_attach_operator(operator_cfg);
        TimerConnection {
            timer: self.timer,
            operator0: operator,
            operator1: self.operator1,
            operator2: self.operator2
        }
    }
}

impl<U, T, O0, O2> TimerConnection<U, T, O0, NoOperator, O2> {
    fn attatch_operator1<O: OperatorConfig<U, T>>(mut self, operator: O) -> TimerConnection<U, T, O0, O, O2> {
        let operator = self.init_and_attach_operator(operator_cfg);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: operator,
            operator2: self.operator2
        }
    }
}

impl<U, T, O0, O1> TimerConnection<U, T, O0, O1, NoOperator> {
    fn attatch_operator2<O: OperatorConfig<U, T>>(mut self, operator: O) -> TimerConnection<U, T, O0, O1, O> {
        let operator = self.init_and_attach_operator(operator_cfg);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator
        }
    }
}

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
