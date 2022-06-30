//! Motor Control Pulse Width Modulator peripheral
//!
//! Interface to the [Motor Control Pulse Width Modulator peripheral (MCPWM)
//! peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)
//!
//! This is an initial implementation supporting !::!:!:!::!:!:!:!
//! TODO: write stuff here
//!
//! TODO: Come up with nice example
//! # Examples
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
use esp_idf_sys::{esp, mcpwm_config_t, EspError};
use esp_idf_sys::{
    mcpwm_counter_type_t,
    mcpwm_counter_type_t_MCPWM_DOWN_COUNTER,
    //mcpwm_counter_type_t_MCPWM_FREEZE_COUNTER,
    mcpwm_counter_type_t_MCPWM_UP_COUNTER,
    mcpwm_counter_type_t_MCPWM_UP_DOWN_COUNTER,
};
use esp_idf_sys::{
    mcpwm_duty_type_t, mcpwm_duty_type_t_MCPWM_DUTY_MODE_0, mcpwm_duty_type_t_MCPWM_DUTY_MODE_1,
};
use esp_idf_sys::{
    mcpwm_io_signals_t, mcpwm_io_signals_t_MCPWM0A, mcpwm_io_signals_t_MCPWM0B,
    mcpwm_io_signals_t_MCPWM1A, mcpwm_io_signals_t_MCPWM1B, mcpwm_io_signals_t_MCPWM2A,
    mcpwm_io_signals_t_MCPWM2B,
};
use esp_idf_sys::{
    mcpwm_timer_t, mcpwm_timer_t_MCPWM_TIMER_0, mcpwm_timer_t_MCPWM_TIMER_1,
    mcpwm_timer_t_MCPWM_TIMER_2,
};
use esp_idf_sys::{mcpwm_unit_t, mcpwm_unit_t_MCPWM_UNIT_0, mcpwm_unit_t_MCPWM_UNIT_1};

/// The Motor Control Pulse Width Modulator peripheral
pub struct Peripheral<U: Unit> {
    pub mcpwm: MCPWM<U>, // TODO: Is there a better way to name or structure this?
    pub operator0: OPERATOR0<U>,
    pub operator1: OPERATOR1<U>,
    pub operator2: OPERATOR2<U>,
}

impl<U: Unit> Peripheral<U> {
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

impl Into<mcpwm_duty_type_t> for DutyMode {
    fn into(self) -> mcpwm_duty_type_t {
        match self {
            DutyMode::ActiveHigh => mcpwm_duty_type_t_MCPWM_DUTY_MODE_0,
            DutyMode::ActiveLow => mcpwm_duty_type_t_MCPWM_DUTY_MODE_1,
        }
    }
}

// TODO: For UpDown, frequency is half of MCPWM frequency set
#[derive(Clone, Copy, Debug)]
pub enum CounterMode {
    //Frozen,
    Up,
    Down,
    UpDown,
}

impl Into<mcpwm_counter_type_t> for CounterMode {
    fn into(self) -> mcpwm_counter_type_t {
        match self {
            // TODO: This seems to be new to IDF 4.4?
            //CounterMode::Frozen => mcpwm_counter_type_t_MCPWM_FREEZE_COUNTER,
            CounterMode::Up => mcpwm_counter_type_t_MCPWM_UP_COUNTER,
            CounterMode::Down => mcpwm_counter_type_t_MCPWM_DOWN_COUNTER,
            CounterMode::UpDown => mcpwm_counter_type_t_MCPWM_UP_DOWN_COUNTER,
        }
    }
}

pub struct OperatorConfig {
    frequency: Hertz,
    duty_a: Duty,
    duty_b: Duty,

    #[cfg(idf_newer_than_or_equal_v4_4_0)]
    lowest_frequency: Hertz,
    duty_mode: DutyMode,
    counter_mode: CounterMode,
    //deadtime: DeadtimeConfig,
}

impl OperatorConfig {
    #[must_use]
    pub fn frequency(mut self, frequency: Hertz) -> Self {
        self.frequency = frequency;
        self
    }

    #[cfg(idf_newer_than_or_equal_v4_4_0)]
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

    /*
    #[must_use]
    pub fn deadtime_config(mut self, deadtime_config: DeadtimeConfig) -> Self {
        todo!()
    }*/
}

impl Default for OperatorConfig {
    fn default() -> Self {
        Self {
            frequency: 1000.Hz(),
            duty_a: 50.0,
            duty_b: 50.0,

            #[cfg(idf_newer_than_or_equal_v4_4_0)]
            lowest_frequency: 1.Hz(),

            duty_mode: DutyMode::ActiveHigh,
            counter_mode: CounterMode::Up,
        }
    }
}

#[derive(Default)]
pub struct UnitZero;

#[derive(Default)]
pub struct UnitOne;

pub type Duty = f32;
const MAX_DUTY: Duty = 100.0;

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
// TODO: How do we want capture module to fit into this?

pub struct Mcpwm<U: Unit> {
    _instance: MCPWM<U>,
}

impl<U: Unit> Mcpwm<U> {
    pub fn new(instance: MCPWM<U>) -> Result<Self, EspError> {
        Ok(Self {
            _instance: instance,
        })
    }

    // TODO: I do not at all understand the motivation behind exposing the group prescaler as
    // "resolution"? From what i can see, duty is exposed as a percentage so we are not talking
    // about the pulse length in terms of cycles as is the case for ledc, right? What am I missing?
    // Anyway, my reasoning:
    // * High res  => small prescaler  => better maintan accuracy at high frequency, unable to reach lower freq
    // * Lower res => larger prescaler => worse accuracy at high frequency(still as good at low frequency),
    //   able to reach lower freq
    // Would it make more sense to expose it as "lowest reachable frequency"
    // Also why are there no checks other than prescaler >= 1 in the idf?
    // What happens if prescaler register value does not fit into the least significant 8 bits?
    #[cfg(idf_newer_than_or_equal_v4_4_0)]
    pub fn lowest_frequency(mut self, lowest_frequency: Hertz) -> Result<Self, EspError> {
        // MCPWM clock source frequency for ESP32 and ESP32-s3
        const MCPWM_CLOCK_SOURCE_FREQUENCY: u32 = 160_000_000;
        // Max PWM timer prescaler
        const MAX_PWM_TIMER_PRESCALE: u32 = 0x1_00;
        // Max PWM timer period
        const MAX_PWM_TIMER_PERIOD: u32 = 0x1_00_00;

        // let lowest_frequency = MCPWM_CLOCK_SOURCE_FREQUENCY / group_prescaler_factor / MAX_PWM_TIMER_PRESCALE / MAX_PWM_TIMER_PERIOD;
        // let MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * frequency = MCPWM_CLOCK_SOURCE_FREQUENCY / group_prescaler_factor;
        // let group_prescaler_factor = MCPWM_CLOCK_SOURCE_FREQUENCY / (MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * frequency);

        // let resolution = MCPWM_CLOCK_SOURCE_FREQUENCY / group_prescaler;

        // let resolution = MCPWM_CLOCK_SOURCE_FREQUENCY / (MCPWM_CLOCK_SOURCE_FREQUENCY / (MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * frequency));
        // let resolution = (MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * frequency) * 160_000_000 / MCPWM_CLOCK_SOURCE_FREQUENCY;
        let resolution = MAX_PWM_TIMER_PRESCALE * MAX_PWM_TIMER_PERIOD * lowest_reachable_frequency;

        mcpwm_group_set_resolution();
        todo!()
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
            /// It is safe to instantiate this operator exactly one time.
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
// TODO: How do we want deadtime to fit into this?
// TODO: How do we want carrier to fit into this?

pub struct Operator<U: Unit, O: HwOperator<U>, M: Borrow<Mcpwm<U>>, PA: OutputPin, PB: OutputPin> {
    _instance: O,
    _unit: U,
    _mcpwm_module: M,

    _pin_a: Option<PA>,
    _pin_b: Option<PB>,
}

impl<U: Unit, O: HwOperator<U>, M: Borrow<Mcpwm<U>>, PA: OutputPin, PB: OutputPin>
    Operator<U, O, M, PA, PB>
{
    pub fn new<A: Into<Option<PA>>, B: Into<Option<PB>>>(
        operator: O,
        mcpwm_module: M,
        config: &OperatorConfig,
        pin_a: A,
        pin_b: B,
    ) -> Result<Self, EspError> {
        // TODO: Dont forget to make these cases work
        #[cfg(idf_newer_than_or_equal_v4_4_0)]
        {
            if config.frequency < config.lowest_frequency {
                return Err(panic!("TODO: Invalid parameter, should this be checked in OperatorConfig or here or hope that the IDF? will handle the error checking"));
            }
            unsafe {
                esp_idf_sys::mcpwm_timer_set_resolution();
            }
        }

        // TODO: Handle half pwm frequency when counter_mode = UpDown here?

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

    pub fn get_duty_a(&self) -> Duty {
        unsafe {
            esp_idf_sys::mcpwm_get_duty(
                U::unit(),
                O::timer(),
                esp_idf_sys::mcpwm_generator_t_MCPWM_GEN_A,
            )
        }
    }

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

    pub fn set_frequency(frequency: u32) -> Result<(), EspError> {
        unsafe {
            esp!(esp_idf_sys::mcpwm_set_frequency(
                U::unit(),
                O::timer(),
                frequency
            ))
        }
    }
    pub fn get_frequency() -> u32 {
        unsafe { esp_idf_sys::mcpwm_get_frequency(U::unit(), O::timer()) }
    }
}

pub enum Generator {
    A,
    B,
}

// TODO: would we like to also implement PwmPin for something like Foo<&Operator, G: Generator>
// this would allow things like:
// let operator = Operator::new(...);
// let gen_a = operator.generator_a_as_pwm_pin();
// let gen_b = operator.generator_b_as_pwm_pin();
// function_expecting_pwm_pin(&gen_a);
// gen_a.set_duty()
// gen_a.set_duty()
impl<U: Unit, O: HwOperator<U>, M: Borrow<Mcpwm<U>>, PA: OutputPin, PB: OutputPin>
    embedded_hal_0_2::Pwm for Operator<U, O, M, PA, PB>
{
    type Channel = Generator;
    type Duty = Duty;
    type Time = ();

    fn disable(&mut self, _channel: Self::Channel) {
        todo!()
    }

    fn enable(&mut self, _channel: Self::Channel) {
        todo!()
    }

    fn get_period(&self) -> Self::Time {
        todo!()
    }

    fn get_duty(&self, channel: Self::Channel) -> Self::Duty {
        match channel {
            Generator::A => self.get_duty_a(),
            Generator::B => self.get_duty_b(),
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        MAX_DUTY
    }

    fn set_duty(&mut self, channel: Self::Channel, duty: Self::Duty) {
        match channel {
            Generator::A => self
                .set_duty_a(duty)
                .expect("Failed to set duty for generator A"),
            Generator::B => self
                .set_duty_b(duty)
                .expect("Failed to set duty for generator B"),
        }
    }

    fn set_period<P>(&mut self, _period: P) {
        todo!()
    }
}
