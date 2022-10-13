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
//! let timer_config = TimerConfig::default().frequency(25.kHz());
//! let operator_config = OperatorConfig::default();
//! let timer = Mcpwm::new(peripherals.mcpwm0.timer, timer_config)?;
//! 
//! let timer = timer.into_connection()
//!     .attatch_operator0(
//!         peripherals.mcpwm0.operator0,
//!         operator_config,
//!         peripherals.pins.gpio4,
//!         peripherals.pins.gpio5,
//!     )?;
//! 
//! let (timer, operator, _, _) = timer.split();
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

mod timer;
mod operator;
mod timer_connection;

use core::ffi;

pub use self::{
    operator::{
        OPERATOR,
        Operator,
        OperatorConfig,
    },
    timer::{
        TIMER,
        Timer,
        TimerConfig
    },
    timer_connection::{
        TimerConnection
    }
};

// MCPWM clock source frequency for ESP32 and ESP32-s3
const MCPWM_CLOCK_SOURCE_FREQUENCY: u32 = 160_000_000;

// Max PWM timer prescaler
const MAX_PWM_TIMER_PRESCALE: u32 = 0x1_00;

// Max PWM timer period
const MAX_PWM_TIMER_PERIOD: u32 = 0x1_00_00;

/// The Motor Control Pulse Width Modulator peripheral
pub struct MCPWM<G: Group> {
    pub timer0: TIMER<0, G>,
    pub timer1: TIMER<1, G>,
    pub timer2: TIMER<2, G>,

    pub operator0: OPERATOR<0, G>,
    pub operator1: OPERATOR<1, G>,
    pub operator2: OPERATOR<2, G>,
}

impl<G: Group> MCPWM<G> {
    /// # Safety
    ///
    /// It is safe to instantiate this exactly one time per `Group`.
    pub unsafe fn new() -> Self {
        Self {
            timer0: TIMER::new(),
            timer1: TIMER::new(),
            timer2: TIMER::new(),
            operator0: OPERATOR::new(),
            operator1: OPERATOR::new(),
            operator2: OPERATOR::new(),
        }
    }
}

#[derive(Default)]
pub struct Group0;

#[derive(Default)]
pub struct Group1;

pub type Duty = u16;

// Note this was called `Unit` in IDF < 5.0
pub trait Group: Default {
    const ID: ffi::c_int;
}

impl Group for Group0 {
    const ID: ffi::c_int = 0;
}

impl Group for Group1 {
    const ID: ffi::c_int = 1;
}