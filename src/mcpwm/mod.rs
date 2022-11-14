//! Motor Control Pulse Width Modulator peripheral
//!
//! Interface to the [Motor Control Pulse Width Modulator peripheral (MCPWM)
//! peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)
//!
//! ```
//!   --------------------------------------------------------------------------
//!  |                             MCPWM Group N                                |
//!  |                       -------------------------                          |
//!  |                                                                          |
//!  |  ---------            -------------------------------------------------  |
//!  | | Timer 0 |-------*  |              OPERATOR  0                        | |
//!  |  ---------        |  |            ---------------                      | |
//!  |  ---------        |  |                                  -------------  | |
//!  | | Timer 1 |----*  |  |        *----------------------->|             | | |
//!  |  ---------     |  |  |        |                *------>| GENERATOR 0 |-|-|--> To Output pin
//!  |  ---------     |  *--|-|\     |                | *---->|             | | |
//!  | | Timer 2 |-*  |  |  | |  \   *->Comparator 0>-* |      -------------  | |
//!  |  ---------  |  *-----|-|   |>-|                | |                     | |
//!  |             |  |  |  | |  /   *->Comparator 1>-|-*      -------------  | |
//!  |             *--------|-|/     |                *-|---->|             | |-|--> To Output pin
//!  |             |  |  |  |        |                  *---->| GENERATOR 1 | | |
//!  |             |  |  |  |        *----------------------->|             | | |
//!  |             |  |  |  |                                  -------------  | |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  |                                                      |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  *--|              OPERATOR  1                        | |
//!  |             |  |  |  |            ---------------                      |-|--> To Output pin
//!  |             |  *-----|                                                 | |
//!  |             |  |  |  |                  ...                            |-|--> To Output pin
//!  |             *--------|                                                 | |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  |                                                      |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  *--|              OPERATOR  2                        | |
//!  |             |  |  |  |            ---------------                      |-|--> To Output pin
//!  |             |  *-----|                                                 | |
//!  |             |  |  |  |                  ...                            |-|--> To Output pin
//!  |             *--------|                                                 | |
//!  |                       -------------------------------------------------  |
//!  |                                                                          |
//!  |                                                                          |
//!   -------------------------------------------------------------------------
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

mod operator;
mod generator;
mod comparator;
mod timer;
mod timer_connection;

use core::ffi;

pub use self::{
    operator::{Operator, OperatorConfig, OPERATOR},
    timer::{Timer, TimerConfig, TIMER},
    timer_connection::TimerConnection,
};

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

#[cfg(not(esp32c6))]
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

#[cfg(not(esp32c6))]
impl Group for Group1 {
    const ID: ffi::c_int = 1;
}
