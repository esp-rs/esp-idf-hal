use crate::mcpwm::Group;

use super::{
    operator::{NoOperator, OperatorConfig, OptionalOperator, OPERATOR},
    timer::{Timer, TIMER},
    Operator,
};

// TODO: How do we want fault module to fit into this?
/// Created by `Timer::into_connection()`
pub struct TimerConnection<const N: u8, G: Group, O0, O1, O2>
where
    O0: OptionalOperator<0, G>,
    O1: OptionalOperator<1, G>,
    O2: OptionalOperator<2, G>,
{
    timer: Timer<N, G>,
    operator0: O0,
    operator1: O1,
    operator2: O2,
}

impl<const N: u8, G: Group> TimerConnection<N, G, NoOperator, NoOperator, NoOperator> {
    pub(crate) fn new(timer: Timer<N, G>) -> Self {
        Self {
            timer,
            operator0: NoOperator,
            operator1: NoOperator,
            operator2: NoOperator,
        }
    }
}

// Since there can only ever by one instance of every operator type (except NoOperator)
// we know that there can be no mem::swap or similar to cause any problems.
//
// Thus we know that after split is called nothing can be added/removed while still having access to
// the individual objects. We also garantuee that the operators wont live longer than the timer
impl<
        const N: u8,
        G: Group,
        O0: OptionalOperator<0, G>,
        O1: OptionalOperator<1, G>,
        O2: OptionalOperator<2, G>,
    > TimerConnection<N, G, O0, O1, O2>
{
    pub fn split(&mut self) -> (&mut Timer<N, G>, &mut O0, &mut O1, &mut O2) {
        (
            &mut self.timer,
            &mut self.operator0,
            &mut self.operator1,
            &mut self.operator2,
        )
    }
}
// TODO: Do something more builder-pattern like for making the operator?
impl<const N: u8, G: Group, O1: OptionalOperator<1, G>, O2: OptionalOperator<2, G>>
    TimerConnection<N, G, NoOperator, O1, O2>
{
    pub fn attatch_operator0<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        self,
        operator_handle: OPERATOR<0, G>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<N, G, Operator<0, G, PA, PB>, O1, O2> {
        let operator = todo!(); //self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: operator,
            operator1: self.operator1,
            operator2: self.operator2,
        }
    }
}

impl<const N: u8, G: Group, O0: OptionalOperator<0, G>, O2: OptionalOperator<2, G>>
    TimerConnection<N, G, O0, NoOperator, O2>
{
    pub fn attatch_operator1<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        self,
        operator_handle: OPERATOR<1, G>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<N, G, O0, Operator<1, G, PA, PB>, O2> {
        let operator = todo!(); //self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: operator,
            operator2: self.operator2,
        }
    }
}

impl<const N: u8, G: Group, O0: OptionalOperator<0, G>, O1: OptionalOperator<1, G>>
    TimerConnection<N, G, O0, O1, NoOperator>
{
    pub fn attatch_operator2<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        self,
        operator_handle: OPERATOR<2, G>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<N, G, O0, O1, Operator<2, G, PA, PB>> {
        let operator = todo!(); //self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator,
        }
    }
}

pub struct NoPin;

pub trait OptionalOutputPin {}

impl<P: crate::gpio::OutputPin> OptionalOutputPin for P {}
