use crate::mcpwm::Group;

use super::{
    operator::{
        HwOperator0, HwOperator1, HwOperator2, NoOperator, OperatorConfig, OptionalOperator,
    },
    timer::{HwTimer, Timer},
};

// TODO: How do we want fault module to fit into this?
/// Created by `Timer::into_connection()`
pub struct TimerConnection<U: Group, T: HwTimer<U>, O0, O1, O2>
where
    O0: OptionalOperator<U, HwOperator0<U>>,
    O1: OptionalOperator<U, HwOperator1<U>>,
    O2: OptionalOperator<U, HwOperator2<U>>,
{
    timer: Timer<U, T>,
    operator0: O0,
    operator1: O1,
    operator2: O2,
}

impl<U, T> TimerConnection<U, T, NoOperator, NoOperator, NoOperator> {
    pub(crate) fn new(timer: T) -> Self {
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
        U,
        T,
        O0: OptionalOperator<U, HwOperator0>,
        O1: OptionalOperator<U, HwOperator1>,
        O2: OptionalOperator<U, HwOperator2>,
    > TimerConnection<U, T, O0, O1, O2>
{
    fn split(&mut self) -> (&mut Timer, &mut O0, &mut O1, &mut O2) {
        (
            &mut self.timer,
            &mut self.operator0,
            &mut self.operator1,
            &mut self.operator2,
        )
    }
}
// TODO: Do something more builder-pattern like for making the operator?
impl<U, T, O1, O2> TimerConnection<U, T, NoOperator, O1, O2> {
    fn attatch_operator0<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        mut self,
        operator_handle: HwOperator0<U>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<U, T, HwOperator0<U>, O1, O2> {
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: operator,
            operator1: self.operator1,
            operator2: self.operator2,
        }
    }
}

impl<U, T, O0, O2> TimerConnection<U, T, O0, NoOperator, O2> {
    fn attatch_operator1<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        mut self,
        operator_handle: HwOperator1<U>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<U, T, O0, HwOperator1<U>, O2> {
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: operator,
            operator2: self.operator2,
        }
    }
}

impl<U, T, O0, O1> TimerConnection<U, T, O0, O1, NoOperator> {
    fn attatch_operator2<PA: OptionalOutputPin, PB: OptionalOutputPin>(
        mut self,
        operator_handle: HwOperator2<U>,
        operator_cfg: OperatorConfig,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<U, T, O0, O1, HwOperator2<U>> {
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
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
