use crate::mcpwm::Group;

use super::{
    comparator::OptionalCmpCfg,
    generator::OptionalGenCfg,
    operator::{self, NoOperator, OptionalOperator, OPERATOR},
    timer::Timer,
    Operator, OperatorConfig,
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
impl<const N: u8, G, O1, O2> TimerConnection<N, G, NoOperator, O1, O2>
where
    G: Group,
    O1: OptionalOperator<1, G>,
    O2: OptionalOperator<2, G>,
{
    pub fn attatch_operator0<CMPX, CMPY, GENA, GENB>(
        self,
        operator_handle: OPERATOR<0, G>,
        operator_cfg: OperatorConfig<CMPX, CMPY, GENA, GENB>,
    ) -> TimerConnection<N, G, Operator<0, G, CMPX::Cmp, CMPY::Cmp, GENA::Gen, GENB::Gen>, O1, O2>
    where
        CMPX: OptionalCmpCfg,
        CMPY: OptionalCmpCfg,
        GENA: OptionalGenCfg,
        GENB: OptionalGenCfg,
    {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) };
        TimerConnection {
            timer: self.timer,
            operator0: operator,
            operator1: self.operator1,
            operator2: self.operator2,
        }
    }
}

impl<const N: u8, G, O0, O2> TimerConnection<N, G, O0, NoOperator, O2>
where
    G: Group,
    O0: OptionalOperator<0, G>,
    O2: OptionalOperator<2, G>,
{
    pub fn attatch_operator1<CMPX, CMPY, GENA, GENB>(
        self,
        operator_handle: OPERATOR<1, G>,
        operator_cfg: OperatorConfig<CMPX, CMPY, GENA, GENB>,
    ) -> TimerConnection<N, G, O0, Operator<1, G, CMPX::Cmp, CMPY::Cmp, GENA::Gen, GENB::Gen>, O2>
    where
        CMPX: OptionalCmpCfg,
        CMPY: OptionalCmpCfg,
        GENA: OptionalGenCfg,
        GENB: OptionalGenCfg,
    {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) };
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: operator,
            operator2: self.operator2,
        }
    }
}

impl<const N: u8, G, O0, O1> TimerConnection<N, G, O0, O1, NoOperator>
where
    G: Group,
    O0: OptionalOperator<0, G>,
    O1: OptionalOperator<1, G>,
{
    pub fn attatch_operator2<CMPX, CMPY, GENA, GENB>(
        self,
        operator_handle: OPERATOR<2, G>,
        operator_cfg: OperatorConfig<CMPX, CMPY, GENA, GENB>,
    ) -> TimerConnection<N, G, O0, O1, Operator<2, G, CMPX::Cmp, CMPY::Cmp, GENA::Gen, GENB::Gen>>
    where
        CMPX: OptionalCmpCfg,
        CMPY: OptionalCmpCfg,
        GENA: OptionalGenCfg,
        GENB: OptionalGenCfg,
    {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) };
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator,
        }
    }
}

/*
// TODO: Adding this prevents moving values out of the type as is done in attatch_operator()
// how do we make this work?
impl<const N: u8, G, O0, O1, O2> Drop for TimerConnection<N, G, O0, O1, O2>
    where G: Group,
    O0: OptionalOperator<0, G>,
    O1: OptionalOperator<1, G>,
    O2: OptionalOperator<2, G>,
{
    fn drop(&mut self) {
        todo!()
    }
}
*/

// TODO: Should this be moved somewhere else?
pub trait OptionalOutputPin {}

impl<P: crate::gpio::OutputPin> OptionalOutputPin for P {}
