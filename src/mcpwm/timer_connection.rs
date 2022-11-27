use crate::mcpwm::Group;

use super::{
    comparator::OptionalCmp,
    generator::OptionalGen,
    operator::{NoOperator, OptionalOperator, OPERATOR},
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
    pub fn attatch_operator0<CMP_X, CMP_Y, GEN_A, GEN_B, PA, PB>(
        self,
        operator_handle: OPERATOR<0, G>,
        operator_cfg: OperatorConfig<CMP_X, CMP_Y, GEN_A, GEN_B>,
    ) -> TimerConnection<N, G, Operator<0, G, CMP_X, CMP_Y, GEN_A, GEN_B>, O1, O2>
    where
        CMP_X: OptionalCmp,
        CMP_Y: OptionalCmp,
        GEN_A: OptionalGen,
        GEN_B: OptionalGen,
        PA: OptionalOutputPin,
        PB: OptionalOutputPin,
    {
        let operator = unsafe { Operator::new(operator_handle, self.timer.timer(), operator_config) };
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
    pub fn attatch_operator1<CMP_X, CMP_Y, GEN_A, GEN_B, PA, PB>(
        self,
        operator_handle: OPERATOR<1, G>,
        operator_cfg: OperatorConfig<CMP_X, CMP_Y, GEN_A, GEN_B>,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<N, G, O0, Operator<1, G, CMP_X, CMP_Y, GEN_A, GEN_B>, O2>
    where
        CMP_X: OptionalCmp,
        CMP_Y: OptionalCmp,
        GEN_A: OptionalGen,
        GEN_B: OptionalGen,
        PA: OptionalOutputPin,
        PB: OptionalOutputPin,
    {
        let operator = todo!(); //self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
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
    pub fn attatch_operator2<CMP_X, CMP_Y, GEN_A, GEN_B, PA, PB>(
        self,
        operator_handle: OPERATOR<2, G>,
        operator_cfg: OperatorConfig<CMP_X, CMP_Y, GEN_A, GEN_B>,
        pin_a: PA,
        pin_b: PB,
    ) -> TimerConnection<N, G, O0, O1, Operator<2, G, CMP_X, CMP_Y, GEN_A, GEN_B>>
    where
        CMP_X: OptionalCmp,
        CMP_Y: OptionalCmp,
        GEN_A: OptionalGen,
        GEN_B: OptionalGen,
        PA: OptionalOutputPin,
        PB: OptionalOutputPin,
    {
        let operator = todo!(); //self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator,
        }
    }
}

// TODO: Should this be moved somewhere else?
pub struct NoPin;

// TODO: Should this be moved somewhere else?
pub trait OptionalOutputPin {}

impl<P: crate::gpio::OutputPin> OptionalOutputPin for P {}
