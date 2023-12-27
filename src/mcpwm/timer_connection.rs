use esp_idf_sys::EspError;

use crate::mcpwm::Group;

use super::{
    operator::{self, NoOperator, OptionalOperator, OPERATOR},
    timer::TimerDriver,
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
    timer: TimerDriver<N, G>,
    operator0: O0,
    operator1: O1,
    operator2: O2,
}

impl<const N: u8, G: Group> TimerConnection<N, G, NoOperator, NoOperator, NoOperator> {
    pub(crate) fn new(timer: TimerDriver<N, G>) -> Self {
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
    pub fn split(&mut self) -> (&mut TimerDriver<N, G>, &mut O0, &mut O1, &mut O2) {
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
    #[allow(clippy::type_complexity)]
    pub fn attach_operator0(
        self,
        operator_handle: OPERATOR<0, G>,
        operator_cfg: OperatorConfig<'_>,
    ) -> Result<TimerConnection<N, G, Operator<'_, 0, G>, O1, O2>, EspError> {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) }?;
        Ok(TimerConnection {
            timer: self.timer,
            operator0: operator,
            operator1: self.operator1,
            operator2: self.operator2,
        })
    }
}

impl<const N: u8, G, O0, O2> TimerConnection<N, G, O0, NoOperator, O2>
where
    G: Group,
    O0: OptionalOperator<0, G>,
    O2: OptionalOperator<2, G>,
{
    #[allow(clippy::type_complexity)]
    pub fn attach_operator1(
        self,
        operator_handle: OPERATOR<1, G>,
        operator_cfg: OperatorConfig<'_>,
    ) -> Result<TimerConnection<N, G, O0, Operator<'_, 1, G>, O2>, EspError> {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) }?;
        Ok(TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: operator,
            operator2: self.operator2,
        })
    }
}

impl<const N: u8, G, O0, O1> TimerConnection<N, G, O0, O1, NoOperator>
where
    G: Group,
    O0: OptionalOperator<0, G>,
    O1: OptionalOperator<1, G>,
{
    #[allow(clippy::type_complexity)]
    pub fn attach_operator2(
        self,
        operator_handle: OPERATOR<2, G>,
        operator_cfg: OperatorConfig<'_>,
    ) -> Result<TimerConnection<N, G, O0, O1, Operator<'_, 2, G>>, EspError> {
        let operator = unsafe { operator::new(operator_handle, self.timer.timer(), operator_cfg) }?;
        Ok(TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator,
        })
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
