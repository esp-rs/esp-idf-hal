// TODO: How do we want fault module to fit into this?
/// Created by `Timer::into_connection()`
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

impl<U, T> TimerConnection<U, T, NoOperator, NoOperator, NoOperator> {
    pub(crate) fn new(timer: T) -> Self {
        Self {
            timer,
            operator0: NoOperator,
            operator1: NoOperator,
            operator2: NoOperator
        }
    }
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

impl<U, T, O1, O2> TimerConnection<U, T, NoOperator, O1, O2> {
    fn attatch_operator0<O: OperatorConfig<U, T>>(mut self, operator_cfg: O) -> TimerConnection<U, T, O, O1, O2> {
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
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
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
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
        let operator = self.init_and_attach_operator(operator_cfg, pin_a, pin_b);
        TimerConnection {
            timer: self.timer,
            operator0: self.operator0,
            operator1: self.operator1,
            operator2: operator
        }
    }
}