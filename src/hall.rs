use core::marker::PhantomData;

use crate::adc;

pub struct HallSensor(PhantomData<*const ()>);

impl HallSensor {
    /// # Safety
    ///
    /// Care should be taken not to instnatiate this Hall Sensor instance, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        HallSensor(PhantomData)
    }
}

unsafe impl Send for HallSensor {}

impl embedded_hal::adc::Channel<adc::ADC1> for HallSensor {
    type ID = ();

    fn channel() -> Self::ID {
        ()
    }
}
