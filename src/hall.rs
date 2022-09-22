use crate::adc;

impl embedded_hal_0_2::adc::Channel<adc::ADC1> for HallSensor {
    type ID = ();

    fn channel() -> Self::ID {}
}

crate::impl_peripheral!(HallSensor);
