#[cfg(not(feature = "ulp"))]
use crate::mutex;

#[cfg(feature = "ulp")]
use crate::ulp::mutex;

use crate::adc;
#[cfg(not(feature = "ulp"))]
use crate::can;
use crate::gpio;
#[cfg(esp32)]
use crate::hall;
#[cfg(not(feature = "ulp"))]
use crate::i2c;
#[cfg(not(feature = "ulp"))]
use crate::serial;
#[cfg(not(feature = "ulp"))]
use crate::spi;

pub struct Peripherals {
    pub pins: gpio::Pins,
    #[cfg(not(feature = "ulp"))]
    pub uart0: serial::UART0,
    #[cfg(not(feature = "ulp"))]
    pub uart1: serial::UART1,
    #[cfg(all(esp32, not(feature = "ulp")))]
    pub uart2: serial::UART2,
    #[cfg(not(feature = "ulp"))]
    pub i2c0: i2c::I2C0,
    #[cfg(all(not(esp32c3), not(feature = "ulp")))]
    pub i2c1: i2c::I2C1,
    #[cfg(not(feature = "ulp"))]
    pub spi1: spi::SPI1,
    #[cfg(not(feature = "ulp"))]
    pub spi2: spi::SPI2,
    #[cfg(all(not(esp32c3), not(feature = "ulp")))]
    pub spi3: spi::SPI3,
    pub adc1: adc::ADC1,
    pub adc2: adc::ADC2,
    #[cfg(esp32)]
    pub hall_sensor: hall::HallSensor,
    #[cfg(not(feature = "ulp"))]
    pub can: can::CAN,
}

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

impl Peripherals {
    pub fn take() -> Option<Self> {
        let mut taken = TAKEN.lock();

        if *taken {
            None
        } else {
            *taken = true;
            Some(unsafe { Peripherals::new() })
        }
    }

    /// # Safety
    ///
    /// Care should be taken not to instantiate the Peripherals structure, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Self {
            pins: gpio::Pins::new(),
            #[cfg(not(feature = "ulp"))]
            uart0: serial::UART0::new(),
            #[cfg(not(feature = "ulp"))]
            uart1: serial::UART1::new(),
            #[cfg(all(esp32, not(feature = "ulp")))]
            uart2: serial::UART2::new(),
            #[cfg(not(feature = "ulp"))]
            i2c0: i2c::I2C0::new(),
            #[cfg(all(not(esp32c3), not(feature = "ulp")))]
            i2c1: i2c::I2C1::new(),
            #[cfg(not(feature = "ulp"))]
            spi1: spi::SPI1::new(),
            #[cfg(not(feature = "ulp"))]
            spi2: spi::SPI2::new(),
            #[cfg(all(not(esp32c3), not(feature = "ulp")))]
            spi3: spi::SPI3::new(),
            adc1: adc::ADC1::new(),
            adc2: adc::ADC2::new(),
            #[cfg(esp32)]
            hall_sensor: hall::HallSensor::new(),
            #[cfg(not(feature = "ulp"))]
            can: can::CAN::new(),
        }
    }
}
