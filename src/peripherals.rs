#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::mutex;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::mutex;

use crate::adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::can;
use crate::gpio;
#[cfg(esp32)]
use crate::hall;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::i2c;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::ledc;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::serial;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::spi;
#[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
use crate::ulp;

pub struct Peripherals {
    pub pins: gpio::Pins,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub uart0: serial::UART0,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub uart1: serial::UART1,
    #[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
    pub uart2: serial::UART2,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub i2c0: i2c::I2C0,
    #[cfg(all(not(esp32c3), not(feature = "riscv-ulp-hal")))]
    pub i2c1: i2c::I2C1,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub spi1: spi::SPI1,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub spi2: spi::SPI2,
    #[cfg(all(not(esp32c3), not(feature = "riscv-ulp-hal")))]
    pub spi3: spi::SPI3,
    pub adc1: adc::ADC1,
    pub adc2: adc::ADC2,
    #[cfg(esp32)]
    pub hall_sensor: hall::HallSensor,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub can: can::CAN,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub ledc: ledc::Peripheral,
    #[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
    pub ulp: ulp::ULP,
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
            #[cfg(not(feature = "riscv-ulp-hal"))]
            uart0: serial::UART0::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            uart1: serial::UART1::new(),
            #[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
            uart2: serial::UART2::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            i2c0: i2c::I2C0::new(),
            #[cfg(all(not(esp32c3), not(feature = "riscv-ulp-hal")))]
            i2c1: i2c::I2C1::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            spi1: spi::SPI1::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            spi2: spi::SPI2::new(),
            #[cfg(all(not(esp32c3), not(feature = "riscv-ulp-hal")))]
            spi3: spi::SPI3::new(),
            adc1: adc::ADC1::new(),
            adc2: adc::ADC2::new(),
            #[cfg(esp32)]
            hall_sensor: hall::HallSensor::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            can: can::CAN::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            ledc: ledc::Peripheral::new(),
            #[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
            ulp: ulp::ULP::new(),
        }
    }
}
