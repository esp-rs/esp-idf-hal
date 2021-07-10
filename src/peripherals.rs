use mutex_trait::*;

use esp_idf_sys::EspMutex;

use crate::gpio;
use crate::i2c;
use crate::serial;
use crate::spi;

pub struct Peripherals {
    pub pins: gpio::Pins,
    pub uart0: serial::UART0,
    pub uart1: serial::UART1,
    #[cfg(esp32)]
    pub uart2: serial::UART2,
    pub i2c0: i2c::I2C0,
    #[cfg(not(esp32c3))]
    pub i2c1: i2c::I2C1,
    pub spi1: spi::SPI1,
    pub spi2: spi::SPI2,
    #[cfg(not(esp32c3))]
    pub spi3: spi::SPI3,
}

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

impl Peripherals {
    pub fn take() -> Option<Self> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    None
                } else {
                    *taken = true;
                    Some(Peripherals::new())
                }
            })
        }
    }

    pub unsafe fn new() -> Self {
        Self {
            pins: gpio::Pins::new(),
            uart0: serial::UART0::new(),
            uart1: serial::UART1::new(),
            #[cfg(esp32)]
            uart2: serial::UART2::new(),
            i2c0: i2c::I2C0::new(),
            #[cfg(not(esp32c3))]
            i2c1: i2c::I2C1::new(),
            spi1: spi::SPI1::new(),
            spi2: spi::SPI2::new(),
            #[cfg(not(esp32c3))]
            spi3: spi::SPI3::new(),
        }
    }
}
