use crate::adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::can;
use crate::gpio;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::i2c;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::ledc;
#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
use crate::mac;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::modem;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::rmt;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::serial;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::spi;
#[cfg(all(
    any(esp32, esp32s2, esp32s3),
    not(feature = "riscv-ulp-hal"),
    esp_idf_comp_ulp_enabled
))]
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
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub hall_sensor: crate::hall::HallSensor,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub can: can::CAN,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub ledc: ledc::Peripheral,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub rmt: rmt::Peripheral,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(feature = "riscv-ulp-hal"),
        esp_idf_comp_ulp_enabled
    ))]
    pub ulp: ulp::ULP,
    #[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
    pub mac: mac::Mac,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub modem: modem::Modem,
}

#[cfg(feature = "riscv-ulp-hal")]
static mut TAKEN: bool = false;

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN_CS: crate::cs::CriticalSection = crate::cs::CriticalSection::new();

impl Peripherals {
    #[cfg(feature = "riscv-ulp-hal")]
    pub fn take() -> Option<Self> {
        if unsafe { TAKEN } {
            None
        } else {
            unsafe {
                TAKEN = true;
            }
            Some(unsafe { Peripherals::new() })
        }
    }

    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub fn take() -> Option<Self> {
        if TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
            None
        } else {
            let _ = TAKEN_CS.enter();

            if !TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
                TAKEN.store(true, core::sync::atomic::Ordering::SeqCst);

                Some(unsafe { Peripherals::new() })
            } else {
                None
            }
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
            #[cfg(all(esp32, esp_idf_version_major = "4"))]
            hall_sensor: crate::hall::HallSensor::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            can: can::CAN::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            ledc: ledc::Peripheral::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            rmt: rmt::Peripheral::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(feature = "riscv-ulp-hal"),
                esp_idf_comp_ulp_enabled
            ))]
            ulp: ulp::ULP::new(),
            #[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
            mac: mac::Mac::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            modem: modem::Modem::new(),
        }
    }
}
