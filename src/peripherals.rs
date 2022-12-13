use crate::adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::can;
use crate::gpio;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::i2c;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::ledc;
#[cfg(all(
    any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
    not(feature = "riscv-ulp-hal")
))]
use crate::mac;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::modem;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::rmt;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::spi;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::timer;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3),
    not(feature = "riscv-ulp-hal"),
    esp_idf_comp_ulp_enabled
))]
use crate::ulp;

pub struct Peripherals {
    pub pins: gpio::Pins,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub uart0: uart::UART0,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub uart1: uart::UART1,
    #[cfg(all(any(esp32, esp32s3), not(feature = "riscv-ulp-hal")))]
    pub uart2: uart::UART2,
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
    pub ledc: ledc::LEDC,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub rmt: rmt::RMT,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(feature = "riscv-ulp-hal"),
        esp_idf_comp_ulp_enabled
    ))]
    pub ulp: ulp::ULP,
    #[cfg(all(
        any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
        not(feature = "riscv-ulp-hal")
    ))]
    pub mac: mac::MAC,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub modem: modem::Modem,
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer00")
    ))]
    pub timer00: timer::TIMER00,
    #[cfg(all(
        not(esp32c3),
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer01")
    ))]
    pub timer01: timer::TIMER01,
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer10")
    ))]
    pub timer10: timer::TIMER10,
    #[cfg(all(
        not(esp32c3),
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer11")
    ))]
    pub timer11: timer::TIMER11,
}

#[cfg(feature = "riscv-ulp-hal")]
static mut TAKEN: bool = false;

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

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
            let _guard = TAKEN_CS.enter();

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
            uart0: uart::UART0::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            uart1: uart::UART1::new(),
            #[cfg(all(any(esp32, esp32s3), not(feature = "riscv-ulp-hal")))]
            uart2: uart::UART2::new(),
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
            ledc: ledc::LEDC::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            rmt: rmt::RMT::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(feature = "riscv-ulp-hal"),
                esp_idf_comp_ulp_enabled
            ))]
            ulp: ulp::ULP::new(),
            #[cfg(all(
                any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
                not(feature = "riscv-ulp-hal")
            ))]
            mac: mac::MAC::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            modem: modem::Modem::new(),
            #[cfg(all(
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer00")
            ))]
            timer00: timer::TIMER00::new(),
            #[cfg(all(
                not(esp32c3),
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer01")
            ))]
            timer01: timer::TIMER01::new(),
            #[cfg(all(
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer10")
            ))]
            timer10: timer::TIMER10::new(),
            #[cfg(all(
                not(esp32c3),
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer11")
            ))]
            timer11: timer::TIMER11::new(),
        }
    }
}
