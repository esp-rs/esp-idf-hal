use crate::adc;
use crate::can;
use crate::gpio;
use crate::i2c;
#[cfg(esp_idf_soc_i2s_supported)]
use crate::i2s;
use crate::ledc;
#[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
use crate::mac;
use crate::modem;
#[cfg(any(esp32, esp32s2, esp32s3))]
use crate::pcnt;
use crate::rmt;
use crate::spi;
#[cfg(any(
    all(
        not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
        esp_idf_esp_task_wdt_en
    ),
    any(esp_idf_version_major = "4", esp_idf_version = "5.0")
))]
use crate::task::watchdog;
use crate::timer;
use crate::uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
    esp_idf_comp_ulp_enabled
))]
use crate::ulp;

pub struct Peripherals {
    pub pins: gpio::Pins,
    pub uart0: uart::UART0,
    pub uart1: uart::UART1,
    #[cfg(any(esp32, esp32s3))]
    pub uart2: uart::UART2,
    pub i2c0: i2c::I2C0,
    #[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
    pub i2c1: i2c::I2C1,
    #[cfg(esp_idf_soc_i2s_supported)]
    pub i2s0: i2s::I2S0,
    #[cfg(all(esp_idf_soc_i2s_supported, any(esp32, esp32s3)))]
    pub i2s1: i2s::I2S1,
    pub spi1: spi::SPI1,
    pub spi2: spi::SPI2,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub spi3: spi::SPI3,
    pub adc1: adc::ADC1,
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
    pub adc2: adc::ADC2,
    // TODO: Check the pulse counter story for c2, h2, c5, c6, and p4
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub pcnt0: pcnt::PCNT0,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub pcnt1: pcnt::PCNT1,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub pcnt2: pcnt::PCNT2,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub pcnt3: pcnt::PCNT3,
    #[cfg(esp32)]
    pub pcnt4: pcnt::PCNT4,
    #[cfg(esp32)]
    pub pcnt5: pcnt::PCNT5,
    #[cfg(esp32)]
    pub pcnt6: pcnt::PCNT6,
    #[cfg(esp32)]
    pub pcnt7: pcnt::PCNT7,
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub hall_sensor: crate::hall::HallSensor,
    pub can: can::CAN,
    pub ledc: ledc::LEDC,
    pub rmt: rmt::RMT,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
        esp_idf_comp_ulp_enabled
    ))]
    pub ulp: ulp::ULP,
    #[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
    pub mac: mac::MAC,
    pub modem: modem::Modem,
    // TODO: Check the timer story for c2, h2, c5, c6, and p4
    pub timer00: timer::TIMER00,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub timer01: timer::TIMER01,
    #[cfg(not(esp32c2))]
    pub timer10: timer::TIMER10,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub timer11: timer::TIMER11,
    #[cfg(any(
        all(
            not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
            esp_idf_esp_task_wdt_en
        ),
        any(esp_idf_version_major = "4", esp_idf_version = "5.0")
    ))]
    pub twdt: watchdog::TWDT,
}

static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static TAKEN_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

impl Peripherals {
    pub fn take() -> Result<Self, crate::sys::EspError> {
        if TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
            Err(crate::sys::EspError::from_infallible::<
                { crate::sys::ESP_ERR_INVALID_STATE },
            >())
        } else {
            let _guard = TAKEN_CS.enter();

            if !TAKEN.load(core::sync::atomic::Ordering::SeqCst) {
                TAKEN.store(true, core::sync::atomic::Ordering::SeqCst);

                Ok(unsafe { Peripherals::new() })
            } else {
                Err(crate::sys::EspError::from_infallible::<
                    { crate::sys::ESP_ERR_INVALID_STATE },
                >())
            }
        }
    }

    /// # Safety
    ///
    /// Care should be taken not to instantiate the Peripherals structure, if it is already instantiated and used elsewhere
    pub unsafe fn new() -> Self {
        Self {
            pins: gpio::Pins::new(),
            uart0: uart::UART0::new(),
            uart1: uart::UART1::new(),
            #[cfg(any(esp32, esp32s3))]
            uart2: uart::UART2::new(),
            i2c0: i2c::I2C0::new(),
            #[cfg(not(any(esp32c3, esp32c2, esp32c6)))]
            i2c1: i2c::I2C1::new(),
            #[cfg(esp_idf_soc_i2s_supported)]
            i2s0: i2s::I2S0::new(),
            #[cfg(all(esp_idf_soc_i2s_supported, any(esp32, esp32s3)))]
            i2s1: i2s::I2S1::new(),
            spi1: spi::SPI1::new(),
            spi2: spi::SPI2::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            spi3: spi::SPI3::new(),
            adc1: adc::ADC1::new(),
            #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
            adc2: adc::ADC2::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            pcnt0: pcnt::PCNT0::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            pcnt1: pcnt::PCNT1::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            pcnt2: pcnt::PCNT2::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            pcnt3: pcnt::PCNT3::new(),
            #[cfg(esp32)]
            pcnt4: pcnt::PCNT4::new(),
            #[cfg(esp32)]
            pcnt5: pcnt::PCNT5::new(),
            #[cfg(esp32)]
            pcnt6: pcnt::PCNT6::new(),
            #[cfg(esp32)]
            pcnt7: pcnt::PCNT7::new(),
            #[cfg(all(esp32, esp_idf_version_major = "4"))]
            hall_sensor: crate::hall::HallSensor::new(),
            can: can::CAN::new(),
            ledc: ledc::LEDC::new(),
            rmt: rmt::RMT::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
                esp_idf_comp_ulp_enabled
            ))]
            ulp: ulp::ULP::new(),
            #[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
            mac: mac::MAC::new(),
            modem: modem::Modem::new(),
            timer00: timer::TIMER00::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            timer01: timer::TIMER01::new(),
            #[cfg(not(esp32c2))]
            timer10: timer::TIMER10::new(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            timer11: timer::TIMER11::new(),
            #[cfg(any(
                all(
                    not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
                    esp_idf_esp_task_wdt_en
                ),
                any(esp_idf_version_major = "4", esp_idf_version = "5.0")
            ))]
            twdt: watchdog::TWDT::new(),
        }
    }
}
