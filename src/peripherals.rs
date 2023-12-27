use crate::adc;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::can;
use crate::gpio;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::i2c;
#[cfg(all(
    not(feature = "riscv-ulp-hal"),
    esp_idf_soc_i2s_supported,
    esp_idf_comp_driver_enabled
))]
use crate::i2s;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::ledc;
#[cfg(all(
    any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
    not(feature = "riscv-ulp-hal")
))]
use crate::mac;
#[cfg(all(
    any(esp32, esp32s3),
    esp_idf_version_major = "5",
    not(feature = "riscv-ulp-hal")
))]
use crate::mcpwm;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::modem;
#[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
use crate::pcnt;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::rmt;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::spi;
#[cfg(all(
    not(feature = "riscv-ulp-hal"),
    any(
        all(
            not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
            esp_idf_esp_task_wdt_en
        ),
        any(esp_idf_version_major = "4", esp_idf_version = "5.0")
    )
))]
use crate::task::watchdog;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::timer;
#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
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
    #[cfg(all(not(any(esp32c3, esp32c2, esp32c6)), not(feature = "riscv-ulp-hal")))]
    pub i2c1: i2c::I2C1,
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        esp_idf_soc_i2s_supported,
        esp_idf_comp_driver_enabled
    ))]
    pub i2s0: i2s::I2S0,
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        esp_idf_soc_i2s_supported,
        esp_idf_comp_driver_enabled,
        any(esp32, esp32s3)
    ))]
    pub i2s1: i2s::I2S1,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub spi1: spi::SPI1,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub spi2: spi::SPI2,
    #[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
    pub spi3: spi::SPI3,
    pub adc1: adc::ADC1,
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
    pub adc2: adc::ADC2,
    // TODO: Check the pulse counter story for c2, h2, c5, c6, and p4
    #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
    pub pcnt0: pcnt::PCNT0,
    #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
    pub pcnt1: pcnt::PCNT1,
    #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
    pub pcnt2: pcnt::PCNT2,
    #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
    pub pcnt3: pcnt::PCNT3,
    #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
    pub pcnt4: pcnt::PCNT4,
    #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
    pub pcnt5: pcnt::PCNT5,
    #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
    pub pcnt6: pcnt::PCNT6,
    #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
    pub pcnt7: pcnt::PCNT7,
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub hall_sensor: crate::hall::HallSensor,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub can: can::CAN,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub ledc: ledc::LEDC,
    #[cfg(all(
        any(esp32, esp32s3),
        esp_idf_version_major = "5",
        not(feature = "riscv-ulp-hal")
    ))]
    pub mcpwm0: mcpwm::MCPWM<mcpwm::Group0>,
    #[cfg(all(
        any(esp32, esp32s3),
        esp_idf_version_major = "5",
        not(feature = "riscv-ulp-hal")
    ))]
    pub mcpwm1: mcpwm::MCPWM<mcpwm::Group1>,
    #[cfg(not(feature = "riscv-ulp-hal"))]
    pub rmt: rmt::RMT,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
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
    // TODO: Check the timer story for c2, h2, c5, c6, and p4
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer00")
    ))]
    pub timer00: timer::TIMER00,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer01")
    ))]
    pub timer01: timer::TIMER01,
    #[cfg(all(
        not(esp32c2),
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer10")
    ))]
    pub timer10: timer::TIMER10,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(feature = "riscv-ulp-hal"),
        not(feature = "embassy-time-isr-queue-timer11")
    ))]
    pub timer11: timer::TIMER11,
    #[cfg(all(
        not(feature = "riscv-ulp-hal"),
        any(
            all(
                not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
                esp_idf_esp_task_wdt_en
            ),
            any(esp_idf_version_major = "4", esp_idf_version = "5.0")
        )
    ))]
    pub twdt: watchdog::TWDT,
}

#[cfg(feature = "riscv-ulp-hal")]
static mut TAKEN: bool = false;

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(not(feature = "riscv-ulp-hal"))]
static TAKEN_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

impl Peripherals {
    #[cfg(feature = "riscv-ulp-hal")]
    pub fn take() -> Result<Self, crate::sys::EspError> {
        if unsafe { TAKEN } {
            panic!("Peripheral already taken")
        } else {
            unsafe {
                TAKEN = true;
            }
            Ok(unsafe { Peripherals::new() })
        }
    }

    #[cfg(not(feature = "riscv-ulp-hal"))]
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
            #[cfg(not(feature = "riscv-ulp-hal"))]
            uart0: uart::UART0::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            uart1: uart::UART1::new(),
            #[cfg(all(any(esp32, esp32s3), not(feature = "riscv-ulp-hal")))]
            uart2: uart::UART2::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            i2c0: i2c::I2C0::new(),
            #[cfg(all(not(any(esp32c3, esp32c2, esp32c6)), not(feature = "riscv-ulp-hal")))]
            i2c1: i2c::I2C1::new(),
            #[cfg(all(
                not(feature = "riscv-ulp-hal"),
                esp_idf_soc_i2s_supported,
                esp_idf_comp_driver_enabled
            ))]
            i2s0: i2s::I2S0::new(),
            #[cfg(all(
                not(feature = "riscv-ulp-hal"),
                esp_idf_soc_i2s_supported,
                esp_idf_comp_driver_enabled,
                any(esp32, esp32s3)
            ))]
            i2s1: i2s::I2S1::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            spi1: spi::SPI1::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            spi2: spi::SPI2::new(),
            #[cfg(all(any(esp32, esp32s2, esp32s3), not(feature = "riscv-ulp-hal")))]
            spi3: spi::SPI3::new(),
            adc1: adc::ADC1::new(),
            #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
            adc2: adc::ADC2::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
            pcnt0: pcnt::PCNT0::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
            pcnt1: pcnt::PCNT1::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
            pcnt2: pcnt::PCNT2::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), any(esp32, esp32s2, esp32s3)))]
            pcnt3: pcnt::PCNT3::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
            pcnt4: pcnt::PCNT4::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
            pcnt5: pcnt::PCNT5::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
            pcnt6: pcnt::PCNT6::new(),
            #[cfg(all(not(feature = "riscv-ulp-hal"), esp32))]
            pcnt7: pcnt::PCNT7::new(),
            #[cfg(all(esp32, esp_idf_version_major = "4"))]
            hall_sensor: crate::hall::HallSensor::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            can: can::CAN::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            ledc: ledc::LEDC::new(),
            #[cfg(all(
                any(esp32, esp32s3),
                not(feature = "riscv-ulp-hal"),
                esp_idf_version_major = "5"
            ))]
            mcpwm0: mcpwm::MCPWM::<mcpwm::Group0>::new(),
            #[cfg(all(
                any(esp32, esp32s3),
                not(feature = "riscv-ulp-hal"),
                esp_idf_version_major = "5"
            ))]
            mcpwm1: mcpwm::MCPWM::<mcpwm::Group1>::new(),
            #[cfg(not(feature = "riscv-ulp-hal"))]
            rmt: rmt::RMT::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
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
                any(esp32, esp32s2, esp32s3),
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer01")
            ))]
            timer01: timer::TIMER01::new(),
            #[cfg(all(
                not(esp32c2),
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer10")
            ))]
            timer10: timer::TIMER10::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(feature = "riscv-ulp-hal"),
                not(feature = "embassy-time-isr-queue-timer11")
            ))]
            timer11: timer::TIMER11::new(),
            #[cfg(all(
                not(feature = "riscv-ulp-hal"),
                any(
                    all(
                        not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
                        esp_idf_esp_task_wdt_en
                    ),
                    any(esp_idf_version_major = "4", esp_idf_version = "5.0")
                )
            ))]
            twdt: watchdog::TWDT::new(),
        }
    }
}
