use crate::adc;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::can;
use crate::gpio;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::i2c;
#[cfg(all(
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
    esp_idf_soc_i2s_supported,
    esp_idf_comp_driver_enabled
))]
use crate::i2s;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::ledc;
#[cfg(all(
    any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
))]
use crate::mac;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::modem;
#[cfg(all(
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
    any(esp32, esp32s2, esp32s3)
))]
use crate::pcnt;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::rmt;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::spi;
#[cfg(all(
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
    any(
        all(
            not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
            esp_idf_esp_task_wdt_en
        ),
        any(esp_idf_version_major = "4", esp_idf_version = "5.0")
    )
))]
use crate::task::watchdog;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::timer;
#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
use crate::uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
    not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
    esp_idf_comp_ulp_enabled
))]
use crate::ulp;

pub struct Peripherals {
    pub pins: gpio::Pins,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub uart0: uart::UART0,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub uart1: uart::UART1,
    #[cfg(all(
        any(esp32, esp32s3),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
    ))]
    pub uart2: uart::UART2,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub i2c0: i2c::I2C0,
    #[cfg(all(
        not(any(esp32c3, esp32c2, esp32c6)),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
    ))]
    pub i2c1: i2c::I2C1,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp_idf_soc_i2s_supported,
        esp_idf_comp_driver_enabled
    ))]
    pub i2s0: i2s::I2S0,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp_idf_soc_i2s_supported,
        esp_idf_comp_driver_enabled,
        any(esp32, esp32s3)
    ))]
    pub i2s1: i2s::I2S1,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub spi1: spi::SPI1,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub spi2: spi::SPI2,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
    ))]
    pub spi3: spi::SPI3,
    pub adc1: adc::ADC1,
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
    pub adc2: adc::ADC2,
    // TODO: Check the pulse counter story for c2, h2, c5, c6, and p4
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        any(esp32, esp32s2, esp32s3)
    ))]
    pub pcnt0: pcnt::PCNT0,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        any(esp32, esp32s2, esp32s3)
    ))]
    pub pcnt1: pcnt::PCNT1,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        any(esp32, esp32s2, esp32s3)
    ))]
    pub pcnt2: pcnt::PCNT2,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        any(esp32, esp32s2, esp32s3)
    ))]
    pub pcnt3: pcnt::PCNT3,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp32
    ))]
    pub pcnt4: pcnt::PCNT4,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp32
    ))]
    pub pcnt5: pcnt::PCNT5,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp32
    ))]
    pub pcnt6: pcnt::PCNT6,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp32
    ))]
    pub pcnt7: pcnt::PCNT7,
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub hall_sensor: crate::hall::HallSensor,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub can: can::CAN,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub ledc: ledc::LEDC,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub rmt: rmt::RMT,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        esp_idf_comp_ulp_enabled
    ))]
    pub ulp: ulp::ULP,
    #[cfg(all(
        any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
    ))]
    pub mac: mac::MAC,
    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
    pub modem: modem::Modem,
    // TODO: Check the timer story for c2, h2, c5, c6, and p4
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        not(feature = "embassy-time-isr-queue-timer00")
    ))]
    pub timer00: timer::TIMER00,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        not(feature = "embassy-time-isr-queue-timer01")
    ))]
    pub timer01: timer::TIMER01,
    #[cfg(all(
        not(esp32c2),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        not(feature = "embassy-time-isr-queue-timer10")
    ))]
    pub timer10: timer::TIMER10,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3),
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
        not(feature = "embassy-time-isr-queue-timer11")
    ))]
    pub timer11: timer::TIMER11,
    #[cfg(all(
        not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
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

#[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
static mut TAKEN: bool = false;

#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
static TAKEN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
static TAKEN_CS: crate::task::CriticalSection = crate::task::CriticalSection::new();

impl Peripherals {
    #[cfg(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))]
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

    #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
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
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            uart0: uart::UART0::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            uart1: uart::UART1::new(),
            #[cfg(all(
                any(esp32, esp32s3),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
            ))]
            uart2: uart::UART2::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            i2c0: i2c::I2C0::new(),
            #[cfg(all(
                not(any(esp32c3, esp32c2, esp32c6)),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
            ))]
            i2c1: i2c::I2C1::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp_idf_soc_i2s_supported,
                esp_idf_comp_driver_enabled
            ))]
            i2s0: i2s::I2S0::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp_idf_soc_i2s_supported,
                esp_idf_comp_driver_enabled,
                any(esp32, esp32s3)
            ))]
            i2s1: i2s::I2S1::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            spi1: spi::SPI1::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            spi2: spi::SPI2::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
            ))]
            spi3: spi::SPI3::new(),
            adc1: adc::ADC1::new(),
            #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
            adc2: adc::ADC2::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                any(esp32, esp32s2, esp32s3)
            ))]
            pcnt0: pcnt::PCNT0::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                any(esp32, esp32s2, esp32s3)
            ))]
            pcnt1: pcnt::PCNT1::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                any(esp32, esp32s2, esp32s3)
            ))]
            pcnt2: pcnt::PCNT2::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                any(esp32, esp32s2, esp32s3)
            ))]
            pcnt3: pcnt::PCNT3::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp32
            ))]
            pcnt4: pcnt::PCNT4::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp32
            ))]
            pcnt5: pcnt::PCNT5::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp32
            ))]
            pcnt6: pcnt::PCNT6::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp32
            ))]
            pcnt7: pcnt::PCNT7::new(),
            #[cfg(all(esp32, esp_idf_version_major = "4"))]
            hall_sensor: crate::hall::HallSensor::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            can: can::CAN::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            ledc: ledc::LEDC::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            rmt: rmt::RMT::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c6, esp32p4),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                esp_idf_comp_ulp_enabled
            ))]
            ulp: ulp::ULP::new(),
            #[cfg(all(
                any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys")))
            ))]
            mac: mac::MAC::new(),
            #[cfg(not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))))]
            modem: modem::Modem::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                not(feature = "embassy-time-isr-queue-timer00")
            ))]
            timer00: timer::TIMER00::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                not(feature = "embassy-time-isr-queue-timer01")
            ))]
            timer01: timer::TIMER01::new(),
            #[cfg(all(
                not(esp32c2),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                not(feature = "embassy-time-isr-queue-timer10")
            ))]
            timer10: timer::TIMER10::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3),
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
                not(feature = "embassy-time-isr-queue-timer11")
            ))]
            timer11: timer::TIMER11::new(),
            #[cfg(all(
                not(all(feature = "riscv-ulp-hal", not(feature = "esp-idf-sys"))),
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
