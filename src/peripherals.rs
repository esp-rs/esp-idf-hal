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
#[cfg(not(esp_idf_version_at_least_6_0_0))]
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
    feature = "pcnt-legacy"
))]
use crate::pcnt;
#[cfg(feature = "rmt-legacy")]
use crate::rmt_legacy;
#[cfg(esp_idf_soc_sdmmc_host_supported)]
use crate::sd;
use crate::spi;
#[cfg(any(
    all(
        not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
        esp_idf_esp_task_wdt_en
    ),
    any(esp_idf_version_major = "4", esp_idf_version = "5.0")
))]
use crate::task::watchdog;
#[cfg(all(esp_idf_soc_temp_sensor_supported, esp_idf_version_major = "5"))]
use crate::temp_sensor;
#[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
use crate::timer;
use crate::uart;
#[cfg(all(
    any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32p4),
    esp_idf_comp_ulp_enabled
))]
use crate::ulp;
#[cfg(esp_idf_soc_usb_serial_jtag_supported)]
use crate::usb_serial;

pub struct Peripherals {
    pub pins: gpio::Pins,
    pub uart0: uart::UART0<'static>,
    pub uart1: uart::UART1<'static>,
    #[cfg(any(esp32, esp32s3))]
    pub uart2: uart::UART2<'static>,
    pub i2c0: i2c::I2C0<'static>,
    #[cfg(not(any(esp32c3, esp32c2, esp32c5, esp32c6, esp32c61)))]
    pub i2c1: i2c::I2C1<'static>,
    #[cfg(esp_idf_soc_i2s_supported)]
    pub i2s0: i2s::I2S0<'static>,
    #[cfg(all(esp_idf_soc_i2s_supported, any(esp32, esp32s3)))]
    pub i2s1: i2s::I2S1<'static>,
    pub spi1: spi::SPI1<'static>,
    pub spi2: spi::SPI2<'static>,
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub spi3: spi::SPI3<'static>,
    pub adc1: adc::ADC1<'static>,
    #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
    pub adc2: adc::ADC2<'static>,
    // TODO: Check the pulse counter story for c2, h2, c5, and p4
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
        feature = "pcnt-legacy"
    ))]
    pub pcnt0: pcnt::PCNT0<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
        feature = "pcnt-legacy"
    ))]
    pub pcnt1: pcnt::PCNT1<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
        feature = "pcnt-legacy"
    ))]
    pub pcnt2: pcnt::PCNT2<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
        feature = "pcnt-legacy"
    ))]
    pub pcnt3: pcnt::PCNT3<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(esp32, feature = "pcnt-legacy"))]
    pub pcnt4: pcnt::PCNT4<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(esp32, feature = "pcnt-legacy"))]
    pub pcnt5: pcnt::PCNT5<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(esp32, feature = "pcnt-legacy"))]
    pub pcnt6: pcnt::PCNT6<'static>,
    #[cfg(not(esp_idf_version_at_least_6_0_0))]
    #[cfg(all(esp32, feature = "pcnt-legacy"))]
    pub pcnt7: pcnt::PCNT7<'static>,
    pub can: can::CAN<'static>,
    pub ledc: ledc::LEDC,
    #[cfg(esp32)]
    pub hledc: ledc::HLEDC,
    #[cfg(feature = "rmt-legacy")]
    pub rmt: rmt_legacy::RMT,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32p4),
        esp_idf_comp_ulp_enabled
    ))]
    pub ulp: ulp::ULP<'static>,
    #[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
    pub mac: mac::MAC<'static>,
    pub modem: modem::Modem<'static>,
    #[cfg(esp_idf_soc_sdmmc_host_supported)]
    pub sdmmc0: sd::mmc::SDMMC0<'static>,
    #[cfg(esp_idf_soc_sdmmc_host_supported)]
    pub sdmmc1: sd::mmc::SDMMC1<'static>,
    #[cfg(all(esp_idf_soc_temp_sensor_supported, esp_idf_version_major = "5"))]
    pub temp_sensor: temp_sensor::TempSensor<'static>,
    // TODO: Check the timer story for c2, h2, c5, c6, and p4
    #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
    pub timer00: timer::TIMER00<'static>,
    #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub timer01: timer::TIMER01<'static>,
    #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
    #[cfg(not(esp32c2))]
    pub timer10: timer::TIMER10<'static>,
    #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
    #[cfg(any(esp32, esp32s2, esp32s3))]
    pub timer11: timer::TIMER11<'static>,
    #[cfg(any(
        all(
            not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
            esp_idf_esp_task_wdt_en
        ),
        any(esp_idf_version_major = "4", esp_idf_version = "5.0")
    ))]
    pub twdt: watchdog::TWDT<'static>,
    #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
    pub usb_serial: usb_serial::USB_SERIAL<'static>,
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

                Ok(unsafe { Peripherals::steal() })
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
    pub unsafe fn steal() -> Self {
        Self {
            pins: gpio::Pins::new(),
            uart0: uart::UART0::steal(),
            uart1: uart::UART1::steal(),
            #[cfg(any(esp32, esp32s3))]
            uart2: uart::UART2::steal(),
            i2c0: i2c::I2C0::steal(),
            #[cfg(not(any(esp32c3, esp32c2, esp32c5, esp32c6, esp32c61)))]
            i2c1: i2c::I2C1::steal(),
            #[cfg(esp_idf_soc_i2s_supported)]
            i2s0: i2s::I2S0::steal(),
            #[cfg(all(esp_idf_soc_i2s_supported, any(esp32, esp32s3)))]
            i2s1: i2s::I2S1::steal(),
            spi1: spi::SPI1::steal(),
            spi2: spi::SPI2::steal(),
            #[cfg(any(esp32, esp32s2, esp32s3))]
            spi3: spi::SPI3::steal(),
            adc1: adc::ADC1::steal(),
            #[cfg(any(esp32, esp32s2, esp32s3, esp32c3))]
            adc2: adc::ADC2::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
                feature = "pcnt-legacy"
            ))]
            pcnt0: pcnt::PCNT0::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
                feature = "pcnt-legacy"
            ))]
            pcnt1: pcnt::PCNT1::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
                feature = "pcnt-legacy"
            ))]
            pcnt2: pcnt::PCNT2::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32c61),
                feature = "pcnt-legacy"
            ))]
            pcnt3: pcnt::PCNT3::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(esp32, feature = "pcnt-legacy"))]
            pcnt4: pcnt::PCNT4::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(esp32, feature = "pcnt-legacy"))]
            pcnt5: pcnt::PCNT5::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(esp32, feature = "pcnt-legacy"))]
            pcnt6: pcnt::PCNT6::steal(),
            #[cfg(not(esp_idf_version_at_least_6_0_0))]
            #[cfg(all(esp32, feature = "pcnt-legacy"))]
            pcnt7: pcnt::PCNT7::steal(),
            can: can::CAN::steal(),
            ledc: ledc::LEDC::new(),
            #[cfg(esp32)]
            hledc: ledc::HLEDC::new(),
            #[cfg(feature = "rmt-legacy")]
            rmt: rmt_legacy::RMT::new(),
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c5, esp32c6, esp32p4),
                esp_idf_comp_ulp_enabled
            ))]
            ulp: ulp::ULP::steal(),
            #[cfg(any(all(esp32, esp_idf_eth_use_esp32_emac), esp_idf_eth_use_openeth))]
            mac: mac::MAC::steal(),
            modem: modem::Modem::steal(),
            #[cfg(esp_idf_soc_sdmmc_host_supported)]
            sdmmc0: sd::mmc::SDMMC0::steal(),
            #[cfg(esp_idf_soc_sdmmc_host_supported)]
            sdmmc1: sd::mmc::SDMMC1::steal(),
            #[cfg(all(esp_idf_soc_temp_sensor_supported, esp_idf_version_major = "5"))]
            temp_sensor: temp_sensor::TempSensor::steal(),
            #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
            timer00: timer::TIMER00::steal(),
            #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
            #[cfg(any(esp32, esp32s2, esp32s3))]
            timer01: timer::TIMER01::steal(),
            #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
            #[cfg(not(esp32c2))]
            timer10: timer::TIMER10::steal(),
            #[cfg(all(feature = "timer-legacy", not(esp_idf_version_at_least_6_0_0)))]
            #[cfg(any(esp32, esp32s2, esp32s3))]
            timer11: timer::TIMER11::steal(),
            #[cfg(any(
                all(
                    not(any(esp_idf_version_major = "4", esp_idf_version = "5.0")),
                    esp_idf_esp_task_wdt_en
                ),
                any(esp_idf_version_major = "4", esp_idf_version = "5.0")
            ))]
            twdt: watchdog::TWDT::steal(),
            #[cfg(esp_idf_soc_usb_serial_jtag_supported)]
            usb_serial: usb_serial::USB_SERIAL::steal(),
        }
    }
}
