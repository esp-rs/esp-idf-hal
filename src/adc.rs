use core::marker::PhantomData;

#[cfg(not(feature = "riscv-ulp-hal"))]
use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::sys::*;

#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::gpio::ADCPin;

#[cfg(not(feature = "riscv-ulp-hal"))]
use crate::peripheral::{Peripheral, PeripheralRef};

#[cfg(not(feature = "riscv-ulp-hal"))]
pub type AdcConfig = config::Config;

pub trait Adc: Send {
    fn unit() -> adc_unit_t;
}

pub trait Attenuation<ADC: Adc>: Send {
    fn attenuation() -> adc_atten_t;
}

pub struct Atten0dB<ADC: Adc>(PhantomData<ADC>);
pub struct Atten2p5dB<ADC: Adc>(PhantomData<ADC>);
pub struct Atten6dB<ADC: Adc>(PhantomData<ADC>);
pub struct Atten11dB<ADC: Adc>(PhantomData<ADC>);

impl<ADC: Adc> Attenuation<ADC> for Atten0dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_0
    }
}

impl<ADC: Adc> Attenuation<ADC> for Atten2p5dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_2_5
    }
}

impl<ADC: Adc> Attenuation<ADC> for Atten6dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_6
    }
}

impl<ADC: Adc> Attenuation<ADC> for Atten11dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_11
    }
}

/// ADC configuration
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod config {
    use esp_idf_sys::*;

    /// The sampling/readout resolution of the ADC
    #[derive(Debug, PartialEq, Eq, Clone, Copy)]
    pub enum Resolution {
        #[cfg(esp32)]
        Resolution9Bit,
        #[cfg(esp32)]
        Resolution10Bit,
        #[cfg(esp32)]
        Resolution11Bit,
        #[cfg(any(esp32, esp32c3, esp32s3, esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
        Resolution12Bit,
        #[cfg(esp32s2)]
        Resolution13Bit,
    }

    impl Default for Resolution {
        #[cfg(any(esp32, esp32c3, esp32s3, esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
        fn default() -> Self {
            Self::Resolution12Bit
        }

        #[cfg(esp32s2)]
        fn default() -> Self {
            Self::Resolution13Bit
        }
    }

    impl From<Resolution> for adc_bits_width_t {
        fn from(resolution: Resolution) -> Self {
            match resolution {
                #[cfg(esp32)]
                Resolution::Resolution9Bit => adc_bits_width_t_ADC_WIDTH_BIT_9,
                #[cfg(esp32)]
                Resolution::Resolution10Bit => adc_bits_width_t_ADC_WIDTH_BIT_10,
                #[cfg(esp32)]
                Resolution::Resolution11Bit => adc_bits_width_t_ADC_WIDTH_BIT_11,
                #[cfg(any(esp32, esp32s3, esp32c3, esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
                Resolution::Resolution12Bit => adc_bits_width_t_ADC_WIDTH_BIT_12,
                #[cfg(esp32s2)]
                Resolution::Resolution13Bit => adc_bits_width_t_ADC_WIDTH_BIT_13,
            }
        }
    }

    #[derive(Debug, Copy, Clone, Default)]
    pub struct Config {
        pub resolution: Resolution,
        #[cfg(any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled))]
        pub calibration: bool,
    }

    impl Config {
        pub fn new() -> Self {
            Default::default()
        }

        #[must_use]
        pub fn resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }

        #[cfg(any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled))]
        #[must_use]
        pub fn calibration(mut self, calibration: bool) -> Self {
            self.calibration = calibration;
            self
        }
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
pub struct AdcChannelDriver<'d, T: ADCPin, ATTEN> {
    pin: PeripheralRef<'d, T>,
    _atten: PhantomData<ATTEN>,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, T: ADCPin, ATTEN> AdcChannelDriver<'d, T, ATTEN>
where
    ATTEN: Attenuation<T::Adc>,
{
    #[inline]
    pub fn new(
        pin: impl Peripheral<P = T> + 'd,
    ) -> Result<AdcChannelDriver<'d, T, ATTEN>, EspError> {
        crate::into_ref!(pin);

        unsafe {
            crate::gpio::rtc_reset_pin(pin.pin())?;
        }

        if T::Adc::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_channel_atten(pin.adc_channel(), ATTEN::attenuation()) })?;
        } else {
            #[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))]
            esp!(unsafe { adc2_config_channel_atten(pin.adc_channel(), ATTEN::attenuation()) })?;

            #[cfg(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
            unreachable!();
        }

        Ok(Self {
            pin,
            _atten: PhantomData,
        })
    }

    fn pin(&mut self) -> &mut PeripheralRef<'d, T> {
        &mut self.pin
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, T: ADCPin, ATTEN> embedded_hal_0_2::adc::Channel<ATTEN>
    for AdcChannelDriver<'d, T, ATTEN>
{
    type ID = u8;

    fn channel() -> Self::ID {
        T::CHANNEL as _
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
pub struct AdcDriver<'d, ADC: Adc> {
    _adc: PeripheralRef<'d, ADC>,
    #[allow(dead_code)]
    resolution: config::Resolution,
    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c3),
        any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
    ))]
    cal_characteristics:
        Option<[Option<esp_adc_cal_characteristics_t>; adc_atten_t_ADC_ATTEN_DB_11 as usize + 1]>,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
unsafe impl<'d, ADC: Adc> Send for AdcDriver<'d, ADC> {}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, ADC: Adc> AdcDriver<'d, ADC> {
    #[cfg(all(
        esp32,
        any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
    ))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_VREF;

    #[cfg(all(
        any(esp32c3, esp32s2),
        any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
    ))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP;

    #[cfg(all(
        esp32s3,
        any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
    ))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t =
        esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP_FIT;

    #[cfg(not(esp32s2))]
    const MAX_READING: u32 = 4095;

    #[cfg(esp32s2)]
    const MAX_READING: u32 = 8191;

    pub fn new(
        adc: impl Peripheral<P = ADC> + 'd,
        config: &config::Config,
    ) -> Result<Self, EspError> {
        crate::into_ref!(adc);

        #[cfg(all(
            any(esp32, esp32s2, esp32s3, esp32c3),
            any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
        ))]
        if config.calibration {
            esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;
        }

        if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_width(config.resolution.into()) })?;
        }

        Ok(Self {
            _adc: adc,
            resolution: config.resolution,
            #[cfg(all(
                any(esp32, esp32s2, esp32s3, esp32c3),
                any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
            ))]
            cal_characteristics: if config.calibration {
                Some(Default::default())
            } else {
                None
            },
        })
    }

    pub fn read<T, ATTEN>(
        &mut self,
        pin: &mut AdcChannelDriver<'_, T, ATTEN>,
    ) -> Result<u16, EspError>
    where
        T: ADCPin,
        ATTEN: Attenuation<T::Adc>,
    {
        self.read_internal(ADC::unit(), pin.pin().adc_channel(), ATTEN::attenuation())
    }

    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub fn read_hall(
        &mut self,
        _hall_sensor: &mut crate::hall::HallSensor,
    ) -> Result<u16, EspError> {
        let measurement = unsafe { hall_sensor_read() };

        self.raw_to_voltage(measurement, adc_atten_t_ADC_ATTEN_DB_0)
    }

    fn read_internal(
        &mut self,
        unit: adc_unit_t,
        channel: adc_channel_t,
        atten: adc_atten_t,
    ) -> Result<u16, EspError> {
        #[allow(unused_assignments)]
        let mut measurement = 0_i32;

        if unit == adc_unit_t_ADC_UNIT_1 {
            measurement = unsafe { adc1_get_raw(channel) };
        } else {
            #[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))]
            esp!(unsafe { adc2_get_raw(channel, self.resolution.into(), &mut measurement) })?;

            #[cfg(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
            unreachable!();
        };

        self.raw_to_voltage(measurement, atten)
    }

    fn raw_to_voltage(
        &mut self,
        measurement: core::ffi::c_int,
        attenuation: adc_atten_t,
    ) -> Result<u16, EspError> {
        #[cfg(all(
            any(esp32, esp32s2, esp32s3, esp32c3),
            any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
        ))]
        let mv = if let Some(cal) = self.get_cal_characteristics(attenuation)? {
            unsafe { esp_adc_cal_raw_to_voltage(measurement as u32, &cal) as u16 }
        } else {
            (measurement as u32 * Self::get_max_mv(attenuation) / Self::MAX_READING) as u16
        };

        #[cfg(not(all(
            any(esp32, esp32s2, esp32s3, esp32c3),
            any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
        )))]
        let mv = (measurement as u32 * Self::get_max_mv(attenuation) / Self::MAX_READING) as u16;

        Ok(mv)
    }

    #[allow(non_upper_case_globals)]
    fn get_max_mv(attenuation: adc_atten_t) -> u32 {
        #[cfg(esp32)]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 950,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1250,
            adc_atten_t_ADC_ATTEN_DB_6 => 1750,
            adc_atten_t_ADC_ATTEN_DB_11 => 2450,
            other => panic!("Unknown attenuation: {}", other),
        };

        #[cfg(any(esp32c3, esp32s2, esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 750,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1050,
            adc_atten_t_ADC_ATTEN_DB_6 => 1300,
            adc_atten_t_ADC_ATTEN_DB_11 => 2500,
            other => panic!("Unknown attenuation: {}", other),
        };

        #[cfg(esp32s3)]
        let mv = match attenuation {
            adc_atten_t_ADC_ATTEN_DB_0 => 950,
            adc_atten_t_ADC_ATTEN_DB_2_5 => 1250,
            adc_atten_t_ADC_ATTEN_DB_6 => 1750,
            adc_atten_t_ADC_ATTEN_DB_11 => 3100,
            other => panic!("Unknown attenuation: {}", other),
        };

        mv
    }

    #[cfg(all(
        any(esp32, esp32s2, esp32s3, esp32c3),
        any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled)
    ))]
    fn get_cal_characteristics(
        &mut self,
        attenuation: adc_atten_t,
    ) -> Result<Option<esp_adc_cal_characteristics_t>, EspError> {
        if let Some(characteristics) = &mut self.cal_characteristics {
            if let Some(cal) = characteristics[attenuation as usize] {
                Ok(Some(cal))
            } else {
                esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;

                let mut cal: esp_adc_cal_characteristics_t = Default::default();
                unsafe {
                    esp_adc_cal_characterize(
                        ADC::unit(),
                        attenuation,
                        self.resolution.into(),
                        0,
                        &mut cal,
                    )
                };

                characteristics[attenuation as usize] = Some(cal);

                Ok(Some(cal))
            }
        } else {
            Ok(None)
        }
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, ADC, ATTEN, PIN> embedded_hal_0_2::adc::OneShot<ATTEN, u16, PIN> for AdcDriver<'d, ADC>
where
    ADC: Adc,
    ATTEN: Attenuation<ADC>,
    PIN: embedded_hal_0_2::adc::Channel<ATTEN, ID = u8>,
{
    type Error = EspError;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        self.read_internal(
            ADC::unit(),
            PIN::channel() as adc_channel_t,
            ATTEN::attenuation(),
        )
        .map_err(to_nb_err)
    }
}

#[cfg(all(esp32, esp_idf_version_major = "4", not(feature = "riscv-ulp-hal")))]
impl<'d> embedded_hal_0_2::adc::OneShot<ADC1, u16, crate::hall::HallSensor>
    for AdcDriver<'d, ADC1>
{
    type Error = EspError;

    fn read(&mut self, hall_sensor: &mut crate::hall::HallSensor) -> nb::Result<u16, Self::Error> {
        AdcDriver::read_hall(self, hall_sensor).map_err(to_nb_err)
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
fn to_nb_err(err: EspError) -> nb::Error<EspError> {
    if err.code() == ESP_ERR_INVALID_STATE {
        nb::Error::WouldBlock
    } else {
        nb::Error::Other(err)
    }
}

macro_rules! impl_adc {
    ($adc:ident: $unit:expr) => {
        crate::impl_peripheral!($adc);

        impl Adc for $adc {
            #[inline(always)]
            fn unit() -> adc_unit_t {
                $unit
            }
        }
    };
}

impl_adc!(ADC1: adc_unit_t_ADC_UNIT_1);
#[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))] // TODO: CVheck for esp32c5 and esp32p4
impl_adc!(ADC2: adc_unit_t_ADC_UNIT_2);
