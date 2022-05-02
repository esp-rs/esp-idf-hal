use core::marker::PhantomData;

#[cfg(not(feature = "riscv-ulp-hal"))]
use esp_idf_sys::*;

#[cfg(feature = "riscv-ulp-hal")]
use crate::riscv_ulp_hal::sys::*;

#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
use crate::hall;

pub trait Adc: Send {
    fn unit() -> adc_unit_t;
}

pub trait Analog<ADC: Adc>: Send {
    fn attenuation() -> adc_atten_t;
}

pub struct Atten0dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten2p5dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten6dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

pub struct Atten11dB<ADC: Adc> {
    _adc: PhantomData<ADC>,
}

impl<ADC: Adc> Analog<ADC> for Atten0dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_0
    }
}

impl<ADC: Adc> Analog<ADC> for Atten2p5dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_2_5
    }
}

impl<ADC: Adc> Analog<ADC> for Atten6dB<ADC> {
    fn attenuation() -> adc_atten_t {
        adc_atten_t_ADC_ATTEN_DB_6
    }
}

impl<ADC: Adc> Analog<ADC> for Atten11dB<ADC> {
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
        #[cfg(any(esp32, esp32c3, esp32s3))]
        Resolution12Bit,
        #[cfg(esp32s2)]
        Resolution13Bit,
    }

    impl Default for Resolution {
        #[cfg(any(esp32, esp32c3, esp32s3))]
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
                #[cfg(any(esp32, esp32s3, esp32c3))]
                Resolution::Resolution12Bit => adc_bits_width_t_ADC_WIDTH_BIT_12,
                #[cfg(esp32s2)]
                Resolution::Resolution13Bit => adc_bits_width_t_ADC_WIDTH_BIT_13,
            }
        }
    }

    #[derive(Debug, Copy, Clone, Default)]
    pub struct Config {
        pub resolution: Resolution,
        #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
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

        #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
        #[must_use]
        pub fn calibration(mut self, calibration: bool) -> Self {
            self.calibration = calibration;
            self
        }
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
pub struct PoweredAdc<ADC: Adc> {
    adc: ADC,
    resolution: config::Resolution,
    #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
    cal_characteristics:
        Option<[Option<esp_adc_cal_characteristics_t>; adc_atten_t_ADC_ATTEN_MAX as usize + 1]>,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
unsafe impl<ADC: Adc> Send for PoweredAdc<ADC> {}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<ADC: Adc> PoweredAdc<ADC> {
    #[cfg(all(esp32, esp_idf_comp_esp_adc_cal_enabled))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_VREF;

    #[cfg(all(any(esp32c3, esp32s2), esp_idf_comp_esp_adc_cal_enabled))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP;

    #[cfg(all(esp32s3, esp_idf_comp_esp_adc_cal_enabled))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t =
        esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP_FIT;

    #[cfg(not(esp32s2))]
    const MAX_READING: u32 = 4095;

    #[cfg(esp32s2)]
    const MAX_READING: u32 = 8191;

    pub fn new(adc: ADC, config: config::Config) -> Result<Self, EspError> {
        #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
        if config.calibration {
            esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;
        }

        if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_width(config.resolution.into()) })?;
        }

        Ok(Self {
            adc,
            resolution: config.resolution,
            #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
            cal_characteristics: if config.calibration {
                Some(Default::default())
            } else {
                None
            },
        })
    }

    pub fn release(self) -> ADC {
        self.adc
    }

    fn raw_to_voltage(
        &mut self,
        measurement: c_types::c_int,
        attenuation: adc_atten_t,
    ) -> Result<u16, EspError> {
        #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
        let mv = if let Some(cal) = self.get_cal_characteristics(attenuation)? {
            unsafe { esp_adc_cal_raw_to_voltage(measurement as u32, &cal as *const _) as u16 }
        } else {
            (measurement as u32 * Self::get_max_mv(attenuation) / Self::MAX_READING) as u16
        };

        #[cfg(not(esp_idf_comp_esp_adc_cal_enabled))]
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

        #[cfg(any(esp32c3, esp32s2))]
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

    #[cfg(esp_idf_comp_esp_adc_cal_enabled)]
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
                        &mut cal as *mut _,
                    )
                };

                characteristics[attenuation as usize] = Some(cal);

                Ok(Some(cal))
            }
        } else {
            Ok(None)
        }
    }

    fn read(
        &mut self,
        unit: adc_unit_t,
        channel: adc_channel_t,
        atten: adc_atten_t,
    ) -> nb::Result<u16, EspError> {
        let mut measurement = 0_i32;

        if unit == adc_unit_t_ADC_UNIT_1 {
            measurement = unsafe { adc1_get_raw(channel) };
        } else {
            let res = unsafe {
                adc2_get_raw(channel, self.resolution.into(), &mut measurement as *mut _)
            };

            if res == ESP_ERR_INVALID_STATE as i32 {
                return Err(nb::Error::WouldBlock);
            } else if res < 0 {
                return Err(nb::Error::Other(EspError::from(res).unwrap()));
            }
        };

        Ok(self.raw_to_voltage(measurement, atten)?)
    }

    #[cfg(esp32)]
    fn read_hall(&mut self) -> nb::Result<u16, EspError> {
        let measurement = unsafe { hall_sensor_read() };

        Ok(self.raw_to_voltage(measurement, adc_atten_t_ADC_ATTEN_DB_0)?)
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<ADC, AN, PIN> embedded_hal_0_2::adc::OneShot<AN, u16, PIN> for PoweredAdc<ADC>
where
    ADC: Adc,
    AN: Analog<ADC>,
    PIN: embedded_hal_0_2::adc::Channel<AN, ID = u8>,
{
    type Error = EspError;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        self.read(
            ADC::unit(),
            PIN::channel() as adc_channel_t,
            AN::attenuation(),
        )
    }
}

#[cfg(all(esp32, not(feature = "riscv-ulp-hal")))]
impl embedded_hal_0_2::adc::OneShot<ADC1, u16, hall::HallSensor> for PoweredAdc<ADC1> {
    type Error = EspError;

    fn read(&mut self, _hall_sensor: &mut hall::HallSensor) -> nb::Result<u16, Self::Error> {
        self.read_hall()
    }
}

macro_rules! impl_adc {
    ($adc:ident: $unit:expr) => {
        pub struct $adc(::core::marker::PhantomData<*const ()>);

        impl $adc {
            /// # Safety
            ///
            /// Care should be taken not to instnatiate this ADC instance, if it is already instantiated and used elsewhere
            pub unsafe fn new() -> Self {
                $adc(::core::marker::PhantomData)
            }
        }

        unsafe impl Send for $adc {}

        impl Adc for $adc {
            #[inline(always)]
            fn unit() -> adc_unit_t {
                $unit
            }
        }
    };
}

impl_adc!(ADC1: adc_unit_t_ADC_UNIT_1);
impl_adc!(ADC2: adc_unit_t_ADC_UNIT_2);
