use core::marker::PhantomData;

#[cfg(not(feature = "ulp"))]
use esp_idf_sys::*;

#[cfg(feature = "ulp")]
use crate::ulp::sys::*;

#[cfg(all(esp32, not(feature = "ulp")))]
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
#[cfg(not(feature = "ulp"))]
pub mod config {
    use esp_idf_sys::*;

    /// The sampling/readout resolution of the ADC
    #[derive(PartialEq, Eq, Clone, Copy)]
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

    #[derive(Copy, Clone)]
    pub struct Config {
        pub resolution: Resolution,
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

        #[must_use]
        pub fn calibration(mut self, calibration: bool) -> Self {
            self.calibration = calibration;
            self
        }
    }

    impl Default for Config {
        fn default() -> Self {
            Self {
                resolution: Default::default(),
                calibration: false,
            }
        }
    }
}

#[cfg(not(feature = "ulp"))]
pub struct PoweredAdc<ADC: Adc> {
    adc: ADC,
    resolution: config::Resolution,
    cal_characteristics:
        Option<[Option<esp_adc_cal_characteristics_t>; adc_atten_t_ADC_ATTEN_MAX as usize + 1]>,
}

unsafe impl<ADC: Adc> Send for PoweredAdc<ADC> {}

#[cfg(not(feature = "ulp"))]
impl<ADC: Adc> PoweredAdc<ADC> {
    #[cfg(esp32)]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_VREF;

    #[cfg(any(esp32c3, esp32s2))]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t = esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP;

    #[cfg(esp32s3)]
    const CALIBRATION_SCHEME: esp_adc_cal_value_t =
        esp_adc_cal_value_t_ESP_ADC_CAL_VAL_EFUSE_TP_FIT;

    #[cfg(not(esp32s2))]
    const MAX_READING: u32 = 4095;

    #[cfg(esp32s2)]
    const MAX_READING: u32 = 8191;

    pub fn new(adc: ADC, config: config::Config) -> Result<Self, EspError> {
        if config.calibration {
            esp!(unsafe { esp_adc_cal_check_efuse(Self::CALIBRATION_SCHEME) })?;
        }

        if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_width(config.resolution.into()) })?;
        }

        Ok(Self {
            adc,
            resolution: config.resolution,
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
        let mv = if let Some(cal) = self.get_cal_characteristics(attenuation)? {
            unsafe { esp_adc_cal_raw_to_voltage(measurement as u32, &cal as *const _) as u16 }
        } else {
            (measurement as u32 * Self::get_max_mv(attenuation) / Self::MAX_READING) as u16
        };

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
}

#[cfg(not(feature = "ulp"))]
impl<ADC, AN, PIN> embedded_hal::adc::OneShot<AN, u16, PIN> for PoweredAdc<ADC>
where
    ADC: Adc,
    AN: Analog<ADC>,
    PIN: embedded_hal::adc::Channel<AN, ID = u8>,
{
    type Error = EspError;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        let mut measurement = 0 as c_types::c_int;

        if ADC::unit() == adc_unit_t_ADC_UNIT_1 {
            measurement = unsafe { adc1_get_raw(PIN::channel() as adc_channel_t) };
        } else {
            let res = unsafe {
                adc2_get_raw(
                    PIN::channel() as adc_channel_t,
                    self.resolution.into(),
                    &mut measurement as *mut _,
                )
            };

            if res == ESP_ERR_INVALID_STATE as i32 {
                return Err(nb::Error::WouldBlock);
            } else if res < 0 {
                return Err(nb::Error::Other(EspError::from(res).unwrap()));
            }
        };

        Ok(self.raw_to_voltage(measurement, AN::attenuation())?)
    }
}

#[cfg(all(esp32, not(feature = "ulp")))]
impl embedded_hal::adc::OneShot<ADC1, u16, hall::HallSensor> for PoweredAdc<ADC1> {
    type Error = EspError;

    fn read(&mut self, _hall_sensor: &mut hall::HallSensor) -> nb::Result<u16, Self::Error> {
        let measurement = unsafe { hall_sensor_read() };

        Ok(self.raw_to_voltage(measurement, adc_atten_t_ADC_ATTEN_DB_0)?)
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
