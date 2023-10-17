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

#[cfg(all(
    not(feature = "riscv-ulp-hal"),
    not(esp_idf_version_major = "4"),
    not(esp32c2)
))]
pub use continuous::{
    config as cont_config, config::Config as AdcContConfig, AdcChannels, AdcChannelsArray,
    AdcDriver as AdcContDriver, AdcMeasurement, Atten11dB, Atten2p5dB, Atten6dB, AttenNone,
    Attenuated, ChainedAdcChannels, EmptyAdcChannels,
};

pub trait Adc: Send {
    fn unit() -> adc_unit_t;
}

// NOTE: Will be changed to an enum once C-style enums are usable as const generics
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod attenuation {
    pub use esp_idf_sys::{
        adc_atten_t, adc_atten_t_ADC_ATTEN_DB_0, adc_atten_t_ADC_ATTEN_DB_11,
        adc_atten_t_ADC_ATTEN_DB_2_5, adc_atten_t_ADC_ATTEN_DB_6,
    };

    pub const NONE: adc_atten_t = adc_atten_t_ADC_ATTEN_DB_0;
    pub const DB_2_5: adc_atten_t = adc_atten_t_ADC_ATTEN_DB_2_5;
    pub const DB_6: adc_atten_t = adc_atten_t_ADC_ATTEN_DB_6;
    pub const DB_11: adc_atten_t = adc_atten_t_ADC_ATTEN_DB_11;
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
pub struct AdcChannelDriver<'d, const A: adc_atten_t, T: ADCPin> {
    pin: PeripheralRef<'d, T>,
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, const A: adc_atten_t, T: ADCPin> AdcChannelDriver<'d, A, T> {
    pub fn new(pin: impl Peripheral<P = T> + 'd) -> Result<Self, EspError> {
        crate::into_ref!(pin);

        unsafe {
            crate::gpio::rtc_reset_pin(pin.pin())?;
        }

        if T::Adc::unit() == adc_unit_t_ADC_UNIT_1 {
            esp!(unsafe { adc1_config_channel_atten(pin.adc_channel(), A) })?;
        } else {
            #[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))]
            esp!(unsafe { adc2_config_channel_atten(pin.adc_channel(), A) })?;

            #[cfg(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
            unreachable!();
        }

        Ok(Self { pin })
    }

    fn pin(&mut self) -> &mut PeripheralRef<'d, T> {
        &mut self.pin
    }
}

#[cfg(not(feature = "riscv-ulp-hal"))]
impl<'d, const A: adc_atten_t, T: ADCPin> embedded_hal_0_2::adc::Channel<T::Adc>
    for AdcChannelDriver<'d, A, T>
{
    type ID = (adc_channel_t, adc_atten_t);

    fn channel() -> Self::ID {
        (T::CHANNEL, A)
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

    #[inline(always)]
    pub fn read<const A: adc_atten_t, T>(
        &mut self,
        pin: &mut AdcChannelDriver<'_, A, T>,
    ) -> Result<u16, EspError>
    where
        T: ADCPin<Adc = ADC>,
    {
        self.read_internal(ADC::unit(), pin.pin().adc_channel(), A)
    }

    #[inline(always)]
    pub fn read_raw<const A: adc_atten_t, T>(
        &mut self,
        pin: &mut AdcChannelDriver<'_, A, T>,
    ) -> Result<u16, EspError>
    where
        T: ADCPin<Adc = ADC>,
    {
        self.read_internal_raw(ADC::unit(), pin.pin().adc_channel())
    }

    #[inline(always)]
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub fn read_hall(
        &mut self,
        hall_sensor: &mut crate::hall::HallSensor,
    ) -> Result<u16, EspError> {
        let measurement = self.read_hall_raw(hall_sensor);

        self.raw_to_voltage(measurement, adc_atten_t_ADC_ATTEN_DB_0)
    }

    #[inline(always)]
    #[cfg(all(esp32, esp_idf_version_major = "4"))]
    pub fn read_hall_raw(&mut self, _hall_sensor: &mut crate::hall::HallSensor) -> u16 {
        unsafe { hall_sensor_read() as u16 }
    }

    #[inline(always)]
    fn read_internal(
        &mut self,
        unit: adc_unit_t,
        channel: adc_channel_t,
        atten: adc_atten_t,
    ) -> Result<u16, EspError> {
        let measurement = self.read_internal_raw(unit, channel)?;
        self.raw_to_voltage(measurement, atten)
    }

    #[inline(always)]
    fn read_internal_raw(
        &mut self,
        unit: adc_unit_t,
        channel: adc_channel_t,
    ) -> Result<u16, EspError> {
        if unit == adc_unit_t_ADC_UNIT_1 {
            Ok(unsafe { adc1_get_raw(channel) } as _)
        } else {
            #[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))]
            {
                let mut measurement = 0;
                esp!(unsafe { adc2_get_raw(channel, self.resolution.into(), &mut measurement) })?;

                Ok(measurement as _)
            }

            #[cfg(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))]
            unreachable!();
        }
    }

    #[inline(always)]
    fn raw_to_voltage(
        &mut self,
        measurement: u16,
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

    #[inline(always)]
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
impl<'d, 'c, const A: adc_atten_t, T>
    embedded_hal_0_2::adc::OneShot<T::Adc, u16, AdcChannelDriver<'c, A, T>>
    for AdcDriver<'d, T::Adc>
where
    T: ADCPin,
{
    type Error = EspError;

    fn read(&mut self, pin: &mut AdcChannelDriver<'c, A, T>) -> nb::Result<u16, Self::Error> {
        self.read_internal(T::Adc::unit(), pin.pin.adc_channel(), A)
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
#[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))] // TODO: Check for esp32c5 and esp32p4
impl_adc!(ADC2: adc_unit_t_ADC_UNIT_2);

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp_idf_version_major = "4")))]
pub mod oneshot {
    use core::borrow::Borrow;

    use esp_idf_sys::*;

    use crate::gpio::ADCPin;
    use crate::peripheral::Peripheral;
    use crate::peripheral::PeripheralRef;

    use super::attenuation::adc_atten_t;
    use super::config::Resolution;
    use super::to_nb_err;
    use super::Adc;

    pub mod config {
        use super::adc_atten_t;
        use super::Resolution;

        #[derive(Debug, Copy, Clone, Default)]
        pub struct AdcChannelConfig {
            pub attenuation: adc_atten_t,
            pub resolution: Resolution,
            #[cfg(any(esp_idf_comp_esp_adc_cal_enabled, esp_idf_comp_esp_adc_enabled))]
            pub calibration: bool,
        }

        impl AdcChannelConfig {
            pub fn new() -> Self {
                Default::default()
            }
        }
    }

    pub struct AdcChannelDriver<'d, T, M>
    where
        T: ADCPin,
        M: Borrow<AdcDriver<'d, T::Adc>>,
    {
        adc: M,
        _pin: PeripheralRef<'d, T>,
        calibration: Option<adc_cali_handle_t>,
    }

    impl<'d, T, M> AdcChannelDriver<'d, T, M>
    where
        T: ADCPin,
        M: Borrow<AdcDriver<'d, T::Adc>>,
    {
        pub fn new(
            adc: M,
            pin: impl Peripheral<P = T> + 'd,
            config: &config::AdcChannelConfig,
        ) -> Result<Self, EspError> {
            crate::into_ref!(pin);

            unsafe {
                crate::gpio::rtc_reset_pin(pin.pin())?;
            }

            let chan_config = adc_oneshot_chan_cfg_t {
                atten: config.attenuation,
                bitwidth: config.resolution.into(),
            };

            unsafe {
                esp!(adc_oneshot_config_channel(
                    adc.borrow().handle,
                    pin.adc_channel(),
                    &chan_config
                ))?
            };

            let mut calibration = Self::get_curve_calibration_handle(
                T::Adc::unit() as u8,
                pin.adc_channel(),
                config.attenuation,
                config.resolution.into(),
            );
            if calibration.is_none() {
                calibration = Self::get_line_calibration_handle(
                    T::Adc::unit() as u8,
                    config.attenuation,
                    config.resolution.into(),
                );
            }
            Ok(Self {
                adc,
                _pin: pin,
                calibration,
            })
        }

        #[allow(unused_variables)]
        fn get_curve_calibration_handle(
            unit_id: u8,
            chan: adc_channel_t,
            atten: adc_atten_t,
            bitwidth: adc_bits_width_t,
        ) -> Option<adc_cali_handle_t> {
            // it would be nice if esp-idf-sys could export some cfg values to replicate these two defines
            // from esp-idf:
            // ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            // ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
            // then we wouuld not need the uglyness for the esp32c6
            #[cfg(any(
                esp32c3,
                all(
                    esp32c6,
                    not(all(esp_idf_version_major = "5", esp_idf_version_minor = "0")),
                    not(esp_idf_version_full = "5.1.0")
                ),
                esp32s3,
            ))]
            {
                let cal_config = adc_cali_curve_fitting_config_t {
                    unit_id: unit_id as u32,
                    #[cfg(all(
                        esp_idf_version_major = "5",
                        not(esp_idf_version_minor = "0"),
                        not(all(esp_idf_version_minor = "1", esp_idf_version_patch = "0"))
                    ))]
                    chan,
                    atten,
                    bitwidth,
                };
                let mut cal_handle: adc_cali_handle_t = core::ptr::null_mut();
                if let Err(_err) = unsafe {
                    esp!(esp_idf_sys::adc_cali_create_scheme_curve_fitting(
                        &cal_config,
                        &mut cal_handle
                    ))
                } {
                    // I'd log a warning but the log crate is not available here
                    None
                } else {
                    Some(cal_handle)
                }
            }
            #[cfg(not(any(
                esp32c3,
                all(
                    esp32c6,
                    not(all(esp_idf_version_major = "5", esp_idf_version_minor = "0")),
                    not(esp_idf_version_full = "5.1.0")
                ),
                esp32s3,
            )))]
            None
        }

        #[allow(unused_variables)]
        fn get_line_calibration_handle(
            unit_id: u8,
            atten: adc_atten_t,
            bitwidth: adc_bits_width_t,
        ) -> Option<adc_cali_handle_t> {
            #[cfg(any(esp32, esp32c2, esp32s2))]
            {
                // esp32 has an additional field that the exanple defalts
                // to using fuse values for vref. Maybe we should expose
                // this as a config option?
                #[allow(clippy::needless_update)]
                let cal_config = adc_cali_line_fitting_config_t {
                    unit_id: unit_id as u32,
                    atten,
                    bitwidth,
                    ..Default::default()
                };
                let mut cal_handle: adc_cali_handle_t = core::ptr::null_mut();
                if let Err(_err) = unsafe {
                    esp!(esp_idf_sys::adc_cali_create_scheme_line_fitting(
                        &cal_config,
                        &mut cal_handle
                    ))
                } {
                    // I'd log a warning but the log crate is not available here
                    None
                } else {
                    Some(cal_handle)
                }
            }
            #[cfg(not(any(esp32, esp32c2, esp32s2)))]
            None
        }

        #[inline(always)]
        pub fn read(&mut self) -> Result<u16, EspError> {
            let raw = self.read_raw()?;
            self.raw_to_cal(raw)
        }

        #[inline(always)]
        pub fn read_raw(&mut self) -> Result<u16, EspError> {
            let channel = T::CHANNEL;
            self.adc.borrow().read_raw_internal(channel)
        }

        #[inline(always)]
        pub fn raw_to_cal(&self, raw: u16) -> Result<u16, EspError> {
            if let Some(calibration) = &self.calibration {
                self.adc.borrow().raw_to_cal_internal(*calibration, raw)
            } else {
                Ok(raw)
            }
        }
    }

    impl<'d, T, M> embedded_hal_0_2::adc::Channel<T::Adc> for AdcChannelDriver<'d, T, M>
    where
        T: ADCPin,
        M: Borrow<AdcDriver<'d, T::Adc>>,
    {
        type ID = adc_channel_t;

        fn channel() -> Self::ID {
            T::CHANNEL
        }
    }

    unsafe impl<'d, T, M> Send for AdcChannelDriver<'d, T, M>
    where
        T: ADCPin,
        M: Borrow<AdcDriver<'d, T::Adc>>,
    {
    }

    pub struct AdcDriver<'d, ADC: Adc> {
        handle: adc_oneshot_unit_handle_t,
        _adc: PeripheralRef<'d, ADC>,
    }

    impl<'d, ADC: Adc> AdcDriver<'d, ADC> {
        pub fn new(adc: impl Peripheral<P = ADC> + 'd) -> Result<Self, EspError> {
            crate::into_ref!(adc);
            let config = adc_oneshot_unit_init_cfg_t {
                unit_id: ADC::unit(),
                ..Default::default()
            };
            let mut handle: adc_oneshot_unit_handle_t = core::ptr::null_mut();
            unsafe { esp!(adc_oneshot_new_unit(&config, &mut handle))? };
            Ok(Self { handle, _adc: adc })
        }

        #[inline(always)]
        pub fn read<T, M>(&self, channel: &mut AdcChannelDriver<'d, T, M>) -> Result<u16, EspError>
        where
            T: ADCPin,
            M: Borrow<AdcDriver<'d, T::Adc>>,
        {
            let raw = self.read_raw(channel)?;
            self.raw_to_cal(channel, raw)
        }

        #[inline(always)]
        pub fn read_raw<T, M>(
            &self,
            _channel: &mut AdcChannelDriver<'d, T, M>,
        ) -> Result<u16, EspError>
        where
            T: ADCPin,
            M: Borrow<AdcDriver<'d, T::Adc>>,
        {
            self.read_raw_internal(T::CHANNEL)
        }

        #[inline(always)]
        fn read_raw_internal(&self, channel: adc_channel_t) -> Result<u16, EspError> {
            let mut measurement = 0;
            unsafe { esp!(adc_oneshot_read(self.handle, channel, &mut measurement)) }?;
            Ok(measurement as u16)
        }

        #[inline(always)]
        pub fn raw_to_cal<T, M>(
            &self,
            channel: &AdcChannelDriver<'d, T, M>,
            raw: u16,
        ) -> Result<u16, EspError>
        where
            T: ADCPin,
            M: Borrow<AdcDriver<'d, T::Adc>>,
        {
            if let Some(calibration) = &channel.calibration {
                self.raw_to_cal_internal(*calibration, raw)
            } else {
                Ok(raw)
            }
        }

        #[inline(always)]
        fn raw_to_cal_internal(
            &self,
            calibration: adc_cali_handle_t,
            raw: u16,
        ) -> Result<u16, EspError> {
            let mut mv = 0i32;
            unsafe {
                esp!(adc_cali_raw_to_voltage(calibration, raw as i32, &mut mv))?;
            };
            Ok(mv as u16)
        }
    }

    impl<'d, ADC: Adc> Drop for AdcDriver<'d, ADC> {
        fn drop(&mut self) {
            unsafe { esp!(adc_oneshot_del_unit(self.handle)) }.unwrap();
        }
    }

    impl<'d, T, M> embedded_hal_0_2::adc::OneShot<T::Adc, u16, AdcChannelDriver<'d, T, M>>
        for AdcDriver<'d, T::Adc>
    where
        T: ADCPin,
        M: Borrow<AdcDriver<'d, T::Adc>>,
    {
        type Error = EspError;

        fn read(&mut self, pin: &mut AdcChannelDriver<'d, T, M>) -> nb::Result<u16, Self::Error> {
            AdcDriver::read(self, pin).map_err(to_nb_err)
        }
    }

    unsafe impl<'d, ADC: Adc> Send for AdcDriver<'d, ADC> {}
    unsafe impl<'d, ADC: Adc> Sync for AdcDriver<'d, ADC> {}
}

#[cfg(all(
    not(feature = "riscv-ulp-hal"),
    not(esp_idf_version_major = "4"),
    not(esp32c2)
))]
pub mod continuous {
    use core::ffi::c_void;
    use core::fmt::{self, Debug, Display};
    use core::marker::PhantomData;

    use esp_idf_sys::*;

    use crate::delay::{self, TickType};
    use crate::gpio::{sealed::ADCPin as _, ADCPin};
    use crate::interrupt::asynch::HalIsrNotification;
    use crate::io::EspIOError;
    use crate::peripheral::Peripheral;

    use super::{attenuation, Adc};

    pub struct Attenuated<const A: adc_atten_t, T>(T);

    impl<T> Attenuated<{ attenuation::NONE }, T> {
        pub const fn none(t: T) -> Self {
            Self(t)
        }
    }

    impl<T> Attenuated<{ attenuation::DB_2_5 }, T> {
        pub const fn db2_5(t: T) -> Self {
            Self(t)
        }
    }

    impl<T> Attenuated<{ attenuation::DB_6 }, T> {
        pub const fn db6(t: T) -> Self {
            Self(t)
        }
    }

    impl<T> Attenuated<{ attenuation::DB_11 }, T> {
        pub const fn db11(t: T) -> Self {
            Self(t)
        }
    }

    impl<const A: adc_atten_t, T> Attenuated<A, T> {
        pub fn atten(channel: (adc_channel_t, adc_atten_t)) -> (adc_channel_t, adc_atten_t) {
            (channel.0, A)
        }
    }

    pub type AttenNone<T> = Attenuated<{ attenuation::NONE }, T>;
    pub type Atten2p5dB<T> = Attenuated<{ attenuation::DB_2_5 }, T>;
    pub type Atten6dB<T> = Attenuated<{ attenuation::DB_6 }, T>;
    pub type Atten11dB<T> = Attenuated<{ attenuation::DB_11 }, T>;

    pub trait AdcChannels {
        type Adc: Adc;
        type Iterator<'a>: Iterator<Item = (adc_channel_t, adc_atten_t)>
        where
            Self: 'a;

        fn iter(&self) -> Self::Iterator<'_>;
    }

    impl<'d, P> AdcChannels for P
    where
        P: Peripheral,
        P::P: ADCPin + 'd,
    {
        type Adc = <<P as Peripheral>::P as ADCPin>::Adc;

        type Iterator<'a> = core::iter::Once<(adc_channel_t, adc_atten_t)> where Self: 'a;

        fn iter(&self) -> Self::Iterator<'_> {
            core::iter::once((P::P::CHANNEL, attenuation::NONE))
        }
    }

    impl<'d, const A: adc_atten_t, C> AdcChannels for Attenuated<A, C>
    where
        C: AdcChannels + 'd,
    {
        type Adc = C::Adc;

        type Iterator<'a> = core::iter::Map<
            C::Iterator<'a>,
            fn((adc_channel_t, adc_atten_t)) -> (adc_channel_t, adc_atten_t),
        > where Self: 'a;

        fn iter(&self) -> Self::Iterator<'_> {
            self.0.iter().map(Attenuated::<A, C>::atten)
        }
    }

    pub struct AdcChannelsArray<C, const N: usize>(pub [C; N]);

    impl<C, const N: usize> AdcChannels for AdcChannelsArray<C, N>
    where
        C: AdcChannels,
    {
        type Adc = C::Adc;

        type Iterator<'a> = core::iter::FlatMap<core::slice::Iter<'a, C>, <C as AdcChannels>::Iterator<'a>, fn(&'a C) -> C::Iterator<'a>> where Self: 'a;

        fn iter(&self) -> Self::Iterator<'_> {
            self.0.iter().flat_map(AdcChannels::iter)
        }
    }

    pub struct EmptyAdcChannels<A>(PhantomData<A>);

    impl<A> EmptyAdcChannels<A> {
        pub fn chain<O>(other: O) -> ChainedAdcChannels<Self, O>
        where
            A: Adc,
            O: AdcChannels<Adc = A>,
        {
            ChainedAdcChannels {
                first: Self(PhantomData),
                second: other,
            }
        }
    }

    impl<A> AdcChannels for EmptyAdcChannels<A>
    where
        A: Adc,
    {
        type Adc = A;

        type Iterator<'a> = core::iter::Empty<(adc_channel_t, adc_atten_t)> where Self: 'a;

        fn iter(&self) -> Self::Iterator<'_> {
            core::iter::empty()
        }
    }

    pub struct ChainedAdcChannels<F, S> {
        first: F,
        second: S,
    }

    impl<F, S> ChainedAdcChannels<F, S> {
        pub fn chain<O>(self, other: O) -> ChainedAdcChannels<Self, O>
        where
            F: AdcChannels,
            S: AdcChannels<Adc = F::Adc>,
            O: AdcChannels<Adc = F::Adc>,
        {
            ChainedAdcChannels {
                first: self,
                second: other,
            }
        }
    }

    impl<F, S> AdcChannels for ChainedAdcChannels<F, S>
    where
        F: AdcChannels,
        S: AdcChannels<Adc = F::Adc>,
    {
        type Adc = F::Adc;

        type Iterator<'a> = core::iter::Chain<F::Iterator<'a>, S::Iterator<'a>> where Self: 'a;

        fn iter(&self) -> Self::Iterator<'_> {
            self.first.iter().chain(self.second.iter())
        }
    }

    #[derive(Copy, Clone)]
    #[repr(transparent)]
    pub struct AdcMeasurement(adc_digi_output_data_t);

    impl AdcMeasurement {
        pub const INIT: Self = AdcMeasurement(unsafe {
            core::mem::transmute([0u8; core::mem::size_of::<adc_digi_output_data_t>()])
        });

        pub const fn new() -> Self {
            Self::INIT
        }

        #[cfg(any(esp32, esp32s2))]
        pub fn data(&self) -> u16 {
            unsafe { self.0.__bindgen_anon_1.type1.data() as _ }
        }

        #[cfg(any(esp32, esp32s2))]
        pub fn channel(&self) -> adc_channel_t {
            unsafe { self.0.__bindgen_anon_1.type1.channel() as _ }
        }

        #[cfg(not(any(esp32, esp32s2)))]
        pub fn data(&self) -> u16 {
            unsafe { self.0.__bindgen_anon_1.type2.data() as _ }
        }

        #[cfg(not(any(esp32, esp32s2)))]
        pub fn channel(&self) -> adc_channel_t {
            unsafe { self.0.__bindgen_anon_1.type2.channel() as _ }
        }

        #[cfg(not(any(esp32, esp32s2, esp32h2, esp32c6)))]
        pub fn unit(&self) -> adc_unit_t {
            unsafe { self.0.__bindgen_anon_1.type2.unit() as _ }
        }

        #[cfg(any(esp32, esp32s2))]
        pub fn nullify(&mut self) {
            self.0.__bindgen_anon_1.val = self.data();
        }

        #[cfg(not(any(esp32, esp32s2)))]
        pub fn nullify(&mut self) {
            self.0.__bindgen_anon_1.val = self.data() as _;
        }

        #[cfg(any(esp32, esp32s2))]
        pub fn as_pcm16(data: &mut [AdcMeasurement]) -> &mut [u16] {
            for measurement in data.iter_mut() {
                measurement.nullify();
            }

            unsafe { core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut _, data.len()) }
        }

        #[cfg(not(any(esp32, esp32s2)))]
        pub fn as_pcm32(data: &mut [AdcMeasurement]) -> &mut [u32] {
            for measurement in data.iter_mut() {
                measurement.nullify();
            }

            unsafe { core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut _, data.len()) }
        }
    }

    impl Display for AdcMeasurement {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(
                f,
                "ADC Reading {{channel={}, data={}}}",
                self.channel(),
                self.data()
            )
        }
    }

    impl Debug for AdcMeasurement {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(
                f,
                "AdcMeasurement {{channel: {}, data: {}}}",
                self.channel(),
                self.data()
            )
        }
    }

    pub mod config {
        use crate::units::*;

        #[derive(Debug, Copy, Clone)]
        pub struct Config {
            pub sample_freq: Hertz,
            pub frame_measurements: usize,
            pub frames_count: usize,
        }

        impl Config {
            pub const fn new() -> Self {
                Self {
                    sample_freq: Hertz(20000),
                    frame_measurements: 100,
                    frames_count: 10,
                }
            }

            #[must_use]
            pub fn sample_freq(mut self, sample_freq: Hertz) -> Self {
                self.sample_freq = sample_freq;
                self
            }

            #[must_use]
            pub fn frame_measurements(mut self, frame_measurements: usize) -> Self {
                self.frame_measurements = frame_measurements;
                self
            }

            #[must_use]
            pub fn frames_count(mut self, frames_count: usize) -> Self {
                self.frames_count = frames_count;
                self
            }
        }

        impl Default for Config {
            fn default() -> Self {
                Self::new()
            }
        }
    }

    pub struct AdcDriver<'d> {
        handle: adc_continuous_handle_t,
        adc: u8,
        _ref: PhantomData<&'d ()>,
    }

    impl<'d> AdcDriver<'d> {
        #[cfg(esp32)]
        pub fn new(
            adc: impl Peripheral<P = super::ADC1> + 'd,
            _i2s: impl Peripheral<P = crate::i2s::I2S0> + 'd,
            config: &config::Config,
            channels: impl AdcChannels<Adc = super::ADC1> + 'd,
        ) -> Result<Self, EspError> {
            Self::internal_new(adc, config, channels)
        }

        #[cfg(esp32s2)]
        pub fn new(
            adc: impl Peripheral<P = super::ADC1> + 'd,
            _spi: impl Peripheral<P = crate::spi::SPI3> + 'd,
            config: &config::Config,
            channels: impl AdcChannels<Adc = super::ADC1> + 'd,
        ) -> Result<Self, EspError> {
            Self::internal_new(adc, config, channels)
        }

        #[cfg(not(any(esp32, esp32s2)))]
        pub fn new<A: Adc>(
            adc: impl Peripheral<P = A> + 'd,
            config: &config::Config,
            channels: impl AdcChannels<Adc = A> + 'd,
        ) -> Result<Self, EspError> {
            Self::internal_new(adc, config, channels)
        }

        fn internal_new<A: Adc>(
            _adc: impl Peripheral<P = A> + 'd,
            config: &config::Config,
            channels: impl AdcChannels<Adc = A> + 'd,
        ) -> Result<Self, EspError> {
            let mut patterns = [adc_digi_pattern_config_t::default(); 32];

            for (index, (channel, atten)) in channels.iter().enumerate() {
                if index >= patterns.len() {
                    return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
                }

                patterns[index].atten = atten as _;
                patterns[index].channel = channel as _;
                patterns[index].unit = A::unit() as _;
                patterns[index].bit_width = 12; // For now only 12 bits is supported
            }

            let mut handle: adc_continuous_handle_t = core::ptr::null_mut();

            #[allow(clippy::needless_update)]
            esp!(unsafe {
                adc_continuous_new_handle(
                    &adc_continuous_handle_cfg_t {
                        max_store_buf_size: SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                            * (config.frame_measurements as u32)
                            * (config.frames_count as u32),
                        conv_frame_size: SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                            * (config.frame_measurements as u32),
                        ..Default::default()
                    },
                    &mut handle,
                )
            })?;

            let conv_mode = if A::unit() == 0 {
                adc_digi_convert_mode_t_ADC_CONV_SINGLE_UNIT_1
            } else {
                adc_digi_convert_mode_t_ADC_CONV_SINGLE_UNIT_2
            };

            #[cfg(any(esp32, esp32s2))]
            let format = adc_digi_output_format_t_ADC_DIGI_OUTPUT_FORMAT_TYPE1;

            #[cfg(not(any(esp32, esp32s2)))]
            let format = adc_digi_output_format_t_ADC_DIGI_OUTPUT_FORMAT_TYPE2;

            esp!(unsafe {
                adc_continuous_config(
                    handle,
                    &adc_continuous_config_t {
                        pattern_num: channels.iter().count() as _,
                        adc_pattern: &patterns as *const _ as *mut _,
                        sample_freq_hz: config.sample_freq.into(),
                        conv_mode,
                        format,
                    },
                )
            })?;

            #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
            {
                esp!(unsafe {
                    adc_continuous_register_event_callbacks(
                        handle,
                        &adc_continuous_evt_cbs_t {
                            on_conv_done: Some(Self::handle_isr),
                            on_pool_ovf: Some(Self::handle_isr),
                        },
                        &NOTIFIER[A::unit() as usize] as *const _ as *mut _,
                    )
                })?;
            }

            Ok(Self {
                handle,
                adc: A::unit() as _,
                _ref: PhantomData,
            })
        }

        pub fn handle(&self) -> adc_continuous_handle_t {
            self.handle
        }

        pub fn unit(&self) -> adc_unit_t {
            self.adc as _
        }

        pub fn start(&mut self) -> Result<(), EspError> {
            esp!(unsafe { adc_continuous_start(self.handle) })
        }

        pub fn stop(&mut self) -> Result<(), EspError> {
            esp!(unsafe { adc_continuous_stop(self.handle) })
        }

        pub fn read(
            &mut self,
            buf: &mut [AdcMeasurement],
            timeout: TickType_t,
        ) -> Result<usize, EspError> {
            let mut read: u32 = 0;

            esp!(unsafe {
                adc_continuous_read(
                    self.handle,
                    buf.as_mut_ptr() as *mut _,
                    core::mem::size_of_val(buf) as _,
                    &mut read,
                    TickType(timeout).as_millis_u32(),
                )
            })?;

            Ok(read as usize / core::mem::size_of::<AdcMeasurement>())
        }

        pub fn read_bytes(
            &mut self,
            buf: &mut [u8],
            timeout: TickType_t,
        ) -> Result<usize, EspError> {
            let mut read: u32 = 0;

            esp!(unsafe {
                adc_continuous_read(
                    self.handle,
                    buf.as_mut_ptr() as *mut _,
                    core::mem::size_of_val(buf) as _,
                    &mut read,
                    TickType(timeout).as_millis_u32(),
                )
            })?;

            Ok(read as usize)
        }

        #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
        pub async fn read_async(&mut self, buf: &mut [AdcMeasurement]) -> Result<usize, EspError> {
            loop {
                match self.read(buf, delay::NON_BLOCK) {
                    Ok(len) if len > 0 => return Ok(len),
                    Err(e) if e.code() != ESP_ERR_TIMEOUT => return Err(e),
                    _ => {
                        NOTIFIER[self.adc as usize].wait().await;
                    }
                }
            }
        }

        #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
        pub async fn read_bytes_async(&mut self, buf: &mut [u8]) -> Result<usize, EspError> {
            loop {
                match self.read_bytes(buf, delay::NON_BLOCK) {
                    Ok(len) if len > 0 => return Ok(len),
                    Err(e) if e.code() != ESP_ERR_TIMEOUT => return Err(e),
                    _ => {
                        NOTIFIER[self.adc as usize].wait().await;
                    }
                }
            }
        }

        #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
        extern "C" fn handle_isr(
            _handle: adc_continuous_handle_t,
            _data: *const adc_continuous_evt_data_t,
            user_data: *mut c_void,
        ) -> bool {
            let notifier: &HalIsrNotification =
                unsafe { (user_data as *const HalIsrNotification).as_ref() }.unwrap();

            notifier.notify_lsb()
        }
    }

    impl<'d> Drop for AdcDriver<'d> {
        fn drop(&mut self) {
            let _ = self.stop();

            #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
            {
                esp!(unsafe {
                    adc_continuous_register_event_callbacks(
                        self.handle,
                        core::ptr::null(),
                        core::ptr::null_mut(),
                    )
                })
                .unwrap();
            }

            esp!(unsafe { adc_continuous_deinit(self.handle) }).unwrap();

            NOTIFIER[self.adc as usize].reset();
        }
    }

    unsafe impl<'d> Send for AdcDriver<'d> {}

    impl<'d> embedded_io::ErrorType for AdcDriver<'d> {
        type Error = EspIOError;
    }

    impl<'d> embedded_io::Read for AdcDriver<'d> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_bytes(buf, delay::BLOCK).map_err(EspIOError)
        }
    }

    #[cfg(feature = "nightly")]
    #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
    impl<'d> embedded_io_async::Read for AdcDriver<'d> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_bytes_async(buf).await.map_err(EspIOError)
        }
    }

    #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
    #[cfg(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4))] // TODO: Check for esp32c5 and esp32p4
    static NOTIFIER: [HalIsrNotification; 1] = [HalIsrNotification::new()];

    #[cfg(not(esp_idf_adc_continuous_isr_iram_safe))]
    #[cfg(not(any(esp32c2, esp32h2, esp32c5, esp32c6, esp32p4)))] // TODO: Check for esp32c5 and esp32p4
    static NOTIFIER: [HalIsrNotification; 2] =
        [HalIsrNotification::new(), HalIsrNotification::new()];
}
