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

// NOTE: Will be changed to an enum once C-style enums are usable as const generics
#[cfg(not(feature = "riscv-ulp-hal"))]
pub mod attenuation {
    pub use esp_idf_sys::*;

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
    pub fn new(pin: impl Peripheral<P = T> + 'd) -> Result<AdcChannelDriver<'d, A, T>, EspError> {
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

    pub fn read<const A: adc_atten_t, T>(
        &mut self,
        pin: &mut AdcChannelDriver<'_, A, T>,
    ) -> Result<u16, EspError>
    where
        T: ADCPin,
    {
        self.read_internal(ADC::unit(), pin.pin().adc_channel(), A)
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
impl<'d, ADC, PIN> embedded_hal_0_2::adc::OneShot<ADC, u16, PIN> for AdcDriver<'d, ADC>
where
    ADC: Adc,
    PIN: embedded_hal_0_2::adc::Channel<ADC, ID = (adc_channel_t, adc_atten_t)>,
{
    type Error = EspError;

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        self.read_internal(ADC::unit(), PIN::channel().0, PIN::channel().1)
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

#[cfg(all(not(feature = "riscv-ulp-hal"), not(esp_idf_version_major = "4")))]
pub mod continuous {
    use core::ffi::c_void;
    use core::marker::PhantomData;
    use core::sync::atomic::{AtomicBool, Ordering};

    use esp_idf_sys::*;

    use crate::delay;
    use crate::private::notification::Notification;
    use crate::{
        gpio::{sealed::ADCPin as _, ADCPin},
        peripheral::Peripheral,
    };

    use super::{attenuation, Adc};

    pub struct Attenuated<const A: adc_atten_t, T>(T);

    impl<const A: adc_atten_t, T> Attenuated<A, T> {
        pub fn override_atten(
            channel: (adc_channel_t, adc_atten_t),
        ) -> (adc_channel_t, adc_atten_t) {
            (channel.0, A)
        }
    }

    pub type AttenNone<T> = Attenuated<{ attenuation::NONE }, T>;
    pub type Atten2p5dB<T> = Attenuated<{ attenuation::DB_2_5 }, T>;
    pub type Atten6dB<T> = Attenuated<{ attenuation::DB_6 }, T>;
    pub type Atten11dB<T> = Attenuated<{ attenuation::DB_11 }, T>;

    pub trait AdcChannels {
        type Adc: Adc;
        type Iterator: Iterator<Item = (adc_channel_t, adc_atten_t)>;

        fn iter(&self) -> Self::Iterator;
    }

    // impl<T> AdcChannels for &T
    // where
    //     T: AdcChannels,
    // {
    //     type Adc = T::Adc;

    //     fn get(&self) -> [adc_channel_t; N] {
    //         (**self).get()
    //     }
    // }

    // impl<T> AdcChannels for &mut T
    // where
    //     T: AdcChannels,
    // {
    //     type Adc = T::Adc;

    //     fn get(&self) -> [adc_channel_t; N] {
    //         (**self).get()
    //     }
    // }

    impl<'d, P> AdcChannels for P
    where
        P: Peripheral,
        P::P: ADCPin + 'd,
    {
        type Adc = <<P as Peripheral>::P as ADCPin>::Adc;

        type Iterator = core::iter::Once<(adc_channel_t, adc_atten_t)>;

        fn iter(&self) -> Self::Iterator {
            core::iter::once((P::P::CHANNEL, attenuation::NONE))
        }
    }

    impl<'d, const A: adc_atten_t, C> AdcChannels for Attenuated<A, C>
    where
        C: AdcChannels + 'd,
    {
        type Adc = C::Adc;

        type Iterator = core::iter::Map<
            C::Iterator,
            fn((adc_channel_t, adc_atten_t)) -> (adc_channel_t, adc_atten_t),
        >;

        fn iter(&self) -> Self::Iterator {
            self.0.iter().map(Attenuated::<A, C>::override_atten)
        }
    }

    pub struct EmptyAdcChannels<A>(PhantomData<A>);

    impl<A> EmptyAdcChannels<A> {
        pub fn chain<O>(self, other: O) -> ChainedAdcChannels<Self, O>
        where
            A: Adc,
            O: AdcChannels<Adc = A>,
        {
            ChainedAdcChannels {
                first: self,
                second: other,
            }
        }
    }

    impl<A> AdcChannels for EmptyAdcChannels<A>
    where
        A: Adc,
    {
        type Adc = A;

        type Iterator = core::iter::Empty<(adc_channel_t, adc_atten_t)>;

        fn iter(&self) -> Self::Iterator {
            core::iter::empty()
        }
    }

    pub struct ChainedAdcChannels<F, S> {
        first: F,
        second: S,
    }

    impl<F, S> AdcChannels for ChainedAdcChannels<F, S>
    where
        F: AdcChannels,
        S: AdcChannels<Adc = F::Adc>,
    {
        type Adc = F::Adc;

        type Iterator = core::iter::Chain<F::Iterator, S::Iterator>;

        fn iter(&self) -> Self::Iterator {
            self.first.iter().chain(self.second.iter())
        }
    }

    // impl<A, C, const N: usize> AdcChannels for [C; N]
    // where
    //     A: Adc,
    //     C: AdcChannels<Adc = A>,
    // {
    //     type Adc = A;

    //     type Iterator = core::iter::Slice
    //     fn len(&self) -> usize {
    //         self.iter().map(|channels| channels.len()).sum()
    //     }

    //     fn channel(&self, index: usize) -> Result<adc_channel_t, EspError> {
    //         let mut offset = 0;

    //         for channels in self {
    //             if index >= offset && index < offset + channels.len() {
    //                 return channels.channel(index - offset);
    //             }

    //             offset += channels.len();
    //         }

    //         Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
    //     }

    //     fn attenuation(&self, index: usize) -> Result<adc_atten_t, EspError> {
    //         let mut offset = 0;

    //         for channels in self {
    //             if index >= offset && index < offset + channels.len() {
    //                 return channels.attenuation(index - offset);
    //             }

    //             offset += channels.len();
    //         }

    //         Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
    //     }
    // }

    #[repr(transparent)]
    pub struct AdcData(adc_digi_output_data_t);

    pub struct AdcDriver<'d> {
        handle: adc_continuous_handle_t,
        adc: adc_unit_t,
        _ref: PhantomData<&'d ()>,
    }

    impl<'d> AdcDriver<'d> {
        pub fn new<const F: usize, const R: usize, A: Adc>(
            adc: impl Peripheral<P = A> + 'd,
            channels: impl AdcChannels<Adc = A> + 'd,
        ) -> Result<Self, EspError> {
            let mut patterns = [adc_digi_pattern_config_t::default(); 20]; // TODO

            for (index, (channel, atten)) in channels.iter().enumerate() {
                if index >= patterns.len() {
                    return Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>());
                }

                patterns[index].atten = atten as _;
                patterns[index].channel = channel as _;
                patterns[index].unit = A::unit() as _;
                patterns[index].bit_width = 12; // TODO
            }

            let mut handle: adc_continuous_handle_t = core::ptr::null_mut();

            esp!(unsafe {
                adc_continuous_new_handle(
                    &adc_continuous_handle_cfg_t {
                        max_store_buf_size: SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                            * (R as u32)
                            * (F as u32),
                        conv_frame_size: SOC_ADC_DIGI_DATA_BYTES_PER_CONV * (R as u32),
                    },
                    &mut handle,
                )
            })?;

            esp!(unsafe {
                adc_continuous_config(
                    handle,
                    &adc_continuous_config_t {
                        pattern_num: channels.iter().count() as _,
                        adc_pattern: &patterns as *const _ as *mut _,
                        sample_freq_hz: 0, // TODO
                        conv_mode: adc_digi_convert_mode_t_ADC_CONV_SINGLE_UNIT_2,
                        format: adc_digi_output_format_t_ADC_DIGI_OUTPUT_FORMAT_TYPE1, // TODO
                    },
                )
            })?;

            Ok(Self {
                handle,
                adc: A::unit(),
                _ref: PhantomData,
            })
        }

        pub unsafe fn subscribe<F: FnMut() + 'static>(
            &mut self,
            handler: F,
        ) -> Result<(), EspError> {
            todo!()
        }

        pub fn unsubscribe(&mut self) -> Result<(), EspError> {
            self.internal_unsubscribe()
        }

        pub fn start(&mut self) -> Result<(), EspError> {
            esp!(unsafe { adc_continuous_start(self.handle) })
        }

        pub fn stop(&mut self) -> Result<(), EspError> {
            esp!(unsafe { adc_continuous_stop(self.handle) })
        }

        pub fn read(
            &mut self,
            buf: &mut [AdcData],
            timeout: TickType_t,
        ) -> Result<usize, EspError> {
            let mut read: u32 = 0;

            esp!(unsafe {
                adc_continuous_read(
                    self.handle,
                    buf.as_mut_ptr() as *mut _,
                    buf.len() as _,
                    &mut read,
                    timeout,
                )
            })?;

            Ok(read as _)
        }

        pub async fn read_async(&mut self, buf: &mut [AdcData]) -> Result<usize, EspError> {
            let (subscribed, notifier) = &NOTIFIER[self.adc as usize];

            if !subscribed.load(Ordering::SeqCst) {
                let _ = self.internal_unsubscribe();
                subscribed.store(true, Ordering::SeqCst);

                esp!(unsafe {
                    adc_continuous_register_event_callbacks(
                        self.handle,
                        &adc_continuous_evt_cbs_t {
                            on_conv_done: Some(Self::async_notifier),
                            on_pool_ovf: Some(Self::async_notifier),
                        },
                        notifier as *const _ as *mut _,
                    )
                })?;
            }

            loop {
                match self.read(buf, delay::NON_BLOCK) {
                    Ok(len) if len > 0 => return Ok(len),
                    Err(e) if e.code() != ESP_ERR_TIMEOUT => return Err(e),
                    _ => notifier.wait().await,
                }
            }
        }

        fn internal_unsubscribe(&mut self) -> Result<(), EspError> {
            esp!(unsafe {
                adc_continuous_register_event_callbacks(
                    self.handle,
                    core::ptr::null(),
                    core::ptr::null_mut(),
                )
            })?;

            for (subscribed, _) in &NOTIFIER {
                subscribed.store(false, Ordering::SeqCst);
            }

            Ok(())
        }

        extern "C" fn async_notifier(
            _handle: adc_continuous_handle_t,
            _data: *const adc_continuous_evt_data_t,
            user_data: *mut c_void,
        ) -> bool {
            let notifier: &Notification =
                unsafe { (user_data as *const Notification).as_ref() }.unwrap();

            notifier.notify();
            true
        }
    }

    impl<'d> Drop for AdcDriver<'d> {
        fn drop(&mut self) {
            let _ = self.stop();
            let _ = self.internal_unsubscribe();
        }
    }

    static NOTIFIER: [(AtomicBool, Notification); 2] = [
        // TODO
        (AtomicBool::new(false), Notification::new()),
        (AtomicBool::new(false), Notification::new()),
    ];
}
