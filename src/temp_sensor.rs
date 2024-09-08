use esp_idf_sys::{
    esp, soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
    soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_RC_FAST,
    soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_XTAL,
    temperature_sensor_clk_src_t, temperature_sensor_config_t, temperature_sensor_disable,
    temperature_sensor_enable, temperature_sensor_get_celsius, temperature_sensor_handle_t,
    temperature_sensor_install, temperature_sensor_uninstall, EspError,
};

// -- TemperatureSensorClockSource --

#[derive(Copy, Clone)]
/// Rust translation of `temperature_sensor_clk_src_t`
pub enum TemperatureSensorClockSource {
    Default,
    XTAL,
    RcFast,
}

impl From<TemperatureSensorClockSource> for temperature_sensor_clk_src_t {
    fn from(value: TemperatureSensorClockSource) -> Self {
        match value {
            TemperatureSensorClockSource::XTAL => {
                soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_XTAL
            }
            TemperatureSensorClockSource::RcFast => {
                soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_RC_FAST
            }
            TemperatureSensorClockSource::Default => {
                soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_DEFAULT
            }
        }
    }
}

impl From<temperature_sensor_clk_src_t> for TemperatureSensorClockSource {
    fn from(value: temperature_sensor_clk_src_t) -> Self {
        match value {
            #[allow(non_upper_case_globals)]
            soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_XTAL => Self::XTAL,
            #[allow(non_upper_case_globals)]
            soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_RC_FAST => {
                Self::RcFast
            }
            // TODO: Perhaps the default value should be mapped explicitly
            // and all other (u32) values should cause a failure
            _ => Self::Default,
        }
    }
}

// -- TemperatureSensorConfig --

#[derive(Copy, Clone)]
/// Rust wrapper for `temperature_sensor_config_t`
pub struct TemperatureSensorConfig {
    // TODO: check int size
    pub range_min: i32,
    pub range_max: i32,
    pub clk_src: TemperatureSensorClockSource,
}

impl From<temperature_sensor_config_t> for TemperatureSensorConfig {
    fn from(value: temperature_sensor_config_t) -> Self {
        TemperatureSensorConfig {
            range_min: value.range_min,
            range_max: value.range_max,
            clk_src: value.clk_src.into(),
        }
    }
}

impl Into<temperature_sensor_config_t> for TemperatureSensorConfig {
    fn into(self) -> temperature_sensor_config_t {
        temperature_sensor_config_t {
            clk_src: self.clk_src.into(),
            range_max: self.range_max,
            range_min: self.range_min,
        }
    }
}

impl Default for TemperatureSensorConfig {
    fn default() -> Self {
        TemperatureSensorConfig {
            range_min: -10,
            range_max: 80,
            clk_src: TemperatureSensorClockSource::Default,
        }
    }
}

// -- TemperatureSensorDriver --

pub struct TemperatureSensorDriver {
    ptr: temperature_sensor_handle_t,
}

impl TemperatureSensorDriver {
    pub fn new(config: TemperatureSensorConfig) -> Result<Self, EspError> {
        let mut sensor = core::ptr::null_mut();
        esp!(unsafe { temperature_sensor_install(&config.into(), &mut sensor) })?;
        Ok(TemperatureSensorDriver { ptr: sensor })
    }

    pub fn enable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { temperature_sensor_enable(self.ptr) })
    }

    pub fn disable(&mut self) -> Result<(), EspError> {
        esp!(unsafe { temperature_sensor_disable(self.ptr) })
    }

    pub fn get_celsius(&self) -> Result<f32, EspError> {
        let mut val = 0.0;
        esp!(unsafe { temperature_sensor_get_celsius(self.ptr, &mut val) })?;
        Ok(val)
    }

    pub fn get_fahrenheit(&self) -> Result<f32, EspError> {
        let celsius = self.get_celsius()?;
        Ok((celsius * 1.8) + 32.0)
    }

    pub fn get_kelvin(&self) -> Result<f32, EspError> {
        let celsius = self.get_celsius()?;
        Ok(celsius + 273.15)
    }
}

impl Drop for TemperatureSensorDriver {
    fn drop(&mut self) {
        let _ = self.disable();
        unsafe {
            temperature_sensor_uninstall(self.ptr);
        }
    }
}
