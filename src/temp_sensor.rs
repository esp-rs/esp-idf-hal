#[cfg(not(any(esp32)))]
use esp_idf_sys::{
    soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_DEFAULT,
    soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_RC_FAST,
    soc_periph_temperature_sensor_clk_src_t_TEMPERATURE_SENSOR_CLK_SRC_XTAL,
    temperature_sensor_clk_src_t, temperature_sensor_config_t, temperature_sensor_disable,
    temperature_sensor_enable, temperature_sensor_get_celsius, temperature_sensor_handle_t,
    temperature_sensor_install, temperature_sensor_uninstall,
};

// -- TemperatureSensorClockSource --

#[derive(Copy, Clone)]
#[cfg(not(any(esp32)))]
/// Rust translation of `temperature_sensor_clk_src_t`
pub enum TemperatureSensorClockSource {
    Default,
    XTAL,
    RcFast,
}

#[cfg(not(any(esp32)))]
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

#[cfg(not(any(esp32)))]
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

#[cfg(not(any(esp32)))]
#[derive(Copy, Clone)]
/// Rust wrapper for `temperature_sensor_config_t`
pub struct TemperatureSensorConfig {
    // TODO: check int size
    pub range_min: i32,
    pub range_max: i32,
    pub clk_src: TemperatureSensorClockSource,
}

#[cfg(not(any(esp32)))]
impl From<temperature_sensor_config_t> for TemperatureSensorConfig {
    fn from(value: temperature_sensor_config_t) -> Self {
        TemperatureSensorConfig {
            range_min: value.range_min,
            range_max: value.range_max,
            clk_src: value.clk_src.into(),
        }
    }
}

#[cfg(not(any(esp32)))]
impl Into<temperature_sensor_config_t> for TemperatureSensorConfig {
    fn into(self) -> temperature_sensor_config_t {
        temperature_sensor_config_t {
            clk_src: self.clk_src.into(),
            range_max: self.range_max,
            range_min: self.range_min,
        }
    }
}

#[cfg(not(any(esp32)))]
impl Default for TemperatureSensorConfig {
    fn default() -> Self {
        TemperatureSensorConfig {
            range_min: -10,
            range_max: 80,
            clk_src: TemperatureSensorClockSource::Default,
        }
    }
}

// -- TemperatureSensor --

#[cfg(not(any(esp32)))]
pub struct TemperatureSensor {
    ptr: temperature_sensor_handle_t,
    // To track current state (to avoid double enable/disable)
    enabled: bool,
}

#[cfg(not(any(esp32)))]
impl TemperatureSensor {
    pub fn new(config: TemperatureSensorConfig) -> Self {
        let mut sensor = std::ptr::null_mut();
        unsafe {
            temperature_sensor_install(&config.into(), &mut sensor);
        }
        TemperatureSensor {
            ptr: sensor,
            enabled: false,
        }
    }

    pub fn enable(&mut self) {
        // TODO: Perhaps handing the "double enable" error to the user makes more sense
        if self.enabled {
            return;
        }
        self.enabled = true;
        unsafe { temperature_sensor_enable(self.ptr) };
    }

    pub fn disable(&mut self) {
        // TODO: Perhaps handing the "double disable" error to the user makes more sense
        if !self.enabled {
            return;
        }
        self.enabled = false;
        unsafe { temperature_sensor_disable(self.ptr) };
    }

    pub fn get_celsius(&self) -> f32 {
        let mut val = 0.0;
        unsafe { temperature_sensor_get_celsius(self.ptr, &mut val) };
        val
    }

    pub fn get_fahrenheit(&self) -> f32 {
        let celsius = self.get_celsius();
        (celsius * 1.8) + 32.0
    }

    pub fn get_kelvin(&self) -> f32 {
        self.get_celsius() + 273.15
    }
}

#[cfg(not(any(esp32)))]
impl Drop for TemperatureSensor {
    fn drop(&mut self) {
        unsafe {
            self.disable();
            temperature_sensor_uninstall(self.ptr);
        };
    }
}
