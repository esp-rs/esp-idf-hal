use esp_idf_sys::*;

pub struct LightSleep;

impl LightSleep {
    pub fn sleep_us(us: u64) -> Result<(), EspError> {
        unsafe {
            esp!(esp_sleep_enable_timer_wakeup(us))?;
            esp!(esp_light_sleep_start())
        }
    }
}
