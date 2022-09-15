use core::time::Duration;

use embedded_svc::sys_time::SystemTime;

use esp_idf_sys::*;

pub struct EspSystemTime;

impl EspSystemTime {
    pub fn now(&self) -> Duration {
        let mut tv_now: timeval = Default::default();

        unsafe {
            gettimeofday(&mut tv_now as *mut _, core::ptr::null_mut());
        }

        Duration::from_micros(tv_now.tv_sec as u64 * 1000000_u64 + tv_now.tv_usec as u64)
    }
}

impl SystemTime for EspSystemTime {
    fn now(&self) -> Duration {
        EspSystemTime::now(self)
    }
}
