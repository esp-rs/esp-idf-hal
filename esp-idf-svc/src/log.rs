use std::ffi::{CStr, CString};

use log::{Level, LevelFilter, Metadata, Record};

use esp_idf_sys::*;

pub struct Logger;

unsafe impl Send for Logger {}
unsafe impl Sync for Logger {}

impl Logger {
    pub fn initialize(&self) {
        log::set_max_level(self.get_max_level());
    }

    pub fn get_max_level(&self) -> LevelFilter {
        Self::to_level_filter(CONFIG_LOG_DEFAULT_LEVEL)
    }

    pub fn set_target_level<T: AsRef<str>>(&self, target: T, level_filter: LevelFilter) {
        let ctarget = CString::new(target.as_ref()).unwrap();

        unsafe {esp_log_level_set(ctarget.as_c_str().as_ptr(), Self::from_level_filter(level_filter))};
    }

    #[allow(non_upper_case_globals)]
    fn to_level_filter(level: esp_log_level_t) -> LevelFilter {
        match level {
            esp_log_level_t_ESP_LOG_NONE => LevelFilter::Off,
            esp_log_level_t_ESP_LOG_ERROR => LevelFilter::Error,
            esp_log_level_t_ESP_LOG_WARN => LevelFilter::Warn,
            esp_log_level_t_ESP_LOG_INFO => LevelFilter::Info,
            esp_log_level_t_ESP_LOG_DEBUG => LevelFilter::Debug,
            esp_log_level_t_ESP_LOG_VERBOSE => LevelFilter::Trace,
            _ => LevelFilter::Trace
        }
    }

    fn from_level_filter(level_filter: LevelFilter) -> esp_log_level_t {
        match level_filter {
            LevelFilter::Off => esp_log_level_t_ESP_LOG_NONE,
            LevelFilter::Error => esp_log_level_t_ESP_LOG_ERROR,
            LevelFilter::Warn => esp_log_level_t_ESP_LOG_WARN,
            LevelFilter::Info => esp_log_level_t_ESP_LOG_INFO,
            LevelFilter::Debug => esp_log_level_t_ESP_LOG_DEBUG,
            LevelFilter::Trace => esp_log_level_t_ESP_LOG_VERBOSE
        }
    }

    #[allow(non_upper_case_globals)]
    fn to_level(level: esp_log_level_t) -> Level {
        match level {
            esp_log_level_t_ESP_LOG_ERROR => Level::Error,
            esp_log_level_t_ESP_LOG_WARN => Level::Warn,
            esp_log_level_t_ESP_LOG_INFO => Level::Info,
            esp_log_level_t_ESP_LOG_DEBUG => Level::Debug,
            esp_log_level_t_ESP_LOG_VERBOSE => Level::Trace,
            _ => Level::Trace
        }
    }

    fn from_level(level: Level) -> esp_log_level_t {
        match level {
            Level::Error => esp_log_level_t_ESP_LOG_ERROR,
            Level::Warn => esp_log_level_t_ESP_LOG_WARN,
            Level::Info => esp_log_level_t_ESP_LOG_INFO,
            Level::Debug => esp_log_level_t_ESP_LOG_DEBUG,
            Level::Trace => esp_log_level_t_ESP_LOG_VERBOSE
        }
    }

    fn get_marker(level: Level) -> &'static CStr {
        CStr::from_bytes_with_nul(match level {
            Level::Error => b"E\0",
            Level::Warn => b"W\0",
            Level::Info => b"I\0",
            Level::Debug => b"D\0",
            Level::Trace => b"V\0"
        }).unwrap()
    }

    fn get_color(level: Level) -> Option<u8> {
        if CONFIG_LOG_COLORS == 0 {
            None
        } else {
            match level {
                Level::Error => Some(30),  // LOG_COLOR_RED
                Level::Warn => Some(33),   // LOG_COLOR_BROWN
                Level::Info => Some(32),   // LOG_COLOR_GREEN,
                _ => None
            }
        }
    }
}

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Self::to_level(CONFIG_LOG_DEFAULT_LEVEL)
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let output = format!("{}", record.args());

            let coutput = CString::new(output).unwrap();
            let ctarget = CString::new(record.metadata().target()).unwrap();

            if let Some(color) = Self::get_color(record.level()) {
                unsafe {
                    esp_log_write(
                        Self::from_level(record.level()),
                        b"rust-logging\0" as *const u8 as *const i8, // TODO: ctarget.as_c_str().as_ptr() as *const u8 as *const i8,
                        b"\x1b[0;%dm%s (%d) %s: %s\x1b[0m\n\0" as *const u8 as *const i8,
                        color as u32,
                        Self::get_marker(record.metadata().level()).as_ptr(),
                        esp_log_timestamp(),
                        ctarget.as_c_str().as_ptr(),
                        coutput.as_c_str().as_ptr());
                }
            } else {
                unsafe {
                    esp_log_write(
                        Self::from_level(record.level()),
                        b"rust-logging\0" as *const u8 as *const i8, // TODO: ctarget.as_c_str().as_ptr() as *const u8 as *const i8,
                        b"%s (%d) %s: %s\n\0" as *const u8 as *const i8,
                        Self::get_marker(record.metadata().level()).as_ptr(),
                        esp_log_timestamp(),
                        ctarget.as_c_str().as_ptr(),
                        coutput.as_c_str().as_ptr());
                }
            }
        }
    }

    fn flush(&self) {}
}
