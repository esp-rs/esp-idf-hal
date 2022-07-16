use ::log::{Level, LevelFilter, Metadata, Record};

use esp_idf_sys::*;

use crate::private::common::*;
use crate::private::cstr::*;

#[allow(non_upper_case_globals)]
impl From<Newtype<esp_log_level_t>> for LevelFilter {
    fn from(level: Newtype<esp_log_level_t>) -> Self {
        match level.0 {
            esp_log_level_t_ESP_LOG_NONE => LevelFilter::Off,
            esp_log_level_t_ESP_LOG_ERROR => LevelFilter::Error,
            esp_log_level_t_ESP_LOG_WARN => LevelFilter::Warn,
            esp_log_level_t_ESP_LOG_INFO => LevelFilter::Info,
            esp_log_level_t_ESP_LOG_DEBUG => LevelFilter::Debug,
            esp_log_level_t_ESP_LOG_VERBOSE => LevelFilter::Trace,
            _ => LevelFilter::Trace,
        }
    }
}

impl From<LevelFilter> for Newtype<esp_log_level_t> {
    fn from(level: LevelFilter) -> Self {
        Newtype(match level {
            LevelFilter::Off => esp_log_level_t_ESP_LOG_NONE,
            LevelFilter::Error => esp_log_level_t_ESP_LOG_ERROR,
            LevelFilter::Warn => esp_log_level_t_ESP_LOG_WARN,
            LevelFilter::Info => esp_log_level_t_ESP_LOG_INFO,
            LevelFilter::Debug => esp_log_level_t_ESP_LOG_DEBUG,
            LevelFilter::Trace => esp_log_level_t_ESP_LOG_VERBOSE,
        })
    }
}

#[allow(non_upper_case_globals)]
impl From<Newtype<esp_log_level_t>> for Level {
    fn from(level: Newtype<esp_log_level_t>) -> Self {
        match level.0 {
            esp_log_level_t_ESP_LOG_ERROR => Level::Error,
            esp_log_level_t_ESP_LOG_WARN => Level::Warn,
            esp_log_level_t_ESP_LOG_INFO => Level::Info,
            esp_log_level_t_ESP_LOG_DEBUG => Level::Debug,
            esp_log_level_t_ESP_LOG_VERBOSE => Level::Trace,
            _ => Level::Trace,
        }
    }
}

impl From<Level> for Newtype<esp_log_level_t> {
    fn from(level: Level) -> Self {
        Newtype(match level {
            Level::Error => esp_log_level_t_ESP_LOG_ERROR,
            Level::Warn => esp_log_level_t_ESP_LOG_WARN,
            Level::Info => esp_log_level_t_ESP_LOG_INFO,
            Level::Debug => esp_log_level_t_ESP_LOG_DEBUG,
            Level::Trace => esp_log_level_t_ESP_LOG_VERBOSE,
        })
    }
}

static LOGGER: EspLogger = EspLogger;

pub struct EspLogger;

unsafe impl Send for EspLogger {}
unsafe impl Sync for EspLogger {}

impl EspLogger {
    pub fn initialize_default() {
        ::log::set_logger(&LOGGER)
            .map(|()| LOGGER.initialize())
            .unwrap();
    }

    pub fn initialize(&self) {
        ::log::set_max_level(self.get_max_level());
    }

    pub fn get_max_level(&self) -> LevelFilter {
        LevelFilter::from(Newtype(CONFIG_LOG_DEFAULT_LEVEL))
    }

    pub fn set_target_level(&self, target: impl AsRef<str>, level_filter: LevelFilter) {
        let ctarget = CString::new(target.as_ref()).unwrap();

        unsafe {
            esp_log_level_set(
                ctarget.as_c_str().as_ptr(),
                Newtype::<esp_log_level_t>::from(level_filter).0,
            )
        };
    }

    fn get_marker(level: Level) -> &'static CStr {
        CStr::from_bytes_with_nul(match level {
            Level::Error => b"E\0",
            Level::Warn => b"W\0",
            Level::Info => b"I\0",
            Level::Debug => b"D\0",
            Level::Trace => b"V\0",
        })
        .unwrap()
    }

    fn get_color(level: Level) -> Option<u8> {
        #[cfg(esp_idf_log_colors)]
        {
            match level {
                Level::Error => Some(31), // LOG_COLOR_RED
                Level::Warn => Some(33),  // LOG_COLOR_BROWN
                Level::Info => Some(32),  // LOG_COLOR_GREEN,
                _ => None,
            }
        }

        #[cfg(not(esp_idf_log_colors))]
        {
            None
        }
    }
}

impl ::log::Log for EspLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= LevelFilter::from(Newtype(CONFIG_LOG_DEFAULT_LEVEL))
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            // TODO: Get rid of all allocations, if possible
            // How?
            // - retire the call to esp_log_write() in favor of the following:
            // - esp_log_level_get() - to decide whether to log
            // - for printing - just println!(), because ESP-IDF uses vprintf by default as well

            let output = format!("{}", record.args());

            let coutput = CString::new(output).unwrap();
            let ctarget = CString::new(record.metadata().target()).unwrap();

            if let Some(color) = Self::get_color(record.level()) {
                unsafe {
                    esp_log_write(
                        Newtype::<esp_log_level_t>::from(record.level()).0,
                        b"rust-logging\0" as *const u8 as *const _, // TODO: ctarget.as_c_str().as_ptr() as *const u8 as *const _,
                        b"\x1b[0;%dm%s (%d) %s: %s\x1b[0m\n\0" as *const u8 as *const _,
                        color as u32,
                        Self::get_marker(record.metadata().level()).as_ptr(),
                        esp_log_timestamp(),
                        ctarget.as_c_str().as_ptr(),
                        coutput.as_c_str().as_ptr(),
                    );
                }
            } else {
                unsafe {
                    esp_log_write(
                        Newtype::<esp_log_level_t>::from(record.level()).0,
                        b"rust-logging\0" as *const u8 as *const _, // TODO: ctarget.as_c_str().as_ptr() as *const u8 as *const _,
                        b"%s (%d) %s: %s\n\0" as *const u8 as *const _,
                        Self::get_marker(record.metadata().level()).as_ptr(),
                        esp_log_timestamp(),
                        ctarget.as_c_str().as_ptr(),
                        coutput.as_c_str().as_ptr(),
                    );
                }
            }
        }
    }

    fn flush(&self) {}
}
