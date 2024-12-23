//! Logging
use core::fmt::Write;

use alloc::collections::BTreeMap;
use alloc::string::String;

use ::log::{Level, LevelFilter, Metadata, Record};

use crate::private::common::*;
use crate::private::cstr::*;
use crate::private::mutex::Mutex;
use crate::sys::*;

extern crate alloc;

/// Exposes the newlib stdout file descriptor to allow writing formatted
/// messages to stdout without a std dependency or allocation
///
/// Does lock the `stdout` file descriptor on `new` and does release the lock on `drop`,
/// so that the logging does not get interleaved with other output due to multithreading
struct EspStdout(*mut FILE);

impl EspStdout {
    fn new() -> Self {
        let stdout = unsafe { __getreent().as_mut() }.unwrap()._stdout;

        let file = unsafe { stdout.as_mut() }.unwrap();

        // Copied from here:
        // https://github.com/bminor/newlib/blob/master/newlib/libc/stdio/local.h#L80
        // https://github.com/bminor/newlib/blob/3bafe2fae7a0878598a82777c623edb2faa70b74/newlib/libc/include/sys/stdio.h#L13
        if (file._flags2 & __SNLK as i32) == 0 && (file._flags & __SSTR as i16) == 0 {
            unsafe {
                _lock_acquire_recursive(&mut file._lock);
            }
        }

        Self(stdout)
    }
}

impl Drop for EspStdout {
    fn drop(&mut self) {
        let file = unsafe { self.0.as_mut() }.unwrap();

        // Copied from here:
        // https://github.com/bminor/newlib/blob/master/newlib/libc/stdio/local.h#L85
        // https://github.com/bminor/newlib/blob/3bafe2fae7a0878598a82777c623edb2faa70b74/newlib/libc/include/sys/stdio.h#L21
        if (file._flags2 & __SNLK as i32) == 0 && (file._flags & __SSTR as i16) == 0 {
            unsafe {
                _lock_release_recursive(&mut file._lock);
            }
        }
    }
}

impl core::fmt::Write for EspStdout {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let slice = s.as_bytes();
        unsafe {
            fwrite(slice.as_ptr() as *const _, 1, slice.len() as u32, self.0);
        }

        Ok(())
    }
}

#[allow(non_upper_case_globals)]
#[allow(non_snake_case)]
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
#[allow(non_snake_case)]
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

static LOGGER: EspLogger = EspLogger::new();

pub struct EspLogger {
    // esp-idf function `esp_log_level_get` builds a cache using the address
    // of the target and not doing a string compare. This means we need to
    // build a cache of our own mapping the str value to a consistant
    // Cstr value.
    cache: Mutex<BTreeMap<String, CString>>,
}

unsafe impl Send for EspLogger {}
unsafe impl Sync for EspLogger {}

impl EspLogger {
    /// Public in case user code would like to compose this logger in their own one
    pub const fn new() -> Self {
        Self {
            cache: Mutex::new(BTreeMap::new()),
        }
    }

    pub fn initialize_default() {
        ::log::set_logger(&LOGGER)
            .map(|()| LOGGER.initialize())
            .unwrap();
    }

    pub fn initialize(&self) {
        ::log::set_max_level(self.get_max_level());
    }

    pub fn get_max_level(&self) -> LevelFilter {
        LevelFilter::from(Newtype(CONFIG_LOG_MAXIMUM_LEVEL))
    }

    pub fn set_target_level(
        &self,
        target: impl AsRef<str>,
        level_filter: LevelFilter,
    ) -> Result<(), EspError> {
        let target = target.as_ref();

        let mut cache = self.cache.lock();

        let ctarget = loop {
            if let Some(ctarget) = cache.get(target) {
                break ctarget;
            }

            let ctarget = to_cstring_arg(target)?;

            cache.insert(target.into(), ctarget);
        };

        unsafe {
            esp_log_level_set(
                ctarget.as_c_str().as_ptr(),
                Newtype::<esp_log_level_t>::from(level_filter).0,
            );
        }

        Ok(())
    }

    fn get_marker(level: Level) -> &'static str {
        match level {
            Level::Error => "E",
            Level::Warn => "W",
            Level::Info => "I",
            Level::Debug => "D",
            Level::Trace => "V",
        }
    }

    fn get_color(_level: Level) -> Option<u8> {
        #[cfg(esp_idf_log_colors)]
        {
            match _level {
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

    fn should_log(&self, record: &Record) -> bool {
        let level = Newtype::<esp_log_level_t>::from(record.level()).0;

        let mut cache = self.cache.lock();

        let ctarget = loop {
            if let Some(ctarget) = cache.get(record.target()) {
                break ctarget;
            }

            if let Ok(ctarget) = to_cstring_arg(record.target()) {
                cache.insert(record.target().into(), ctarget);
            } else {
                return true;
            }
        };

        let max_level = unsafe { esp_log_level_get(ctarget.as_c_str().as_ptr()) };
        level <= max_level
    }
}

impl Default for EspLogger {
    fn default() -> Self {
        Self::new()
    }
}

impl ::log::Log for EspLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= LevelFilter::from(Newtype(CONFIG_LOG_MAXIMUM_LEVEL))
    }

    fn log(&self, record: &Record) {
        let metadata = record.metadata();

        if self.enabled(metadata) && self.should_log(record) {
            let marker = Self::get_marker(metadata.level());
            let target = record.metadata().target();
            let args = record.args();
            let color = Self::get_color(record.level());

            let mut stdout = EspStdout::new();

            if let Some(color) = color {
                write!(stdout, "\x1b[0;{}m", color).unwrap();
            }
            write!(stdout, "{} (", marker).unwrap();
            if cfg!(esp_idf_log_timestamp_source_rtos) {
                let timestamp = unsafe { esp_log_timestamp() };
                write!(stdout, "{}", timestamp).unwrap();
            } else if cfg!(esp_idf_log_timestamp_source_system) {
                // TODO: https://github.com/esp-rs/esp-idf-svc/pull/494 - official usage of
                // `esp_log_timestamp_str()` should be tracked and replace the not thread-safe
                // `esp_log_system_timestamp()` which has a race condition flaw due to
                // returning a pointer to a static buffer containing the c-string.
                let timestamp =
                    unsafe { CStr::from_ptr(esp_log_system_timestamp()).to_str().unwrap() };
                write!(stdout, "{}", timestamp).unwrap();
            }
            write!(stdout, ") {}: {}", target, args).unwrap();
            if color.is_some() {
                write!(stdout, "\x1b[0m").unwrap();
            }
            writeln!(stdout).unwrap();
        }
    }

    fn flush(&self) {}
}

pub fn set_target_level(
    target: impl AsRef<str>,
    level_filter: LevelFilter,
) -> Result<(), EspError> {
    LOGGER.set_target_level(target, level_filter)
}
