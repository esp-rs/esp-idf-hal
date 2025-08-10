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

const RUST_LOG: Option<&str> = option_env!("RUST_LOG");

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

/// Trait for a log filter backend that can be used with the `EspIdfLogger`.
pub trait LogFilterBackend {
    /// Initialize the log filter backend.
    fn initialize(&self) {}

    /// Check if logging for the given metadata is enabled.
    fn enabled(&self, metadata: &Metadata) -> bool;
}

impl<T> LogFilterBackend for &T
where
    T: LogFilterBackend,
{
    fn initialize(&self) {
        (**self).initialize()
    }

    fn enabled(&self, metadata: &Metadata) -> bool {
        (**self).enabled(metadata)
    }
}

/// Log filter backend based on the ESP-IDF logging configuration.
///
/// This filter is useful when the user would like to control the verbosity
/// of the logging system based on the ESP-IDF configuration settings, which
/// should apply both to the ESP-IDF native C logging, as well as to logging from Rust.
///
/// This backend uses the ESP-IDF logging system to filter log messages based on their target and level.
/// Specifically:
/// - The `log` crate is set to max level equal to the `CONFIG_LOG_MAXIMUM_LEVEL` ESP-IDF configuration.
/// - The `set_target_level` method allows setting the log level for specific log targets
///   (both targets based on Rust logging - i.e. most often than not Rust modules, as well as native ESP-IDF targets).
pub struct EspIdfLogFilter {
    cache: Mutex<BTreeMap<String, CString>>,
}

impl EspIdfLogFilter {
    /// Create a new instance of `EspIdfLogFilter`.
    pub const fn new() -> Self {
        Self {
            cache: Mutex::new(BTreeMap::new()),
        }
    }

    /// Initialize the ESP-IDF log filter backend.
    pub fn initialize(&self) {
        ::log::set_max_level(self.get_max_level());
    }

    /// Return the maximum log level configured in the ESP-IDF.
    pub fn get_max_level(&self) -> LevelFilter {
        LevelFilter::from(Newtype(CONFIG_LOG_MAXIMUM_LEVEL))
    }

    /// Set the log level for a specific target.
    ///
    /// Arguments:
    /// - `target`: The target for which to set the log level. This can be a Rust log target, or an ESP-IDF native target.
    /// - `level_filter`: The log level to set for the target.
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

    /// Check if logging for the given metadata is enabled,
    /// based on the ESP-IDF current log level, including taeget-specific log levels.
    pub fn enabled(&self, metadata: &Metadata) -> bool {
        let level = Newtype::<esp_log_level_t>::from(metadata.level()).0;

        let mut cache = self.cache.lock();

        let ctarget = loop {
            if let Some(ctarget) = cache.get(metadata.target()) {
                break ctarget;
            }

            if let Ok(ctarget) = to_cstring_arg(metadata.target()) {
                cache.insert(metadata.target().into(), ctarget);
            } else {
                return true;
            }
        };

        let max_level = unsafe { esp_log_level_get(ctarget.as_c_str().as_ptr()) };
        level <= max_level
    }
}

impl Default for EspIdfLogFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl LogFilterBackend for EspIdfLogFilter {
    fn initialize(&self) {
        self.initialize();
    }

    fn enabled(&self, metadata: &Metadata) -> bool {
        self.enabled(metadata)
    }
}

/// A log filter backend that does not consider the ESP-IDF configuration settings
/// that control the log verbosity and does not filter anything.
///
/// This way, the control of the log verbosity from within Rust is completely disconnected
/// from the log verbosity for the ESP-IDF native C code.
impl LogFilterBackend for () {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Logging verbosity is controlled by the Rust log crate settings
        true
    }
}

static INTEGRATED_LOGGER: EspIdfLogger<EspIdfLogFilter> = EspIdfLogger::new(EspIdfLogFilter::new());
static LOGGER: EspIdfLogger = EspIdfLogger::new(());

/// A type alias for the ESP-IDf logger configured with the ESP-IDF log filter.
///
/// For backwards compatibility.
pub type EspLogger = EspIdfLogger<EspIdfLogFilter>;

impl EspIdfLogger<EspIdfLogFilter> {
    /// For backwards compatibility
    ///
    /// Equivalent to calling `init_from_esp_idf()`
    pub fn initialize_default() {
        init_from_esp_idf();
    }
}

/// A logger that integrates with the ESP-IDF logging system.
///
/// Specifically:
/// - It logs to `stdout`/`stderr` just like the ESP-IDF native C logging functions
/// - The format of the logs matches the ESP-IDF native C logging format
/// - If the `EspIdfLogFilter` backend is used, it respects the ESP-IDF log level configuration
#[derive(Debug)]
pub struct EspIdfLogger<T = ()> {
    filter: T,
}

impl<T> EspIdfLogger<T> {
    /// Create a new instance of `EspIdfLogger` with the specified log filter backend.
    ///
    /// # Arguments
    /// - `filter`: The log filter backend to use for filtering log messages.
    pub const fn new(filter: T) -> Self {
        Self { filter }
    }

    /// Return a reference to the log filter backend used by this logger.
    pub fn filter(&self) -> &T {
        &self.filter
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
}

impl<T> ::log::Log for EspIdfLogger<T>
where
    T: LogFilterBackend + Send + Sync,
{
    fn enabled(&self, metadata: &Metadata) -> bool {
        self.filter.enabled(metadata)
    }

    fn log(&self, record: &Record) {
        let metadata = record.metadata();

        if self.enabled(metadata) {
            let marker = Self::get_marker(metadata.level());
            let target = record.metadata().target();
            let args = record.args();
            let color = Self::get_color(record.level());

            let mut stdout = EspStdout::new();

            if let Some(color) = color {
                write!(stdout, "\x1b[0;{color}m").unwrap();
            }
            write!(stdout, "{marker} (").unwrap();
            if cfg!(esp_idf_log_timestamp_source_rtos) {
                let timestamp = unsafe { esp_log_timestamp() };
                write!(stdout, "{timestamp}").unwrap();
            } else if cfg!(esp_idf_log_timestamp_source_system) {
                // TODO: https://github.com/esp-rs/esp-idf-svc/pull/494 - official usage of
                // `esp_log_timestamp_str()` should be tracked and replace the not thread-safe
                // `esp_log_system_timestamp()` which has a race condition flaw due to
                // returning a pointer to a static buffer containing the c-string.
                let timestamp =
                    unsafe { CStr::from_ptr(esp_log_system_timestamp()).to_str().unwrap() };
                write!(stdout, "{timestamp}").unwrap();
            }
            write!(stdout, ") {target}: {args}").unwrap();
            if color.is_some() {
                write!(stdout, "\x1b[0m").unwrap();
            }
            writeln!(stdout).unwrap();
        }
    }

    fn flush(&self) {}
}

/// Initialize the Rust logging system with the ESP-IDF logger and with the noop log filter backend
/// (i.e. logging verbosity is controlled by the Rust log crate settings and disconnected from the ESP-IDF configuration settings).
///
/// Arguments:
/// - `filter`: The log level filter to set in the `log` crate.
pub fn init(filter: LevelFilter) -> &'static EspIdfLogger<()> {
    init_with_logger(&LOGGER);

    ::log::set_max_level(filter);

    &LOGGER
}

/// Initialize the Rust logging system with the ESP-IDF logger and with the noop log filter backend
/// (i.e. logging verbosity is controlled by the `RUST_LOG` environment variable).
///
/// This function reads the `RUST_LOG` environment variable to determine the log level.
pub fn init_from_env() -> &'static EspIdfLogger<()> {
    let level = match RUST_LOG.unwrap_or("info").to_ascii_lowercase().as_str() {
        "off" | "none" => LevelFilter::Off,
        "error" => LevelFilter::Error,
        "warn" | "warning" => LevelFilter::Warn,
        "info" => LevelFilter::Info,
        "debug" => LevelFilter::Debug,
        "trace" => LevelFilter::Trace,
        _ => LevelFilter::Info, // Default to Info if the level is not recognized
    };

    init(level)
}

/// Initialize the Rust logging system with the ESP-IDF logger and with the ESP-IDF log filter backend
/// (i.e. logging verbosity is controlled by the ESP-IDF configuration settings).
pub fn init_from_esp_idf() -> &'static EspIdfLogger<EspIdfLogFilter> {
    init_with_logger(&INTEGRATED_LOGGER);

    &INTEGRATED_LOGGER
}

/// Initialize the Rust logging system with the provided ESP-IDF logger.
fn init_with_logger<T>(logger: &'static EspIdfLogger<T>)
where
    T: LogFilterBackend + Send + Sync,
{
    ::log::set_logger(logger)
        .map(|()| logger.filter().initialize())
        .unwrap();
}
