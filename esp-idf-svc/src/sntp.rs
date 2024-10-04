//! SNTP Time Synchronization

use core::cmp::min;
use core::marker::PhantomData;
use core::time::Duration;

use ::log::*;

use crate::private::cstr::to_cstring_arg;
use crate::private::cstr::CString;
use crate::private::mutex;

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(not(any(
    esp_idf_version_major = "4",
    all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
)))] // For ESP-IDF v5.1 and later
mod esp_sntp {
    use super::OperatingMode;
    pub use crate::sys::*;

    impl From<esp_sntp_operatingmode_t> for OperatingMode {
        #[allow(non_upper_case_globals)]
        fn from(from: esp_sntp_operatingmode_t) -> Self {
            match from {
                esp_sntp_operatingmode_t_ESP_SNTP_OPMODE_POLL => OperatingMode::Poll,
                esp_sntp_operatingmode_t_ESP_SNTP_OPMODE_LISTENONLY => OperatingMode::ListenOnly,
                _ => unreachable!(),
            }
        }
    }

    impl From<OperatingMode> for esp_sntp_operatingmode_t {
        #[allow(non_upper_case_globals)]
        fn from(from: OperatingMode) -> Self {
            match from {
                OperatingMode::Poll => esp_sntp_operatingmode_t_ESP_SNTP_OPMODE_POLL,
                OperatingMode::ListenOnly => esp_sntp_operatingmode_t_ESP_SNTP_OPMODE_LISTENONLY,
            }
        }
    }

    pub use esp_sntp_init as sntp_init;
    pub use esp_sntp_setoperatingmode as sntp_setoperatingmode;
    pub use esp_sntp_setservername as sntp_setservername;
    pub use esp_sntp_stop as sntp_stop;
}

#[cfg(any(
    esp_idf_version_major = "4",
    all(esp_idf_version_major = "5", esp_idf_version_minor = "0"),
))] // Up to ESP-IDF v5.0.x
mod esp_sntp {
    use super::OperatingMode;
    pub use crate::sys::*;

    impl From<u8_t> for OperatingMode {
        fn from(from: u8_t) -> Self {
            match from as u32 {
                SNTP_OPMODE_POLL => OperatingMode::Poll,
                SNTP_OPMODE_LISTENONLY => OperatingMode::ListenOnly,
                _ => unreachable!(),
            }
        }
    }

    impl From<OperatingMode> for u8_t {
        fn from(from: OperatingMode) -> Self {
            match from {
                OperatingMode::Poll => SNTP_OPMODE_POLL as u8_t,
                OperatingMode::ListenOnly => SNTP_OPMODE_LISTENONLY as u8_t,
            }
        }
    }
}

use esp_sntp::*;

const SNTP_SERVER_NUM: usize = SNTP_MAX_SERVERS as usize;

const DEFAULT_SERVERS: [&str; 4] = [
    "0.pool.ntp.org",
    "1.pool.ntp.org",
    "2.pool.ntp.org",
    "3.pool.ntp.org",
];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
pub enum OperatingMode {
    Poll,
    ListenOnly,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
pub enum SyncMode {
    Smooth,
    Immediate,
}

impl From<sntp_sync_mode_t> for SyncMode {
    #[allow(non_upper_case_globals)]
    fn from(from: sntp_sync_mode_t) -> Self {
        match from {
            sntp_sync_mode_t_SNTP_SYNC_MODE_SMOOTH => SyncMode::Smooth,
            sntp_sync_mode_t_SNTP_SYNC_MODE_IMMED => SyncMode::Immediate,
            _ => unreachable!(),
        }
    }
}

impl From<SyncMode> for sntp_sync_mode_t {
    fn from(from: SyncMode) -> Self {
        match from {
            SyncMode::Smooth => sntp_sync_mode_t_SNTP_SYNC_MODE_SMOOTH,
            SyncMode::Immediate => sntp_sync_mode_t_SNTP_SYNC_MODE_IMMED,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
pub enum SyncStatus {
    Reset,
    Completed,
    InProgress,
}

impl From<sntp_sync_status_t> for SyncStatus {
    #[allow(non_upper_case_globals)]
    fn from(from: sntp_sync_status_t) -> Self {
        match from {
            sntp_sync_status_t_SNTP_SYNC_STATUS_RESET => SyncStatus::Reset,
            sntp_sync_status_t_SNTP_SYNC_STATUS_COMPLETED => SyncStatus::Completed,
            sntp_sync_status_t_SNTP_SYNC_STATUS_IN_PROGRESS => SyncStatus::InProgress,
            _ => unreachable!(),
        }
    }
}

pub struct SntpConf<'a> {
    pub servers: [&'a str; SNTP_SERVER_NUM],
    pub operating_mode: OperatingMode,
    pub sync_mode: SyncMode,
}

impl Default for SntpConf<'_> {
    fn default() -> Self {
        let mut servers: [&str; SNTP_SERVER_NUM] = Default::default();
        let copy_len = min(servers.len(), DEFAULT_SERVERS.len());

        servers[..copy_len].copy_from_slice(&DEFAULT_SERVERS[..copy_len]);

        Self {
            servers,
            operating_mode: OperatingMode::Poll,
            sync_mode: SyncMode::Immediate,
        }
    }
}

#[cfg(feature = "alloc")]
type SyncCallback = alloc::boxed::Box<dyn FnMut(Duration) + Send + 'static>;
#[cfg(feature = "alloc")]
static SYNC_CB: mutex::Mutex<Option<SyncCallback>> = mutex::Mutex::new(None);
static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

pub struct EspSntp<'a> {
    // Needs to be kept around because the C bindings only have a pointer.
    _sntp_servers: [CString; SNTP_SERVER_NUM],
    _ref: PhantomData<&'a ()>,
}

impl EspSntp<'static> {
    pub fn new_default() -> Result<Self, EspError> {
        Self::new(&Default::default())
    }

    pub fn new(conf: &SntpConf) -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        let sntp = Self::init(conf)?;

        *taken = true;
        Ok(sntp)
    }

    #[cfg(feature = "alloc")]
    pub fn new_with_callback<F>(conf: &SntpConf, callback: F) -> Result<Self, EspError>
    where
        F: FnMut(Duration) + Send + 'static,
    {
        Self::internal_new_with_callback(conf, callback)
    }
}

impl<'a> EspSntp<'a> {
    /// # Safety
    ///
    /// This method - in contrast to method `new_with_callback` - allows the user to set
    /// a non-static callback/closure into the returned `EspSntp` service. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    ///   scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent to a hidden ESP IDF thread.
    /// This means that if the service is forgotten, Rust is free to e.g. unwind the stack
    /// and the closure now owned by this other thread will end up with references to variables that no longer exist.
    ///
    /// The destructor of the service takes care - prior to the service being dropped and e.g.
    /// the stack being unwind - to remove the closure from the hidden thread and destroy it.
    /// Unfortunately, when the service is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    #[cfg(feature = "alloc")]
    pub unsafe fn new_nonstatic_with_callback<F>(
        conf: &SntpConf,
        callback: F,
    ) -> Result<Self, EspError>
    where
        F: FnMut(Duration) + Send + 'a,
    {
        Self::internal_new_with_callback(conf, callback)
    }

    #[cfg(feature = "alloc")]
    fn internal_new_with_callback<F>(conf: &SntpConf, callback: F) -> Result<Self, EspError>
    where
        F: FnMut(Duration) + Send + 'a,
    {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE)?;
        }

        #[allow(clippy::type_complexity)]
        let callback: alloc::boxed::Box<dyn FnMut(Duration) + Send + 'a> =
            alloc::boxed::Box::new(callback);
        #[allow(clippy::type_complexity)]
        let callback: alloc::boxed::Box<dyn FnMut(Duration) + Send + 'static> =
            unsafe { core::mem::transmute(callback) };

        *SYNC_CB.lock() = Some(callback);
        let sntp = Self::init(conf)?;

        *taken = true;
        Ok(sntp)
    }

    fn init(conf: &SntpConf) -> Result<Self, EspError> {
        info!("Initializing");

        unsafe { sntp_setoperatingmode(conf.operating_mode.into()) };
        unsafe { sntp_set_sync_mode(sntp_sync_mode_t::from(conf.sync_mode)) };

        let mut c_servers: [CString; SNTP_SERVER_NUM] = Default::default();
        for (i, s) in conf.servers.iter().enumerate() {
            let c_server = to_cstring_arg(s)?;
            unsafe { sntp_setservername(i as u8, c_server.as_ptr()) };
            c_servers[i] = c_server;
        }

        unsafe {
            sntp_set_time_sync_notification_cb(Some(Self::sync_cb));

            sntp_init();
        };

        info!("Initialization complete");

        Ok(Self {
            _sntp_servers: c_servers,
            _ref: PhantomData,
        })
    }

    #[cfg(feature = "alloc")]
    fn unsubscribe(&mut self) {
        *SYNC_CB.lock() = None;
    }

    pub fn get_sync_status(&self) -> SyncStatus {
        SyncStatus::from(unsafe { sntp_get_sync_status() })
    }

    unsafe extern "C" fn sync_cb(tv: *mut timeval) {
        debug!(
            " Sync cb called: sec: {}, usec: {}",
            (*tv).tv_sec,
            (*tv).tv_usec,
        );

        #[cfg(feature = "alloc")]
        if let Some(cb) = &mut *SYNC_CB.lock() {
            let duration = Duration::from_secs((*tv).tv_sec as u64)
                + Duration::from_micros((*tv).tv_usec as u64);

            cb(duration);
        }
    }
}

impl Drop for EspSntp<'_> {
    fn drop(&mut self) {
        {
            let mut taken = TAKEN.lock();

            unsafe { sntp_stop() };

            #[cfg(feature = "alloc")]
            self.unsubscribe();

            *taken = false;
        }

        info!("Dropped");
    }
}
