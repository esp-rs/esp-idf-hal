use core::cmp::min;

use ::log::*;

use esp_idf_sys::*;

use crate::private::cstr::CString;
use crate::private::mutex;

const SNTP_SERVER_NUM: usize = SNTP_MAX_SERVERS as usize;

const DEFAULT_SERVERS: [&str; 4] = [
    "0.pool.ntp.org",
    "1.pool.ntp.org",
    "2.pool.ntp.org",
    "3.pool.ntp.org",
];

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
pub enum OperatingMode {
    Poll,
    ListenOnly,
}

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "std", derive(Hash))]
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
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
#[cfg_attr(feature = "use_serde", derive(Serialize, Deserialize))]
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

impl<'a> Default for SntpConf<'a> {
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

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::wrap(mutex::RawMutex::new(), false);

pub struct EspSntp {
    // Needs to be kept around because the C bindings only have a pointer.
    _sntp_servers: [CString; SNTP_SERVER_NUM],
}

impl EspSntp {
    pub fn new_default() -> Result<Self, EspError> {
        Self::new(&Default::default())
    }

    pub fn new(conf: &SntpConf) -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE)?;
        }

        let sntp = Self::init(conf)?;

        *taken = true;
        Ok(sntp)
    }

    fn init(conf: &SntpConf) -> Result<Self, EspError> {
        info!("Initializing");

        unsafe { sntp_setoperatingmode(u8_t::from(conf.operating_mode)) };
        unsafe { sntp_set_sync_mode(sntp_sync_mode_t::from(conf.sync_mode)) };

        let mut c_servers: [CString; SNTP_SERVER_NUM] = Default::default();
        for (i, s) in conf.servers.iter().enumerate() {
            let c_server = CString::new(*s).unwrap();
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
        })
    }

    pub fn get_sync_status(&self) -> SyncStatus {
        SyncStatus::from(unsafe { sntp_get_sync_status() })
    }

    unsafe extern "C" fn sync_cb(tv: *mut esp_idf_sys::timeval) {
        debug!(
            " Sync cb called: sec: {}, usec: {}",
            (*tv).tv_sec,
            (*tv).tv_usec,
        );
    }
}

impl Drop for EspSntp {
    fn drop(&mut self) {
        {
            let mut taken = TAKEN.lock();

            unsafe { sntp_stop() };
            *taken = false;
        }

        info!("Dropped");
    }
}
