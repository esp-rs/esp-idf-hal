extern crate alloc;
use alloc::string::String;

use log::*;
use mutex_trait::Mutex;

use esp_idf_sys::*;

use crate::private::cstr::CString;

const SNTP_SERVER_NUM: usize = SNTP_MAX_SERVERS as usize;

#[derive(Copy, Clone, Debug, PartialEq)]
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

#[derive(Copy, Clone, Debug, PartialEq)]
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

#[derive(Copy, Clone, Debug, PartialEq)]
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

pub struct SntpConf {
    pub servers: [String; SNTP_SERVER_NUM],
    pub operating_mode: OperatingMode,
    pub sync_mode: SyncMode,
}

impl Default for SntpConf {
    fn default() -> Self {
        let mut servers: [String; SNTP_SERVER_NUM] = Default::default();
        // Only 0-3 are valid ntp pool domain names
        for (i, item) in servers.iter_mut().enumerate().take(SNTP_SERVER_NUM.min(4)) {
            *item = format!("{}.pool.ntp.org", i);
        }

        Self {
            servers,
            operating_mode: OperatingMode::Poll,
            sync_mode: SyncMode::Immediate,
        }
    }
}

static mut TAKEN: EspMutex<bool> = EspMutex::new(false);

pub struct EspSntp {
    // Needs to be kept around because the C bindings only have a pointer.
    _sntp_servers: [CString; SNTP_SERVER_NUM],
}

impl EspSntp {
    pub fn new_default() -> Result<Self, EspError> {
        Self::new(&Default::default())
    }

    pub fn new(conf: &SntpConf) -> Result<Self, EspError> {
        unsafe {
            TAKEN.lock(|taken| {
                if *taken {
                    Err(EspError::from(ESP_ERR_INVALID_STATE as i32).unwrap())
                } else {
                    let sntp = Self::init(conf)?;

                    *taken = true;
                    Ok(sntp)
                }
            })
        }
    }

    fn init(conf: &SntpConf) -> Result<Self, EspError> {
        info!("Initializing");

        unsafe { sntp_setoperatingmode(u8_t::from(conf.operating_mode)) };
        unsafe { sntp_set_sync_mode(sntp_sync_mode_t::from(conf.sync_mode)) };

        let mut c_servers: [CString; SNTP_SERVER_NUM] = Default::default();
        for (i, s) in conf.servers.iter().enumerate() {
            let c_server = CString::new(s.as_str()).unwrap();
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
        unsafe {
            TAKEN.lock(|taken| {
                sntp_stop();
                *taken = false;
            });
        }

        info!("Dropped");
    }
}
