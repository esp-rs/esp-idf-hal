use ::log::info;

use alloc::boxed::Box;

use esp_idf_sys::*;

use crate::private::mutex::{Mutex, RawMutex};

type Singleton<T> = Mutex<Option<Box<T>>>;

pub const BROADCAST: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

#[allow(clippy::type_complexity)]
static RECV_CALLBACK: Singleton<dyn FnMut(&[u8], &[u8]) + Send> =
    Mutex::wrap(RawMutex::new(), None);
#[allow(clippy::type_complexity)]
static SEND_CALLBACK: Singleton<dyn FnMut(&[u8], SendStatus) + Send> =
    Mutex::wrap(RawMutex::new(), None);

static TAKEN: Mutex<bool> = Mutex::wrap(RawMutex::new(), false);

#[derive(Debug)]
pub enum SendStatus {
    SUCCESS = 0,
    FAIL,
}

impl From<u32> for SendStatus {
    fn from(val: u32) -> Self {
        match val {
            0 => SendStatus::SUCCESS,
            1 => SendStatus::FAIL,
            _ => panic!("Wrong status code"),
        }
    }
}

pub type PeerInfo = esp_now_peer_info_t;

pub struct EspNow(());

impl EspNow {
    pub fn take() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
        }

        // disable modem sleep, otherwise messages queue up and we're not able
        // to send any esp-now data after a few messages
        // esp-idf bug report: https://github.com/espressif/esp-idf/issues/7496
        esp!(unsafe { esp_wifi_set_ps(0) })?;

        info!("Initializing ESP NOW");
        esp!(unsafe { esp_now_init() })?;

        *taken = true;

        Ok(Self(()))
    }

    pub fn send(&self, peer_addr: [u8; 6], data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_idf_sys::esp_now_send(
                peer_addr.as_ptr() as *const u8,
                data.as_ptr() as *const u8,
                data.len() as size_t,
            )
        })?;
        Ok(())
    }

    pub fn add_peer(&self, peer_info: PeerInfo) -> Result<(), EspError> {
        esp!(unsafe { esp_now_add_peer(&peer_info) })?;
        Ok(())
    }

    pub fn del_peer(&self, peer_addr: [u8; 6]) -> Result<(), EspError> {
        esp!(unsafe { esp_now_del_peer(&peer_addr as *const u8) })?;
        Ok(())
    }

    pub fn mod_peer(&self, peer_info: PeerInfo) -> Result<(), EspError> {
        esp!(unsafe { esp_now_mod_peer(&peer_info) })?;
        Ok(())
    }

    pub fn get_peer(&self, peer_addr: [u8; 6]) -> Result<PeerInfo, EspError> {
        let mut peer_info = PeerInfo::default();
        esp!(unsafe {
            esp_now_get_peer(
                &peer_addr as *const u8,
                &mut peer_info as *mut esp_now_peer_info_t,
            )
        })?;
        Ok(peer_info)
    }

    pub fn peer_exists(&self, peer_addr: [u8; 6]) -> Result<bool, EspError> {
        Ok(unsafe { esp_now_is_peer_exist(&peer_addr as *const u8) })
    }

    pub fn get_peers_number(&self) -> Result<(usize, usize), EspError> {
        let mut num = esp_now_peer_num_t::default();
        esp!(unsafe { esp_now_get_peer_num(&mut num as *mut esp_now_peer_num_t) })?;
        Ok((num.total_num as usize, num.encrypt_num as usize))
    }

    pub fn set_pmk(&self, pmk: &[u8]) -> Result<(), EspError> {
        esp!(unsafe { esp_now_set_pmk(pmk.as_ptr()) })?;
        Ok(())
    }

    pub fn get_version(&self) -> Result<u32, EspError> {
        let mut version: u32 = 0;
        esp!(unsafe { esp_now_get_version(&mut version as *mut u32) })?;
        Ok(version)
    }

    pub fn register_recv_cb(
        &self,
        callback: impl for<'b, 'c> FnMut(&'b [u8], &'c [u8]) + 'static + Send,
    ) -> Result<(), EspError> {
        *RECV_CALLBACK.lock() = Some(Box::new(callback));
        esp!(unsafe { esp_now_register_recv_cb(Some(Self::recv_callback)) })?;
        Ok(())
    }

    pub fn unregister_recv_cb(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_now_unregister_recv_cb() })?;
        *RECV_CALLBACK.lock() = None;
        Ok(())
    }

    pub fn register_send_cb(
        &self,
        callback: impl for<'b, 'c> FnMut(&'b [u8], SendStatus) + 'static + Send,
    ) -> Result<(), EspError> {
        *SEND_CALLBACK.lock() = Some(Box::new(callback));
        esp!(unsafe { esp_now_register_send_cb(Some(Self::send_callback)) })?;
        Ok(())
    }

    pub fn unregister_send_cb(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_now_unregister_send_cb() })?;
        *SEND_CALLBACK.lock() = None;
        Ok(())
    }

    extern "C" fn send_callback(mac_addr: *const u8, status: esp_now_send_status_t) {
        let c_mac = unsafe { core::slice::from_raw_parts(mac_addr, 6usize) };

        if let Some(ref mut callback) = *SEND_CALLBACK.lock() {
            callback(c_mac, status.into())
        } else {
            panic!("EspNow callback not available");
        }
    }

    extern "C" fn recv_callback(mac_addr: *const u8, data: *const u8, data_len: c_types::c_int) {
        let c_mac = unsafe { core::slice::from_raw_parts(mac_addr, 6usize) };
        let c_data = unsafe { core::slice::from_raw_parts(data, data_len as usize) };

        if let Some(ref mut callback) = *RECV_CALLBACK.lock() {
            callback(c_mac, c_data)
        } else {
            panic!("EspNow callback not available");
        }
    }
}

impl Drop for EspNow {
    fn drop(&mut self) {
        let mut taken = TAKEN.lock();

        esp!(unsafe { esp_now_deinit() }).unwrap();

        let send_cb = &mut *SEND_CALLBACK.lock();
        if send_cb.is_some() {
            *send_cb = None;
        }

        let recv_cb = &mut *RECV_CALLBACK.lock();
        if recv_cb.is_some() {
            *recv_cb = None;
        }

        *taken = false;
    }
}
