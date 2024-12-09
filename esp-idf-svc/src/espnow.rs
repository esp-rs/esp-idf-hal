//! ESP-NOW
//!
//! ESP-NOW is a kind of connectionless Wi-Fi communication protocol that is
//! defined by Espressif. In ESP-NOW, application data is encapsulated in a
//! vendor-specific action frame and then transmitted from one Wi-Fi device to
//! another without connection. CTR with CBC-MAC Protocol(CCMP) is used to
//! protect the action frame for security. ESP-NOW is widely used in smart
//! light, remote controlling, sensor, etc.
use core::marker::PhantomData;

use ::log::info;

use alloc::boxed::Box;

use crate::sys::*;

use crate::private::mutex::Mutex;

type Singleton<T> = Mutex<Option<Box<T>>>;

pub const BROADCAST: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

#[derive(Debug, Clone)]
pub struct ReceiveInfo<'a> {
    pub src_addr: &'a [u8; 6],
    pub dst_addr: &'a [u8; 6],
}

#[allow(clippy::type_complexity)]
static RECV_CALLBACK: Singleton<dyn FnMut(&ReceiveInfo, &[u8]) + Send + 'static> = Mutex::new(None);
#[allow(clippy::type_complexity)]
static SEND_CALLBACK: Singleton<dyn FnMut(&[u8], SendStatus) + Send + 'static> = Mutex::new(None);

static TAKEN: Mutex<bool> = Mutex::new(false);

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

pub struct EspNow<'a>(PhantomData<&'a ()>);

impl EspNow<'static> {
    pub fn take() -> Result<Self, EspError> {
        Self::internal_take()
    }
}

impl<'a> EspNow<'a> {
    /// # Safety
    ///
    /// This method - in contrast to method `take` - allows the user to set
    /// non-static callbacks/closures into the returned `EspNow` service. This enables users to borrow
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
    pub unsafe fn take_nonstatic() -> Result<Self, EspError> {
        Self::internal_take()
    }

    fn internal_take() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        // disable modem sleep, otherwise messages queue up and we're not able
        // to send any esp-now data after a few messages
        // esp-idf bug report: https://github.com/espressif/esp-idf/issues/7496
        esp!(unsafe { esp_wifi_set_ps(0) })?;

        info!("Initializing ESP NOW");
        esp!(unsafe { esp_now_init() })?;

        *taken = true;

        Ok(Self(PhantomData))
    }

    pub fn send(&self, peer_addr: [u8; 6], data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe { crate::sys::esp_now_send(peer_addr.as_ptr(), data.as_ptr(), data.len(),) })?;

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

    pub fn fetch_peer(&self, from_head: bool) -> Result<PeerInfo, EspError> {
        let mut peer_info = PeerInfo::default();
        esp!(unsafe { esp_now_fetch_peer(from_head, &mut peer_info as *mut esp_now_peer_info_t) })?;

        Ok(peer_info)
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

    pub fn register_recv_cb<F>(&self, callback: F) -> Result<(), EspError>
    where
        F: FnMut(&ReceiveInfo, &[u8]) + Send + 'a,
    {
        #[allow(clippy::type_complexity)]
        let callback: Box<dyn FnMut(&ReceiveInfo, &[u8]) + Send + 'a> = Box::new(callback);
        #[allow(clippy::type_complexity)]
        let callback: Box<dyn FnMut(&ReceiveInfo, &[u8]) + Send + 'static> =
            unsafe { core::mem::transmute(callback) };

        *RECV_CALLBACK.lock() = Some(Box::new(callback));
        esp!(unsafe { esp_now_register_recv_cb(Some(Self::recv_callback)) })?;

        Ok(())
    }

    pub fn unregister_recv_cb(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_now_unregister_recv_cb() })?;
        *RECV_CALLBACK.lock() = None;

        Ok(())
    }

    pub fn register_send_cb<F>(&self, callback: F) -> Result<(), EspError>
    where
        F: FnMut(&[u8], SendStatus) + Send + 'a,
    {
        #[allow(clippy::type_complexity)]
        let callback: Box<dyn FnMut(&[u8], SendStatus) + Send + 'a> = Box::new(callback);
        #[allow(clippy::type_complexity)]
        let callback: Box<dyn FnMut(&[u8], SendStatus) + Send + 'static> =
            unsafe { core::mem::transmute(callback) };

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

    extern "C" fn recv_callback(
        #[cfg(esp_idf_version_major = "4")] src_addr: *const u8,
        #[cfg(not(esp_idf_version_major = "4"))] esp_now_info: *const esp_now_recv_info_t,
        data: *const u8,
        data_len: core::ffi::c_int,
    ) {
        #[cfg(not(any(esp_idf_version_major = "4")))]
        let src_addr = unsafe { *esp_now_info }.src_addr.cast_const();
        #[cfg(not(any(esp_idf_version_major = "4")))]
        let dst_addr = unsafe { *esp_now_info }.des_addr.cast_const();
        let c_src_addr = unsafe { &*(src_addr as *const [u8; 6]) };
        #[cfg(not(any(esp_idf_version_major = "4")))]
        let c_dst_addr = unsafe { &*(dst_addr as *const [u8; 6]) };
        let c_data = unsafe { core::slice::from_raw_parts(data, data_len as usize) };

        if let Some(ref mut callback) = *RECV_CALLBACK.lock() {
            callback(
                &ReceiveInfo {
                    src_addr: c_src_addr,
                    #[cfg(esp_idf_version_major = "4")]
                    dst_addr: &[0u8; 6],
                    #[cfg(not(any(esp_idf_version_major = "4")))]
                    dst_addr: c_dst_addr,
                },
                c_data,
            )
        } else {
            panic!("EspNow callback not available");
        }
    }
}

impl Drop for EspNow<'_> {
    fn drop(&mut self) {
        let mut taken = TAKEN.lock();

        esp!(unsafe { esp_now_deinit() }).unwrap();

        let send_cb = &mut *SEND_CALLBACK.lock();
        *send_cb = None;

        let recv_cb = &mut *RECV_CALLBACK.lock();
        *recv_cb = None;

        *taken = false;
    }
}
