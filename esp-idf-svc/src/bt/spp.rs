#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]

use core::borrow::Borrow;
use core::marker::PhantomData;

use ::log::trace;

use core::fmt::{self, Debug};
use num_enum::TryFromPrimitive;

use crate::sys::*;
use crate::{
    bt::{BtClassicEnabled, BtDriver},
    private::cstr::to_cstring_arg,
};

use super::{BdAddr, BtSingleton};

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum Security {
    None = ESP_SPP_SEC_NONE,
    Authenticate = ESP_SPP_SEC_AUTHENTICATE,
    Encrypt = ESP_SPP_SEC_AUTHENTICATE | ESP_SPP_SEC_ENCRYPT,
    Acceptor16Digit = ESP_SPP_SEC_IN_16_DIGITS,
    Acceptor16DigitAuthenticate = ESP_SPP_SEC_IN_16_DIGITS | ESP_SPP_SEC_AUTHENTICATE,
    Acceptor16DigitEncrypt =
        ESP_SPP_SEC_IN_16_DIGITS | ESP_SPP_SEC_AUTHENTICATE | ESP_SPP_SEC_ENCRYPT,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum Status {
    Success = esp_spp_status_t_ESP_SPP_SUCCESS,
    Failure = esp_spp_status_t_ESP_SPP_FAILURE,
    Busy = esp_spp_status_t_ESP_SPP_BUSY,
    NoData = esp_spp_status_t_ESP_SPP_NO_DATA,
    NoResource = esp_spp_status_t_ESP_SPP_NO_RESOURCE,
    NeedInit = esp_spp_status_t_ESP_SPP_NEED_INIT,
    NeedDeinit = esp_spp_status_t_ESP_SPP_NEED_DEINIT,
    NoConnection = esp_spp_status_t_ESP_SPP_NO_CONNECTION,
    NoServer = esp_spp_status_t_ESP_SPP_NO_SERVER,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum Role {
    Master = esp_spp_role_t_ESP_SPP_ROLE_MASTER,
    Slave = esp_spp_role_t_ESP_SPP_ROLE_SLAVE,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u32)]
pub enum Mode {
    Cb = esp_spp_mode_t_ESP_SPP_MODE_CB,
    Vfs = esp_spp_mode_t_ESP_SPP_MODE_VFS,
}

#[derive(Debug)]
pub struct SppConfig {
    pub mode: Mode,
    pub enable_l2cap_ertm: bool,
    pub tx_buffer_size: u16,
}

impl Default for SppConfig {
    fn default() -> Self {
        Self {
            mode: Mode::Vfs,
            enable_l2cap_ertm: true,
            tx_buffer_size: ESP_SPP_MAX_TX_BUFFER_SIZE as _,
        }
    }
}

impl From<SppConfig> for esp_spp_cfg_t {
    fn from(value: SppConfig) -> Self {
        esp_spp_cfg_t {
            mode: value.mode as _,
            enable_l2cap_ertm: value.enable_l2cap_ertm,
            tx_buffer_size: value.tx_buffer_size,
        }
    }
}

pub struct EventRawData<'a>(pub &'a esp_spp_cb_param_t);

impl<'a> Debug for EventRawData<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("RawData").finish()
    }
}

#[derive(Debug)]
pub enum SppEvent<'a> {
    Init(Status),
    Uninit(Status),
    DiscoveryComp {
        status: Status,
        scn_num: u8, // number of scn discovered
        scn: &'a [u8],
        service_name: &'a [*const ::core::ffi::c_char],
    },
    Open {
        status: Status,
        handle: u32,
        fd: i32,
        rem_bda: BdAddr,
    },
    ServerOpen {
        status: Status,
        handle: u32,
        listen_handle: u32,
        fd: i32,
        rem_bda: BdAddr,
    },
    Close {
        status: Status,
        port_status: u32,
        handle: u32,
        async_: bool,
    },
    Start {
        status: Status,
        handle: u32,
        sec_id: u8,
        scn: u8,
        use_co: bool,
    },
    ServerStop {
        status: Status,
        scn: u8,
    },
    ClInit {
        status: Status,
        handle: u32,
        sec_id: u8,
        use_co: bool,
    },
    Write {
        status: Status,
        handle: u32,
        length: i32,
        cong: bool,
    },
    DataInd {
        status: Status,
        handle: u32,
        length: u16,
        data: *mut u8,
    },
    Cong {
        status: Status,
        handle: u32,
        cong: bool,
    },
    VfsRegister(Status),
    VfsUnregister(Status),
    Other {
        raw_event: esp_spp_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_spp_cb_event_t, &'a esp_spp_cb_param_t)> for SppEvent<'a> {
    fn from(value: (esp_spp_cb_event_t, &'a esp_spp_cb_param_t)) -> Self {
        let (event, param) = value;

        unsafe {
            match event {
                esp_spp_cb_event_t_ESP_SPP_INIT_EVT => {
                    Self::Init(param.init.status.try_into().unwrap())
                }
                esp_spp_cb_event_t_ESP_SPP_UNINIT_EVT => {
                    Self::Uninit(param.uninit.status.try_into().unwrap())
                }
                esp_spp_cb_event_t_ESP_SPP_DISCOVERY_COMP_EVT => Self::DiscoveryComp {
                    status: param.disc_comp.status.try_into().unwrap(),
                    scn_num: param.disc_comp.scn_num,
                    scn: param.disc_comp.scn.as_ref(),
                    service_name: param.disc_comp.service_name.as_ref(),
                },
                esp_spp_cb_event_t_ESP_SPP_OPEN_EVT => Self::Open {
                    status: param.open.status.try_into().unwrap(),
                    handle: param.open.handle,
                    fd: param.open.fd,
                    rem_bda: param.open.rem_bda.into(),
                },
                esp_spp_cb_event_t_ESP_SPP_SRV_OPEN_EVT => Self::ServerOpen {
                    status: param.srv_open.status.try_into().unwrap(),
                    handle: param.srv_open.handle,
                    listen_handle: param.srv_open.new_listen_handle,
                    fd: param.srv_open.fd,
                    rem_bda: param.srv_open.rem_bda.into(),
                },
                esp_spp_cb_event_t_ESP_SPP_CLOSE_EVT => Self::Close {
                    status: param.close.status.try_into().unwrap(),
                    port_status: param.close.port_status,
                    handle: param.close.handle,
                    async_: param.close.async_,
                },
                esp_spp_cb_event_t_ESP_SPP_START_EVT => Self::Start {
                    status: param.start.status.try_into().unwrap(),
                    handle: param.start.handle,
                    sec_id: param.start.sec_id,
                    scn: param.start.scn,
                    use_co: param.start.use_co,
                },
                esp_spp_cb_event_t_ESP_SPP_SRV_STOP_EVT => Self::ServerStop {
                    status: param.srv_stop.status.try_into().unwrap(),
                    scn: param.srv_stop.scn,
                },
                esp_spp_cb_event_t_ESP_SPP_CL_INIT_EVT => Self::ClInit {
                    status: param.cl_init.status.try_into().unwrap(),
                    handle: param.cl_init.handle,
                    sec_id: param.cl_init.sec_id,
                    use_co: param.cl_init.use_co,
                },
                esp_spp_cb_event_t_ESP_SPP_WRITE_EVT => Self::Write {
                    status: param.write.status.try_into().unwrap(),
                    handle: param.write.handle,
                    length: param.write.len,
                    cong: param.write.cong,
                },
                esp_spp_cb_event_t_ESP_SPP_DATA_IND_EVT => Self::DataInd {
                    status: param.data_ind.status.try_into().unwrap(),
                    handle: param.data_ind.handle,
                    length: param.data_ind.len,
                    data: param.data_ind.data,
                },
                esp_spp_cb_event_t_ESP_SPP_CONG_EVT => Self::Cong {
                    status: param.cong.status.try_into().unwrap(),
                    handle: param.cong.handle,
                    cong: param.cong.cong,
                },
                esp_spp_cb_event_t_ESP_SPP_VFS_REGISTER_EVT => {
                    Self::VfsRegister(param.vfs_register.status.try_into().unwrap())
                }
                esp_spp_cb_event_t_ESP_SPP_VFS_UNREGISTER_EVT => {
                    Self::VfsUnregister(param.vfs_unregister.status.try_into().unwrap())
                }
                _ => Self::Other {
                    raw_event: event,
                    raw_data: EventRawData(param),
                },
            }
        }
    }
}

pub struct EspSpp<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspSpp<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    pub fn new(driver: T, config: &SppConfig) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_spp_register_callback(Some(Self::event_handler)) })?;
        esp!(unsafe { esp_spp_enhanced_init(config as *const _ as *mut _) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn start_discovery(&self, bd_addr: &BdAddr) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_start_discovery(bd_addr as *const _ as *mut _) })
    }

    pub fn connect(
        &self,
        security: Security,
        role: Role,
        remote_scn: u8,
        peer_bd_addr: &BdAddr,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_spp_connect(
                security as _,
                role as _,
                remote_scn,
                peer_bd_addr as *const _ as *mut _,
            )
        })
    }

    pub fn disconnect(&self, handle: u32) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_disconnect(handle) })
    }

    pub fn start_server(
        &self,
        security: Security,
        role: Role,
        local_scn: u8,
        name: &str,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            let name = to_cstring_arg(name)?;

            esp_spp_start_srv(security as _, role as _, local_scn, name.as_ptr())
        })
    }

    pub fn stop_server(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_stop_srv() })
    }

    pub fn stop_server_scn(&self, scn: u8) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_stop_srv_scn(scn) })
    }

    pub fn write(&self, handle: u32, data: &mut [u8]) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_write(handle, data.len() as _, data as *const _ as *mut _) })
    }

    pub fn vfs_register(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_vfs_register() })
    }

    pub fn vfs_unregister(&self) -> Result<(), EspError> {
        esp!(unsafe { esp_spp_vfs_unregister() })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut(SppEvent) + Send + 'static,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    /// # Safety
    ///
    /// This method - in contrast to method `subscribe` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
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
    pub unsafe fn subscribe_nonstatic<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut(SppEvent) + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    unsafe extern "C" fn event_handler(event: esp_spp_cb_event_t, param: *mut esp_spp_cb_param_t) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = SppEvent::from((event, param));

        trace!("Got event {{ {event:#?} }}");

        SINGLETON.call(event);
    }
}

impl<'d, M, T> Drop for EspSpp<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>>,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        esp!(unsafe { esp_spp_deinit() }).unwrap();

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T> Send for EspSpp<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T> Sync for EspSpp<'d, M, T>
where
    M: BtClassicEnabled,
    T: Borrow<BtDriver<'d, M>> + Send,
{
}

static SINGLETON: BtSingleton<SppEvent, ()> = BtSingleton::new(());
