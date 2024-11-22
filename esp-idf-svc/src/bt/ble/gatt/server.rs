use core::borrow::Borrow;
use core::fmt::{self, Debug};
use core::marker::PhantomData;

use ::log::{debug, trace};

use crate::bt::{BdAddr, BleEnabled, BtDriver, BtSingleton, BtUuid};
use crate::sys::*;

use super::{
    GattCharacteristic, GattConnParams, GattConnReason, GattDescriptor, GattInterface,
    GattResponse, GattServiceId, GattStatus, Handle,
};

pub type AppId = u16;
pub type ConnectionId = u16;
pub type TransferId = u32;

pub struct EventRawData<'a>(pub &'a esp_ble_gatts_cb_param_t);

impl Debug for EventRawData<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("EventRawData").finish()
    }
}

#[derive(Debug)]
pub enum GattsEvent<'a> {
    ServiceRegistered {
        /// Operation status
        status: GattStatus,
        /// Application id which input in register API
        app_id: AppId,
    },
    Read {
        /// Connection id
        conn_id: ConnectionId,
        /// Transfer id
        trans_id: TransferId,
        /// The bluetooth device address which been read
        addr: BdAddr,
        /// The attribute handle
        handle: Handle,
        /// Offset of the value, if the value is too long
        offset: u16,
        /// The value is too long or not
        is_long: bool,
        /// The read operation need to do response
        need_rsp: bool,
    },
    Write {
        /// Connection id
        conn_id: ConnectionId,
        /// Transfer id
        trans_id: TransferId,
        /// The bluetooth device address which been written
        addr: BdAddr,
        /// The attribute handle
        handle: Handle,
        /// Offset of the value, if the value is too long
        offset: u16,
        /// The write operation need to do response
        need_rsp: bool,
        /// This write operation is prepare write
        is_prep: bool,
        /// The write attribute value
        value: &'a [u8],
    },
    ExecWrite {
        /// Connection id
        conn_id: ConnectionId,
        /// Transfer id
        trans_id: TransferId,
        /// The bluetooth device address which been written
        addr: BdAddr,
        /// Whether execution was canceled
        canceled: bool,
    },
    Mtu {
        /// Connection id
        conn_id: ConnectionId,
        /// MTU size
        mtu: u16,
    },
    Confirm {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
        /// attribute handle
        handle: Handle,
        /// The indication or notification value, value is valid when send notification or indication failed
        value: Option<&'a [u8]>,
    },
    ServiceUnregistered {
        /// Operation status
        status: GattStatus,
        /// Service attribute handle
        service_handle: Handle,
        /// Service id, include service uuid and other information
        service_id: GattServiceId,
    },
    ServiceCreated {
        /// Operation status
        status: GattStatus,
        /// Service attribute handle
        service_handle: Handle,
        /// Service id, include service uuid and other information
        service_id: GattServiceId,
    },
    IncludedServiceAdded {
        /// Operation status
        status: GattStatus,
        /// Included service attribute handle
        attr_handle: Handle,
        /// Service attribute handle
        service_handle: Handle,
    },
    CharacteristicAdded {
        /// Operation status
        status: GattStatus,
        /// Characteristic attribute handle
        attr_handle: Handle,
        /// Service attribute handle
        service_handle: Handle,
        /// Characteristic uuid
        char_uuid: BtUuid,
    },
    DescriptorAdded {
        /// Operation status
        status: GattStatus,
        /// Descriptor attribute handle
        attr_handle: Handle,
        /// Service attribute handle
        service_handle: Handle,
        /// Characteristic descriptor uuid
        descr_uuid: BtUuid,
    },
    ServiceDeleted {
        /// Operation status
        status: GattStatus,
        /// Service attribute handle
        service_handle: Handle,
    },
    ServiceStarted {
        /// Operation status
        status: GattStatus,
        /// Service attribute handle
        service_handle: Handle,
    },
    ServiceStopped {
        /// Operation status
        status: GattStatus,
        /// Service attribute handle
        service_handle: Handle,
    },
    PeerConnected {
        /// Connection id
        conn_id: ConnectionId,
        /// Link role : master role = 0  ; slave role = 1
        link_role: u8,
        /// Remote bluetooth device address
        addr: BdAddr,
        /// Current Connection parameters
        conn_params: GattConnParams,
    },
    PeerDisconnected {
        /// Connection id
        conn_id: ConnectionId,
        /// Remote bluetooth device address
        addr: BdAddr,
        /// Indicate the reason of disconnection
        reason: GattConnReason,
    },
    Open {
        /// Operation status
        status: GattStatus,
    },
    Close {
        /// Operation status
        status: GattStatus,
        /// Connection id
        conn_id: ConnectionId,
    },
    // TODO: Are the parameters below correct?
    Listen {
        /// Connection id
        conn_id: ConnectionId,
        /// Congested or not
        congested: bool,
    },
    Congest {
        /// Connection id
        conn_id: ConnectionId,
        /// Congested or not
        congested: bool,
    },
    ResponseComplete {
        /// Operation status
        status: GattStatus,
        /// Attribute handle which send response
        handle: Handle,
    },
    AttributeTableCreated {
        /// Operation status
        status: GattStatus,
        /// Service UUID type
        svc_uuid: BtUuid,
        /// Service id
        svc_inst_id: u8,
        /// The handles
        handles: &'a [Handle],
    },
    AttributeValueModified {
        /// The service handle
        srvc_handle: Handle,
        /// The attribute  handle
        attr_handle: Handle,
        /// Operation status
        status: GattStatus,
    },
    ServiceChanged {
        /// Operation status
        status: GattStatus,
    },
    Other {
        raw_event: esp_gatts_cb_event_t,
        raw_data: EventRawData<'a>,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_gatts_cb_event_t, &'a esp_ble_gatts_cb_param_t)> for GattsEvent<'a> {
    fn from(value: (esp_gatts_cb_event_t, &'a esp_ble_gatts_cb_param_t)) -> Self {
        let (event, param) = value;

        match event {
            esp_gatts_cb_event_t_ESP_GATTS_REG_EVT => unsafe {
                Self::ServiceRegistered {
                    status: param.reg.status.try_into().unwrap(),
                    app_id: param.reg.app_id,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_READ_EVT => unsafe {
                Self::Read {
                    conn_id: param.read.conn_id,
                    trans_id: param.read.trans_id,
                    addr: param.read.bda.into(),
                    handle: param.read.handle,
                    offset: param.read.offset,
                    is_long: param.read.is_long,
                    need_rsp: param.read.need_rsp,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_WRITE_EVT => unsafe {
                Self::Write {
                    conn_id: param.write.conn_id,
                    trans_id: param.write.trans_id,
                    addr: param.write.bda.into(),
                    handle: param.write.handle,
                    offset: param.write.offset,
                    need_rsp: param.write.need_rsp,
                    is_prep: param.write.is_prep,
                    value: core::slice::from_raw_parts(param.write.value, param.write.len as _),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_EXEC_WRITE_EVT => unsafe {
                Self::ExecWrite {
                    conn_id: param.exec_write.conn_id,
                    addr: param.exec_write.bda.into(),
                    trans_id: param.exec_write.trans_id,
                    canceled: param.exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_CANCEL as _,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_MTU_EVT => unsafe {
                Self::Mtu {
                    conn_id: param.mtu.conn_id,
                    mtu: param.mtu.mtu,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CONF_EVT => unsafe {
                Self::Confirm {
                    status: param.conf.status.try_into().unwrap(),
                    conn_id: param.conf.conn_id,
                    handle: param.conf.handle,
                    value: if !matches!(param.conf.status.try_into().unwrap(), GattStatus::Ok) {
                        Some(core::slice::from_raw_parts(
                            param.conf.value,
                            param.conf.len as _,
                        ))
                    } else {
                        None
                    },
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_UNREG_EVT => unsafe {
                Self::ServiceUnregistered {
                    status: param.create.status.try_into().unwrap(),
                    service_handle: param.create.service_handle,
                    service_id: param.create.service_id.into(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CREATE_EVT => unsafe {
                Self::ServiceCreated {
                    status: param.create.status.try_into().unwrap(),
                    service_handle: param.create.service_handle,
                    service_id: param.create.service_id.into(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_ADD_INCL_SRVC_EVT => unsafe {
                Self::IncludedServiceAdded {
                    status: param.add_incl_srvc.status.try_into().unwrap(),
                    attr_handle: param.add_incl_srvc.attr_handle,
                    service_handle: param.add_incl_srvc.service_handle,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_ADD_CHAR_EVT => unsafe {
                Self::CharacteristicAdded {
                    status: param.add_char.status.try_into().unwrap(),
                    attr_handle: param.add_char.attr_handle,
                    service_handle: param.add_char.service_handle,
                    char_uuid: param.add_char.char_uuid.into(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_ADD_CHAR_DESCR_EVT => unsafe {
                Self::DescriptorAdded {
                    status: param.add_char_descr.status.try_into().unwrap(),
                    attr_handle: param.add_char_descr.attr_handle,
                    service_handle: param.add_char_descr.service_handle,
                    descr_uuid: param.add_char_descr.descr_uuid.into(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_DELETE_EVT => unsafe {
                Self::ServiceDeleted {
                    status: param.del.status.try_into().unwrap(),
                    service_handle: param.del.service_handle,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_START_EVT => unsafe {
                Self::ServiceStarted {
                    status: param.start.status.try_into().unwrap(),
                    service_handle: param.start.service_handle,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_STOP_EVT => unsafe {
                Self::ServiceStopped {
                    status: param.stop.status.try_into().unwrap(),
                    service_handle: param.stop.service_handle,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CONNECT_EVT => unsafe {
                Self::PeerConnected {
                    conn_id: param.connect.conn_id,
                    link_role: param.connect.link_role,
                    addr: param.connect.remote_bda.into(),
                    conn_params: GattConnParams {
                        interval_ms: param.connect.conn_params.interval as u32 * 125 / 100,
                        latency_ms: param.connect.conn_params.latency as u32 * 125 / 100,
                        timeout_ms: param.connect.conn_params.timeout as u32 * 10,
                    },
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_DISCONNECT_EVT => unsafe {
                Self::PeerDisconnected {
                    conn_id: param.disconnect.conn_id,
                    addr: param.disconnect.remote_bda.into(),
                    reason: param.disconnect.reason.try_into().unwrap(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_OPEN_EVT => unsafe {
                Self::Open {
                    status: param.open.status.try_into().unwrap(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CLOSE_EVT => unsafe {
                Self::Close {
                    status: param.close.status.try_into().unwrap(),
                    conn_id: param.close.conn_id,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_LISTEN_EVT => unsafe {
                Self::Listen {
                    conn_id: param.congest.conn_id,
                    congested: param.congest.congested,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CONGEST_EVT => unsafe {
                Self::Congest {
                    conn_id: param.congest.conn_id,
                    congested: param.congest.congested,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_RESPONSE_EVT => unsafe {
                Self::ResponseComplete {
                    status: param.rsp.status.try_into().unwrap(),
                    handle: param.rsp.handle,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_CREAT_ATTR_TAB_EVT => unsafe {
                Self::AttributeTableCreated {
                    status: param.add_attr_tab.status.try_into().unwrap(),
                    svc_uuid: param.add_attr_tab.svc_uuid.into(),
                    svc_inst_id: param.add_attr_tab.svc_inst_id,
                    handles: core::slice::from_raw_parts(
                        param.add_attr_tab.handles,
                        param.add_attr_tab.num_handle as _,
                    ),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_SET_ATTR_VAL_EVT => unsafe {
                Self::AttributeValueModified {
                    srvc_handle: param.set_attr_val.srvc_handle,
                    attr_handle: param.set_attr_val.attr_handle,
                    status: param.set_attr_val.status.try_into().unwrap(),
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_SEND_SERVICE_CHANGE_EVT => unsafe {
                Self::ServiceChanged {
                    status: param.service_change.status.try_into().unwrap(),
                }
            },
            _ => Self::Other {
                raw_event: event,
                raw_data: EventRawData(param),
            },
        }
    }
}

pub struct EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    _driver: T,
    _p: PhantomData<&'d ()>,
    _m: PhantomData<M>,
}

impl<'d, M, T> EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    pub fn new(driver: T) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { esp_ble_gatts_register_callback(Some(Self::event_handler)) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
    where
        F: FnMut((GattInterface, GattsEvent)) + Send + 'static,
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
        F: FnMut((GattInterface, GattsEvent)) + Send + 'd,
    {
        SINGLETON.subscribe(events_cb);

        Ok(())
    }

    pub fn unsubscribe(&self) -> Result<(), EspError> {
        SINGLETON.unsubscribe();

        Ok(())
    }

    pub fn register_app(&self, app_id: AppId) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_app_register(app_id) })
    }

    pub fn unregister_app(&self, gatts_if: GattInterface) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_app_unregister(gatts_if) })
    }

    pub fn create_service(
        &self,
        gatt_if: GattInterface,
        service_id: &GattServiceId,
        num_handles: u16,
    ) -> Result<(), EspError> {
        let service_id: esp_gatt_srvc_id_t = service_id.clone().into();

        esp!(unsafe {
            esp_ble_gatts_create_service(gatt_if, &service_id as *const _ as *mut _, num_handles)
        })
    }

    pub fn delete_service(&self, service_handle: Handle) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_delete_service(service_handle) })
    }

    pub fn start_service(&self, service_handle: Handle) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_start_service(service_handle) })
    }

    pub fn stop_service(&self, service_handle: Handle) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_stop_service(service_handle) })
    }

    pub fn add_characteristic(
        &self,
        service_handle: Handle,
        characteristic: &GattCharacteristic,
        data: &[u8],
    ) -> Result<(), EspError> {
        if data.len() > characteristic.max_len {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;
        }

        let value = esp_attr_value_t {
            attr_max_len: characteristic.max_len as _,
            attr_len: data.len() as _,
            attr_value: if data.is_empty() {
                core::ptr::null_mut()
            } else {
                data.as_ptr() as *const _ as *mut _
            },
        };

        let auto_rsp = esp_attr_control_t {
            auto_rsp: characteristic.auto_rsp as _,
        };

        esp!(unsafe {
            esp_ble_gatts_add_char(
                service_handle,
                &characteristic.uuid.raw() as *const _ as *mut _,
                characteristic.permissions.as_repr(),
                characteristic.properties.as_repr(),
                &value as *const esp_attr_value_t as *mut _,
                &auto_rsp as *const esp_attr_control_t as *mut _,
            )
        })
    }

    pub fn add_descriptor(
        &self,
        service_handle: Handle,
        descriptor: &GattDescriptor,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_add_char_descr(
                service_handle,
                &descriptor.uuid.raw() as *const _ as *mut _,
                descriptor.permissions.as_repr(),
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
        })
    }

    pub fn get_attr(&self, attr_handle: Handle, buf: &mut [u8]) -> Result<usize, EspError> {
        let mut len: u16 = 0;
        let mut data: *const u8 = core::ptr::null_mut();

        unsafe {
            esp!(esp_ble_gatts_get_attr_value(
                attr_handle,
                &mut len,
                &mut data
            ))?;

            let data = core::slice::from_raw_parts(data, len as _);
            trace!("len: {:?}, data: {:p}", len, data);

            if buf.len() < len as _ {
                Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;
            } else {
                buf[..len as _].copy_from_slice(data);
            }

            Ok(len as _)
        }
    }

    pub fn set_attr(&self, attr_handle: Handle, data: &[u8]) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_set_attr_value(attr_handle, data.len() as _, data.as_ptr() as *const _)
        })
    }

    pub fn send_response(
        &self,
        gatts_if: GattInterface,
        conn_id: ConnectionId,
        trans_id: TransferId,
        status: GattStatus,
        response: Option<&GattResponse>,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_send_response(
                gatts_if,
                conn_id,
                trans_id,
                status as _,
                response
                    .map(|response| &response.0 as *const _)
                    .unwrap_or(core::ptr::null()) as *mut _,
            )
        })
    }

    pub fn indicate(
        &self,
        gatts_if: GattInterface,
        conn_id: ConnectionId,
        attr_handle: Handle,
        data: &[u8],
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_send_indicate(
                gatts_if,
                conn_id,
                attr_handle,
                data.len() as _,
                data.as_ptr() as *const _ as *mut _,
                true as _,
            )
        })
    }

    pub fn notify(
        &self,
        gatts_if: GattInterface,
        conn_id: ConnectionId,
        attr_handle: Handle,
        data: &[u8],
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_send_indicate(
                gatts_if,
                conn_id,
                attr_handle,
                data.len() as _,
                data.as_ptr() as *const _ as *mut _,
                false as _,
            )
        })
    }

    unsafe extern "C" fn event_handler(
        event: esp_gap_ble_cb_event_t,
        gatts_if: esp_gatt_if_t,
        param: *mut esp_ble_gatts_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = GattsEvent::from((event, param));

        debug!("Got event {{ {:#?} }}", event);

        SINGLETON.call((gatts_if, event));
    }
}

impl<'d, M, T> Drop for EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    fn drop(&mut self) {
        self.unsubscribe().unwrap();

        esp!(unsafe { esp_ble_gatts_register_callback(None) }).unwrap();

        SINGLETON.release().unwrap();
    }
}

unsafe impl<'d, M, T> Send for EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

// Safe because the ESP IDF Bluedroid APIs all do message passing
// to a dedicated Bluedroid task
unsafe impl<'d, M, T> Sync for EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>> + Send,
    M: BleEnabled,
{
}

static SINGLETON: BtSingleton<(GattInterface, GattsEvent), ()> = BtSingleton::new(());
