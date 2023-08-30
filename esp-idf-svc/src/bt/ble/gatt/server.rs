use core::{borrow::Borrow, marker::PhantomData};

use crate::sys::*;
use log::info;

use crate::bt::{BleEnabled, BtCallback, BtDriver, BtUuid};

use super::{GattCharacteristic, GattDescriptor};

#[derive(Debug)]
pub struct GattService {
    pub(crate) is_primary: bool,
    pub(crate) id: BtUuid,
    pub(crate) instance_id: u8,
    pub(crate) handle: u16,
}

impl GattService {
    pub const fn new_primary(id: BtUuid, handle: u16, instance_id: u8) -> Self {
        Self {
            is_primary: true,
            id,
            handle,
            instance_id,
        }
    }

    pub const fn new(id: BtUuid, handle: u16, instance_id: u8) -> Self {
        Self {
            is_primary: false,
            id,
            handle,
            instance_id,
        }
    }
}

#[derive(Clone)]
pub enum GattsEvent<'a> {
    Register {
        /// Operation status
        status: esp_gatt_status_t,
        /// Application id which input in register API
        app_id: u16,
    },
    Read {
        /// Connection id
        conn_id: u16,
        /// Transfer id
        trans_id: u32,
        /// The bluetooth device address which been read
        bda: esp_bd_addr_t,
        /// The attribute handle
        handle: u16,
        /// Offset of the value, if the value is too long
        offset: u16,
        /// The value is too long or not
        is_long: bool,
        /// The read operation need to do response
        need_rsp: bool,
    },
    Write {
        /// Connection id
        conn_id: u16,
        /// Transfer id
        trans_id: u32,
        /// The bluetooth device address which been written
        bda: esp_bd_addr_t,
        /// The attribute handle
        handle: u16,
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
        conn_id: u16,
        /// Transfer id
        trans_id: u32,
        /// The bluetooth device address which been written
        bda: esp_bd_addr_t,
        /// Execute write flag
        exec_write_flag: u8,
    },
    Mtu {
        /// Connection id
        conn_id: u16,
        /// MTU size
        mtu: u16,
    },
    Confirm {
        /// Operation status
        status: esp_gatt_status_t,
        /// Connection id
        conn_id: u16,
        /// attribute handle
        handle: u16,
        /// The indication or notification value length, len is valid when send notification or indication failed
        len: u16,
        /// The indication or notification value, value is valid when send notification or indication failed
        value: Option<&'a [u8]>,
    },
    Unregister {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service attribute handle
        service_handle: u16,
        /// Service id, include service uuid and other information
        service_id: esp_gatt_srvc_id_t,
    },
    Create {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service attribute handle
        service_handle: u16,
        /// Service id, include service uuid and other information
        service_id: esp_gatt_srvc_id_t,
    },
    AddIncludedServiceComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Included service attribute handle
        attr_handle: u16,
        /// Service attribute handle
        service_handle: u16,
    },
    AddCharacteristicComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Characteristic attribute handle
        attr_handle: u16,
        /// Service attribute handle
        service_handle: u16,
        /// Characteristic uuid
        char_uuid: esp_bt_uuid_t,
    },
    AddDescriptorComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Descriptor attribute handle
        attr_handle: u16,
        /// Service attribute handle
        service_handle: u16,
        /// Characteristic descriptor uuid
        descr_uuid: esp_bt_uuid_t,
    },
    DeleteComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service attribute handle
        service_handle: u16,
    },
    StartComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service attribute handle
        service_handle: u16,
    },
    StopComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service attribute handle
        service_handle: u16,
    },
    Connect {
        /// Connection id
        conn_id: u16,
        /// Link role : master role = 0  ; slave role = 1
        link_role: u8,
        /// Remote bluetooth device address
        remote_bda: esp_bd_addr_t,
        /// current Connection parameters
        conn_params: esp_gatt_conn_params_t,
    },
    Disconnect {
        /// Connection id
        conn_id: u16,
        /// Link role : master role = 0  ; slave role = 1
        link_role: u8,
        /// Remote bluetooth device address
        remote_bda: esp_bd_addr_t,
        /// Indicate the reason of disconnection
        reason: esp_gatt_conn_reason_t,
    },
    Open {
        /// Operation status
        status: esp_gatt_status_t,
    },
    Close {
        /// Operation status
        status: esp_gatt_status_t,
        /// Connection id
        conn_id: u16,
    },
    Listen {
        /// Connection id
        conn_id: u16,
        /// Congested or not
        congested: bool,
    },
    Congest {
        /// Connection id
        conn_id: u16,
        /// Congested or not
        congested: bool,
    },
    ResponseComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Attribute handle which send response
        handle: u16,
    },
    CreateAttributeTableComplete {
        /// Operation status
        status: esp_gatt_status_t,
        /// Service uuid type
        svc_uuid: esp_bt_uuid_t,
        /// Service id
        svc_inst_id: u8,
        /// The handles
        handles: &'a [u16],
    },
    SetAttributeValueComplete {
        /// The service handle
        srvc_handle: u16,
        /// The attribute  handle
        attr_handle: u16,
        /// Operation status
        status: esp_gatt_status_t,
    },
    SendServiceChangeComplete {
        /// Operation status
        status: esp_gatt_status_t,
    },
}

#[allow(non_upper_case_globals)]
impl<'a> From<(esp_gatts_cb_event_t, &'a esp_ble_gatts_cb_param_t)> for GattsEvent<'a> {
    fn from(value: (esp_gatts_cb_event_t, &esp_ble_gatts_cb_param_t)) -> Self {
        let (event, param) = value;

        match event {
            esp_gatts_cb_event_t_ESP_GATTS_REG_EVT => unsafe {
                Self::Register {
                    status: param.reg.status,
                    app_id: param.reg.app_id,
                }
            },
            esp_gatts_cb_event_t_ESP_GATTS_READ_EVT => unsafe {
                Self::Read {
                    conn_id: param.read.conn_id,
                    trans_id: param.read.trans_id,
                    bda: param.read.bda,
                    handle: param.read.handle,
                    offset: param.read.offset,
                    is_long: param.read.is_long,
                    need_rsp: param.read.need_rsp,
                }
            },
            // esp_gatts_cb_event_t_ESP_GATTS_WRITE_EVT => {
            //     Self::Write(param.write)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_EXEC_WRITE_EVT => {
            //     Self::ExecWrite(param.exec_write)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_MTU_EVT => Self::Mtu(param.mtu),
            // esp_gatts_cb_event_t_ESP_GATTS_CONF_EVT => {
            //     Self::Confirm(param.conf)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_UNREG_EVT => {
            //     Self::Unregister(param.create)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_CREATE_EVT => {
            //     Self::Create(param.create)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_ADD_INCL_SRVC_EVT => {
            //     Self::AddIncludedServiceComplete(param.add_incl_srvc)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_ADD_CHAR_EVT => {
            //     Self::AddCharacteristicComplete(param.add_char)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_ADD_CHAR_DESCR_EVT => {
            //     Self::AddDescriptorComplete(param.add_char_descr)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_DELETE_EVT => {
            //     Self::DeleteComplete(param.del)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_START_EVT => {
            //     Self::StartComplete(param.start)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_STOP_EVT => {
            //     Self::StopComplete(param.stop)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_CONNECT_EVT => {
            //     Self::Connect(param.connect)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_DISCONNECT_EVT => {
            //     Self::Disconnect(param.disconnect)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_OPEN_EVT => {
            //     Self::Open(param.open)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_CLOSE_EVT => {
            //     Self::Close(param.close)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_LISTEN_EVT => {
            //     Self::Listen(param.congest)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_CONGEST_EVT => {
            //     Self::Congest(param.congest)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_RESPONSE_EVT => {
            //     Self::ResponseComplete(param.rsp)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_CREAT_ATTR_TAB_EVT => {
            //     Self::CreateAttributeTableComplete(param.add_attr_tab)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_SET_ATTR_VAL_EVT => {
            //     Self::SetAttributeValueComplete(param.set_attr_val)
            // }
            // esp_gatts_cb_event_t_ESP_GATTS_SEND_SERVICE_CHANGE_EVT => {
            //     Self::SendServiceChangeComplete(param.service_change)
            // }
            _ => {
                log::warn!("Unhandled event: {:?}", event);
                panic!("Unhandled event: {:?}", event)
            }
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
    pub fn new<F>(driver: T, events_cb: F) -> Result<Self, EspError>
    where
        F: Fn((u8, GattsEvent)) + Send + 'static,
    {
        CALLBACK.set(events_cb)?;

        esp!(unsafe { esp_ble_gatts_register_callback(Some(Self::event_handler)) })?;

        Ok(Self {
            _driver: driver,
            _p: PhantomData,
            _m: PhantomData,
        })
    }

    pub fn register_app(&mut self, app_id: u16) -> Result<(), EspError> {
        info!(
            "register_gatt_service_application enter for app_id: {}",
            app_id
        );

        esp!(unsafe { esp_ble_gatts_app_register(app_id) })
    }

    pub fn create_service(&mut self, gatt_if: u8, service: &GattService) -> Result<(), EspError> {
        let mut svc_id: esp_gatt_srvc_id_t = esp_gatt_srvc_id_t {
            is_primary: service.is_primary,
            id: esp_gatt_id_t {
                uuid: service.id.clone().into(),
                inst_id: service.instance_id,
            },
        };

        esp!(unsafe { esp_ble_gatts_create_service(gatt_if, &mut svc_id, service.handle) })
    }

    pub fn start_service(&mut self, service_handle: u16) -> Result<(), EspError> {
        esp!(unsafe { esp_ble_gatts_start_service(service_handle) })
    }

    pub fn add_characteristic<const S: usize>(
        &mut self,
        service_handle: u16,
        characteristic: &GattCharacteristic<S>,
    ) -> Result<(), EspError> {
        let value = (&characteristic.value).into();
        let auto_rsp = characteristic.auto_rsp.into();

        esp!(unsafe {
            esp_ble_gatts_add_char(
                service_handle,
                &characteristic.uuid.raw() as *const _ as *mut _,
                characteristic.permissions,
                characteristic.property,
                &value as *const esp_attr_value_t as *mut _,
                &auto_rsp as *const esp_attr_control_t as *mut _,
            )
        })
    }

    pub fn add_descriptor(
        &mut self,
        service_handle: u16,
        descriptor: &GattDescriptor,
    ) -> Result<(), EspError> {
        esp!(unsafe {
            esp_ble_gatts_add_char_descr(
                service_handle,
                &descriptor.uuid.raw() as *const _ as *mut _,
                descriptor.permissions,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
        })
    }

    pub fn read_attr(&mut self, attr_handle: u16, buf: &mut [u8]) -> Result<usize, EspError> {
        let mut len: u16 = 0;
        let mut data: *const u8 = core::ptr::null_mut();

        unsafe {
            esp!(esp_ble_gatts_get_attr_value(
                attr_handle,
                &mut len,
                &mut data
            ))?;

            let data = core::slice::from_raw_parts(data, len as _);
            info!("len: {:?}, data: {:p}", len, data);

            if buf.len() < len as _ {
                Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())?;
            } else {
                buf[..len as _].copy_from_slice(data);
            }

            Ok(len as _)
        }
    }

    unsafe extern "C" fn event_handler(
        event: esp_gap_ble_cb_event_t,
        gatts_if: esp_gatt_if_t,
        param: *mut esp_ble_gatts_cb_param_t,
    ) {
        let param = unsafe { param.as_ref() }.unwrap();
        let event = GattsEvent::from((event, param));

        //debug!("Got GATTS event {{ {:#?} }}", event);

        CALLBACK.call((gatts_if, event));
    }
}

impl<'d, M, T> Drop for EspGatts<'d, M, T>
where
    T: Borrow<BtDriver<'d, M>>,
    M: BleEnabled,
{
    fn drop(&mut self) {
        esp!(unsafe { esp_ble_gatts_register_callback(None) }).unwrap();

        CALLBACK.clear().unwrap();
    }
}

static CALLBACK: BtCallback<(u8, GattsEvent), ()> = BtCallback::new(());
