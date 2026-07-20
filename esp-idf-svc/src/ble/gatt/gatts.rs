//! NimBLE GATT server: service registration and per-characteristic access.

use core::ffi::{c_int, c_void};
use core::ptr;

use alloc::boxed::Box;
use alloc::vec::Vec;

use enumset::EnumSet;

use crate::sys::*;

use super::super::mbuf::{mbuf_from_slice, Mbuf};
use super::super::{BleError, BleSetup, BleUuid};
use super::{flags_to_repr, BleGattCharFlag, Handle};

pub type ConnectionId = u16;

type AccessClosure<'ble> = dyn FnMut(BleGattAccess) -> i32 + Send + 'ble;

pub enum BleGattAccess<'a> {
    Read {
        conn_handle: ConnectionId,
        attr_handle: Handle,
        reply: Mbuf<'a>,
    },
    Write {
        conn_handle: ConnectionId,
        attr_handle: Handle,
        data: Mbuf<'a>,
    },
}

unsafe extern "C" fn access_trampoline(
    conn_handle: u16,
    attr_handle: u16,
    ctxt: *mut ble_gatt_access_ctxt,
    arg: *mut c_void,
) -> c_int {
    // `arg` is the thin pointer to the characteristic's boxed closure; the
    // laundered lifetime is sound because the closure lives for `'ble`, which
    // outlives the host.
    let closure: &mut AccessClosure = unsafe { &mut **(arg as *mut Box<AccessClosure>) };

    let mbuf = Mbuf::from_raw(unsafe { (*ctxt).om });

    let rc = match unsafe { (*ctxt).op } as u32 {
        BLE_GATT_ACCESS_OP_READ_CHR => closure(BleGattAccess::Read {
            conn_handle,
            attr_handle,
            reply: mbuf,
        }),
        BLE_GATT_ACCESS_OP_WRITE_CHR => closure(BleGattAccess::Write {
            conn_handle,
            attr_handle,
            data: mbuf,
        }),
        _ => BLE_ATT_ERR_UNLIKELY as i32,
    };

    rc as c_int
}

/// A GATT registration event, delivered on the host task via
/// [`BleSetup::on_gatts_register`](super::super::BleSetup::on_gatts_register) as the
/// service table is registered. This is how you learn the attribute handles NimBLE
/// assigns; capture the value handles you need (matching on `uuid`) here.
pub enum BleGattRegister {
    Service {
        uuid: BleUuid,
        handle: Handle,
    },
    Characteristic {
        uuid: BleUuid,
        def_handle: Handle,
        val_handle: Handle,
    },
    Descriptor {
        uuid: BleUuid,
        handle: Handle,
    },
    Other,
}

impl From<&ble_gatt_register_ctxt> for BleGattRegister {
    fn from(ctxt: &ble_gatt_register_ctxt) -> Self {
        let anon = &ctxt.__bindgen_anon_1;

        match ctxt.op as u32 {
            BLE_GATT_REGISTER_OP_SVC => {
                let svc = unsafe { &anon.svc };
                Self::Service {
                    uuid: unsafe { BleUuid::from_raw((*svc.svc_def).uuid) },
                    handle: svc.handle,
                }
            }
            BLE_GATT_REGISTER_OP_CHR => {
                let chr = unsafe { &anon.chr };
                Self::Characteristic {
                    uuid: unsafe { BleUuid::from_raw((*chr.chr_def).uuid) },
                    def_handle: chr.def_handle,
                    val_handle: chr.val_handle,
                }
            }
            BLE_GATT_REGISTER_OP_DSC => {
                let dsc = unsafe { &anon.dsc };
                Self::Descriptor {
                    uuid: unsafe { BleUuid::from_raw((*dsc.dsc_def).uuid) },
                    handle: dsc.handle,
                }
            }
            _ => Self::Other,
        }
    }
}

/// A characteristic in a [`BleGattService`]. `access` is invoked on the host task
/// for every read and write; the value attribute handle NimBLE assigns is reported
/// via [`BleGattRegister`] during registration.
pub struct BleGattCharacteristic<'ble> {
    uuid: BleUuid,
    flags: EnumSet<BleGattCharFlag>,
    access: Box<AccessClosure<'ble>>,
}

impl<'ble> BleGattCharacteristic<'ble> {
    pub fn new<F>(uuid: BleUuid, flags: EnumSet<BleGattCharFlag>, access: F) -> Self
    where
        F: FnMut(BleGattAccess) -> i32 + Send + 'ble,
    {
        Self {
            uuid,
            flags,
            access: Box::new(access),
        }
    }
}

/// A GATT service definition.
pub struct BleGattService<'ble> {
    primary: bool,
    uuid: BleUuid,
    characteristics: Vec<BleGattCharacteristic<'ble>>,
}

impl<'ble> BleGattService<'ble> {
    pub fn new(
        primary: bool,
        uuid: BleUuid,
        characteristics: Vec<BleGattCharacteristic<'ble>>,
    ) -> Self {
        Self {
            primary,
            uuid,
            characteristics,
        }
    }
}

/// This defines GATT services as a tree structure. You allocate this
/// and let the BLE stack borrow it. You can add callbacks to handle reads/writes, and,
/// once the stack is started, learn the attribute handles the BLE stack assigns via
/// [`BleSetup::on_gatts_register`](super::super::BleSetup::on_gatts_register).
pub struct BleGattServices<'ble> {
    // There are dragons here. The overall structure combines safe Rust types with the
    // c-level service definitions NimBLE expects. The structure is self-referential, so
    // it's critical that any pointers in svc_defs and _chr_defs go to locations that are
    // stable even if the BleGattServices structure moves (eg. Boxed places).
    _services: Vec<BleGattService<'ble>>,
    _chr_defs: Vec<Box<[ble_gatt_chr_def]>>,
    svc_defs: Box<[ble_gatt_svc_def]>,
}

impl<'ble> BleGattServices<'ble> {
    pub fn new(mut services: Vec<BleGattService<'ble>>) -> Self {
        let mut chr_storage: Vec<Box<[ble_gatt_chr_def]>> = Vec::with_capacity(services.len());
        let mut svc_defs: Vec<ble_gatt_svc_def> = Vec::with_capacity(services.len() + 1);

        // Build the C def arrays with pointers into `services`; it is moved into
        // `self` afterwards, which relocates only the Vec handle, not the
        // heap-allocated elements the pointers target.
        for service in &mut services {
            let mut chr_defs: Vec<ble_gatt_chr_def> =
                Vec::with_capacity(service.characteristics.len() + 1);

            for chr in &mut service.characteristics {
                let uuid = chr.uuid.as_ptr();
                let flags = flags_to_repr(chr.flags);
                let arg = &mut chr.access as *mut Box<AccessClosure<'ble>> as *mut c_void;

                // `val_handle` is left null: handles are reported via `BleGattRegister`
                // rather than written back through a caller-owned pointer, because I wasn't
                // able to think up a safe API for the in-tree val_handles.
                chr_defs.push(ble_gatt_chr_def {
                    uuid,
                    access_cb: Some(access_trampoline),
                    arg,
                    flags,
                    val_handle: ptr::null_mut(),
                    ..Default::default()
                });
            }
            chr_defs.push(ble_gatt_chr_def::default());

            let chr_defs = chr_defs.into_boxed_slice();
            let chr_ptr = chr_defs.as_ptr();
            chr_storage.push(chr_defs);

            svc_defs.push(ble_gatt_svc_def {
                type_: if service.primary {
                    BLE_GATT_SVC_TYPE_PRIMARY as u8
                } else {
                    BLE_GATT_SVC_TYPE_SECONDARY as u8
                },
                uuid: service.uuid.as_ptr(),
                includes: ptr::null_mut(),
                characteristics: chr_ptr,
            });
        }
        svc_defs.push(ble_gatt_svc_def::default());

        Self {
            _services: services,
            _chr_defs: chr_storage,
            svc_defs: svc_defs.into_boxed_slice(),
        }
    }
}

/// GATT-server setup routines, mapping to the ble_gatts_* family of NimBLE functions;
/// this is available while you have a BleSetup instance, so before you've called start.
pub struct GattsSetup<'a, 'ble> {
    _setup: &'a mut BleSetup<'ble>,
}

impl<'a, 'ble> GattsSetup<'a, 'ble> {
    pub fn new(setup: &'a mut BleSetup<'ble>) -> Self {
        Self { _setup: setup }
    }

    /// Queue `services` to be registered once BleSetup#start() is called.
    pub fn add_services(&mut self, services: &'ble BleGattServices<'ble>) -> Result<(), BleError> {
        let svc_defs = services.svc_defs.as_ptr();

        BleError::from_raw(unsafe { ble_gatts_count_cfg(svc_defs) })?;
        BleError::from_raw(unsafe { ble_gatts_add_svcs(svc_defs) })
    }
}

/// Sends a "free-form" characteristic indication.
pub fn indicate(
    conn_handle: ConnectionId,
    val_handle: Handle,
    data: &[u8],
) -> Result<(), BleError> {
    let om = mbuf_from_slice(data)?;

    // `ble_gatts_indicate_custom` takes ownership of `om` and frees it on all paths (no leak, no double-free).
    BleError::from_raw(unsafe { ble_gatts_indicate_custom(conn_handle, val_handle, om) })
}
