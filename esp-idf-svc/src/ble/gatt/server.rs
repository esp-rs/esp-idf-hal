//! NimBLE GATT server: the service table, its events, and server operations on the [`BleDriver`].

use core::ptr;

use alloc::boxed::Box;
use alloc::vec::Vec;

use enumset::EnumSet;

use crate::sys::*;

use super::super::mbuf::{mbuf_from_slice, Mbuf};
use super::super::{BleDriver, BleError, BleUuid, ConnHandle};
use super::{flags_to_repr, AttrHandle, BleGattCharFlag};

/// A GATT-server event, delivered on the host task to the single
/// [`gatts_subscribe`](BleDriver::gatts_subscribe) hook.
///
/// There are no per-characteristic callbacks: NimBLE dispatches *every* characteristic read and
/// write through one shared trampoline, and they arrive here as [`Read`](Self::Read) /
/// [`Write`](Self::Write), keyed by the globally-unique `attr_handle`. The
/// [`Register`](Self::Register) variants fire as the service table is registered (at `start`).
/// [`SubscriptionChanged`](Self::SubscriptionChanged) and [`NotifyComplete`](Self::NotifyComplete)
/// are server-role connection events that NimBLE delivers on the GAP callback and we demux here.
///
/// The hook returns the ATT status (`0` on success) for `Read`/`Write`; the return is ignored for
/// the others.
pub enum GattsEvent<'a> {
    Register(BleGattRegister),
    /// A peer is reading one of our characteristics; append the value to `reply`.
    ///
    /// This covers every ATT read (Read Request, Read Blob Request, Read By Type Request, Read
    /// Multiple Request) — NimBLE reports them all the same way — as well as *local* reads, which
    /// carry [`CONN_HANDLE_NONE`](crate::ble::CONN_HANDLE_NONE) instead of a real connection.
    Read {
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        /// Non-zero only for a long read (ATT Read Blob Request), where it is the offset the peer
        /// is asking to continue from.
        ///
        /// **Always append the whole value regardless of this field**: NimBLE hands a long read a
        /// scratch buffer and slices `[offset..]` out of it itself. The offset is informational —
        /// use it to tell a continuation from a fresh read (e.g. to snapshot the value at offset
        /// `0` so a multi-blob read stays coherent).
        ///
        /// Always `0` on ESP-IDF < 5.3, where NimBLE does not report the offset at all.
        offset: u16,
        reply: Mbuf<'a>,
    },
    /// A peer wrote one of our characteristics.
    ///
    /// This covers every ATT write — Write Request, Write Command (write-without-response), Signed
    /// Write Command, and a completed Prepare/Execute long write (NimBLE coalesces the queued
    /// fragments into one event) — as NimBLE does not report which opcode carried the write. It
    /// need not be distinguished: for a Write Request the status returned by the hook becomes the
    /// ATT error response, and for a Write Command (which has no response) it is discarded. Local
    /// writes arrive here too, with [`CONN_HANDLE_NONE`](crate::ble::CONN_HANDLE_NONE).
    Write {
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        data: Mbuf<'a>,
    },
    /// A peer's subscription state for one of our characteristics changed: it wrote the CCCD, the
    /// connection is going down, or a bond was restored — see `reason`. The `prev_*` / `cur_*`
    /// pairs give the edge, so no shadow state is needed to tell a subscribe from an unsubscribe.
    SubscriptionChanged {
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        reason: SubscribeReason,
        prev_notify: bool,
        cur_notify: bool,
        prev_indicate: bool,
        cur_indicate: bool,
    },
    /// An indication/notification we sent completed (for an indication, `status` is the peer's
    /// confirmation result).
    NotifyComplete {
        conn_handle: ConnHandle,
        attr_handle: AttrHandle,
        indication: bool,
        status: i32,
    },
}

impl GattsEvent<'static> {
    /// Build the server-role `SubscriptionChanged` / `NotifyComplete` events from a raw GAP event.
    /// Returns `None` for any other event type. Called from the GAP trampoline's demux.
    pub(crate) fn from_gap(event: &ble_gap_event) -> Option<Self> {
        let anon = &event.__bindgen_anon_1;

        match event.type_ as u32 {
            BLE_GAP_EVENT_SUBSCRIBE => {
                let subscribe = unsafe { &anon.subscribe };
                Some(Self::SubscriptionChanged {
                    conn_handle: subscribe.conn_handle,
                    attr_handle: subscribe.attr_handle,
                    reason: SubscribeReason::from_raw(subscribe.reason),
                    prev_notify: subscribe.prev_notify() != 0,
                    cur_notify: subscribe.cur_notify() != 0,
                    prev_indicate: subscribe.prev_indicate() != 0,
                    cur_indicate: subscribe.cur_indicate() != 0,
                })
            }
            BLE_GAP_EVENT_NOTIFY_TX => {
                let notify_tx = unsafe { &anon.notify_tx };
                Some(Self::NotifyComplete {
                    conn_handle: notify_tx.conn_handle,
                    attr_handle: notify_tx.attr_handle,
                    indication: notify_tx.indication() != 0,
                    status: notify_tx.status,
                })
            }
            _ => None,
        }
    }
}

/// Why a peer's subscription state changed (the `reason` of
/// [`GattsEvent::SubscriptionChanged`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum SubscribeReason {
    /// The peer wrote the characteristic's CCCD.
    Write,
    /// The connection is about to be terminated, so NimBLE is clearing the peer's subscription
    /// state. The `cur_*` flags are therefore `false` — this is *not* the peer opting out.
    Term,
    /// A bond was restored from persistence and the peer's subscription state came back with it.
    /// Nothing was written on the wire; the peer is subscribed as of now.
    Restore,
    /// A reason code unknown to this version of the crate.
    Other(u8),
}

impl SubscribeReason {
    fn from_raw(reason: u8) -> Self {
        match reason as u32 {
            BLE_GAP_SUBSCRIBE_REASON_WRITE => Self::Write,
            BLE_GAP_SUBSCRIBE_REASON_TERM => Self::Term,
            BLE_GAP_SUBSCRIBE_REASON_RESTORE => Self::Restore,
            _ => Self::Other(reason),
        }
    }
}

/// A GATT registration event (the payload of [`GattsEvent::Register`]). Capture the value handles
/// you need (matching on `uuid`) here.
pub enum BleGattRegister {
    Service {
        uuid: BleUuid,
        handle: AttrHandle,
    },
    Characteristic {
        uuid: BleUuid,
        def_handle: AttrHandle,
        val_handle: AttrHandle,
    },
    Descriptor {
        uuid: BleUuid,
        handle: AttrHandle,
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

/// A characteristic in a [`BleGattService`] — just its UUID and flags. Reads and writes are
/// serviced by the single [`gatts_subscribe`](BleDriver::gatts_subscribe) hook (dispatched by the
/// value handle reported via [`BleGattRegister`]), so there is no per-characteristic closure and
/// no per-characteristic allocation.
pub struct BleGattCharacteristic {
    uuid: BleUuid,
    flags: EnumSet<BleGattCharFlag>,
}

impl BleGattCharacteristic {
    pub fn new(uuid: BleUuid, flags: EnumSet<BleGattCharFlag>) -> Self {
        Self { uuid, flags }
    }
}

/// A GATT service definition.
pub struct BleGattService {
    primary: bool,
    uuid: BleUuid,
    characteristics: Vec<BleGattCharacteristic>,
}

impl BleGattService {
    pub fn new(primary: bool, uuid: BleUuid, characteristics: Vec<BleGattCharacteristic>) -> Self {
        Self {
            primary,
            uuid,
            characteristics,
        }
    }
}

/// A GATT service table built **at runtime** (heap-allocated), as the raw NimBLE
/// `ble_gatt_svc_def` tree, ready to hand to
/// [`BleDriver::new_with_services`](crate::ble::BleDriver::new_with_services). Implements
/// `AsRef<[ble_gatt_svc_def]>`, so it is one valid `S`; a `&'static` table or a
/// `Box<[ble_gatt_svc_def]>` are others. For a **compile-time** table (no heap), see the
/// [`gatt_services!`](crate::gatt_services) macro.
pub struct BleGattServices {
    // There are dragons here. The C def arrays hold raw pointers into `_services` (UUIDs) and into
    // `_chr_defs`. Those targets are heap-allocated, so they stay put when this struct's handle
    // moves — which is what keeps the pointers valid. All characteristics share the crate's single
    // access trampoline (`gatts_access_cb`); `arg` and `val_handle` are null.
    _services: Vec<BleGattService>,
    _chr_defs: Vec<Box<[ble_gatt_chr_def]>>,
    svc_defs: Box<[ble_gatt_svc_def]>,
}

impl BleGattServices {
    pub fn new(services: Vec<BleGattService>) -> Self {
        let mut chr_storage: Vec<Box<[ble_gatt_chr_def]>> = Vec::with_capacity(services.len());
        let mut svc_defs: Vec<ble_gatt_svc_def> = Vec::with_capacity(services.len() + 1);

        for service in &services {
            let mut chr_defs: Vec<ble_gatt_chr_def> =
                Vec::with_capacity(service.characteristics.len() + 1);

            for chr in &service.characteristics {
                chr_defs.push(ble_gatt_chr_def {
                    uuid: chr.uuid.as_ptr(),
                    // The one trampoline for every characteristic; `attr_handle` disambiguates.
                    access_cb: Some(super::super::BleSingleton::gatts_access_cb),
                    arg: ptr::null_mut(),
                    flags: flags_to_repr(chr.flags),
                    // Handles are captured from the registration event, not written back here.
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

impl AsRef<[ble_gatt_svc_def]> for BleGattServices {
    fn as_ref(&self) -> &[ble_gatt_svc_def] {
        &self.svc_defs
    }
}

/// GATT-server operations on the [`BleDriver`], available only when the driver was built with a
/// service table (`S: AsRef<[ble_gatt_svc_def]>`) via
/// [`new_with_services`](BleDriver::new_with_services). `&self`, so callable re-entrantly.
#[cfg(esp_idf_bt_nimble_gatt_server)]
impl<'d, S> BleDriver<'d, S>
where
    S: AsRef<[ble_gatt_svc_def]>,
{
    /// Subscribe to GATT-server events ([`GattsEvent`]). Set this **before**
    /// [`start`](BleDriver::start): the `Register` events (carrying the attribute handles NimBLE
    /// assigned) fire during host start.
    pub fn gatts_subscribe<F>(&self, callback: F)
    where
        F: for<'a> FnMut(GattsEvent<'a>) -> u8 + Send + 'static,
    {
        unsafe { self.gatts_subscribe_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The non-`'static` counterpart of [`gatts_subscribe`](Self::gatts_subscribe). See
    /// [`BleDriver::host_subscribe_nonstatic`](crate::ble::BleDriver::host_subscribe_nonstatic) for the borrowing
    /// rules and the `core::mem::forget` hazard.
    pub unsafe fn gatts_subscribe_nonstatic<F>(&self, callback: F)
    where
        F: for<'a> FnMut(GattsEvent<'a>) -> u8 + Send + 'd,
    {
        // The `gatts_register_cb` trampoline is installed once at construction (see
        // `BleDriver::new_with_services`), so subscribing only swaps the mutex-guarded `SINGLETON`
        // slot — no `ble_hs_cfg` write here.
        unsafe { super::super::SINGLETON.gatts.subscribe_nonstatic(callback) };
    }

    /// Stop delivering GATT-server events to the subscribed hook.
    pub fn gatts_unsubscribe(&self) {
        super::super::SINGLETON.gatts.unsubscribe();
    }

    /// Send a "free-form" characteristic indication to `conn_handle`.
    pub fn indicate(
        &self,
        conn_handle: ConnHandle,
        val_handle: AttrHandle,
        data: &[u8],
    ) -> Result<(), BleError> {
        let om = mbuf_from_slice(data)?;

        // `ble_gatts_indicate_custom` takes ownership of `om` and frees it on all paths (no leak, no double-free).
        BleError::from_raw(unsafe { ble_gatts_indicate_custom(conn_handle, val_handle, om) })
    }
}

// -------------------------------------------------------------------------------------------------
// Static (compile-time) service tables — see the `gatt_services!` macro.
//
// The C def structs hold raw pointers, so they are `!Sync` and cannot go in a `static` directly.
// These `#[repr(transparent)]` wrappers add the `unsafe impl Sync`; the macro then builds the
// null-terminated tree bottom-up out of *block-scoped inner statics*, which is what gives every
// pointer target a stable `'static` address (a `const` has no stable address; a `static` does).
// -------------------------------------------------------------------------------------------------

/// A **static** (compile-time) GATT service table: a null-terminated `ble_gatt_svc_def` array
/// wrapped so it can live in a `static`. Build one with the [`gatt_services!`](crate::gatt_services)
/// macro and pass `&MY_SERVICES` to [`new_with_services`](BleDriver::new_with_services) — it needs
/// no heap and lands in flash. `N` (services + terminator) is inferred by the macro; you never write
/// it.
#[repr(transparent)]
pub struct GattServices<const N: usize>([ble_gatt_svc_def; N]);

// SAFETY: the raw pointers in the table only address other items of the same `'static` tree, and
// NimBLE consumes the table read-only (assigned handles are reported via the register events, not
// written back into it).
unsafe impl<const N: usize> Sync for GattServices<N> {}

impl<const N: usize> GattServices<N> {
    /// Wrap a fully-built, null-terminated service array. Prefer the
    /// [`gatt_services!`](crate::gatt_services) macro over calling this directly.
    #[doc(hidden)]
    pub const fn new(defs: [ble_gatt_svc_def; N]) -> Self {
        Self(defs)
    }
}

impl<const N: usize> AsRef<[ble_gatt_svc_def]> for GattServices<N> {
    fn as_ref(&self) -> &[ble_gatt_svc_def] {
        &self.0
    }
}

/// The characteristics of one service, as a null-terminated `ble_gatt_chr_def` array wrapped for
/// `static` storage. An implementation detail of [`gatt_services!`](crate::gatt_services).
#[doc(hidden)]
#[repr(transparent)]
pub struct GattChrs<const N: usize>([ble_gatt_chr_def; N]);

// SAFETY: as for `GattServices`.
unsafe impl<const N: usize> Sync for GattChrs<N> {}

impl<const N: usize> GattChrs<N> {
    pub const fn new(defs: [ble_gatt_chr_def; N]) -> Self {
        Self(defs)
    }

    pub const fn as_ptr(&self) -> *const ble_gatt_chr_def {
        self.0.as_ptr()
    }
}

// All-zero templates. A zeroed `ble_gatt_*_def` is exactly the null terminator, and is the base the
// builders below fill in — the `const` analog of the runtime path's `..Default::default()`, so that
// fields we do not set (and any the bindings gain across ESP-IDF versions, e.g. `cpfd`) are zeroed
// without having to enumerate them. Every field is a pointer / fn-pointer / integer, so all-zero is
// a valid, meaningful value.
const ZERO_SVC: ble_gatt_svc_def = unsafe { core::mem::MaybeUninit::zeroed().assume_init() };
const ZERO_CHR: ble_gatt_chr_def = unsafe { core::mem::MaybeUninit::zeroed().assume_init() };

/// The service-array terminator. Public only for the macro.
#[doc(hidden)]
pub const SVC_SENTINEL: ble_gatt_svc_def = ZERO_SVC;

/// The characteristic-array terminator. Public only for the macro.
#[doc(hidden)]
pub const CHR_SENTINEL: ble_gatt_chr_def = ZERO_CHR;

/// Build one characteristic def for a static table. Public only for the macro. Wires the crate's
/// single access trampoline (as the runtime path does); the assigned handle is learned from the
/// register events, so `val_handle` is null.
#[doc(hidden)]
pub const fn make_chr(uuid: *const ble_uuid_t, flags: ble_gatt_chr_flags) -> ble_gatt_chr_def {
    ble_gatt_chr_def {
        uuid,
        access_cb: Some(super::super::BleSingleton::gatts_access_cb),
        flags,
        ..ZERO_CHR
    }
}

/// Build one service def for a static table. Public only for the macro.
#[doc(hidden)]
pub const fn make_svc(
    primary: bool,
    uuid: *const ble_uuid_t,
    characteristics: *const ble_gatt_chr_def,
) -> ble_gatt_svc_def {
    ble_gatt_svc_def {
        type_: if primary {
            BLE_GATT_SVC_TYPE_PRIMARY as u8
        } else {
            BLE_GATT_SVC_TYPE_SECONDARY as u8
        },
        uuid,
        characteristics,
        ..ZERO_SVC
    }
}

/// Define a **static** (compile-time, heap-free) GATT service table.
///
/// Expands to a `static NAME` holding the null-terminated NimBLE `ble_gatt_svc_def` tree (services,
/// characteristics, UUIDs), all in flash. Pass `&NAME` to
/// [`new_with_services`](crate::ble::BleDriver::new_with_services). Reads and writes are serviced
/// the same way as the runtime table — through the single
/// [`gatts_subscribe`](crate::ble::BleDriver::gatts_subscribe) hook, keyed by the value handle
/// reported in the `Register` events (so learn handles there, exactly as the runtime example does).
///
/// Each UUID slot is any `const` [`BleUuid`](crate::ble::BleUuid) expression — declare your UUIDs as
/// `const FOO: BleUuid = BleUuid::uuid128(..)` (or `uuid16`) and name them. Characteristic flags are
/// any `|`-separated [`BleGattCharFlag`](crate::ble::gatt::BleGattCharFlag) variants (`Read`,
/// `Write`, `Notify`, `Indicate`). Services are `primary(..)` or `secondary(..)`.
///
/// The `NAME` and the body live inside the one macro group (a macro invocation is a single token
/// tree), so it reads `gatt_services!(NAME { .. });` — use `!{ .. }` to drop the trailing `;`.
///
/// ```ignore
/// use esp_idf_svc::ble::BleUuid;
/// use esp_idf_svc::gatt_services;
///
/// const SVC: BleUuid = BleUuid::uuid128(0xad91b201_73474047_9e173bed_82d75f9d);
/// const RECV: BleUuid = BleUuid::uuid128(0xb6fccb50_87be44f3_ae22f854_85ea42c4);
/// const HR: BleUuid = BleUuid::uuid16(0x2A37);
///
/// gatt_services!(SERVICES {
///     primary(SVC) {
///         chr(RECV, Write);
///         chr(HR, Notify | Indicate);
///     }
/// });
/// // ... let driver = BleDriver::new_with_services(modem, &SERVICES)?;
/// ```
#[macro_export]
macro_rules! gatt_services {
    // --- internal helpers (matched before the public arm) ---

    // a `*const ble_uuid_t` backed by a fresh block-scoped `'static` (stable address)
    (@uuid_ptr $uuid:expr) => {{
        static U: $crate::ble::BleUuid = $uuid;
        U.as_ptr()
    }};

    // one characteristic def
    (@chr $uuid:expr, $($flag:ident)|+ ) => {
        $crate::ble::gatt::server::make_chr(
            $crate::gatt_services!(@uuid_ptr $uuid),
            0 $( | $crate::ble::gatt::BleGattCharFlag::$flag.repr() )+,
        )
    };

    // map `primary`/`secondary` to a bool; a unit token used only for counting repetitions
    (@primary primary) => { true };
    (@primary secondary) => { false };
    (@unit $_t:expr) => { () };

    // --- public entry: `gatt_services!(NAME { primary(uuid) { chr(uuid, Flags); .. } .. })` ---
    (
        $vis:vis $NAME:ident {
            $(
                $kind:ident ( $svc_uuid:expr ) {
                    $( chr ( $chr_uuid:expr, $($flag:ident)|+ ) ; )*
                }
            )+
        }
    ) => {
        $vis static $NAME: $crate::ble::gatt::server::GattServices<
            { <[()]>::len(&[ $( $crate::gatt_services!(@unit $svc_uuid) ),+ ]) + 1 }
        > = $crate::ble::gatt::server::GattServices::new([
            $(
                {
                    static CHRS: $crate::ble::gatt::server::GattChrs<
                        { <[()]>::len(&[ $( $crate::gatt_services!(@unit $chr_uuid) ),* ]) + 1 }
                    > = $crate::ble::gatt::server::GattChrs::new([
                        $( $crate::gatt_services!(@chr $chr_uuid, $($flag)|+), )*
                        $crate::ble::gatt::server::CHR_SENTINEL
                    ]);
                    $crate::ble::gatt::server::make_svc(
                        $crate::gatt_services!(@primary $kind),
                        $crate::gatt_services!(@uuid_ptr $svc_uuid),
                        CHRS.as_ptr(),
                    )
                },
            )+
            $crate::ble::gatt::server::SVC_SENTINEL
        ]);
    };
}
