//! Safe wrapper for the ESP-IDF NimBLE BLE host.

use core::cell::UnsafeCell;
use core::ffi::{c_int, c_void};
use core::fmt;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};

use alloc::boxed::Box;
use alloc::sync::Arc;

use crate::hal::modem::BluetoothModemPeripheral;
use crate::private::mutex::Mutex;
use crate::sys::*;

pub mod gap;
pub mod gatt;
pub mod mbuf;

/// A BLE UUID, either 16-bit (assigned) or 128-bit (vendor-specific).
#[derive(Clone, Copy, Debug)]
pub enum BleUuid {
    Uuid16(ble_uuid16_t),
    Uuid128(ble_uuid128_t),
}

impl BleUuid {
    pub const fn uuid16(uuid: u16) -> Self {
        Self::Uuid16(ble_uuid16_t {
            u: ble_uuid_t {
                type_: BLE_UUID_TYPE_16 as u8,
            },
            value: uuid,
        })
    }

    pub const fn uuid128(uuid: u128) -> Self {
        Self::Uuid128(ble_uuid128_t {
            u: ble_uuid_t {
                type_: BLE_UUID_TYPE_128 as u8,
            },
            value: uuid.to_le_bytes(),
        })
    }

    pub fn as_ptr(&self) -> *const ble_uuid_t {
        match self {
            Self::Uuid16(uuid) => &uuid.u as *const ble_uuid_t,
            Self::Uuid128(uuid) => &uuid.u as *const ble_uuid_t,
        }
    }

    /// # Safety
    ///
    /// `uuid` must point to a valid `ble_uuid_t` header and the concrete
    /// 16-/128-bit body it introduces.
    pub(crate) unsafe fn from_raw(uuid: *const ble_uuid_t) -> Self {
        match unsafe { (*uuid).type_ } as u32 {
            BLE_UUID_TYPE_128 => Self::Uuid128(unsafe { *uuid.cast::<ble_uuid128_t>() }),
            // Only 16- and 128-bit UUIDs are modelled; anything else reads as 16-bit.
            _ => Self::Uuid16(unsafe { *uuid.cast::<ble_uuid16_t>() }),
        }
    }
}

impl PartialEq for BleUuid {
    fn eq(&self, other: &Self) -> bool {
        unsafe { ble_uuid_cmp(self.as_ptr(), other.as_ptr()) == 0 }
    }
}

impl Eq for BleUuid {}

#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct BleAddr(ble_addr_t);

impl BleAddr {
    pub const fn new(kind: u8, val: [u8; 6]) -> Self {
        Self(ble_addr_t { type_: kind, val })
    }

    pub const fn raw(&self) -> &ble_addr_t {
        &self.0
    }

    pub const fn kind(&self) -> u8 {
        self.0.type_
    }

    pub const fn val(&self) -> [u8; 6] {
        self.0.val
    }
}

impl fmt::Display for BleAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = &self.0.val;
        write!(
            f,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            v[5], v[4], v[3], v[2], v[1], v[0]
        )
    }
}

impl From<ble_addr_t> for BleAddr {
    fn from(addr: ble_addr_t) -> Self {
        Self(addr)
    }
}

impl From<BleAddr> for ble_addr_t {
    fn from(addr: BleAddr) -> Self {
        addr.0
    }
}

/// Attempt to configure at least one BLE address; how this is done is hardware-specific.
/// If prefer_random is true, prefer using a random address even if a public address is configured.
pub fn ensure_addr(prefer_random: bool) -> Result<(), BleError> {
    BleError::from_raw(unsafe { ble_hs_util_ensure_addr(prefer_random as c_int) })
}

/// Read back the device's identity address of the given type.
pub fn id_copy_addr(kind: u8) -> Result<BleAddr, BleError> {
    let mut val = [0u8; 6];
    BleError::from_raw(unsafe {
        ble_hs_id_copy_addr(kind, val.as_mut_ptr(), core::ptr::null_mut())
    })?;

    Ok(BleAddr::new(kind, val))
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct BleError(c_int);

impl BleError {
    pub const fn new(rc: c_int) -> Self {
        Self(rc)
    }

    pub const fn code(&self) -> c_int {
        self.0
    }

    pub fn from_raw(rc: c_int) -> Result<(), Self> {
        if rc == 0 {
            Ok(())
        } else {
            Err(Self(rc))
        }
    }

    fn name(&self) -> &'static str {
        match self.0 as u32 {
            BLE_HS_EALREADY => "BLE_HS_EALREADY",
            BLE_HS_EDONE => "BLE_HS_EDONE",
            BLE_HS_ENOMEM => "BLE_HS_ENOMEM",
            BLE_HS_ETIMEOUT => "BLE_HS_ETIMEOUT",
            _ => "BLE_HS_E*",
        }
    }
}

impl fmt::Debug for BleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BleError({}, {})", self.0, self.name())
    }
}

impl fmt::Display for BleError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NimBLE error {} ({})", self.0, self.name())
    }
}

#[cfg(feature = "std")]
impl std::error::Error for BleError {}

impl From<BleError> for EspError {
    /// NimBLE host codes (`BLE_HS_E*`) are a separate namespace from `esp_err_t`
    /// with no faithful mapping, so any [`BleError`] collapses to `ESP_FAIL`. Match
    /// on the [`BleError`] directly if you need the specific NimBLE code.
    fn from(_err: BleError) -> Self {
        EspError::from_infallible::<ESP_FAIL>()
    }
}

/// Security Manager (SMP) configuration, applied via
/// [`BleSetup::set_security`](BleSetup::set_security) before the host starts.
#[derive(Clone, Copy)]
pub struct BleSecurity {
    /// Local IO capabilities (`BLE_HS_IO_*`).
    pub io_cap: u8,
    pub oob_data_flag: bool,
    pub bonding: bool,
    pub mitm: bool,
    /// LE Secure Connections.
    pub secure_connections: bool,
    /// Restrict pairing to LE Secure Connections only.
    pub secure_connections_only: bool,
    pub keypress: bool,
    /// Minimum GATT security level (`sm_sec_lvl`); 0 is ignored.
    pub min_sec_level: u8,
    /// Keys we distribute (`BLE_SM_PAIR_KEY_DIST_*` mask).
    pub our_key_dist: u8,
    /// Keys the peer distributes (`BLE_SM_PAIR_KEY_DIST_*` mask).
    pub their_key_dist: u8,
}

impl Default for BleSecurity {
    fn default() -> Self {
        Self {
            io_cap: BLE_HS_IO_NO_INPUT_OUTPUT as u8,
            oob_data_flag: false,
            bonding: false,
            mitm: false,
            secure_connections: false,
            secure_connections_only: false,
            keypress: false,
            min_sec_level: 0,
            our_key_dist: 0,
            their_key_dist: 0,
        }
    }
}

#[allow(dead_code)]
#[allow(clippy::type_complexity)]
pub(crate) struct BleCallback<A, R> {
    callback: Mutex<Option<Arc<UnsafeCell<Box<dyn FnMut(A) -> R>>>>>,
    default_result: R,
}

#[allow(dead_code)]
impl<A, R> BleCallback<A, R>
where
    R: Clone,
{
    pub const fn new(default_result: R) -> Self {
        Self {
            callback: Mutex::new(None),
            default_result,
        }
    }

    pub fn subscribe<F>(&self, callback: F)
    where
        F: FnMut(A) -> R + Send + 'static,
    {
        unsafe { self.subscribe_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The stored slot is `'static`; this erases the callback's lifetime. The
    /// caller must ensure the callback (and everything it borrows) stays valid
    /// until it is unsubscribed via `unsubscribe`.
    pub unsafe fn subscribe_nonstatic<'a, F>(&self, callback: F)
    where
        F: FnMut(A) -> R + Send + 'a,
    {
        let callback: Box<dyn FnMut(A) -> R + 'a> = Box::new(callback);
        let callback: Box<dyn FnMut(A) -> R + 'static> = unsafe { core::mem::transmute(callback) };
        *self.callback.lock() = Some(Arc::new(UnsafeCell::new(callback)));
    }

    pub fn unsubscribe(&self) {
        *self.callback.lock() = None;
    }

    /// # Safety
    ///
    /// Safe to use only from within the NimBLE host task.
    pub unsafe fn call(&self, arg: A) -> R {
        if let Some(callback) = self
            .callback
            .lock()
            .as_ref()
            .map(|callback| callback.clone())
        {
            ((callback.get()).as_mut().unwrap())(arg)
        } else {
            self.default_result.clone()
        }
    }
}

unsafe impl<A, R> Sync for BleCallback<A, R> {}
unsafe impl<A, R> Send for BleCallback<A, R> {}

/// The NimBLE stack has several globally-singleton things; we enforce that by
/// the calling take/release on this. BleSingleton also wraps the globally singleton state
/// that requires well-known static addresses.
#[allow(dead_code)]
pub(crate) struct BleSingleton {
    initialized: AtomicBool,
    sync: BleCallback<(), ()>,
    reset: BleCallback<i32, ()>,
    gap_event: BleCallback<gap::BleGapEvent, i32>,
    gatts_register: BleCallback<gatt::gatts::BleGattRegister, ()>,
}

#[allow(dead_code)]
impl BleSingleton {
    pub const fn new() -> Self {
        Self {
            initialized: AtomicBool::new(false),
            sync: BleCallback::new(()),
            reset: BleCallback::new(()),
            gap_event: BleCallback::new(0),
            gatts_register: BleCallback::new(()),
        }
    }

    pub fn take(&self) -> Result<(), EspError> {
        self.initialized
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .map_err(|_| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;

        Ok(())
    }

    pub fn release(&self) -> Result<(), EspError> {
        self.initialized
            .compare_exchange(true, false, Ordering::SeqCst, Ordering::SeqCst)
            .map_err(|_| EspError::from_infallible::<ESP_ERR_INVALID_STATE>())?;

        Ok(())
    }
}

static SINGLETON: BleSingleton = BleSingleton::new();

unsafe extern "C" fn host_sync_cb() {
    unsafe { SINGLETON.sync.call(()) }
}

unsafe extern "C" fn host_reset_cb(reason: i32) {
    unsafe { SINGLETON.reset.call(reason) }
}

unsafe extern "C" fn gap_event_cb(event: *mut ble_gap_event, _arg: *mut c_void) -> c_int {
    let event = gap::BleGapEvent::from(unsafe { &*event });

    unsafe { SINGLETON.gap_event.call(event) }
}

unsafe extern "C" fn gatts_register_cb(ctxt: *mut ble_gatt_register_ctxt, _arg: *mut c_void) {
    let event = gatt::gatts::BleGattRegister::from(unsafe { &*ctxt });

    unsafe { SINGLETON.gatts_register.call(event) }
}

unsafe extern "C" fn host_task(_arg: *mut c_void) {
    unsafe {
        nimble_port_run();
        nimble_port_freertos_deinit();
    }
}

/// The NimBLE host handle, use BleSetup to create this. The BLE stack stays up until
/// this is dropped. There can only be one of these at a time.
pub struct BleDriver<'ble> {
    started: bool,
    _p: PhantomData<&'ble mut ()>,
}

impl Drop for BleDriver<'_> {
    fn drop(&mut self) {
        if self.started {
            let _ = unsafe { nimble_port_stop() };
        }

        esp!(unsafe { nimble_port_deinit() }).unwrap();

        unsafe {
            let cfg = core::ptr::addr_of_mut!(ble_hs_cfg);
            (*cfg).sync_cb = None;
            (*cfg).reset_cb = None;
            (*cfg).gatts_register_cb = None;
        }

        SINGLETON.sync.unsubscribe();
        SINGLETON.reset.unsubscribe();
        SINGLETON.gap_event.unsubscribe();
        SINGLETON.gatts_register.unsubscribe();
        let _ = SINGLETON.release();
    }
}

/// This is your primary entrypoint to setup BLE. Create a new instance of this, set up
/// callbacks and then start it to get the driver instance.
pub struct BleSetup<'ble> {
    driver: BleDriver<'ble>,
}

impl<'ble> BleSetup<'ble> {
    pub fn new<M: BluetoothModemPeripheral + 'ble>(_modem: M) -> Result<Self, EspError> {
        SINGLETON.take()?;

        esp!(unsafe { nimble_port_init() })?;

        unsafe {
            ble_svc_gap_init();
            ble_svc_gatt_init();
        }

        Ok(Self {
            driver: BleDriver {
                started: false,
                _p: PhantomData,
            },
        })
    }

    /// Once the BLE stack is "in sync", this gets called; you should delay BLE operations until
    /// this gets called. Note the stack can also go out-of-sync for various reasons, indicated by
    /// on_reset being called followed by this being called again, so the callback must be re-entrant.
    /// See https://mynewt.apache.org/latest/network/ble_setup/ble_sync_cb.html
    pub fn on_sync<F>(&self, callback: F)
    where
        F: FnMut() + Send + 'static,
    {
        unsafe { self.on_sync_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// This method - in contrast to `on_sync` - allows a non-`'static` callback,
    /// so it may borrow variables that live as long as the [`BleDriver`] returned
    /// by `start`. The callback stays registered with the running NimBLE host task
    /// until the driver is dropped, which un-subscribes it.
    ///
    /// Care must be taken NOT to `core::mem::forget` the driver: that skips the
    /// un-subscription, leaving the host task holding a callback with dangling
    /// borrows. This "local borrowing" can only be expressed safely once/if `!Leak`
    /// types are introduced to Rust.
    pub unsafe fn on_sync_nonstatic<F>(&self, mut callback: F)
    where
        F: FnMut() + Send + 'ble,
    {
        unsafe { SINGLETON.sync.subscribe_nonstatic(move |()| callback()) };

        unsafe {
            (*core::ptr::addr_of_mut!(ble_hs_cfg)).sync_cb = Some(host_sync_cb);
        }
    }

    /// Called if the BLE host goes out of sync with the controller after on_sync has been called.
    /// See https://mynewt.apache.org/latest/network/ble_setup/ble_sync_cb.html
    pub fn on_reset<F>(&self, callback: F)
    where
        F: FnMut(i32) + Send + 'static,
    {
        unsafe { self.on_reset_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The non-`'static` counterpart of `on_reset`. See
    /// [`on_sync_nonstatic`](Self::on_sync_nonstatic) for the borrowing rules and
    /// the `core::mem::forget` hazard.
    pub unsafe fn on_reset_nonstatic<F>(&self, callback: F)
    where
        F: FnMut(i32) + Send + 'ble,
    {
        unsafe { SINGLETON.reset.subscribe_nonstatic(callback) };

        unsafe {
            (*core::ptr::addr_of_mut!(ble_hs_cfg)).reset_cb = Some(host_reset_cb);
        }
    }

    pub fn on_gap_event<F>(&self, callback: F)
    where
        F: FnMut(gap::BleGapEvent) -> i32 + Send + 'static,
    {
        unsafe { self.on_gap_event_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The non-`'static` counterpart of `on_gap_event`. See
    /// [`on_sync_nonstatic`](Self::on_sync_nonstatic) for the borrowing rules and
    /// the `core::mem::forget` hazard.
    pub unsafe fn on_gap_event_nonstatic<F>(&self, callback: F)
    where
        F: FnMut(gap::BleGapEvent) -> i32 + Send + 'ble,
    {
        unsafe { SINGLETON.gap_event.subscribe_nonstatic(callback) };
    }

    /// Called on the host task as each GATT service, characteristic, and descriptor
    /// is registered, carrying the attribute handles NimBLE assigned. Capture the
    /// value handles you need here (e.g. by UUID).
    pub fn on_gatts_register<F>(&self, callback: F)
    where
        F: FnMut(gatt::gatts::BleGattRegister) + Send + 'static,
    {
        unsafe { self.on_gatts_register_nonstatic(callback) }
    }

    /// # Safety
    ///
    /// The non-`'static` counterpart of `on_gatts_register`. See
    /// [`on_sync_nonstatic`](Self::on_sync_nonstatic) for the borrowing rules and
    /// the `core::mem::forget` hazard.
    pub unsafe fn on_gatts_register_nonstatic<F>(&self, callback: F)
    where
        F: FnMut(gatt::gatts::BleGattRegister) + Send + 'ble,
    {
        unsafe { SINGLETON.gatts_register.subscribe_nonstatic(callback) };

        unsafe {
            (*core::ptr::addr_of_mut!(ble_hs_cfg)).gatts_register_cb = Some(gatts_register_cb);
        }
    }

    /// Configure the Security Manager (SMP) parameters. Must be called before
    /// [`start`](Self::start); the settings take effect once the host task runs.
    pub fn set_security(&self, security: &BleSecurity) {
        unsafe {
            let cfg = core::ptr::addr_of_mut!(ble_hs_cfg);
            (*cfg).sm_io_cap = security.io_cap;
            (*cfg).set_sm_oob_data_flag(security.oob_data_flag as _);
            (*cfg).set_sm_bonding(security.bonding as _);
            (*cfg).set_sm_mitm(security.mitm as _);
            (*cfg).set_sm_sc(security.secure_connections as _);
            (*cfg).set_sm_sc_only(security.secure_connections_only as _);
            (*cfg).set_sm_keypress(security.keypress as _);
            (*cfg).sm_sec_lvl = security.min_sec_level;
            (*cfg).sm_our_key_dist = security.our_key_dist;
            (*cfg).sm_their_key_dist = security.their_key_dist;
        }
    }

    /// Once you are done with setup, call this function to start the BLE task. The task will
    /// start in the background and call [`on_sync`](Self::on_sync) once it's ready for use.
    /// You must retain the driver, the BLE stack is stopped when it drops.
    pub fn start(mut self) -> Result<BleDriver<'ble>, EspError> {
        unsafe { nimble_port_freertos_init(Some(host_task)) };

        self.driver.started = true;

        Ok(self.driver)
    }
}
