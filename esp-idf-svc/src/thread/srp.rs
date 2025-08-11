use core::ffi::{c_void, CStr};
use core::fmt::{self, Display};
use core::marker::PhantomData;
use core::net::{Ipv6Addr, SocketAddrV6};
use core::ptr::addr_of_mut;

use ::log::{debug, info, trace};

use crate::sys::{
    esp, esp_openthread_get_instance, otDnsTxtEntry, otError, otError_OT_ERROR_INVALID_ARGS,
    otError_OT_ERROR_NO_BUFS, otIp6Address, otIp6Address__bindgen_ty_1, otSrpClientAddService,
    otSrpClientClearHostAndServices, otSrpClientClearService, otSrpClientEnableAutoStartMode,
    otSrpClientGetHostInfo, otSrpClientGetServerAddress, otSrpClientGetServices,
    otSrpClientHostInfo, otSrpClientIsAutoStartModeEnabled, otSrpClientIsRunning,
    otSrpClientItemState, otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_ADDING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REFRESHING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REGISTERED,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVING,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_ADD,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REFRESH,
    otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REMOVE, otSrpClientRemoveHostAndServices,
    otSrpClientRemoveService, otSrpClientService, otSrpClientSetHostAddresses,
    otSrpClientSetHostName, otSrpClientStart, otSrpClientStop, EspError, ESP_ERR_INVALID_STATE,
};

#[cfg(not(esp_idf_version_major = "4"))]
use crate::sys::{
    otSrpClientEnableAutoHostAddress, otSrpClientGetKeyLeaseInterval, otSrpClientGetLeaseInterval,
    otSrpClientGetTtl, otSrpClientSetKeyLeaseInterval, otSrpClientSetLeaseInterval,
    otSrpClientSetTtl,
};

use crate::thread::{ot_esp, EspThread, Mode, NetifMode, ThreadDriver};

/// The unique ID of a registered SRP service
pub type SrpServiceSlot = usize;

/// An enum describing the status of either a concrete SRP service, or the SRP host.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
pub enum SrpState {
    /// The service/host is to be added/registered.
    ToAdd,
    /// The service/host is being added/registered.
    Adding,
    /// The service/host is to be refreshed (re-register to renew lease).
    ToRefresh,
    /// The service/host is being refreshed.
    Refreshing,
    /// The service/host is to be removed/unregistered.
    ToRemove,
    /// The service/host is being removed/unregistered.
    Removing,
    /// The service/host has been removed/unregistered.
    Removed,
    /// The service/host is registered.
    Registered,
    /// Any other state.
    Other(otSrpClientItemState),
}

impl Display for SrpState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ToAdd => write!(f, "To add"),
            Self::Adding => write!(f, "Adding"),
            Self::ToRefresh => write!(f, "To refresh"),
            Self::Refreshing => write!(f, "Refreshing"),
            Self::ToRemove => write!(f, "To remove"),
            Self::Removing => write!(f, "Removing"),
            Self::Removed => write!(f, "Removed"),
            Self::Registered => write!(f, "Registered"),
            Self::Other(state) => write!(f, "Other ({state})"),
        }
    }
}

#[allow(non_upper_case_globals)]
#[allow(non_snake_case)]
impl From<otSrpClientItemState> for SrpState {
    fn from(value: otSrpClientItemState) -> Self {
        match value {
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_ADD => Self::ToAdd,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_ADDING => Self::Adding,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REFRESH => Self::ToRefresh,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REFRESHING => Self::Refreshing,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_TO_REMOVE => Self::ToRemove,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVING => Self::Removing,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED => Self::Removed,
            otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REGISTERED => Self::Registered,
            other => Self::Other(other),
        }
    }
}

/// The SRP configuration of the OpenThread stack.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub struct SrpConf<'a> {
    /// SRP hostname
    pub host_name: &'a str,
    /// SRP host Ipv6 addresses.
    /// If empty, the SRP implementation will automatically set the host addresses
    /// by itself, using non-link-local addresses, once these become available.
    pub host_addrs: &'a [Ipv6Addr],
    /// SRP TTL (Time To Live) value.
    pub ttl: u32,
    /// Default lease time for SRP services if they specify 0 for their lease time.
    /// Set to 0 to use the OpenThread default value.
    pub default_lease_secs: u32,
    /// Default key lease time for SRP services' keys if they specify 0 for their key lease time.
    /// Set to 0 to use the OpenThread default value.
    pub default_key_lease_secs: u32,
}

impl SrpConf<'_> {
    /// Create a new `SrpConf` instance, wuth a host named "ot-device",
    /// no explicit host addresses, a TTL of 60 seconds, and default lease times.
    pub const fn new() -> Self {
        Self {
            host_name: "ot-device",
            host_addrs: &[],
            ttl: 60,
            default_lease_secs: 0,
            default_key_lease_secs: 0,
        }
    }

    fn store(&self, ot_srp: &mut otSrpClientHostInfo, buf: &mut [u8]) -> Result<(), EspError> {
        let (addrs, buf) = align_min::<otIp6Address>(buf, self.host_addrs.len())?;

        ot_srp.mName = store_str(self.host_name, buf)?.0.as_ptr();

        for (index, ip) in self.host_addrs.iter().enumerate() {
            let addr = &mut addrs[index];
            addr.mFields.m8 = ip.octets();
        }

        ot_srp.mAddresses = if addrs.is_empty() {
            core::ptr::null_mut()
        } else {
            addrs.as_ptr()
        };
        ot_srp.mNumAddresses = addrs.len() as _;

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            ot_srp.mAutoAddress = addrs.is_empty();
        }

        Ok(())
    }
}

impl Default for SrpConf<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// An SRP service that can be registered with the OpenThread stack.
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub struct SrpService<'a, SI, TI> {
    /// The service name.
    pub name: &'a str,
    /// The instance name.
    pub instance_name: &'a str,
    /// The subtype labels.
    pub subtype_labels: SI,
    /// The TXT entries.
    pub txt_entries: TI,
    /// The service port.
    pub port: u16,
    /// The service priority.
    pub priority: u16,
    /// The service weight.
    pub weight: u16,
    /// The service lease time in seconds.
    /// Set to 0 to use the default value as specified in `SrpConf`.
    pub lease_secs: u32,
    /// The service key lease time in seconds.
    /// Set to 0 to use the default value as specified in `SrpConf`.
    pub key_lease_secs: u32,
}

impl<'a, SI, TI> SrpService<'a, SI, TI>
where
    SI: Iterator<Item = &'a str> + Clone + 'a,
    TI: Iterator<Item = (&'a str, &'a [u8])> + Clone + 'a,
{
    fn store(&self, ot_srp: &mut otSrpClientService, buf: &mut [u8]) -> Result<(), EspError> {
        let subtype_labels_len = self.subtype_labels.clone().count();
        let txt_entries_len = self.txt_entries.clone().count();

        let (txt_entries, buf) = align_min::<otDnsTxtEntry>(buf, txt_entries_len)?;
        let (subtype_labels, buf) = align_min::<*const char>(buf, subtype_labels_len + 1)?;

        let (name, buf) = store_str(self.name, buf)?;
        let (instance_name, buf) = store_str(self.instance_name, buf)?;

        ot_srp.mName = name.as_ptr();
        ot_srp.mInstanceName = instance_name.as_ptr();

        let mut index = 0;
        let mut buf = buf;

        for subtype_label in self.subtype_labels.clone() {
            let (subtype_label, rem_buf) = store_str(subtype_label, buf)?;

            subtype_labels[index] = subtype_label.as_ptr() as *const _;

            buf = rem_buf;
            index += 1;
        }

        subtype_labels[index] = core::ptr::null();

        index = 0;

        for (key, value) in self.txt_entries.clone() {
            let txt_entry = &mut txt_entries[index];

            let (key, rem_buf) = store_str(key, buf)?;
            let (value, rem_buf) = store_data(value, rem_buf)?;

            txt_entry.mKey = key.as_ptr();
            txt_entry.mValue = value.as_ptr();
            txt_entry.mValueLength = value.len() as _;

            buf = rem_buf;
            index += 1;
        }

        ot_srp.mSubTypeLabels = subtype_labels.as_ptr() as *const _;
        ot_srp.mTxtEntries = txt_entries.as_ptr();
        ot_srp.mNumTxtEntries = txt_entries_len as _;
        ot_srp.mPort = self.port;
        ot_srp.mPriority = self.priority;
        ot_srp.mWeight = self.weight;
        #[cfg(not(esp_idf_version_major = "4"))]
        {
            ot_srp.mLease = self.lease_secs;
            ot_srp.mKeyLease = self.key_lease_secs;
        }
        ot_srp.mState = 0;
        ot_srp.mNext = core::ptr::null_mut();

        Ok(())
    }
}

impl<'a, SI, TI> Display for SrpService<'a, SI, TI>
where
    SI: Iterator<Item = &'a str> + Clone,
    TI: Iterator<Item = (&'a str, &'a [u8])> + Clone,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "SRP Service {{name: {}, instance: {}, port: {}, priority: {}, weight: {}, lease: {}, keylease: {}, labels: [",
            self.name,
            self.instance_name,
            self.port,
            self.priority,
            self.weight,
            self.lease_secs,
            self.key_lease_secs
        )?;

        for (index, label) in self.subtype_labels.clone().enumerate() {
            if index > 0 {
                write!(f, ", {label}")?;
            } else {
                write!(f, "{label}")?;
            }
        }

        write!(f, "], txt: [")?;

        for (index, value) in self.txt_entries.clone().enumerate() {
            if index > 0 {
                write!(f, ", {}: {:?}", value.0, value.1)?;
            } else {
                write!(f, "{}: {:?}", value.0, value.1)?;
            }
        }

        write!(f, "]}}")
    }
}

/// Type alias for an SRP service as returned by
/// `OpenThread::srp_services`.
pub type OutSrpService<'a> = SrpService<'a, OutSrpSubtypeLabelsIter<'a>, OutSrpTxtEntriesIter<'a>>;

/// An iterator over the subtype labels of an SRP service
/// as returned by `OpenThread::srp_services`.
#[derive(Clone)]
pub struct OutSrpSubtypeLabelsIter<'a> {
    ptr: *const *const u8,
    index: usize,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for OutSrpSubtypeLabelsIter<'a> {
    type Item = &'a str;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() {
            return None;
        }

        let label = unsafe { *self.ptr.add(self.index) };

        if label.is_null() {
            None
        } else {
            self.index += 1;
            Some(unsafe { CStr::from_ptr(label as _) }.to_str().unwrap())
        }
    }
}

/// An iterator over the TXT entries of an SRP service
/// as returned by `OpenThread::srp_services`.
#[derive(Clone)]
pub struct OutSrpTxtEntriesIter<'a> {
    ptr: *const otDnsTxtEntry,
    size: usize,
    index: usize,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for OutSrpTxtEntriesIter<'a> {
    type Item = (&'a str, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() || self.index == self.size {
            return None;
        }

        let entry = unsafe { self.ptr.add(self.index) };

        self.index += 1;

        let entry = unsafe { &*entry };

        Some((
            unsafe { CStr::from_ptr(entry.mKey) }.to_str().unwrap(),
            unsafe { core::slice::from_raw_parts(entry.mValue, entry.mValueLength as _) },
        ))
    }
}

impl<'a> From<&'a otSrpClientService> for OutSrpService<'a> {
    fn from(ot_srp: &'a otSrpClientService) -> Self {
        #[allow(unused_mut)]
        let mut this = Self {
            name: if !ot_srp.mName.is_null() {
                unsafe { CStr::from_ptr(ot_srp.mName) }.to_str().unwrap()
            } else {
                ""
            },
            instance_name: if !ot_srp.mInstanceName.is_null() {
                unsafe { CStr::from_ptr(ot_srp.mInstanceName) }
                    .to_str()
                    .unwrap()
            } else {
                ""
            },
            subtype_labels: OutSrpSubtypeLabelsIter {
                ptr: ot_srp.mSubTypeLabels as _,
                index: 0,
                _phantom: PhantomData,
            },
            txt_entries: OutSrpTxtEntriesIter {
                ptr: ot_srp.mTxtEntries,
                size: ot_srp.mNumTxtEntries as _,
                index: 0,
                _phantom: PhantomData,
            },
            port: ot_srp.mPort,
            priority: ot_srp.mPriority,
            weight: ot_srp.mWeight,
            lease_secs: 0,
            key_lease_secs: 0,
        };

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            this.lease_secs = ot_srp.mLease;
            this.key_lease_secs = ot_srp.mKeyLease;
        }

        this
    }
}

impl<T> ThreadDriver<'_, T>
where
    T: Mode,
{
    /// Return the current SRP client configuration and SRP client host state to the provided closure.
    ///
    /// Arguments:
    /// - `f`: A closure that takes the SRP configuration and SRP host state as arguments.
    pub fn srp_conf<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&SrpConf, SrpState, bool) -> Result<R, EspError>,
    {
        let inner = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        let info = unsafe { otSrpClientGetHostInfo(instance).as_ref() }.unwrap();

        #[allow(unused_mut)]
        let mut conf = SrpConf {
            host_name: if !info.mName.is_null() {
                unsafe { CStr::from_ptr(info.mName) }.to_str().unwrap()
            } else {
                ""
            },
            host_addrs: if info.mNumAddresses > 0 && !info.mAddresses.is_null() {
                unsafe {
                    core::slice::from_raw_parts(
                        info.mAddresses as *const _,
                        info.mNumAddresses as _,
                    )
                }
            } else {
                &[]
            },
            ttl: 0,
            default_lease_secs: 0,
            default_key_lease_secs: 0,
        };

        #[cfg(not(esp_idf_version_major = "4"))]
        {
            unsafe {
                conf.ttl = otSrpClientGetTtl(instance);
                conf.default_lease_secs = otSrpClientGetLeaseInterval(instance);
                conf.default_key_lease_secs = otSrpClientGetKeyLeaseInterval(instance);
            }
        }

        f(&conf, info.mState.into(), !inner.srp.conf_taken)
    }

    /// Return `true` if there is neither host, nor any service currently registered with the SRP client.
    pub fn srp_is_empty(&self) -> Result<bool, EspError> {
        let inner = self.inner();

        Ok(!inner.srp.conf_taken && inner.srp.services.iter().all(|service| !service.taken))
    }

    /// Set the SRP client configuration.
    ///
    /// Arguments:
    /// - `conf`: The SRP configuration.
    ///
    /// Returns:
    /// - `Ok(())` if the configuration was set successfully.
    /// - `Err(OtError)` if the configuration could not be set. One reason why the configuration setting
    ///   might fail is if the configuration had already been set and then not removed with `srp_remove_all`.
    pub fn srp_set_conf(&self, conf: &SrpConf) -> Result<(), EspError> {
        let mut inner = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        if inner.srp.conf_taken {
            esp!(ESP_ERR_INVALID_STATE)?;
        }

        #[cfg(not(esp_idf_version_major = "4"))]
        unsafe {
            otSrpClientSetLeaseInterval(instance, conf.default_lease_secs);
            otSrpClientSetKeyLeaseInterval(instance, conf.default_key_lease_secs);
            otSrpClientSetTtl(instance, conf.ttl);
        }

        let mut srp_conf = otSrpClientHostInfo {
            mName: core::ptr::null(),
            mAddresses: core::ptr::null(),
            mNumAddresses: 0,
            #[cfg(not(esp_idf_version_major = "4"))]
            mAutoAddress: true,
            mState: 0,
        };

        conf.store(&mut srp_conf, &mut inner.srp.conf_buf)?;
        inner.srp.conf_taken = true;

        ot_esp!(unsafe { otSrpClientSetHostName(instance, srp_conf.mName) })?;

        if !conf.host_addrs.is_empty() {
            ot_esp!(unsafe {
                otSrpClientSetHostAddresses(instance, srp_conf.mAddresses, srp_conf.mNumAddresses)
            })?;
        } else {
            #[cfg(not(esp_idf_version_major = "4"))]
            {
                ot_esp!(unsafe { otSrpClientEnableAutoHostAddress(instance) })?;
            }
        }

        Ok(())
    }

    /// Return `true` if the SRP client is running, `false` otherwise.
    pub fn srp_running(&self) -> Result<bool, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otSrpClientIsRunning(esp_openthread_get_instance()) })
    }

    /// Return `true` if the SRP client is in auto-start mode, `false` otherwise.
    pub fn srp_autostart_enabled(&self) -> Result<bool, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otSrpClientIsAutoStartModeEnabled(esp_openthread_get_instance()) })
    }

    /// Auto-starts the SRP client.
    pub fn srp_autostart(&self) -> Result<(), EspError> {
        let mut inner = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        let srp = &mut inner.srp;

        unsafe {
            otSrpClientEnableAutoStartMode(
                instance,
                Some(OtSrp::plat_c_srp_auto_start_callback),
                srp as *mut _ as *mut _,
            );
        }

        Ok(())
    }

    /// Start the SRP client for the given SRP server address.
    ///
    /// Arguments:
    /// - `server_addr`: The SRP server address.
    pub fn srp_start(&self, server_addr: SocketAddrV6) -> Result<(), EspError> {
        let _lock = self.inner();

        ot_esp!(unsafe {
            otSrpClientStart(esp_openthread_get_instance(), &to_ot_addr(&server_addr))
        })
    }

    /// Stop the SRP client.
    pub fn srp_stop(&self) -> Result<(), EspError> {
        let _lock = self.inner();

        unsafe {
            otSrpClientStop(esp_openthread_get_instance());
        }

        Ok(())
    }

    /// Return the SRP server address, if the SRP client is running and
    /// had connected to a server.
    pub fn srp_server_addr(&self) -> Result<Option<SocketAddrV6>, EspError> {
        let _lock = self.inner();

        let addr =
            unsafe { otSrpClientGetServerAddress(esp_openthread_get_instance()).as_ref() }.unwrap();
        let addr = to_sock_addr(&addr.mAddress, addr.mPort, 0);

        // OT documentation notes that if the SRP client is not running
        // this will return the unspecified addr (0.0.0.0.0.0.0.0)
        Ok((!addr.ip().is_unspecified()).then_some(addr))
    }

    /// Iterate over the SRP services registered with the SRP client.
    ///
    /// Arguments:
    /// - `f`: A closure that receives a tuple of the next SRP service, SRP service state, and SRP service ID.
    ///   If there are no more SRP services, the closure will receive `None`.
    pub fn srp_services<F>(&self, mut f: F) -> Result<(), EspError>
    where
        F: FnMut(Option<(&OutSrpService<'_>, SrpState, SrpServiceSlot)>),
    {
        let inner = self.inner();

        let mut service_ptr: *const otSrpClientService =
            unsafe { otSrpClientGetServices(esp_openthread_get_instance()) };

        while !service_ptr.is_null() {
            let service = unsafe { &*service_ptr };

            let slot = inner
                .srp
                .services
                .iter()
                .position(|s| core::ptr::eq(&s.service, service))
                .unwrap();

            f(Some((&service.into(), service.mState.into(), slot)));

            service_ptr = service.mNext;
        }

        f(None);

        Ok(())
    }

    /// Add an SRP service to the SRP client.
    ///
    /// Arguments:
    /// - `service`: The SRP service to add.
    ///
    /// Returns:
    /// - The SRP service slot, if the service was added successfully.
    /// - `Err(OtError)` if the service could not be added. One reason why the service addition
    ///   might fail is if there are no more slots available for services. This can happen even if all services
    ///   had been removed, as the slots are not freed until the SRP client propagates the removal info to the SRP server.
    pub fn srp_add_service<'a, SI, TI>(
        &self,
        service: &'a SrpService<'a, SI, TI>,
    ) -> Result<SrpServiceSlot, EspError>
    where
        SI: Iterator<Item = &'a str> + Clone + 'a,
        TI: Iterator<Item = (&'a str, &'a [u8])> + Clone + 'a,
    {
        let mut inner = self.inner();

        let slot = inner.srp.services.iter().position(|service| !service.taken);

        let Some(slot) = slot else {
            //return ot_esp!(otError_OT_ERROR_NO_BUFS);
            ot_esp!(otError_OT_ERROR_NO_BUFS).unwrap(); // TODO
            panic!();
        };

        let our_service = &mut inner.srp.services[slot];

        service.store(&mut our_service.service, &mut our_service.buf)?;

        ot_esp!(unsafe {
            otSrpClientAddService(esp_openthread_get_instance(), &mut our_service.service)
        })?;

        debug!("Service added");

        our_service.taken = true;

        Ok(slot)
    }

    /// Remove an SRP service from the SRP client.
    ///
    /// Arguments:
    /// - `slot`: The SRP service to remove.
    /// - `immediate`: If `true`, the service will be removed immediately, otherwise, the service will be removed gracefully
    ///   by propagating the removal info to the SRP server.
    pub fn srp_remove_service(
        &self,
        slot: SrpServiceSlot,
        immediate: bool,
    ) -> Result<(), EspError> {
        let mut inner = self.inner();

        if slot > inner.srp.services.len() || !inner.srp.services[slot].taken {
            ot_esp!(otError_OT_ERROR_INVALID_ARGS)?;
        }

        let service = &mut inner.srp.services[slot];

        if immediate {
            ot_esp!(unsafe {
                otSrpClientClearService(esp_openthread_get_instance(), &mut service.service)
            })?;
            service.taken = false;
            debug!("Service {slot} cleared immeidately");
        } else {
            ot_esp!(unsafe {
                otSrpClientRemoveService(esp_openthread_get_instance(), &mut service.service)
            })?;
            debug!("Service {slot} scheduled for removal");
        }

        Ok(())
    }

    /// Remove the SRP hostname and all SRP services from the SRP client.
    ///
    /// Arguments:
    /// - `immediate`: If `true`, the hostname and services will be removed immediately, otherwise,
    ///   the hostname and services will be removed gracefully by propagating the removal info to the SRP server.
    pub fn srp_remove_all(&self, immediate: bool) -> Result<(), EspError> {
        let mut inner = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        if immediate {
            unsafe {
                otSrpClientClearHostAndServices(instance);
            }

            inner.srp.conf_taken = false;
            for service in &mut inner.srp.services {
                service.taken = false;
            }

            debug!("Hostname and all services cleared immediately");
        } else {
            ot_esp!(unsafe { otSrpClientRemoveHostAndServices(instance, false, true) })?;
            debug!("Hostname and all services scheduled for removal");
        }

        Ok(())
    }

    // /// Wait for the SRP state to change.
    // ///
    // /// This method will wait forever if `OpenThread` is not instantiated with SRP.
    // ///
    // /// NOTE:
    // /// It is not advised to call this method concurrently from multiple async tasks
    // /// because it uses a single waker registration. Thus, while the method will not panic,
    // /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    // TODO
    // pub async fn srp_wait_changed(&self) {
    //     if self.activate().state().srp().is_ok() {
    //         poll_fn(move |cx| {
    //             self.activate().state().srp.as_mut().unwrap()
    //                 .changes
    //                 .poll_wait(cx)
    //         })
    //         .await;
    //     } else {
    //         core::future::pending::<()>().await;
    //     }
    // }
}

impl<T> EspThread<'_, T>
where
    T: NetifMode,
{
    /// Return the current SRP client configuration and SRP client host state to the provided closure.
    ///
    /// Arguments:
    /// - `f`: A closure that takes the SRP configuration and SRP host state as arguments.
    pub fn srp_conf<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&SrpConf, SrpState, bool) -> Result<R, EspError>,
    {
        self.driver().srp_conf(f)
    }

    /// Return `true` if there is neither host, nor any service currently registered with the SRP client.
    pub fn srp_is_empty(&self) -> Result<bool, EspError> {
        self.driver().srp_is_empty()
    }

    /// Set the SRP client configuration.
    ///
    /// Arguments:
    /// - `conf`: The SRP configuration.
    ///
    /// Returns:
    /// - `Ok(())` if the configuration was set successfully.
    /// - `Err(OtError)` if the configuration could not be set. One reason why the configuration setting
    ///   might fail is if the configuration had already been set and then not removed with `srp_remove_all`.
    pub fn srp_set_conf(&self, conf: &SrpConf) -> Result<(), EspError> {
        self.driver().srp_set_conf(conf)
    }

    /// Return `true` if the SRP client is running, `false` otherwise.
    pub fn srp_running(&self) -> Result<bool, EspError> {
        self.driver().srp_running()
    }

    /// Return `true` if the SRP client is in auto-start mode, `false` otherwise.
    pub fn srp_autostart_enabled(&self) -> Result<bool, EspError> {
        self.driver().srp_autostart_enabled()
    }

    /// Auto-starts the SRP client.
    pub fn srp_autostart(&self) -> Result<(), EspError> {
        self.driver().srp_autostart()
    }

    /// Start the SRP client for the given SRP server address.
    ///
    /// Arguments:
    /// - `server_addr`: The SRP server address.
    pub fn srp_start(&self, server_addr: SocketAddrV6) -> Result<(), EspError> {
        self.driver().srp_start(server_addr)
    }

    /// Stop the SRP client.
    pub fn srp_stop(&self) -> Result<(), EspError> {
        self.driver().srp_stop()
    }

    /// Return the SRP server address, if the SRP client is running and
    /// had connected to a server.
    pub fn srp_server_addr(&self) -> Result<Option<SocketAddrV6>, EspError> {
        self.driver().srp_server_addr()
    }

    /// Iterate over the SRP services registered with the SRP client.
    ///
    /// Arguments:
    /// - `f`: A closure that receives a tuple of the next SRP service, SRP service state, and SRP service ID.
    ///   If there are no more SRP services, the closure will receive `None`.
    pub fn srp_services<F>(&self, f: F) -> Result<(), EspError>
    where
        F: FnMut(Option<(&OutSrpService<'_>, SrpState, SrpServiceSlot)>),
    {
        self.driver().srp_services(f)
    }

    /// Add an SRP service to the SRP client.
    ///
    /// Arguments:
    /// - `service`: The SRP service to add.
    ///
    /// Returns:
    /// - The SRP service slot, if the service was added successfully.
    /// - `Err(OtError)` if the service could not be added. One reason why the service addition
    ///   might fail is if there are no more slots available for services. This can happen even if all services
    ///   had been removed, as the slots are not freed until the SRP client propagates the removal info to the SRP server.
    pub fn srp_add_service<'a, SI, TI>(
        &self,
        service: &'a SrpService<'a, SI, TI>,
    ) -> Result<SrpServiceSlot, EspError>
    where
        SI: Iterator<Item = &'a str> + Clone + 'a,
        TI: Iterator<Item = (&'a str, &'a [u8])> + Clone + 'a,
    {
        self.driver().srp_add_service(service)
    }

    /// Remove an SRP service from the SRP client.
    ///
    /// Arguments:
    /// - `slot`: The SRP service to remove.
    /// - `immediate`: If `true`, the service will be removed immediately, otherwise, the service will be removed gracefully
    ///   by propagating the removal info to the SRP server.
    pub fn srp_remove_service(
        &self,
        slot: SrpServiceSlot,
        immediate: bool,
    ) -> Result<(), EspError> {
        self.driver().srp_remove_service(slot, immediate)
    }

    /// Remove the SRP hostname and all SRP services from the SRP client.
    ///
    /// Arguments:
    /// - `immediate`: If `true`, the hostname and services will be removed immediately, otherwise,
    ///   the hostname and services will be removed gracefully by propagating the removal info to the SRP server.
    pub fn srp_remove_all(&self, immediate: bool) -> Result<(), EspError> {
        self.driver().srp_remove_all(immediate)
    }

    // /// Wait for the SRP state to change.
    // ///
    // /// This method will wait forever if `OpenThread` is not instantiated with SRP.
    // ///
    // /// NOTE:
    // /// It is not advised to call this method concurrently from multiple async tasks
    // /// because it uses a single waker registration. Thus, while the method will not panic,
    // /// the tasks will fight with each other by each re-registering its own waker, thus keeping the CPU constantly busy.
    // TODO
    // pub async fn srp_wait_changed(&self) {
    //     self.driver().srp_wait_changed().await
    // }
}

// TODO: Make these configurable with a feature
const SRP_SVCS: usize = 3;
const SRP_SVC_BUF_SIZE: usize = 300;
const SRP_HOST_BUF_SIZE: usize = 300;

pub(crate) struct OtSrp {
    conf_taken: bool,
    conf_buf: [u8; SRP_HOST_BUF_SIZE],
    services: [OtSrpService; SRP_SVCS],
}

impl OtSrp {
    pub(crate) unsafe fn init(this: *mut Self) {
        unsafe {
            addr_of_mut!((*this).conf_taken).write(false);
            addr_of_mut!((*this).conf_buf).write_bytes(0, 1);

            for index in 0..SRP_SVCS {
                let service = addr_of_mut!((*this).services[index]);
                OtSrpService::init(service);
            }
        }
    }

    /// Reclaims the slots of the SRP host and services that are reported as removed
    fn cleanup(
        &mut self,
        host_info: &otSrpClientHostInfo,
        mut removed_services: Option<&otSrpClientService>,
    ) {
        if host_info.mState == otSrpClientItemState_OT_SRP_CLIENT_ITEM_STATE_REMOVED {
            self.conf_taken = false;
            info!("SRP host removed");
        }

        while let Some(service) = removed_services {
            let (slot, our_service) = self
                .services
                .iter_mut()
                .enumerate()
                .find(|(_, s)| core::ptr::eq(&s.service, service))
                .unwrap();

            removed_services = unsafe { service.mNext.as_ref() };

            our_service.taken = false;
            info!("SRP service at slot {slot} removed");
        }
    }

    fn plat_srp_changed(
        &mut self,
        host_info: &otSrpClientHostInfo,
        _services: Option<&otSrpClientService>,
        removed_services: Option<&otSrpClientService>,
    ) {
        trace!("Plat SRP changed callback");

        self.cleanup(host_info, removed_services);
    }

    fn plat_srp_auto_started(&mut self) {
        // TODO - in future, consider if we need to signal the changes here
    }

    pub(crate) unsafe extern "C" fn plat_c_srp_state_change_callback(
        _error: otError,
        host_info: *const crate::sys::otSrpClientHostInfo,
        services: *const crate::sys::otSrpClientService,
        removed_services: *const crate::sys::otSrpClientService,
        context: *mut c_void,
    ) {
        let srp = context as *mut OtSrp;
        let srp = unsafe { srp.as_mut() }.unwrap();

        srp.plat_srp_changed(
            unsafe { &*host_info },
            unsafe { services.as_ref() },
            unsafe { removed_services.as_ref() },
        );
    }

    pub(crate) unsafe extern "C" fn plat_c_srp_auto_start_callback(
        _server_sock_addr: *const crate::sys::otSockAddr,
        context: *mut c_void,
    ) {
        let srp = context as *mut OtSrp;
        let srp = unsafe { srp.as_mut() }.unwrap();

        srp.plat_srp_auto_started();
    }
}

struct OtSrpService {
    taken: bool,
    service: otSrpClientService,
    buf: [u8; SRP_SVC_BUF_SIZE],
}

impl OtSrpService {
    pub(crate) unsafe fn init(this: *mut Self) {
        unsafe {
            addr_of_mut!((*this).taken).write(false);
            addr_of_mut!((*this).buf).write_bytes(0, 1);
            addr_of_mut!((*this).service).write_bytes(0, 1);
        }
    }
}

fn align_min<T>(buf: &mut [u8], count: usize) -> Result<(&mut [T], &mut [u8]), EspError> {
    if count == 0 || core::mem::size_of::<T>() == 0 {
        return Ok((&mut [], buf));
    }

    let (t_leading_buf0, t_buf, _) = unsafe { buf.align_to_mut::<T>() };
    if t_buf.len() < count {
        ot_esp!(otError_OT_ERROR_NO_BUFS)?;
    }

    // Shrink `t_buf` to the number of requested items (count)
    let t_buf = &mut t_buf[..count];
    let t_leading_buf0_len = t_leading_buf0.len();
    let t_buf_size = core::mem::size_of_val(t_buf);

    let (buf0, remaining_buf) = buf.split_at_mut(t_leading_buf0_len + t_buf_size);

    let (t_leading_buf, t_buf, t_remaining_buf) = unsafe { buf0.align_to_mut::<T>() };
    assert_eq!(t_leading_buf0_len, t_leading_buf.len());
    assert_eq!(t_buf.len(), count);
    assert!(t_remaining_buf.is_empty());

    Ok((t_buf, remaining_buf))
}

fn store_str<'t>(str: &str, buf: &'t mut [u8]) -> Result<(&'t CStr, &'t mut [u8]), EspError> {
    let data_len = str.len() + 1;

    if data_len > buf.len() {
        ot_esp!(otError_OT_ERROR_NO_BUFS)?;
    }

    let (str_buf, rem_buf) = buf.split_at_mut(data_len);

    str_buf[..str.len()].copy_from_slice(str.as_bytes());
    str_buf[str.len()] = 0;

    Ok((
        CStr::from_bytes_with_nul(&str_buf[..data_len]).unwrap(),
        rem_buf,
    ))
}

fn store_data<'t>(data: &[u8], buf: &'t mut [u8]) -> Result<(&'t [u8], &'t mut [u8]), EspError> {
    if data.len() > buf.len() {
        ot_esp!(otError_OT_ERROR_NO_BUFS)?;
    }

    let (data_buf, rem_buf) = buf.split_at_mut(data.len());

    data_buf[..data.len()].copy_from_slice(data);

    Ok((data_buf, rem_buf))
}

/// Convert an `otIp6Address`, port and network interface ID to a `SocketAddrV6`.
fn to_sock_addr(addr: &otIp6Address, port: u16, netif: u32) -> SocketAddrV6 {
    SocketAddrV6::new(Ipv6Addr::from(unsafe { addr.mFields.m8 }), port, 0, netif)
}

/// Convert a `SocketAddrV6` to an `otSockAddr`.
fn to_ot_addr(addr: &SocketAddrV6) -> crate::sys::otSockAddr {
    crate::sys::otSockAddr {
        mAddress: otIp6Address {
            mFields: otIp6Address__bindgen_ty_1 {
                m8: addr.ip().octets(),
            },
        },
        mPort: addr.port(),
    }
}
