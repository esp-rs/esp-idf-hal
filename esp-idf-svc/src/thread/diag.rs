//! Diagnostics of the local Thread node: network parameters, neighbor and route tables,
//! the Active Operational Dataset and the MAC counters.

use core::ffi::CStr;
use core::mem::MaybeUninit;

use crate::sys::{
    esp_openthread_get_instance, otDatasetGetActive, otDatasetIsCommissioned,
    otError_OT_ERROR_NONE, otLinkGetChannel, otLinkGetCounters, otLinkGetExtendedAddress,
    otLinkGetPanId, otLinkResetCounters, otMacCounters, otNeighborInfo, otNeighborInfoIterator,
    otNetDataGetStableVersion, otNetDataGetVersion, otOperationalDataset,
    otOperationalDatasetComponents, otRouterInfo, otSecurityPolicy, otThreadGetExtendedPanId,
    otThreadGetLeaderRouterId, otThreadGetLeaderWeight, otThreadGetLinkMode,
    otThreadGetMeshLocalPrefix, otThreadGetNetworkName, otThreadGetNextNeighborInfo,
    otThreadGetPartitionId, otThreadGetRloc16, EspError, OT_NEIGHBOR_INFO_ITERATOR_INIT,
    OT_RADIO_RSSI_INVALID,
};

#[cfg(esp_idf_openthread_ftd)]
use crate::sys::{otThreadGetMaxRouterId, otThreadGetRouterInfo, otThreadIsRouterEligible};

#[cfg(not(esp_idf_openthread_ftd))]
use crate::sys::otThreadGetParentInfo;

#[cfg(all(esp_idf_comp_esp_netif_enabled, not(esp_idf_openthread_radio)))]
use crate::thread::{EspThread, NetifMode};
use crate::thread::{Mode, ThreadDriver};

/// An entry of the Thread neighbor table (`otNeighborInfo`).
///
/// All fields describe the *neighboring* node, as observed by this node.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct NeighborInfo {
    /// IEEE 802.15.4 extended address of the neighbor, big-endian.
    pub ext_address: u64,
    /// Seconds since a frame was last received from the neighbor.
    pub age: u32,
    /// RLOC16 of the neighbor.
    pub rloc16: u16,
    /// Last received MAC frame counter of the neighbor.
    pub link_frame_counter: u32,
    /// Last received MLE frame counter of the neighbor.
    pub mle_frame_counter: u32,
    /// Link Quality In for messages from the neighbor (0..=3).
    pub link_quality_in: u8,
    /// Average RSSI (dBm) of frames from the neighbor, if available.
    pub average_rssi: Option<i8>,
    /// RSSI (dBm) of the last frame from the neighbor (clamped to <= 0), if available.
    pub last_rssi: Option<i8>,
    /// Frame error rate as a percentage (0..=100).
    pub frame_error_rate: u8,
    /// Message error rate as a percentage (0..=100).
    pub message_error_rate: u8,
    /// Whether the neighbor keeps its receiver on when idle.
    pub rx_on_when_idle: bool,
    /// Whether the neighbor is a Full Thread Device.
    pub full_thread_device: bool,
    /// Whether the neighbor requests the full Network Data.
    pub full_network_data: bool,
    /// Whether the neighbor is a child of this node.
    pub is_child: bool,
}

impl NeighborInfo {
    fn load_raw(raw: &otNeighborInfo) -> Self {
        // OpenThread reports error rates as a fraction of 0xffff
        let to_percent = |rate: u16| ((rate as u32 * 100) / 0xffff) as u8;
        let rssi = |v: i8| (v != OT_RADIO_RSSI_INVALID as i8).then_some(v);

        Self {
            ext_address: u64::from_be_bytes(raw.mExtAddress.m8),
            age: raw.mAge,
            rloc16: raw.mRloc16,
            link_frame_counter: raw.mLinkFrameCounter,
            mle_frame_counter: raw.mMleFrameCounter,
            link_quality_in: raw.mLinkQualityIn,
            average_rssi: rssi(raw.mAverageRssi),
            last_rssi: rssi(raw.mLastRssi).map(|v| v.min(0)),
            frame_error_rate: to_percent(raw.mFrameErrorRate),
            message_error_rate: to_percent(raw.mMessageErrorRate),
            rx_on_when_idle: raw.mRxOnWhenIdle(),
            full_thread_device: raw.mFullThreadDevice(),
            full_network_data: raw.mFullNetworkData(),
            is_child: raw.mIsChild(),
        }
    }
}

/// An entry of the Thread route table (`otRouterInfo`).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct RouteInfo {
    /// IEEE 802.15.4 extended address of the router, big-endian.
    pub ext_address: u64,
    /// RLOC16 of the router.
    pub rloc16: u16,
    /// Router ID of the router.
    pub router_id: u8,
    /// Router ID of the next hop towards the router.
    pub next_hop: u8,
    /// Path cost to the router.
    pub path_cost: u8,
    /// Link Quality In for messages from the router (0..=3).
    pub lqi_in: u8,
    /// Link Quality Out for messages to the router (0..=3).
    pub lqi_out: u8,
    /// Seconds since a frame was last received from the router.
    pub age: u8,
    /// Whether the router ID is allocated.
    pub allocated: bool,
    /// Whether a link is established with the router.
    pub link_established: bool,
}

impl RouteInfo {
    fn load_raw(raw: &otRouterInfo) -> Self {
        Self {
            ext_address: u64::from_be_bytes(raw.mExtAddress.m8),
            rloc16: raw.mRloc16,
            router_id: raw.mRouterId,
            next_hop: raw.mNextHop,
            path_cost: raw.mPathCost,
            lqi_in: raw.mLinkQualityIn,
            lqi_out: raw.mLinkQualityOut,
            age: raw.mAge,
            allocated: raw.mAllocated(),
            link_established: raw.mLinkEstablished(),
        }
    }
}

/// IEEE 802.15.4 MAC-layer counters (`otMacCounters`).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct MacCounters {
    pub tx_total: u32,
    pub tx_unicast: u32,
    pub tx_broadcast: u32,
    pub tx_ack_requested: u32,
    pub tx_acked: u32,
    pub tx_no_ack_requested: u32,
    pub tx_data: u32,
    pub tx_data_poll: u32,
    pub tx_beacon: u32,
    pub tx_beacon_request: u32,
    pub tx_other: u32,
    pub tx_retry: u32,
    pub tx_direct_max_retry_expiry: u32,
    pub tx_indirect_max_retry_expiry: u32,
    pub tx_err_cca: u32,
    pub tx_err_abort: u32,
    pub tx_err_busy_channel: u32,
    pub rx_total: u32,
    pub rx_unicast: u32,
    pub rx_broadcast: u32,
    pub rx_data: u32,
    pub rx_data_poll: u32,
    pub rx_beacon: u32,
    pub rx_beacon_request: u32,
    pub rx_other: u32,
    pub rx_address_filtered: u32,
    pub rx_dest_addr_filtered: u32,
    pub rx_duplicated: u32,
    pub rx_err_no_frame: u32,
    pub rx_err_unknown_neighbor: u32,
    pub rx_err_invalid_src_addr: u32,
    pub rx_err_sec: u32,
    pub rx_err_fcs: u32,
    pub rx_err_other: u32,
}

impl From<&otMacCounters> for MacCounters {
    fn from(c: &otMacCounters) -> Self {
        Self {
            tx_total: c.mTxTotal,
            tx_unicast: c.mTxUnicast,
            tx_broadcast: c.mTxBroadcast,
            tx_ack_requested: c.mTxAckRequested,
            tx_acked: c.mTxAcked,
            tx_no_ack_requested: c.mTxNoAckRequested,
            tx_data: c.mTxData,
            tx_data_poll: c.mTxDataPoll,
            tx_beacon: c.mTxBeacon,
            tx_beacon_request: c.mTxBeaconRequest,
            tx_other: c.mTxOther,
            tx_retry: c.mTxRetry,
            tx_direct_max_retry_expiry: c.mTxDirectMaxRetryExpiry,
            tx_indirect_max_retry_expiry: c.mTxIndirectMaxRetryExpiry,
            tx_err_cca: c.mTxErrCca,
            tx_err_abort: c.mTxErrAbort,
            tx_err_busy_channel: c.mTxErrBusyChannel,
            rx_total: c.mRxTotal,
            rx_unicast: c.mRxUnicast,
            rx_broadcast: c.mRxBroadcast,
            rx_data: c.mRxData,
            rx_data_poll: c.mRxDataPoll,
            rx_beacon: c.mRxBeacon,
            rx_beacon_request: c.mRxBeaconRequest,
            rx_other: c.mRxOther,
            rx_address_filtered: c.mRxAddressFiltered,
            rx_dest_addr_filtered: c.mRxDestAddrFiltered,
            rx_duplicated: c.mRxDuplicated,
            rx_err_no_frame: c.mRxErrNoFrame,
            rx_err_unknown_neighbor: c.mRxErrUnknownNeighbor,
            rx_err_invalid_src_addr: c.mRxErrInvalidSrcAddr,
            rx_err_sec: c.mRxErrSec,
            rx_err_fcs: c.mRxErrFcs,
            rx_err_other: c.mRxErrOther,
        }
    }
}

/// The Security Policy of an Operational Dataset (`otSecurityPolicy`).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct SecurityPolicy {
    /// The key rotation time, in hours.
    pub rotation_time: u16,
    pub obtain_network_key_enabled: bool,
    pub native_commissioning_enabled: bool,
    pub routers_enabled: bool,
    pub external_commissioning_enabled: bool,
    pub commercial_commissioning_enabled: bool,
    pub autonomous_enrollment_enabled: bool,
    pub network_key_provisioning_enabled: bool,
    pub toble_link_enabled: bool,
    pub non_ccm_routers_enabled: bool,
    pub version_threshold_for_routing: u8,
}

impl SecurityPolicy {
    fn load_raw(raw: &otSecurityPolicy) -> Self {
        Self {
            rotation_time: raw.mRotationTime,
            obtain_network_key_enabled: raw.mObtainNetworkKeyEnabled(),
            native_commissioning_enabled: raw.mNativeCommissioningEnabled(),
            routers_enabled: raw.mRoutersEnabled(),
            external_commissioning_enabled: raw.mExternalCommissioningEnabled(),
            commercial_commissioning_enabled: raw.mCommercialCommissioningEnabled(),
            autonomous_enrollment_enabled: raw.mAutonomousEnrollmentEnabled(),
            network_key_provisioning_enabled: raw.mNetworkKeyProvisioningEnabled(),
            toble_link_enabled: raw.mTobleLinkEnabled(),
            non_ccm_routers_enabled: raw.mNonCcmRoutersEnabled(),
            version_threshold_for_routing: raw.mVersionThresholdForRouting(),
        }
    }
}

/// Which components are present in an Operational Dataset (`otOperationalDatasetComponents`).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct OperationalDatasetComponents {
    pub active_timestamp_present: bool,
    pub pending_timestamp_present: bool,
    pub network_key_present: bool,
    pub network_name_present: bool,
    pub extended_pan_id_present: bool,
    pub mesh_local_prefix_present: bool,
    pub delay_present: bool,
    pub pan_id_present: bool,
    pub channel_present: bool,
    pub pskc_present: bool,
    pub security_policy_present: bool,
    pub channel_mask_present: bool,
}

impl OperationalDatasetComponents {
    fn load_raw(raw: &otOperationalDatasetComponents) -> Self {
        Self {
            active_timestamp_present: raw.mIsActiveTimestampPresent,
            pending_timestamp_present: raw.mIsPendingTimestampPresent,
            network_key_present: raw.mIsNetworkKeyPresent,
            network_name_present: raw.mIsNetworkNamePresent,
            extended_pan_id_present: raw.mIsExtendedPanIdPresent,
            mesh_local_prefix_present: raw.mIsMeshLocalPrefixPresent,
            delay_present: raw.mIsDelayPresent,
            pan_id_present: raw.mIsPanIdPresent,
            channel_present: raw.mIsChannelPresent,
            pskc_present: raw.mIsPskcPresent,
            security_policy_present: raw.mIsSecurityPolicyPresent,
            channel_mask_present: raw.mIsChannelMaskPresent,
        }
    }
}

/// The diagnostic-relevant parts of the Active Operational Dataset.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash)]
pub struct ActiveDatasetDiag {
    /// The Security Policy, if present in the dataset.
    pub security_policy: Option<SecurityPolicy>,
    /// The page 0 Channel Mask (bit `n` = channel `n`), if present in the dataset.
    pub channel_mask: Option<u32>,
    /// Which components are present in the dataset.
    pub components: OperationalDatasetComponents,
}

impl<T> ThreadDriver<'_, T>
where
    T: Mode,
{
    /// Return `true` if the node has an Active Operational Dataset (`otDatasetIsCommissioned`).
    pub fn is_commissioned(&self) -> Result<bool, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otDatasetIsCommissioned(esp_openthread_get_instance()) })
    }

    /// Return the current IEEE 802.15.4 channel.
    pub fn channel(&self) -> Result<u8, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otLinkGetChannel(esp_openthread_get_instance()) })
    }

    /// Return `true` if the node keeps its receiver on when idle, i.e. is not a sleepy end device.
    pub fn rx_on_when_idle(&self) -> Result<bool, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadGetLinkMode(esp_openthread_get_instance()) }.mRxOnWhenIdle())
    }

    /// Return `true` if the node is router-eligible.
    #[cfg(esp_idf_openthread_ftd)]
    pub fn router_eligible(&self) -> Result<bool, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadIsRouterEligible(esp_openthread_get_instance()) })
    }

    /// Call the provided closure with the Thread network name.
    pub fn network_name<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&str) -> R,
    {
        let _lock = self.inner();

        let name = unsafe { CStr::from_ptr(otThreadGetNetworkName(esp_openthread_get_instance())) };

        Ok(f(name.to_str().unwrap_or("")))
    }

    /// Return the IEEE 802.15.4 PAN ID.
    pub fn pan_id(&self) -> Result<u16, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otLinkGetPanId(esp_openthread_get_instance()) })
    }

    /// Return the Thread extended PAN ID, big-endian.
    pub fn extended_pan_id(&self) -> Result<u64, EspError> {
        let _lock = self.inner();

        let id = unsafe { &*otThreadGetExtendedPanId(esp_openthread_get_instance()) };

        Ok(u64::from_be_bytes(id.m8))
    }

    /// Return the mesh-local /64 prefix.
    pub fn mesh_local_prefix(&self) -> Result<[u8; 8], EspError> {
        let _lock = self.inner();

        Ok(unsafe { &*otThreadGetMeshLocalPrefix(esp_openthread_get_instance()) }.m8)
    }

    /// Return the Thread partition ID.
    pub fn partition_id(&self) -> Result<u32, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadGetPartitionId(esp_openthread_get_instance()) })
    }

    /// Return the leader weight.
    pub fn leader_weight(&self) -> Result<u8, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadGetLeaderWeight(esp_openthread_get_instance()) })
    }

    /// Return the full Network Data version.
    pub fn net_data_version(&self) -> Result<u8, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otNetDataGetVersion(esp_openthread_get_instance()) })
    }

    /// Return the stable Network Data version.
    pub fn net_data_stable_version(&self) -> Result<u8, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otNetDataGetStableVersion(esp_openthread_get_instance()) })
    }

    /// Return the router ID of the leader.
    pub fn leader_router_id(&self) -> Result<u8, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadGetLeaderRouterId(esp_openthread_get_instance()) })
    }

    /// Return the IEEE 802.15.4 extended address of the node, big-endian.
    pub fn ext_address(&self) -> Result<u64, EspError> {
        let _lock = self.inner();

        let addr = unsafe { &*otLinkGetExtendedAddress(esp_openthread_get_instance()) };

        Ok(u64::from_be_bytes(addr.m8))
    }

    /// Return the RLOC16 of the node.
    pub fn rloc16(&self) -> Result<u16, EspError> {
        let _lock = self.inner();

        Ok(unsafe { otThreadGetRloc16(esp_openthread_get_instance()) })
    }

    /// Call the provided closure for each entry of the neighbor table.
    pub fn neighbor_table<F>(&self, mut f: F) -> Result<(), EspError>
    where
        F: FnMut(&NeighborInfo),
    {
        let _lock = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        let mut iter = OT_NEIGHBOR_INFO_ITERATOR_INIT as otNeighborInfoIterator;
        let mut info = MaybeUninit::<otNeighborInfo>::uninit();

        while unsafe { otThreadGetNextNeighborInfo(instance, &mut iter, info.as_mut_ptr()) }
            == otError_OT_ERROR_NONE
        {
            f(&NeighborInfo::load_raw(unsafe { info.assume_init_ref() }));
        }

        Ok(())
    }

    /// Call the provided closure for each entry of the route table.
    ///
    /// On a Full Thread Device, this is the router table. On a Minimal Thread Device,
    /// it is the parent of the node, if it has one.
    pub fn route_table<F>(&self, mut f: F) -> Result<(), EspError>
    where
        F: FnMut(&RouteInfo),
    {
        let _lock = self.inner();

        let instance = unsafe { esp_openthread_get_instance() };

        let mut info = MaybeUninit::<otRouterInfo>::uninit();

        #[cfg(esp_idf_openthread_ftd)]
        {
            let max_router_id = unsafe { otThreadGetMaxRouterId(instance) };

            for router_id in 0..=max_router_id {
                // Unallocated router IDs return an error
                if unsafe { otThreadGetRouterInfo(instance, router_id as _, info.as_mut_ptr()) }
                    == otError_OT_ERROR_NONE
                {
                    f(&RouteInfo::load_raw(unsafe { info.assume_init_ref() }));
                }
            }
        }

        #[cfg(not(esp_idf_openthread_ftd))]
        {
            if unsafe { otThreadGetParentInfo(instance, info.as_mut_ptr()) }
                == otError_OT_ERROR_NONE
            {
                f(&RouteInfo::load_raw(unsafe { info.assume_init_ref() }));
            }
        }

        Ok(())
    }

    /// Return the diagnostic-relevant parts of the Active Operational Dataset,
    /// or `None` if the node has no Active Operational Dataset.
    pub fn active_dataset_diag(&self) -> Result<Option<ActiveDatasetDiag>, EspError> {
        let _lock = self.inner();

        let mut dataset = MaybeUninit::<otOperationalDataset>::uninit();

        if unsafe { otDatasetGetActive(esp_openthread_get_instance(), dataset.as_mut_ptr()) }
            != otError_OT_ERROR_NONE
        {
            return Ok(None);
        }

        let dataset = unsafe { dataset.assume_init_ref() };

        let components = OperationalDatasetComponents::load_raw(&dataset.mComponents);

        Ok(Some(ActiveDatasetDiag {
            security_policy: components
                .security_policy_present
                .then(|| SecurityPolicy::load_raw(&dataset.mSecurityPolicy)),
            channel_mask: components
                .channel_mask_present
                .then_some(dataset.mChannelMask),
            components,
        }))
    }

    /// Return the IEEE 802.15.4 MAC counters.
    pub fn mac_counters(&self) -> Result<MacCounters, EspError> {
        let _lock = self.inner();

        Ok(unsafe { &*otLinkGetCounters(esp_openthread_get_instance()) }.into())
    }

    /// Reset the IEEE 802.15.4 MAC counters.
    pub fn reset_mac_counters(&self) -> Result<(), EspError> {
        let _lock = self.inner();

        unsafe { otLinkResetCounters(esp_openthread_get_instance()) };

        Ok(())
    }
}

#[cfg(all(esp_idf_comp_esp_netif_enabled, not(esp_idf_openthread_radio)))]
impl<T> EspThread<'_, T>
where
    T: NetifMode,
{
    /// Return `true` if the node has an Active Operational Dataset (`otDatasetIsCommissioned`).
    pub fn is_commissioned(&self) -> Result<bool, EspError> {
        self.driver().is_commissioned()
    }

    /// Return the current IEEE 802.15.4 channel.
    pub fn channel(&self) -> Result<u8, EspError> {
        self.driver().channel()
    }

    /// Return `true` if the node keeps its receiver on when idle, i.e. is not a sleepy end device.
    pub fn rx_on_when_idle(&self) -> Result<bool, EspError> {
        self.driver().rx_on_when_idle()
    }

    /// Return `true` if the node is router-eligible.
    #[cfg(esp_idf_openthread_ftd)]
    pub fn router_eligible(&self) -> Result<bool, EspError> {
        self.driver().router_eligible()
    }

    /// Call the provided closure with the Thread network name.
    pub fn network_name<F, R>(&self, f: F) -> Result<R, EspError>
    where
        F: FnOnce(&str) -> R,
    {
        self.driver().network_name(f)
    }

    /// Return the IEEE 802.15.4 PAN ID.
    pub fn pan_id(&self) -> Result<u16, EspError> {
        self.driver().pan_id()
    }

    /// Return the Thread extended PAN ID, big-endian.
    pub fn extended_pan_id(&self) -> Result<u64, EspError> {
        self.driver().extended_pan_id()
    }

    /// Return the mesh-local /64 prefix.
    pub fn mesh_local_prefix(&self) -> Result<[u8; 8], EspError> {
        self.driver().mesh_local_prefix()
    }

    /// Return the Thread partition ID.
    pub fn partition_id(&self) -> Result<u32, EspError> {
        self.driver().partition_id()
    }

    /// Return the leader weight.
    pub fn leader_weight(&self) -> Result<u8, EspError> {
        self.driver().leader_weight()
    }

    /// Return the full Network Data version.
    pub fn net_data_version(&self) -> Result<u8, EspError> {
        self.driver().net_data_version()
    }

    /// Return the stable Network Data version.
    pub fn net_data_stable_version(&self) -> Result<u8, EspError> {
        self.driver().net_data_stable_version()
    }

    /// Return the router ID of the leader.
    pub fn leader_router_id(&self) -> Result<u8, EspError> {
        self.driver().leader_router_id()
    }

    /// Return the IEEE 802.15.4 extended address of the node, big-endian.
    pub fn ext_address(&self) -> Result<u64, EspError> {
        self.driver().ext_address()
    }

    /// Return the RLOC16 of the node.
    pub fn rloc16(&self) -> Result<u16, EspError> {
        self.driver().rloc16()
    }

    /// Call the provided closure for each entry of the neighbor table.
    pub fn neighbor_table<F>(&self, f: F) -> Result<(), EspError>
    where
        F: FnMut(&NeighborInfo),
    {
        self.driver().neighbor_table(f)
    }

    /// Call the provided closure for each entry of the route table.
    ///
    /// On a Full Thread Device, this is the router table. On a Minimal Thread Device,
    /// it is the parent of the node, if it has one.
    pub fn route_table<F>(&self, f: F) -> Result<(), EspError>
    where
        F: FnMut(&RouteInfo),
    {
        self.driver().route_table(f)
    }

    /// Return the diagnostic-relevant parts of the Active Operational Dataset,
    /// or `None` if the node has no Active Operational Dataset.
    pub fn active_dataset_diag(&self) -> Result<Option<ActiveDatasetDiag>, EspError> {
        self.driver().active_dataset_diag()
    }

    /// Return the IEEE 802.15.4 MAC counters.
    pub fn mac_counters(&self) -> Result<MacCounters, EspError> {
        self.driver().mac_counters()
    }

    /// Reset the IEEE 802.15.4 MAC counters.
    pub fn reset_mac_counters(&self) -> Result<(), EspError> {
        self.driver().reset_mac_counters()
    }
}
