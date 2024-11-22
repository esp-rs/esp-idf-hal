#![allow(non_upper_case_globals)]

#[cfg(esp_idf_bt_hfp_client_enable)]
pub mod client {
    use core::borrow::Borrow;
    use core::convert::TryInto;
    use core::ffi;
    use core::fmt::{self, Debug};
    use core::marker::PhantomData;

    use crate::sys::*;

    use ::log::{debug, info};

    use num_enum::TryFromPrimitive;

    use crate::{
        bt::{BdAddr, BtClassicEnabled, BtDriver, BtSingleton},
        private::cstr::{from_cstr_ptr, to_cstring_arg},
    };

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub enum Volume {
        Speaker(u8),
        Microphone(u8),
    }

    #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    pub struct Source {
        pub sample_rate_hz: u32,
        pub bits_per_sample: u8,
        pub stereo: bool,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum ConnectionStatus {
        Disconnected = esp_hf_client_connection_state_t_ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED,
        Connecting = esp_hf_client_connection_state_t_ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING,
        Connected = esp_hf_client_connection_state_t_ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED,
        SlcConnected =
            esp_hf_client_connection_state_t_ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED,
        Disconnecting =
            esp_hf_client_connection_state_t_ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTING,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum AudioStatus {
        Disconnected = esp_hf_client_audio_state_t_ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED,
        Connectng = esp_hf_client_audio_state_t_ESP_HF_CLIENT_AUDIO_STATE_CONNECTING,
        Connected = esp_hf_client_audio_state_t_ESP_HF_CLIENT_AUDIO_STATE_CONNECTED,
        ConnectedMsbc = esp_hf_client_audio_state_t_ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum CallSetupStatus {
        Idle = esp_hf_call_setup_status_t_ESP_HF_CALL_SETUP_STATUS_IDLE,
        Incoming = esp_hf_call_setup_status_t_ESP_HF_CALL_SETUP_STATUS_INCOMING,
        OutgoingDialing = esp_hf_call_setup_status_t_ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING,
        OutgoingAlerting = esp_hf_call_setup_status_t_ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum HoldStatus {
        Held = esp_hf_btrh_status_t_ESP_HF_BTRH_STATUS_HELD,
        Accepted = esp_hf_btrh_status_t_ESP_HF_BTRH_STATUS_ACCEPTED,
        Rejected = esp_hf_btrh_status_t_ESP_HF_BTRH_STATUS_REJECTED,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum CallHeldStatus {
        None = esp_hf_call_held_status_t_ESP_HF_CALL_HELD_STATUS_NONE,
        HeldAndActive = esp_hf_call_held_status_t_ESP_HF_CALL_HELD_STATUS_HELD_AND_ACTIVE,
        Held = esp_hf_call_held_status_t_ESP_HF_CALL_HELD_STATUS_HELD,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum ServiceType {
        Unknown = esp_hf_subscriber_service_type_t_ESP_HF_SUBSCRIBER_SERVICE_TYPE_UNKNOWN,
        Voice = esp_hf_subscriber_service_type_t_ESP_HF_SUBSCRIBER_SERVICE_TYPE_VOICE,
        Fax = esp_hf_subscriber_service_type_t_ESP_HF_SUBSCRIBER_SERVICE_TYPE_FAX,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum AtResponseCode {
        OK = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_OK,
        Er = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_ERR,
        NoCarrier = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_NO_CARRIER,
        Busy = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_BUSY,
        NoAnswer = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_NO_ANSWER,
        Delayed = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_DELAYED,
        DenyListed = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_BLACKLISTED,
        AudioGatewayErr = esp_hf_at_response_code_t_ESP_HF_AT_RESPONSE_CODE_CME,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum AudioGatewayResponseCode {
        AgFailure = esp_hf_cme_err_t_ESP_HF_CME_AG_FAILURE,
        NoConnectionToPhone = esp_hf_cme_err_t_ESP_HF_CME_NO_CONNECTION_TO_PHONE,
        OperationNotAllowed = esp_hf_cme_err_t_ESP_HF_CME_OPERATION_NOT_ALLOWED,
        OperationNotSupported = esp_hf_cme_err_t_ESP_HF_CME_OPERATION_NOT_SUPPORTED,
        PhSimPinRequired = esp_hf_cme_err_t_ESP_HF_CME_PH_SIM_PIN_REQUIRED,
        SimNotInserted = esp_hf_cme_err_t_ESP_HF_CME_SIM_NOT_INSERTED,
        SimPinRequired = esp_hf_cme_err_t_ESP_HF_CME_SIM_PIN_REQUIRED,
        SimPukRequired = esp_hf_cme_err_t_ESP_HF_CME_SIM_PUK_REQUIRED,
        SimFailure = esp_hf_cme_err_t_ESP_HF_CME_SIM_FAILURE,
        SimBusy = esp_hf_cme_err_t_ESP_HF_CME_SIM_BUSY,
        IncorrectPassword = esp_hf_cme_err_t_ESP_HF_CME_INCORRECT_PASSWORD,
        SimPin2Required = esp_hf_cme_err_t_ESP_HF_CME_SIM_PIN2_REQUIRED,
        SimPuk2Required = esp_hf_cme_err_t_ESP_HF_CME_SIM_PUK2_REQUIRED,
        #[cfg(not(esp_idf_version_major = "4"))]
        MemoryFull = esp_hf_cme_err_t_ESP_HF_CME_MEMORY_FULL,
        #[cfg(esp_idf_version_major = "4")]
        MemoryFull = esp_hf_cme_err_t_ESP_HF_CME_MEMEORY_FULL,
        InvalidIndex = esp_hf_cme_err_t_ESP_HF_CME_INVALID_INDEX,
        #[cfg(not(esp_idf_version_major = "4"))]
        MemoryFailure = esp_hf_cme_err_t_ESP_HF_CME_MEMORY_FAILURE,
        #[cfg(esp_idf_version_major = "4")]
        MemoryFailure = esp_hf_cme_err_t_ESP_HF_CME_MEMEORY_FAILURE,
        TextStringTooLong = esp_hf_cme_err_t_ESP_HF_CME_TEXT_STRING_TOO_LONG,
        InvalidCharsInTextString = esp_hf_cme_err_t_ESP_HF_CME_INVALID_CHARACTERS_IN_TEXT_STRING,
        DialStringTooLong = esp_hf_cme_err_t_ESP_HF_CME_DIAL_STRING_TOO_LONG,
        InvalidCharsInDialString = esp_hf_cme_err_t_ESP_HF_CME_INVALID_CHARACTERS_IN_DIAL_STRING,
        NoNetworkService = esp_hf_cme_err_t_ESP_HF_CME_NO_NETWORK_SERVICE,
        NetworkTimeout = esp_hf_cme_err_t_ESP_HF_CME_NETWORK_TIMEOUT,
        NetworkNotAllowed = esp_hf_cme_err_t_ESP_HF_CME_NETWORK_NOT_ALLOWED,
    }

    #[derive(Debug, Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
    #[repr(u32)]
    pub enum CurrentCallStatus {
        Active = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_ACTIVE,
        Held = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_HELD,
        Dialing = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_DIALING,
        Alerting = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_ALERTING,
        Incoming = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_INCOMING,
        Waiting = esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_WAITING,
        HeldByResponseAndHold =
            esp_hf_current_call_status_t_ESP_HF_CURRENT_CALL_STATUS_HELD_BY_RESP_HOLD,
    }

    pub struct EventRawData<'a>(pub &'a esp_hf_client_cb_param_t);

    impl<'a> Debug for EventRawData<'a> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_tuple("RawData").finish()
        }
    }

    #[derive(Debug)]
    pub enum HfpcEvent<'a> {
        ConnectionState {
            bd_addr: BdAddr,
            status: ConnectionStatus,
            peer_features: u32,
            child_features: u32,
        },
        AudioState {
            bd_addr: BdAddr,
            status: AudioStatus,
        },
        CallState(bool),
        CallSetupState(CallSetupStatus),
        VoiceRecognitionEnabled,
        VoiceRecognitionDisabled,
        CallHeld(CallHeldStatus),
        NetworkServiceAvailability(bool),
        SignalStrength(u8),
        Roaming(bool),
        BatteryLevel(u8),
        NetworkOperator(&'a str),
        CallResponseAndHold(HoldStatus),
        CallingLineIdentification(&'a str),
        CallWaiting(&'a str),
        CurrentCall {
            index: usize,
            outgoing: bool,
            status: CurrentCallStatus,
            multi_party: bool,
            number: &'a str,
        },
        VolumeControl(Volume),
        AtResponse {
            code: AtResponseCode,
            extended_code: AudioGatewayResponseCode,
        },
        SubscriberInfo {
            number: &'a str,
            service_type: ServiceType,
        },
        RingTone(bool),
        VoiceInput(&'a str),
        RingIndication,
        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        RecvData(&'a [u8]),
        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        SendData(&'a mut [u8]),
        Other {
            raw_event: esp_hf_client_cb_event_t,
            raw_data: EventRawData<'a>,
        },
    }

    #[allow(non_upper_case_globals)]
    impl<'a> From<(esp_hf_client_cb_event_t, &'a esp_hf_client_cb_param_t)> for HfpcEvent<'a> {
        fn from(value: (esp_hf_client_cb_event_t, &'a esp_hf_client_cb_param_t)) -> Self {
            let (event, param) = value;

            unsafe {
                match event {
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CONNECTION_STATE_EVT => Self::ConnectionState {
                        bd_addr: param.conn_stat.remote_bda.into(),
                        status: param.conn_stat.state.try_into().unwrap(),
                        peer_features: param.conn_stat.peer_feat,
                        child_features: param.conn_stat.chld_feat,
                    },
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_AUDIO_STATE_EVT => Self::AudioState {
                        bd_addr: param.audio_stat.remote_bda.into(),
                        status: param.audio_stat.state.try_into().unwrap(),
                    },
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_BVRA_EVT => {
                        if param.bvra.value == esp_hf_vr_state_t_ESP_HF_VR_STATE_ENABLED {
                            Self::VoiceRecognitionEnabled
                        } else {
                            Self::VoiceRecognitionDisabled
                        }
                    }
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_CALL_EVT => Self::CallState(param.call.status != esp_hf_call_status_t_ESP_HF_CALL_STATUS_NO_CALLS),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_CALL_SETUP_EVT => Self::CallSetupState(param.call_setup.status.try_into().unwrap()),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_CALL_HELD_EVT => Self::CallHeld(param.call_held.status.try_into().unwrap()),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT => Self::NetworkServiceAvailability(param.service_availability.status != esp_hf_network_state_t_ESP_HF_NETWORK_STATE_NOT_AVAILABLE),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT => Self::SignalStrength(param.signal_strength.value as _),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT => Self::Roaming(param.roaming.status != esp_hf_roaming_status_t_ESP_HF_ROAMING_STATUS_INACTIVE),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT => Self::BatteryLevel(param.battery_level.value as _),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT => Self::NetworkOperator(from_cstr_ptr(param.cops.name)),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_BTRH_EVT => Self::CallResponseAndHold(param.btrh.status.try_into().unwrap()),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CLIP_EVT => Self::CallingLineIdentification(from_cstr_ptr(param.clip.number)),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CCWA_EVT => Self::CallWaiting(from_cstr_ptr(param.ccwa.number)),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CLCC_EVT => Self::CurrentCall {
                        index: param.clcc.idx as _,
                        outgoing: param.clcc.dir == esp_hf_current_call_direction_t_ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
                        status: param.clcc.status.try_into().unwrap(),
                        multi_party: param.clcc.mpty != esp_hf_current_call_mpty_type_t_ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
                        number: from_cstr_ptr(param.clcc.number),
                    },
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_VOLUME_CONTROL_EVT => Self::VolumeControl(if param.volume_control.type_ == esp_hf_volume_control_target_t_ESP_HF_VOLUME_CONTROL_TARGET_SPK {
                        Volume::Speaker(param.volume_control.volume as _)
                    } else {
                        Volume::Microphone(param.volume_control.volume as _)
                    }),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_AT_RESPONSE_EVT => Self::AtResponse {
                        code: param.at_response.code.try_into().unwrap(),
                        extended_code: param.at_response.cme.try_into().unwrap(),
                    },
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_CNUM_EVT => Self::SubscriberInfo {
                        number: from_cstr_ptr(param.cnum.number),
                        service_type: param.cnum.type_.try_into().unwrap(),
                    },
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_BSIR_EVT => Self::RingTone(param.bsir.state != esp_hf_client_in_band_ring_state_t_ESP_HF_CLIENT_IN_BAND_RINGTONE_NOT_PROVIDED),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_BINP_EVT => Self::VoiceInput(from_cstr_ptr(param.binp.number)),
                    esp_hf_client_cb_event_t_ESP_HF_CLIENT_RING_IND_EVT => Self::RingIndication,
                    _ => Self::Other {
                        raw_event: event,
                        raw_data: EventRawData(param),
                    },
                }
            }
        }
    }

    pub struct EspHfpc<'d, M, T>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        _driver: T,
        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        resampling_source: Option<Source>,
        _p: PhantomData<&'d ()>,
        _m: PhantomData<M>,
    }

    impl<'d, M, T> EspHfpc<'d, M, T>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        pub fn new(driver: T, resampling_source: Option<Source>) -> Result<Self, EspError> {
            Self::initialize()?;

            if let Some(resampling_source) = resampling_source {
                unsafe {
                    esp_hf_client_pcm_resample_init(
                        resampling_source.sample_rate_hz,
                        resampling_source.bits_per_sample as _,
                        if resampling_source.stereo { 2 } else { 1 },
                    );
                }
            }

            Ok(Self {
                _driver: driver,
                resampling_source,
                _p: PhantomData,
                _m: PhantomData,
            })
        }

        #[cfg(not(esp_idf_bt_hfp_audio_data_path_hci))]
        pub fn new(driver: T) -> Result<Self, EspError> {
            Self::initialize()?;

            Ok(Self {
                _driver: driver,
                _p: PhantomData,
                _m: PhantomData,
            })
        }

        fn initialize() -> Result<(), EspError> {
            SINGLETON.take()?;

            esp!(unsafe { esp_hf_client_register_callback(Some(Self::event_handler)) })?;
            esp!(unsafe { esp_hf_client_init() })?;

            #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
            esp!(unsafe {
                esp_hf_client_register_data_callback(
                    Some(Self::recv_data_handler),
                    Some(Self::send_data_handler),
                )
            })?;

            Ok(())
        }

        pub fn subscribe<F>(&self, events_cb: F) -> Result<(), EspError>
        where
            F: FnMut(HfpcEvent) -> usize + Send + 'static,
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
            F: FnMut(HfpcEvent) -> usize + Send + 'd,
        {
            SINGLETON.subscribe(events_cb);

            Ok(())
        }

        pub fn unsubscribe(&self) -> Result<(), EspError> {
            SINGLETON.unsubscribe();

            Ok(())
        }

        pub fn connect(&self, bd_addr: &esp_bd_addr_t) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_connect(bd_addr as *const _ as *mut _) })
        }

        pub fn disconnect(&self, bd_addr: &esp_bd_addr_t) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_disconnect(bd_addr as *const _ as *mut _) })
        }

        pub fn connect_audio(&self, bd_addr: &esp_bd_addr_t) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_connect_audio(bd_addr as *const _ as *mut _) })
        }

        pub fn disconnect_audio(&self, bd_addr: &esp_bd_addr_t) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_disconnect_audio(bd_addr as *const _ as *mut _) })
        }

        pub fn start_voice_recognition(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_start_voice_recognition() })
        }

        pub fn stop_voice_recognition(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_stop_voice_recognition() })
        }

        pub fn update_volume(&self, volume: Volume) -> Result<(), EspError> {
            let (volume_type, gain) = match volume {
                Volume::Speaker(gain) => (
                    esp_hf_volume_control_target_t_ESP_HF_VOLUME_CONTROL_TARGET_SPK,
                    gain as _,
                ),
                Volume::Microphone(gain) => (
                    esp_hf_volume_control_target_t_ESP_HF_VOLUME_CONTROL_TARGET_MIC,
                    gain as _,
                ),
            };

            esp!(unsafe { esp_hf_client_volume_update(volume_type, gain) })
        }

        pub fn dial(&self, number: &str) -> Result<(), EspError> {
            let number = to_cstring_arg(number)?;

            esp!(unsafe { esp_hf_client_dial(number.as_ptr()) })
        }

        pub fn dial_memory(&self, location: usize) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_dial_memory(location as _) })
        }

        // pub fn hold(&self, location: usize) -> Result<(), EspError> {
        //     esp!(unsafe { esp_hf_client_send_chld_cmd(location as _) })
        // }

        pub fn reply_hold(&self, location: usize) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_send_btrh_cmd(location as _) })
        }

        pub fn dtmf(&self, location: usize) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_send_dtmf(location as _) })
        }

        // pub fn apple(&self, location: usize) -> Result<(), EspError> {
        //     esp!(unsafe { esp_hf_client_send_xapl(location as _) })
        // }

        // pub fn apple_iphone_info(&self, location: usize) -> Result<(), EspError> {
        //     esp!(unsafe { esp_hf_client_send_iphoneaccev(location as _) })
        // }

        pub fn answer(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_answer_call() })
        }

        pub fn reject(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_reject_call() })
        }

        pub fn request_current_calls(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_query_current_calls() })
        }

        pub fn request_current_operator_name(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_query_current_operator_name() })
        }

        pub fn request_subscriber_info(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_retrieve_subscriber_info() })
        }

        pub fn request_last_voice_tag_number(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_request_last_voice_tag_number() })
        }

        pub fn disable_ag_aec(&self) -> Result<(), EspError> {
            esp!(unsafe { esp_hf_client_send_nrec() })
        }

        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        pub fn request_outgoing_data_ready(&self) {
            unsafe {
                esp_hf_client_outgoing_data_ready();
            }
        }

        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        pub fn pcm_resample(&self, src: &[u8], dst: &mut [u8]) -> Result<usize, EspError> {
            if self.resampling_source.is_some() {
                if dst.len() >= src.len() {
                    let samples_ct = unsafe {
                        esp_hf_client_pcm_resample(
                            src.as_ptr() as *mut ffi::c_void,
                            src.len() as _,
                            dst.as_ptr() as *mut ffi::c_void,
                        )
                    };

                    Ok(samples_ct as usize * 2)
                } else {
                    Err(EspError::from_infallible::<ESP_ERR_INVALID_ARG>())
                }
            } else {
                Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
            }
        }

        unsafe extern "C" fn event_handler(
            event: esp_hf_client_cb_event_t,
            param: *mut esp_hf_client_cb_param_t,
        ) {
            if let Some(param) = unsafe { param.as_ref() } {
                let event = HfpcEvent::from((event, param));

                info!("Got event {{ {:#?} }}", event);

                SINGLETON.call(event);
            }
        }

        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        unsafe extern "C" fn recv_data_handler(buf: *const u8, len: u32) {
            let event = HfpcEvent::RecvData(core::slice::from_raw_parts(buf, len as _));
            debug!("Got event {{ {:#?} }}", event);

            SINGLETON.call(event);
        }

        #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
        unsafe extern "C" fn send_data_handler(buf: *mut u8, len: u32) -> u32 {
            let event = HfpcEvent::SendData(core::slice::from_raw_parts_mut(buf, len as _));
            debug!("Got event {{ {:#?} }}", event);

            SINGLETON.call(event) as _
        }
    }

    impl<'d, M, T> Drop for EspHfpc<'d, M, T>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>>,
    {
        fn drop(&mut self) {
            #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
            if self.resampling_source.is_some() {
                unsafe {
                    esp_hf_client_pcm_resample_deinit();
                }
            }

            self.unsubscribe().unwrap();

            // Not possible because this function rejects NULL arguments
            // esp!(unsafe { esp_hf_client_register_callback(None) }).unwrap();

            #[cfg(esp_idf_bt_hfp_audio_data_path_hci)]
            esp!(unsafe { esp_hf_client_register_data_callback(None, None) }).unwrap();

            esp!(unsafe { esp_hf_client_deinit() }).unwrap();

            SINGLETON.release().unwrap();
        }
    }

    unsafe impl<'d, M, T> Send for EspHfpc<'d, M, T>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>> + Send,
    {
    }

    // Safe because the ESP IDF Bluedroid APIs all do message passing
    // to a dedicated Bluedroid task
    unsafe impl<'d, M, T> Sync for EspHfpc<'d, M, T>
    where
        M: BtClassicEnabled,
        T: Borrow<BtDriver<'d, M>> + Send,
    {
    }

    static SINGLETON: BtSingleton<HfpcEvent, usize> = BtSingleton::new(0);
}
