use core::{mem, ptr, time::Duration};

use ::log::*;

use embedded_svc::ipv4;
use embedded_svc::ping::*;

use esp_idf_sys::*;

use crate::private::common::*;
use crate::private::waitable::*;

#[derive(Debug, Default)]
pub struct EspPing(u32);

unsafe impl Send for EspPing {}
unsafe impl Sync for EspPing {}

impl EspPing {
    pub fn new(interface_index: u32) -> Self {
        Self(interface_index)
    }

    pub fn ping(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<Summary, EspError> {
        info!(
            "About to run a summary ping {} with configuration {:?}",
            ip, conf
        );

        let mut tracker = Tracker::new(Some(&nop_callback));

        self.run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.summary)
    }

    pub fn ping_details<F: Fn(&Summary, &Reply)>(
        &mut self,
        ip: ipv4::Ipv4Addr,
        conf: &Configuration,
        reply_callback: &F,
    ) -> Result<Summary, EspError> {
        info!(
            "About to run a detailed ping {} with configuration {:?}",
            ip, conf
        );

        let mut tracker = Tracker::new(Some(reply_callback));

        self.run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.summary)
    }

    fn run_ping<F: Fn(&Summary, &Reply)>(
        &self,
        ip: ipv4::Ipv4Addr,
        conf: &Configuration,
        tracker: &mut Tracker<F>,
    ) -> Result<(), EspError> {
        #[allow(clippy::needless_update)]
        #[allow(clippy::useless_conversion)]
        let config = esp_ping_config_t {
            count: conf.count,
            interval_ms: conf.interval.as_millis() as u32,
            timeout_ms: conf.timeout.as_millis() as u32,
            data_size: conf.data_size,
            tos: conf.tos.into(),
            target_addr: ip_addr_t {
                u_addr: ip_addr__bindgen_ty_1 {
                    ip4: Newtype::<ip4_addr_t>::from(ip).0,
                },
                type_: 0,
            },
            task_stack_size: 4096,
            task_prio: 2,
            interface: self.0,
            ..Default::default()
        };

        let callbacks = esp_ping_callbacks_t {
            on_ping_success: Some(EspPing::on_ping_success::<F>),
            on_ping_timeout: Some(EspPing::on_ping_timeout::<F>),
            on_ping_end: Some(EspPing::on_ping_end::<F>),
            cb_args: tracker as *mut Tracker<F> as *mut c_types::c_void,
        };

        let mut handle: esp_ping_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe {
            esp_ping_new_session(&config, &callbacks, handle_ref as *mut *mut c_types::c_void)
        })?;

        if handle.is_null() {
            return Err(EspError::from(ESP_ERR_INVALID_ARG as _).unwrap());
        }

        info!("Ping session established, got handle {:?}", handle);

        {
            let mut running = tracker.waitable.state.lock();
            *running = true;
        }

        esp!(unsafe { esp_ping_start(handle) })?;
        info!("Ping session started");

        info!("Waiting for the ping session to complete");

        tracker.waitable.wait_while(|running| *running);

        esp!(unsafe { esp_ping_stop(handle) })?;
        info!("Ping session stopped");

        esp!(unsafe { esp_ping_delete_session(handle) })?;

        info!("Ping session {:?} removed", &handle);

        Ok(())
    }

    unsafe extern "C" fn on_ping_success<F: Fn(&Summary, &Reply)>(
        handle: esp_ping_handle_t,
        args: *mut c_types::c_void,
    ) {
        info!("Ping success callback invoked");

        let tracker_ptr: *mut Tracker<F> = args as _;
        let tracker = tracker_ptr.as_mut().unwrap();

        let mut seqno: c_types::c_ushort = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SEQNO,
            &mut seqno as *mut c_types::c_ushort as *mut c_types::c_void,
            mem::size_of_val(&seqno) as u32,
        );

        let mut ttl: c_types::c_uchar = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_TTL,
            &mut ttl as *mut c_types::c_uchar as *mut c_types::c_void,
            mem::size_of_val(&ttl) as u32,
        );

        let mut target_addr_raw = [0_u8; mem::size_of::<ip_addr_t>()];
        let target_addr: &mut ip_addr_t = mem::transmute(&mut target_addr_raw);

        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_IPADDR,
            target_addr as *mut ip_addr_t as *mut c_types::c_void,
            mem::size_of::<ip_addr_t>() as _,
        );

        let mut elapsed_time: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_TIMEGAP,
            &mut elapsed_time as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&elapsed_time) as u32,
        );

        let mut recv_len: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SIZE,
            &mut recv_len as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&recv_len) as u32,
        );

        let addr = ipv4::Ipv4Addr::from(Newtype(target_addr.u_addr.ip4));

        info!(
            "From {} icmp_seq={} ttl={} time={}ms bytes={}",
            addr, seqno, ttl, elapsed_time, recv_len
        );

        if let Some(reply_callback) = tracker.reply_callback {
            Self::update_summary(handle, &mut tracker.summary);

            reply_callback(
                &tracker.summary,
                &Reply::Success(Info {
                    addr,
                    seqno: seqno as u32,
                    ttl,
                    recv_len,
                    elapsed_time: Duration::from_millis(elapsed_time as u64),
                }),
            );
        }
    }

    unsafe extern "C" fn on_ping_timeout<F: Fn(&Summary, &Reply)>(
        handle: esp_ping_handle_t,
        args: *mut c_types::c_void,
    ) {
        info!("Ping timeout callback invoked");

        let tracker_ptr: *mut Tracker<F> = args as _;
        let tracker = tracker_ptr.as_mut().unwrap();

        let mut seqno: c_types::c_ushort = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SEQNO,
            &mut seqno as *mut c_types::c_ushort as *mut c_types::c_void,
            mem::size_of_val(&seqno) as u32,
        );

        let mut target_addr_raw = [0_u8; mem::size_of::<ip_addr_t>()];
        let target_addr: &mut ip_addr_t = mem::transmute(&mut target_addr_raw);

        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_IPADDR,
            target_addr as *mut ip_addr_t as *mut c_types::c_void,
            mem::size_of::<ip_addr_t>() as _,
        );

        info!("From {} icmp_seq={} timeout", "???", seqno);

        if let Some(reply_callback) = tracker.reply_callback {
            Self::update_summary(handle, &mut tracker.summary);

            reply_callback(&tracker.summary, &Reply::Timeout);
        }
    }

    #[allow(clippy::mutex_atomic)]
    unsafe extern "C" fn on_ping_end<F: Fn(&Summary, &Reply)>(
        handle: esp_ping_handle_t,
        args: *mut c_types::c_void,
    ) {
        info!("Ping end callback invoked");

        let tracker_ptr: *mut Tracker<F> = args as _;
        let tracker = tracker_ptr.as_mut().unwrap();

        Self::update_summary(handle, &mut tracker.summary);

        info!(
            "{} packets transmitted, {} received, time {}ms",
            tracker.summary.transmitted,
            tracker.summary.received,
            tracker.summary.time.as_millis()
        );

        let mut running = tracker.waitable.state.lock();
        *running = false;

        tracker.waitable.cvar.notify_all();
    }

    unsafe fn update_summary(handle: esp_ping_handle_t, summary: &mut Summary) {
        let mut transmitted: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_REQUEST,
            &mut transmitted as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&transmitted) as u32,
        );

        let mut received: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_REPLY,
            &mut received as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&received) as u32,
        );

        let mut total_time: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_DURATION,
            &mut total_time as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&total_time) as u32,
        );

        summary.transmitted = transmitted;
        summary.received = received;
        summary.time = Duration::from_millis(total_time as u64);
    }
}

impl Ping for EspPing {
    type Error = EspError;

    fn ping(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<Summary, Self::Error> {
        EspPing::ping(self, ip, conf)
    }

    fn ping_details<F: Fn(&Summary, &Reply)>(
        &mut self,
        ip: ipv4::Ipv4Addr,
        conf: &Configuration,
        reply_callback: &F,
    ) -> Result<Summary, Self::Error> {
        EspPing::ping_details(self, ip, conf, reply_callback)
    }
}

struct Tracker<'a, F: Fn(&Summary, &Reply)> {
    summary: Summary,
    waitable: Waitable<bool>,
    reply_callback: Option<&'a F>,
}

impl<'a, F: Fn(&Summary, &Reply)> Tracker<'a, F> {
    #[allow(clippy::mutex_atomic)]
    pub fn new(reply_callback: Option<&'a F>) -> Self {
        Self {
            summary: Default::default(),
            waitable: Waitable::new(false),
            reply_callback,
        }
    }
}

fn nop_callback(_summary: &Summary, _reply: &Reply) {}
