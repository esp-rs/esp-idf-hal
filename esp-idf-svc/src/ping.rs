use std::{mem, ptr, time::Duration, vec};
use std::sync::{Condvar, Mutex};

use anyhow::*;

use embedded_svc::ipv4;
use embedded_svc::ping::*;
use esp_idf_sys::*;
use log::info;

use crate::common::*;

pub struct EspPing;

unsafe impl Send for EspPing {}
unsafe impl Sync for EspPing {}

impl EspPing {
    fn run_ping(ip: ipv4::Ipv4Addr, conf: &Configuration, tracker: &mut Tracker) -> Result<()> {
        let config = esp_ping_config_t {
            count: conf.count,
            interval_ms: conf.interval.as_millis() as u32,
            timeout_ms: conf.timeout.as_millis() as u32,
            data_size: conf.data_size,
            tos: conf.tos,
            target_addr: ip_addr_t {
                u_addr: ip_addr__bindgen_ty_1 {
                    ip4: Newtype::<ip4_addr_t>::from(ip).0,
                },
                type_: 0,
            },
            task_stack_size: 4096,
            task_prio: 2,
            ..Default::default()
        };

        let callbacks = esp_ping_callbacks_t {
            on_ping_success: Some(EspPing::on_ping_success),
            on_ping_timeout: Some(EspPing::on_ping_timeout),
            on_ping_end: Some(EspPing::on_ping_end),
            cb_args: tracker as *mut Tracker as *mut c_types::c_void,
        };

        let mut handle: esp_ping_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe {esp_ping_new_session(&config, &callbacks, handle_ref as *mut *mut c_types::c_void)})?;

        info!("Ping session established, got handle {:?}", &handle);

        {
            *tracker.running.lock().unwrap() = true;
        }

        esp!(unsafe {esp_ping_start(handle)})?;
        info!("Ping session started");

        info!("Waiting for the ping session to complete");

        // loop {
        //     {
        //         let finished = tracker.lock.lock().unwrap();
        //         if *finished {
        //             break
        //         }
        //     }

        //     thread::sleep(Duration::from_millis(500));
        // }

        let _running = tracker.cvar.wait_while(tracker.running.lock().unwrap(), |running| *running).unwrap();

        esp!(unsafe {esp_ping_stop(handle)})?;
        info!("Ping session stopped");

        esp!(unsafe {esp_ping_delete_session(handle)})?;

        info!("Ping session {:?} removed", &handle);

        Ok(())
    }

    unsafe extern "C" fn on_ping_success(handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        info!("Ping success callback invoked");

        let tracker = (args as *mut Tracker).as_mut().unwrap();

        let mut seqno: c_types::c_ushort = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SEQNO,
            &mut seqno as *mut c_types::c_ushort as *mut c_types::c_void,
            mem::size_of_val(&seqno) as u32);

        let mut ttl: c_types::c_uchar = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_TTL,
            &mut ttl as *mut c_types::c_uchar as *mut c_types::c_void,
            mem::size_of_val(&ttl) as u32);

        let mut target_addr_raw = [0 as u8; mem::size_of::<ip_addr_t>()];
        let target_addr: &mut ip_addr_t = mem::transmute(&mut target_addr_raw);

        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_IPADDR,
            target_addr as *mut ip_addr_t as *mut c_types::c_void,
            mem::size_of::<ip_addr_t> as u32);

        let mut elapsed_time: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_TIMEGAP,
            &mut elapsed_time as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&elapsed_time) as u32);

        let mut recv_len: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SIZE,
            &mut recv_len as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&recv_len) as u32);

        let addr = ipv4::Ipv4Addr::from(Newtype(target_addr.u_addr.ip4));

        info!("From {} icmp_seq={} ttl={} time={}ms bytes={}",
            addr,
            seqno,
            ttl,
            elapsed_time,
            recv_len);

        if let Some(ref mut replies) = tracker.replies {
            replies.push(Reply::Success(Info {
                addr,
                seqno: seqno as u32,
                ttl: ttl as u8,
                recv_len: recv_len as u32,
                elapsed_time: Duration::from_millis(elapsed_time as u64)
            }));
        }
    }

    unsafe extern "C" fn on_ping_timeout(handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        info!("Ping timeout callback invoked");

        let tracker = (args as *mut Tracker).as_mut().unwrap();

        let mut seqno: c_types::c_ushort = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_SEQNO,
            &mut seqno as *mut c_types::c_ushort as *mut c_types::c_void,
            mem::size_of_val(&seqno) as u32);

        let mut target_addr_raw = [0 as u8; mem::size_of::<ip_addr_t>()];
        let target_addr: &mut ip_addr_t = mem::transmute(&mut target_addr_raw);

        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_IPADDR,
            target_addr as *mut ip_addr_t as *mut c_types::c_void,
            mem::size_of::<ip_addr_t> as u32);

        info!("From {} icmp_seq={} timeout", "???", seqno);

        if let Some(ref mut replies) = tracker.replies {
            replies.push(Reply::Timeout);
        }
    }

    unsafe extern "C" fn on_ping_end(handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        info!("Ping end callback invoked");

        let tracker = (args as *mut Tracker).as_mut().unwrap();

        let mut transmitted: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_REQUEST,
            &mut transmitted as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&transmitted) as u32);

        let mut received: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_REPLY,
            &mut received as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&received) as u32);

        let mut total_time: c_types::c_uint = 0;
        esp_ping_get_profile(
            handle,
            esp_ping_profile_t_ESP_PING_PROF_DURATION,
            &mut total_time as *mut c_types::c_uint as *mut c_types::c_void,
            mem::size_of_val(&total_time) as u32);

        info!("{} packets transmitted, {} received, time {}ms", transmitted, received, total_time);

        tracker.summary.transmitted = transmitted;
        tracker.summary.received = received;
        tracker.summary.time = Duration::from_millis(total_time as u64);

        *tracker.running.lock().unwrap() = false;
        tracker.cvar.notify_one();
    }
}

impl Ping for EspPing {
    fn ping(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<vec::Vec<Reply>> {
        info!("About to run a detailed ping {} with configuration {:?}", ip, &conf);

        let mut tracker = Tracker {
            replies: Some(vec::Vec::new()),
            ..Default::default()
        };

        Self::run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.replies.unwrap())
    }

    fn ping_summary(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<Summary> {
        info!("About to run a summary ping {} with configuration {:?}", ip, &conf);

        let mut tracker = Default::default();

        Self::run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.summary)
    }
}

struct Tracker {
    summary: Summary,
    replies: Option<vec::Vec<Reply>>,
    cvar: Condvar,
    running: Mutex<bool>,
}

impl Default for Tracker {
    fn default() -> Self {
        Tracker {
            summary: Summary {
                transmitted: 0,
                received: 0,
                time: Duration::from_secs(0),
            },
            replies: None,
            cvar: Condvar::new(),
            running: Mutex::new(false),
        }
    }
}
