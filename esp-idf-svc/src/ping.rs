use std::{mem, ptr, time::Duration, vec};
use std::sync::{Condvar, Mutex};

use anyhow::*;

use embedded_svc::ipv4;
use embedded_svc::ping::*;
use esp_idf_sys::*;

use crate::common::*;

pub struct EspPing;

struct Tracker {
    summary: Summary,
    replies: Option<vec::Vec<Reply>>,
    cvar: Condvar,
    lock: Mutex<bool>,
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
            lock: Mutex::new(false),
        }
    }
}

impl EspPing {
    pub fn new() -> Self {
        EspPing
    }

    fn run_ping(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration, tracker: &mut Tracker) -> Result<()> {
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
            task_stack_size: 2048,
            task_prio: 2,
        };

        let callbacks = esp_ping_callbacks_t {
            on_ping_success: Some(EspPing::on_ping_success),
            on_ping_timeout: Some(EspPing::on_ping_timeout),
            on_ping_end: Some(EspPing::on_ping_end),
            cb_args: ptr::null_mut(),
        };

        let mut handle: esp_ping_handle_t = ptr::null_mut();
        let handle_ref = &mut handle;

        esp!(unsafe {esp_ping_new_session(&config, &callbacks, handle_ref)})?;

        let mut finished = tracker.lock.lock().unwrap();
        while !*finished {
            finished = tracker.cvar.wait(finished).unwrap();
        }

        esp!(unsafe {esp_ping_delete_session(handle)})?;

        Ok(())
    }

    unsafe extern "C" fn on_ping_success(handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        let tracker: &mut Tracker = mem::transmute(args);

        let mut elapsed_time: c_types::c_uint = 0;
        esp_ping_get_profile(handle, esp_ping_profile_t_ESP_PING_PROF_TIMEGAP, mem::transmute(&mut elapsed_time), mem::size_of_val(&elapsed_time) as u32);

        tracker.summary.transmitted += 1;
        tracker.summary.received += 1;
        tracker.summary.time += Duration::from_millis(elapsed_time as u64);

        if let Some(ref mut replies) = tracker.replies {
            //let mut seqno: c_types::c_ushort;
            //esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, mem::size_of_val(&seqno) as u32);

            let mut ttl: c_types::c_uchar = 0;
            esp_ping_get_profile(handle, esp_ping_profile_t_ESP_PING_PROF_TTL, mem::transmute(&mut ttl), mem::size_of_val(&ttl) as u32);

            let mut recv_len: c_types::c_uint = 0;
            esp_ping_get_profile(handle, esp_ping_profile_t_ESP_PING_PROF_SIZE, mem::transmute(&mut recv_len), mem::size_of_val(&recv_len) as u32);

            replies.push(Reply::Success(Info {
                ttl: ttl,
                elapsed_time: Duration::from_millis(elapsed_time as u64),
                recv_len: recv_len,
            }));
        }
    }

    unsafe extern "C" fn on_ping_timeout(_handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        let tracker: &mut Tracker = mem::transmute(args);

        tracker.summary.transmitted += 1;
        tracker.summary.received += 1;

        if let Some(ref mut replies) = tracker.replies {
            //let mut seqno: c_types::c_ushort;
            //esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, mem::size_of_val(&seqno) as u32);

            replies.push(Reply::Timeout);
        }
    }

    unsafe extern "C" fn on_ping_end(_handle: esp_ping_handle_t, args: *mut c_types::c_void) {
        let tracker: &mut Tracker = mem::transmute(args);

        let mut finished = tracker.lock.lock().unwrap();

        *finished = true;
        tracker.cvar.notify_one();
    }
}

impl Ping for EspPing {
    fn ping(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<vec::Vec<Reply>> {
        let mut tracker = Tracker {
            replies: Some(vec::Vec::new()),
            ..Default::default()
        };

        self.run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.replies.unwrap())
    }

    fn ping_summary(&mut self, ip: ipv4::Ipv4Addr, conf: &Configuration) -> Result<Summary> {
        let mut tracker = Default::default();

        self.run_ping(ip, conf, &mut tracker)?;

        Ok(tracker.summary)
    }
}
