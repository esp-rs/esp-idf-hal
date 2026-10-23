//! Checks that spawning and joining `std::thread`s does not leak memory.
//!
//! Each variant spawns and joins a batch of threads, and compares the free heap before and after the batch.
//! The variants exercise the thread handle in various ways, including `std::thread::current()` being called
//! from the destructor of a thread-local, which should keep working during the thread-local destruction.
//!
//! Rust STD 1.83 and later leaks ~150 bytes per thread on ESP-IDF, unless it has the fix for
//! https://github.com/esp-rs/esp-idf/issues/522

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]
#![allow(clippy::missing_const_for_thread_local)]

use std::thread::{self, Builder};
use std::time::Duration;

use esp_idf_svc::sys::esp_get_free_heap_size;

const THREADS: u32 = 100;

/// Room for the occasional allocation unrelated to the threads themselves
const SLACK: u32 = 256;

struct CurrentOnDrop;

impl Drop for CurrentOnDrop {
    fn drop(&mut self) {
        let _ = thread::current().id();
    }
}

thread_local! {
    static CURRENT_ON_DROP: CurrentOnDrop = const { CurrentOnDrop };
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let unnamed = || Builder::new().stack_size(4096);
    let named = || Builder::new().stack_size(4096).name("leak-check".into());

    let mut leaking = false;

    leaking |= check("plain", unnamed, || ())?;
    leaking |= check("named", named, || ())?;
    leaking |= check("current + park", named, || {
        thread::current().unpark();
        thread::park();
    })?;
    leaking |= check("current() in a TLS destructor", unnamed, || {
        CURRENT_ON_DROP.with(|_| ())
    })?;

    if leaking {
        anyhow::bail!("Spawning threads leaks memory");
    }

    log::info!("No leaks detected");

    Ok(())
}

fn check(variant: &str, builder: fn() -> Builder, body: fn()) -> anyhow::Result<bool> {
    // The first spawn of a variant allocates things which stay around for good,
    // like the TLS keys of the thread-locals it uses
    spawn_join(builder, body)?;

    let before = free_heap();

    for _ in 0..THREADS {
        spawn_join(builder, body)?;
    }

    // Let the idle task free the deleted tasks, if any are left
    thread::sleep(Duration::from_millis(100));

    let leaked = before.saturating_sub(free_heap());
    let leaking = leaked > SLACK;

    if leaking {
        log::error!(
            "{variant}: LEAKING {leaked} bytes after {THREADS} threads (~{} bytes per thread)",
            leaked / THREADS
        );
    } else {
        log::info!("{variant}: OK ({leaked} bytes after {THREADS} threads)");
    }

    Ok(leaking)
}

fn spawn_join(builder: fn() -> Builder, body: fn()) -> anyhow::Result<()> {
    builder()
        .spawn(body)?
        .join()
        .map_err(|_| anyhow::anyhow!("Thread panicked"))
}

fn free_heap() -> u32 {
    unsafe { esp_get_free_heap_size() }
}
