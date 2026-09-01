//! Restoring the POSIX binding between the C standard streams and file
//! descriptors 0, 1 and 2.
//!
//! ESP-IDF attaches `stdin`/`stdout`/`stderr` to the `/dev/console` VFS device.
//! Opening `/dev/console` internally opens its backing devices (the primary
//! console - usually UART - and, on chips which have one, the USB-Serial-JTAG
//! secondary console) *first*, and VFS descriptors are handed out
//! lowest-free-first. As a result, the standard streams end up on descriptors
//! 2/3/4 (or similar), while descriptors 0/1/2 point to the raw backing devices.
//!
//! Anything which assumes the POSIX descriptor binding - most prominently the
//! Rust Standard Library, which reads `stdin` from descriptor 0 and writes
//! `println!`/`eprintln!` output to descriptors 1/2 - either loses its output,
//! or panics with "failed printing to stdout: Success (os error 0)", as
//! descriptor 1 is usually the raw USB-Serial-JTAG secondary console device,
//! whose `write` fails without setting `errno` when no USB host is attached.
//!
//! Note that all of the above only applies when the I/O support of the VFS
//! component is compiled in (`CONFIG_VFS_SUPPORT_IO`). Without it, ESP-IDF
//! initializes the standard streams with the stock `__sinit` of the C library,
//! which does bind them to descriptors 0, 1 and 2 natively.
//!
//! How the descriptors are re-arranged depends on the C library, because the
//! two libraries ESP-IDF supports own their standard streams very differently:
//!
//! - With newlib, the streams are `fopen`-ed by ESP-IDF, so they can be closed
//!   and re-created, which is what the restoration does: it re-creates them at
//!   a point where descriptors 0, 1 and 2 are the lowest free ones.
//! - With picolibc (the default from ESP-IDF v6.0 on), the streams are
//!   statically allocated `FILE`s owned by ESP-IDF and set up without picolibc's
//!   "allocated by stdio" flag. They must not be `fclose`-ed: `fclose` would
//!   neither close their descriptors nor free them, but it *would* destroy their
//!   locks, leaving each stream with a dangling handle to a deleted FreeRTOS
//!   mutex - the next stdio call on such a stream spins in `taskENTER_CRITICAL`
//!   on freed memory until the interrupt watchdog fires. And re-running
//!   ESP-IDF's stdio initialization would not re-create them either; it only
//!   re-opens the console and stamps the descriptors into the static streams -
//!   asserting, on top of that, that `stdin` does *not* land on descriptor 0.
//!   So for picolibc the standard streams are left alone (only their console
//!   handles are re-opened) and descriptors 0, 1 and 2 are claimed by console
//!   handles of our own.

/// Restores the POSIX binding between the C standard streams and file
/// descriptors 0, 1 and 2, so that reads from descriptor 0 and writes to
/// descriptors 1 and 2 go to `/dev/console`.
///
/// Returns `true` if the binding is in place when the function returns
/// (either because it was already in place, or because it was successfully
/// restored), and `false` otherwise.
///
/// The function is idempotent. It is called automatically from the `app_main`
/// glue of the `binstart`/`libstart` features, so calling it explicitly is
/// only necessary with a custom `app_main`.
///
/// On ESP-IDF versions older than v5.3 the restoration is not attempted (the
/// function only reports whether the binding happens to be in place): the
/// console of those versions does not refcount its open/close calls, which
/// the re-arrangement of the descriptors relies on. For the older versions,
/// the `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y` sdkconfig setting is a build-time
/// alternative, as it results in descriptors 1 and 2 landing on the console.
///
/// NOTE: the standard streams are detached from the console and re-attached to
/// it while the function runs, so it must be called *before* any other thread
/// might be using them - which is why the `app_main` glue calls it first thing.
#[allow(clippy::needless_bool)]
pub fn restore_posix_stdio_fds() -> bool {
    unsafe {
        // Nothing to do if the streams are already on their POSIX descriptors:
        // - a second call (newlib),
        // - an ESP-IDF which binds them correctly,
        // - VFS I/O support not compiled in, in which case ESP-IDF initializes
        //   the streams with the stock `__sinit` of the C library, which does
        //   bind them to descriptors 0, 1 and 2
        if imp::bound() {
            return true;
        }

        #[cfg(all(
            esp_idf_comp_vfs_enabled,
            esp_idf_vfs_support_io,
            esp_idf_version_at_least_5_3_0
        ))]
        {
            imp::restore()
        }

        #[cfg(not(all(
            esp_idf_comp_vfs_enabled,
            esp_idf_vfs_support_io,
            esp_idf_version_at_least_5_3_0
        )))]
        {
            false
        }
    }
}

mod imp {
    use crate::*;

    /// Returns `true` if the standard streams are bound to descriptors 0, 1
    /// and 2
    pub(super) unsafe fn bound() -> bool {
        matches!(
            streams(),
            Some((si, so, se)) if fileno(si) == 0 && fileno(so) == 1 && fileno(se) == 2
        )
    }

    #[cfg(not(esp_idf_libc_picolibc))]
    pub(super) unsafe fn streams() -> Option<(*mut FILE, *mut FILE, *mut FILE)> {
        // The global reentrancy structure owns the standard streams; the reent
        // of every task points to the same streams, courtesy of `esp_reent_init`
        let g = _global_impure_ptr;
        if g.is_null() {
            return None;
        }

        let (si, so, se) = ((*g)._stdin, (*g)._stdout, (*g)._stderr);

        (!si.is_null() && !so.is_null() && !se.is_null()).then_some((si, so, se))
    }

    #[cfg(esp_idf_libc_picolibc)]
    pub(super) unsafe fn streams() -> Option<(*mut FILE, *mut FILE, *mut FILE)> {
        (!stdin.is_null() && !stdout.is_null() && !stderr.is_null())
            .then_some((stdin, stdout, stderr))
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    pub(super) use restore::restore;

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    mod restore {
        use core::ffi::c_int;
        use core::ptr;

        use crate::*;

        use super::streams;

        pub(in crate::stdio) const CONSOLE: &core::ffi::CStr = c"/dev/console";

        /// Claims the lowest three free descriptors with placeholder entries of
        /// a dummy VFS, so that the console lands on descriptors >= 3 when it
        /// re-opens its backing devices.
        ///
        /// Returns the ID of the dummy VFS, to be passed to
        /// [`release_low_fds`], or `None` if the descriptors could not be
        /// claimed.
        pub(in crate::stdio) unsafe fn claim_low_fds() -> Option<esp_vfs_id_t> {
            let mut vfs_id: esp_vfs_id_t = -1;
            let vfs = core::mem::zeroed::<esp_vfs_t>();

            if esp_vfs_register_with_id(&vfs, ptr::null_mut(), &mut vfs_id) != ESP_OK {
                return None;
            }

            for _ in 0..3 {
                let mut placeholder: c_int = -1;
                if esp_vfs_register_fd(vfs_id, &mut placeholder) != ESP_OK {
                    break;
                }
            }

            Some(vfs_id)
        }

        /// Releases the placeholder descriptors claimed by [`claim_low_fds`]
        pub(in crate::stdio) unsafe fn release_low_fds(vfs_id: esp_vfs_id_t) {
            // (also releases the placeholder descriptors)
            esp_vfs_unregister_with_id(vfs_id);
        }

        #[cfg(esp_idf_version_at_least_5_5_0)]
        pub(in crate::stdio) unsafe fn init_global_stdio() {
            esp_libc_init_global_stdio(CONSOLE.as_ptr());
        }

        #[cfg(not(esp_idf_version_at_least_5_5_0))]
        pub(in crate::stdio) unsafe fn init_global_stdio() {
            esp_newlib_init_global_stdio(CONSOLE.as_ptr());
        }

        /// With newlib, ESP-IDF `fopen`-s the standard streams, so the binding
        /// is restored by tearing them down and re-creating them at a point
        /// where descriptors 0, 1 and 2 are the lowest free ones.
        #[cfg(not(esp_idf_libc_picolibc))]
        pub(in crate::stdio) unsafe fn restore() -> bool {
            let Some((si, so, se)) = streams() else {
                return false;
            };

            // Close the standard streams. This drops the console refcount to
            // zero, which makes the console close the descriptors of its
            // backing devices too, so all low descriptors become free
            fclose(si);
            fclose(so);
            if se != so {
                fclose(se);
            }

            let placeholders = claim_low_fds();

            // Open the console once: this makes it re-open - and re-latch - the
            // descriptors of its backing devices, above the placeholders
            let probe = open(CONSOLE.as_ptr(), O_WRONLY as c_int);

            // Release the placeholders and re-run the stdio initialization of
            // ESP-IDF: the standard streams now claim the freed descriptors
            // 0, 1 and 2, in that order
            if let Some(vfs_id) = placeholders {
                release_low_fds(vfs_id);
            }

            init_global_stdio();

            if probe >= 0 {
                close(probe);
            }

            // Re-point the streams of the current task's reent to the
            // re-created global ones (`esp_reent_init` had copied the old, now
            // stale, pointers)
            let g = _global_impure_ptr;
            let r = __getreent();
            if !g.is_null() && !r.is_null() && r != g {
                (*r)._stdin = (*g)._stdin;
                (*r)._stdout = (*g)._stdout;
                (*r)._stderr = (*g)._stderr;
            }

            matches!(streams(), Some((_, so, _)) if fileno(so) == 1)
        }

        /// With picolibc, the standard streams are statically allocated `FILE`s
        /// owned by ESP-IDF which cannot be torn down and re-created (see the
        /// module documentation). Only their console handles are re-opened
        /// here; descriptors 0, 1 and 2 are then claimed by console handles of
        /// our own, which is all the Rust Standard Library needs.
        #[cfg(esp_idf_libc_picolibc)]
        pub(in crate::stdio) unsafe fn restore() -> bool {
            use core::sync::atomic::{AtomicU8, Ordering};

            const UNTRIED: u8 = 0;
            const FAILED: u8 = 1;
            const RESTORED: u8 = 2;

            // The restoration leaves the standard streams on descriptors >= 3,
            // so - unlike with newlib - `bound()` cannot detect a second call
            static STATE: AtomicU8 = AtomicU8::new(UNTRIED);

            match STATE.load(Ordering::Relaxed) {
                FAILED => return false,
                RESTORED => return true,
                _ => (),
            }

            let Some((si, so, _)) = streams() else {
                STATE.store(FAILED, Ordering::Relaxed);
                return false;
            };

            // `stderr` shares the stream - and thus the descriptor - of `stdout`
            let (sifd, sofd) = (fileno(si), fileno(so));

            fflush(so);

            // Close the console handles of the standard streams *without*
            // closing the streams themselves. This drops the console refcount
            // to zero, which makes the console close the descriptors of its
            // backing devices too, so all low descriptors become free
            if sifd >= 0 {
                close(sifd);
            }
            if sofd >= 0 && sofd != sifd {
                close(sofd);
            }

            let placeholders = claim_low_fds();

            // Open the console once: this makes it re-open - and re-latch - the
            // descriptors of its backing devices, above the placeholders
            let probe = open(CONSOLE.as_ptr(), O_WRONLY as c_int);

            // Re-attach the standard streams to the console *while* the
            // placeholders are still in place: all the ESP-IDF stdio
            // initialization does under picolibc is re-opening the console and
            // storing the descriptors in the (static) streams - and it asserts
            // that the `stdin` descriptor is not 0
            init_global_stdio();

            if let Some(vfs_id) = placeholders {
                release_low_fds(vfs_id);
            }

            // Descriptors 0, 1 and 2 are only needed by users of the raw POSIX
            // descriptors (the Rust Standard Library among them), so claim them
            // with console handles of their own. These are deliberately never
            // closed - they are the process' standard descriptors
            let i = open(CONSOLE.as_ptr(), O_RDONLY as c_int);
            let o = open(CONSOLE.as_ptr(), O_WRONLY as c_int);
            let e = open(CONSOLE.as_ptr(), O_WRONLY as c_int);

            if probe >= 0 {
                close(probe);
            }

            let restored = i == 0 && o == 1 && e == 2;

            if !restored {
                // Do not leak the handles which did not land where they were
                // supposed to
                for fd in [i, o, e] {
                    if fd >= 3 {
                        close(fd);
                    }
                }
            }

            // The streams were writing to closed descriptors for the duration
            // of the shuffle above, which may have latched their error flag
            clearerr(si);
            clearerr(so);

            STATE.store(if restored { RESTORED } else { FAILED }, Ordering::Relaxed);

            restored
        }
    }
}
