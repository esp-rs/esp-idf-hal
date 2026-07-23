//! Restoring the POSIX binding between the C standard streams and file
//! descriptors 0, 1 and 2.
//!
//! ESP-IDF creates `stdin`/`stdout`/`stderr` by `fopen`-ing the `/dev/console`
//! VFS device. Opening `/dev/console` internally opens its backing devices
//! (the primary console - usually UART - and, on chips which have one, the
//! USB-Serial-JTAG secondary console) *first*, and VFS descriptors are handed
//! out lowest-free-first. As a result, the standard streams end up on
//! descriptors 2/3/4 (or similar), while descriptors 0/1/2 point to the raw
//! backing devices.
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

/// Restores the POSIX binding between the C standard streams and file
/// descriptors 0, 1 and 2, by re-creating the standard streams in a way where
/// their underlying `/dev/console` descriptors are numbered 0, 1 and 2.
///
/// Returns `true` if the binding is in place when the function returns
/// (either because it was already in place, or because it was successfully
/// restored), and `false` otherwise, in which case the standard streams are in
/// the same state as if the function had not been called at all.
///
/// The function is idempotent. It is called automatically from the `app_main`
/// glue of the `binstart`/`libstart` features, so calling it explicitly is
/// only necessary with a custom `app_main`.
///
/// On ESP-IDF versions older than v5.3 the restoration is not attempted (the
/// function only reports whether the binding happens to be in place): the
/// console of those versions does not refcount its open/close calls, which
/// the re-creation of the standard streams relies on. For the older versions,
/// the `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y` sdkconfig setting is a build-time
/// alternative, as it results in descriptors 1 and 2 landing on the console.
///
/// NOTE: the standard streams are torn down and re-created while the function
/// runs, so it must be called *before* any other thread might be using them -
/// which is why the `app_main` glue calls it first thing.
pub fn restore_posix_stdio_fds() -> bool {
    unsafe {
        // Nothing to do if the streams are already on their POSIX descriptors:
        // - a second call,
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
    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    use core::ffi::c_int;
    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    use core::ptr;

    use crate::*;

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    const CONSOLE: &core::ffi::CStr = c"/dev/console";

    /// Returns `true` if the standard streams are bound to descriptors 0, 1
    /// and 2
    pub(super) unsafe fn bound() -> bool {
        matches!(
            streams(),
            Some((si, so, se)) if fileno(si) == 0 && fileno(so) == 1 && fileno(se) == 2
        )
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0
    ))]
    pub(super) unsafe fn restore() -> bool {
        let Some((si, so, se)) = streams() else {
            return false;
        };

        // Close the standard streams. This drops the console refcount to zero,
        // which makes the console close the descriptors of its backing devices
        // too, so all low descriptors become free
        fclose(si);
        fclose(so);
        if se != so {
            // (under picolibc, `stderr` and `stdout` share a single stream)
            fclose(se);
        }

        // Claim descriptors 0-2 with placeholder entries, so that the backing
        // devices of the console land on descriptors >= 3 when it re-opens them
        let mut vfs_id: esp_vfs_id_t = -1;
        let vfs = core::mem::zeroed::<esp_vfs_t>();
        let placeholders = esp_vfs_register_with_id(&vfs, ptr::null_mut(), &mut vfs_id) == ESP_OK;
        if placeholders {
            for _ in 0..3 {
                let mut placeholder: c_int = -1;
                if esp_vfs_register_fd(vfs_id, &mut placeholder) != ESP_OK {
                    break;
                }
            }
        }

        // Open the console once: this makes it re-open - and re-latch - the
        // descriptors of its backing devices, above the placeholders
        let probe = open(CONSOLE.as_ptr(), O_WRONLY as c_int);

        // Release the placeholders and re-run the stdio initialization of
        // ESP-IDF: the standard streams now claim the freed descriptors
        // 0, 1 and 2, in that order
        if placeholders {
            // (also releases the placeholder descriptors)
            esp_vfs_unregister_with_id(vfs_id);
        }
        init_global_stdio();
        if probe >= 0 {
            close(probe);
        }

        finish();

        matches!(streams(), Some((_, so, _)) if fileno(so) == 1)
    }

    #[cfg(not(esp_idf_libc_picolibc))]
    unsafe fn streams() -> Option<(*mut FILE, *mut FILE, *mut FILE)> {
        // The global reentrancy structure owns the standard streams; the reent
        // of every task points to the same streams, courtesy of `esp_reent_init`
        let g = _global_impure_ptr;
        if g.is_null() {
            return None;
        }

        let (si, so, se) = ((*g)._stdin, (*g)._stdout, (*g)._stderr);

        (!si.is_null() && !so.is_null() && !se.is_null()).then_some((si, so, se))
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0,
        not(esp_idf_libc_picolibc)
    ))]
    unsafe fn finish() {
        // Re-point the streams of the current task's reent to the re-created
        // global ones (`esp_reent_init` had copied the old, now stale, pointers)
        let g = _global_impure_ptr;
        let r = __getreent();
        if !g.is_null() && !r.is_null() && r != g {
            (*r)._stdin = (*g)._stdin;
            (*r)._stdout = (*g)._stdout;
            (*r)._stderr = (*g)._stderr;
        }
    }

    #[cfg(esp_idf_libc_picolibc)]
    unsafe fn streams() -> Option<(*mut FILE, *mut FILE, *mut FILE)> {
        (!stdin.is_null() && !stdout.is_null() && !stderr.is_null())
            .then_some((stdin, stdout, stderr))
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0,
        esp_idf_libc_picolibc
    ))]
    unsafe fn finish() {
        // Under picolibc `stderr` shares the stream - and thus the descriptor -
        // of `stdout`, which leaves descriptor 2 unclaimed; claim it with one
        // more console handle (deliberately never closed), so that raw writes
        // to descriptor 2 - e.g. Rust's `eprintln!` - work as well
        let fd = open(CONSOLE.as_ptr(), O_WRONLY as c_int);
        if fd >= 0 && fd != 2 {
            close(fd);
        }
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0,
        esp_idf_version_at_least_5_5_0
    ))]
    unsafe fn init_global_stdio() {
        esp_libc_init_global_stdio(CONSOLE.as_ptr());
    }

    #[cfg(all(
        esp_idf_comp_vfs_enabled,
        esp_idf_vfs_support_io,
        esp_idf_version_at_least_5_3_0,
        not(esp_idf_version_at_least_5_5_0)
    ))]
    unsafe fn init_global_stdio() {
        esp_newlib_init_global_stdio(CONSOLE.as_ptr());
    }
}
