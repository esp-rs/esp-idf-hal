//! Minimal startup / runtime for ESP32-SXX RISC-V ULPs
//! Adapted from riscv-rt/src/lib.rs

#![deny(missing_docs)]

use super::sys::cpu;

#[export_name = "error: ulp_start appears more than once in the dependency graph"]
#[doc(hidden)]
pub static __ONCE__: () = ();

/// # Safety
///
/// Rust entry point (_start_rust)
/// This function is NOT supposed to be called from use code
///
/// Calls main. This function never returns.
#[link_section = ".start.rust"]
#[export_name = "_start_rust"]
pub unsafe extern "C" fn start_rust() -> ! {
    #[rustfmt::skip]
    extern "Rust" {
        // This symbol will be provided by the user
        fn main();
    }

    cpu::rescue_from_monitor();

    main();

    cpu::shutdown();
}

/// Registers saved in trap handler
#[allow(missing_docs)]
#[repr(C)]
pub struct TrapFrame {
    pub ra: usize,
    pub t0: usize,
    pub t1: usize,
    pub t2: usize,
    pub t3: usize,
    pub t4: usize,
    pub t5: usize,
    pub t6: usize,
    pub a0: usize,
    pub a1: usize,
    pub a2: usize,
    pub a3: usize,
    pub a4: usize,
    pub a5: usize,
    pub a6: usize,
    pub a7: usize,
}

/// # Safety
///
/// Trap entry point rust (_start_trap_rust)
/// This function is NOT supposed to be called from use code
///
/// `mcause` is read to determine the cause of the trap. XLEN-1 bit indicates
/// if it's an interrupt or an exception. The result is examined and ExceptionHandler
/// or one of the core interrupt handlers is called.
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust"]
pub unsafe extern "C" fn start_trap_rust(trap_frame: *const TrapFrame) {
    // use riscv::register::mcause;

    extern "C" {
        fn ExceptionHandler(trap_frame: &TrapFrame);
        #[allow(dead_code)]
        fn DefaultHandler();
    }

    // let cause = mcause::read();
    // if cause.is_exception() {
    ExceptionHandler(trap_frame.as_ref().unwrap())
    // } else {
    //     let code = cause.code();
    //     if code < __INTERRUPTS.len() {
    //         let h = &__INTERRUPTS[code];
    //         if h.reserved == 0 {
    //             DefaultHandler();
    //         } else {
    //             (h.handler)();
    //         }
    //     } else {
    //         DefaultHandler();
    //     }
    // }
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultExceptionHandler(trap_frame: &TrapFrame) -> ! {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}

#[doc(hidden)]
#[no_mangle]
#[allow(unused_variables, non_snake_case)]
pub fn DefaultInterruptHandler() {
    loop {
        // Prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        continue;
    }
}

/* Interrupts */
#[doc(hidden)]
pub enum Interrupt {
    UserSoft,
    SupervisorSoft,
    MachineSoft,
    UserTimer,
    SupervisorTimer,
    MachineTimer,
    UserExternal,
    SupervisorExternal,
    MachineExternal,
}

pub use self::Interrupt as interrupt;

extern "C" {
    fn UserSoft();
    fn SupervisorSoft();
    fn MachineSoft();
    fn UserTimer();
    fn SupervisorTimer();
    fn MachineTimer();
    fn UserExternal();
    fn SupervisorExternal();
    fn MachineExternal();
}

#[doc(hidden)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

#[doc(hidden)]
#[allow(dead_code)]
#[no_mangle]
pub static __INTERRUPTS: [Vector; 12] = [
    Vector { handler: UserSoft },
    Vector {
        handler: SupervisorSoft,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineSoft,
    },
    Vector { handler: UserTimer },
    Vector {
        handler: SupervisorTimer,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineTimer,
    },
    Vector {
        handler: UserExternal,
    },
    Vector {
        handler: SupervisorExternal,
    },
    Vector { reserved: 0 },
    Vector {
        handler: MachineExternal,
    },
];
