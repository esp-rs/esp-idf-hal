#![cfg(all(not(feature = "std"), feature = "panic_handler"))]

#[cfg_attr(not(feature = "std"), panic_handler)]
#[allow(dead_code)]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe { crate::abort() }
}
