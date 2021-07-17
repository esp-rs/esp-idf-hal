// Support for ESP32-C3
// Temporary, until ESP-IDF V4.4 is released.
// V4.4 will have the very same functions currently implemented here as part of
// `components/newlib/stdatomic.c`

// Single core SoC: atomics can be implemented using portENTER_CRITICAL_NESTED
// and portEXIT_CRITICAL_NESTED, which disable and enable interrupts.

// TODO: Figure out how to express the IRAM_ATTR attribute in Rust, so that the functions are placed in IRAM

use crate::*;

struct CriticalSection(i32);

impl CriticalSection {
    #[inline(always)]
    #[link_section = ".rwtext"]
    unsafe fn new() -> Self {
        Self(vPortSetInterruptMask())
    }
}

impl Drop for CriticalSection {
    #[inline(always)]
    #[link_section = ".rwtext"]
    fn drop(&mut self) {
        unsafe {
            vPortClearInterruptMask(self.0);
        }
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_load<T: Copy>(mem: *const T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    *mem.as_ref().unwrap()
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_store<T: Copy>(mem: *mut T, val: T, _memorder: i32) {
    let _cs = CriticalSection::new();

    *mem.as_mut().unwrap() = val;
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_exchange<T: Copy>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = val;

    prev
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_compare_exchange<T: Copy + Eq>(
    mem: *mut T,
    expect: *mut T,
    desired: T,
    _weak: bool,
    _success: i32,
    _failure: i32,
) -> bool {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();
    let expect = expect.as_mut().unwrap();

    if *mem == *expect {
        *mem = desired;
        true
    } else {
        *expect = *mem;
        false
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_fetch_add<T: Copy + core::ops::Add<Output = T>>(
    mem: *mut T,
    val: T,
    _memorder: i32,
) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev + val;

    prev
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_fetch_sub<T: Copy + core::ops::Sub<Output = T>>(
    mem: *mut T,
    val: T,
    _memorder: i32,
) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev - val;

    prev
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_fetch_and<T: Copy + core::ops::BitAnd<Output = T>>(
    mem: *mut T,
    val: T,
    _memorder: i32,
) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev & val;

    prev
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_fetch_or<T: Copy + core::ops::BitOr<Output = T>>(
    mem: *mut T,
    val: T,
    _memorder: i32,
) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev | val;

    prev
}

#[inline(always)]
#[link_section = ".rwtext"]
unsafe fn atomic_fetch_xor<T: Copy + core::ops::BitXor<Output = T>>(
    mem: *mut T,
    val: T,
    _memorder: i32,
) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev ^ val;

    prev
}

macro_rules! impl_atomics {
    ($t:ty: $s: ident) => {
        paste::item! {
            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_load $s>](mem: *const $t, memorder: i32) -> $t {
                atomic_load(mem, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_store $s>](mem: *mut $t, val: $t, memorder: i32) {
                atomic_store(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_exchange $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_exchange(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_compare_exchange $s>](mem: *mut $t, expect: *mut $t, desired: $t, weak: bool, success: i32, failure: i32) -> bool {
                atomic_compare_exchange(mem, expect, desired, weak, success, failure)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_fetch_add $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_add(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_fetch_sub $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_sub(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_fetch_and $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_and(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_fetch_or $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_or(mem, val, memorder)
            }

            #[no_mangle]
            #[inline(never)]
            #[link_section = ".rwtext"]
            pub unsafe extern "C" fn [<__atomic_fetch_xor $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_xor(mem, val, memorder)
            }
        }
    };
}

impl_atomics!(u8: _1);
impl_atomics!(u16: _2);
impl_atomics!(u32: _4);
