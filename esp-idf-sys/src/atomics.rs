// Support for ESP32-S2 and ESP32-C3
// Temporary, until ESP-IDF V4.4 is released.
// V4.4 will have the very same functions implemented here as part of
// `components/newlib/stdatomic.c`

// Single core SoC: atomics can be implemented using portENTER_CRITICAL_NESTED
// and portEXIT_CRITICAL_NESTED, which disable and enable interrupts.

use crate::*;

struct CriticalSection(i32);

impl CriticalSection {
    #[inline(always)]
    unsafe fn new() -> Self {
        Self(vPortSetInterruptMask())
    }
}

impl Drop for CriticalSection {
    #[inline(always)]
    fn drop(&mut self) {
        unsafe {
            vPortClearInterruptMask(self.0);
        }
    }
}

#[inline(always)]
unsafe fn atomic_load<T: Copy>(mem: *const T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    *mem.as_ref().unwrap()
}

#[inline(always)]
unsafe fn atomic_store<T: Copy>(mem: *mut T, val: T, _memorder: i32) {
    let _cs = CriticalSection::new();

    *mem.as_mut().unwrap() = val;
}

#[inline(always)]
unsafe fn atomic_exchange<T: Copy>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = val;

    prev
}

#[inline(always)]
unsafe fn atomic_compare_exchange<T: Copy + Eq>(mem: *mut T, expect: *mut T, desired: T, _weak: bool, _success: i32, _failure: i32) -> bool {
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
unsafe fn atomic_fetch_add<T: Copy + core::ops::Add<Output = T>>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev + val;

    prev
}

#[inline(always)]
unsafe fn atomic_fetch_sub<T: Copy + core::ops::Sub<Output = T>>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev - val;

    prev
}

#[inline(always)]
unsafe fn atomic_fetch_and<T: Copy + core::ops::BitAnd<Output = T>>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev & val;

    prev
}

#[inline(always)]
unsafe fn atomic_fetch_or<T: Copy + core::ops::BitOr<Output = T>>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev | val;

    prev
}

#[inline(always)]
unsafe fn atomic_fetch_xor<T: Copy + core::ops::BitXor<Output = T>>(mem: *mut T, val: T, _memorder: i32) -> T {
    let _cs = CriticalSection::new();

    let mem = mem.as_mut().unwrap();

    let prev = *mem;
    *mem = prev ^ val;

    prev
}

// #[inline(always)]
// unsafe fn sync_fetch_and<T: Copy>(mem: *mut T, val: T) -> T {
//     atomic_fetch(mem, val, 5/*__ATOMIC_SEQ_CST*/)
// }

// #[inline(always)]
// unsafe fn sync_bool_compare_and_swap<T: Copy + Eq>(mem: *mut T, old_val: T, new_val: T) -> bool {
//     let cs = CriticalSection::new();

//     if *mem.as_ref().unwrap() == old_val {
//         *mem.as_ref().unwrap() = new_val;
//         true
//     } else {
//         false
//     }
// }

// #[inline(always)]
// unsafe fn sync_val_compare_and_swap<T: Copy + Eq>(mem: *mut T, old_val: T, new_val: T) -> T {
//     let _cs = CriticalSection::new();

//     let current_val = *mem.as_ref().unwrap();

//     if current_val == old_val {
//         *mem.as_mut().unwrap() = new_val;
//     }

//     current_val
// }

macro_rules! impl_atomics {
    ($t:ty: $s: ident) => {
        paste::item! {
            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_load $s>](mem: *const $t, memorder: i32) -> $t {
                atomic_load(mem, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_store $s>](mem: *mut $t, val: $t, memorder: i32) {
                atomic_store(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_exchange $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_exchange(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_compare_exchange $s>](mem: *mut $t, expect: *mut $t, desired: $t, weak: bool, success: i32, failure: i32) -> bool {
                atomic_compare_exchange(mem, expect, desired, weak, success, failure)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_fetch_add $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_add(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_fetch_sub $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_sub(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_fetch_and $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_and(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_fetch_or $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_or(mem, val, memorder)
            }

            #[no_mangle]
            pub unsafe extern "C" fn [<__atomic_fetch_xor $s>](mem: *mut $t, val: $t, memorder: i32) -> $t {
                atomic_fetch_xor(mem, val, memorder)
            }

            // #[no_mangle]
            // pub unsafe extern "C" fn [<__sync_fetch_and $s>](mem: *mut $t, val: $t) -> $t {
            //     sync_fetch_and(mem, val)
            // }

            // #[no_mangle]
            // pub unsafe extern "C" fn [<__sync_bool_compare_and_swap $s>](mem: *mut $t, old_val: $t, new_val: $t) -> bool {
            //     sync_bool_compare_and_swap(mem, old_val, new_val)
            // }

            // #[no_mangle]
            // pub unsafe extern "C" fn [<__sync_val_compare_and_swap $s>](mem: *mut $t, old_val: $t, new_val: $t) -> $t {
            //     sync_val_compare_and_swap(mem, old_val, new_val)
            // }
        }
    };
}

impl_atomics!(u8: _1);
impl_atomics!(u16: _2);
impl_atomics!(u32: _4);
