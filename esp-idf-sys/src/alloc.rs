use core::alloc::{GlobalAlloc, Layout};

use crate::*;

#[cfg_attr(not(feature = "std"), global_allocator)]
#[allow(dead_code)]
static HEAP: Esp32Alloc = Esp32Alloc;

#[cfg_attr(not(feature = "std"), alloc_error_handler)]
#[allow(dead_code)]
fn on_oom(_layout: Layout) -> ! {
    unsafe {
        crate::abort();
        core::hint::unreachable_unchecked();
    }
}

struct Esp32Alloc;

unsafe impl Sync for Esp32Alloc {}

unsafe impl GlobalAlloc for Esp32Alloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        heap_caps_malloc(layout.size() as u32, MALLOC_CAP_8BIT as _) as *mut _
    }

    unsafe fn realloc(&self, ptr: *mut u8, _layout: Layout, new_size: usize) -> *mut u8 {
        heap_caps_realloc(ptr as *mut _, new_size as u32, MALLOC_CAP_8BIT as _) as *mut _
    }

    unsafe fn dealloc(&self, ptr: *mut u8, _layout: Layout) {
        heap_caps_free(ptr as *mut _);
    }
}
