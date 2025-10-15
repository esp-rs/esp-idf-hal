use core::alloc::Layout;
use core::ptr::{self, NonNull};

use esp_idf_sys::*;

/// Helper struct to allocate and deallocate memory for RMT encoders.
pub(super) struct RmtEncoderAllocator;

unsafe fn rmt_aligned_alloc_encoder_mem(alignment: usize, size: usize) -> *mut core::ffi::c_void {
    // The provided rmt_alloc_encoder_mem allocates the bytes with
    // - heap_caps_calloc(1, size, RMT_MEM_ALLOC_CAPS)
    // -> heap_caps_calloc_base(1, size, RMT_MEM_ALLOC_CAPS)
    // -> heap_caps_aligned_alloc_base(UNALIGNED_MEM_ALIGNMENT_BYTES, size, RMT_MEM_ALLOC_CAPS)
    //
    // So the alignment is always UNALIGNED_MEM_ALIGNMENT_BYTES (=4)
    //
    // Rust allocator wants that the values are aligned -> we need to implement this with heap_caps_aligned_alloc

    // NOTE: This is technically part of the internal API, but there is no public API to allocate the encoder aligned.

    #[cfg(esp_idf_rmt_obj_cache_safe)]
    const RMT_MEM_ALLOC_CAPS: u32 = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    #[cfg(not(esp_idf_rmt_obj_cache_safe))]
    const RMT_MEM_ALLOC_CAPS: u32 = MALLOC_CAP_DEFAULT;

    heap_caps_aligned_calloc(alignment, 1, size, RMT_MEM_ALLOC_CAPS)
}

impl RmtEncoderAllocator {
    pub fn allocate_value<T>(&self, value: T) -> Result<NonNull<T>, EspError> {
        let layout = Layout::new::<T>();
        let memory = self.allocate(layout)?.cast::<T>();
        // SAFETY: The memory got allocated, should still be valid and is properly aligned.
        unsafe {
            ptr::write(memory.as_ptr(), value);
        }
        Ok(memory)
    }

    pub fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, EspError> {
        let ptr =
            unsafe { rmt_aligned_alloc_encoder_mem(layout.align(), layout.size()) } as *mut u8;

        let result = NonNull::new(ptr::slice_from_raw_parts_mut(ptr, layout.size()));
        if let Some(memory) = result {
            let ptr = memory.as_ptr();
            debug_assert_eq!(
                // NOTE: this is ptr::is_aligned_to impl, which is unstable
                (ptr.cast::<()>() as usize) & (layout.align() - 1),
                0,
                "ptr {ptr:p} not aligned to {}",
                layout.align()
            );

            Ok(memory)
        } else {
            Err(EspError::from_infallible::<ESP_ERR_NO_MEM>())
        }
    }

    pub unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: Layout) {
        heap_caps_free(ptr.as_ptr() as *mut core::ffi::c_void);
    }
}
