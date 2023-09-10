use core::{
    marker::PhantomData,
    mem::{size_of, MaybeUninit},
};

use esp_idf_sys::TickType_t;

use crate::sys;

pub struct Queue<'a, T> {
    ptr: sys::QueueHandle_t,
    is_owned: bool,
    _marker: PhantomData<&'a mut T>,
}

unsafe impl<'a, T> Send for Queue<'a, T> where T: Send + Sync {}
unsafe impl<'a, T> Sync for Queue<'a, T> where T: Send + Sync {}

impl<'a, T> Queue<'a, T>
where
    // ensures the contained elements are not `Drop`
    // might be able to lift restriction in the future
    T: Copy,
{
    pub fn new(size: usize) -> Self {
        Queue {
            ptr: unsafe { sys::xQueueGenericCreate(size as u32, size_of::<T>() as u32, 0) },
            is_owned: true,
            _marker: PhantomData,
        }
    }

    /// Create a new queue which is not deleted on `Drop`, but owned by somebody else.
    ///
    /// # Safety
    ///
    /// Care must be taken that the queue is valid for the constructed
    /// lifetime.
    pub unsafe fn new_borrowed(ptr: sys::QueueHandle_t) -> Self {
        assert!(!ptr.is_null());

        Queue {
            ptr,
            is_owned: false,
            _marker: PhantomData,
        }
    }

    pub fn as_raw(&self) -> sys::QueueHandle_t {
        self.ptr
    }

    /// Copy item to back of queue, blocking for `timeout` ticks if full.
    pub fn send_back(&self, item: T, timeout: TickType_t) -> bool {
        unsafe { sys::xQueueGenericSend(self.ptr, &item as *const T as *const _, timeout, 0) == 1 }
    }

    /// Copy item to front of queue, blocking for `timeout` ticks if full.
    ///
    /// This is meant for high priority messages.
    pub fn send_front(&self, item: T, timeout: TickType_t) -> bool {
        unsafe { sys::xQueueGenericSend(self.ptr, &item as *const T as *const _, timeout, 0) == 1 }
    }

    /// Block for `timeout` until a message is available, removing it from the queue.
    pub fn recv_front(&self, timeout: TickType_t) -> Option<T> {
        let mut buf = MaybeUninit::uninit();

        unsafe {
            if sys::xQueueReceive(self.ptr, buf.as_mut_ptr() as *mut _, timeout) == 1 {
                Some(buf.assume_init())
            } else {
                None
            }
        }
    }

    /// Block for `timeout` until a message is available.
    pub fn peek_front(&self, timeout: TickType_t) -> Option<T> {
        let mut buf = MaybeUninit::uninit();

        unsafe {
            if sys::xQueuePeek(self.ptr, buf.as_mut_ptr() as *mut _, timeout) == 1 {
                Some(buf.assume_init())
            } else {
                None
            }
        }
    }
}

impl<'a, T> Drop for Queue<'a, T> {
    fn drop(&mut self) {
        if self.is_owned {
            unsafe { sys::vQueueDelete(self.ptr) }
        }
    }
}
