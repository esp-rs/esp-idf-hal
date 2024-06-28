use core::ffi::CStr;
use core::marker::PhantomData;

use alloc::boxed::Box;
use alloc::sync::Arc;

use esp_idf_hal::cpu::Core;
use esp_idf_hal::sys::{EspError, TaskHandle_t};
use esp_idf_hal::task;
use esp_idf_hal::task::asynch::Notification;

use super::mutex::Mutex;
use super::zerocopy::{Channel, Receiver};

pub struct Unblocker<T>
where
    T: Send + 'static,
{
    receiver: Receiver<T>,
    task: TaskHandle_t,
}

impl<T> Unblocker<T>
where
    T: Send + 'static,
{
    pub fn new<F>(
        task_name: &'static CStr,
        stack_size: usize,
        priority: Option<u8>,
        pin_to_core: Option<Core>,
        mut worker: F,
    ) -> Result<Self, EspError>
    where
        F: FnOnce(Arc<Channel<T>>) + Send + 'static,
        T: Send + 'static,
    {
        let (channel, receiver) = Channel::new();

        let worker: Box<Box<dyn FnOnce() + Send + 'static>> = Box::new(Box::new(move || {
            worker(channel);
        }));

        let worker: *mut Box<dyn FnOnce() + Send + 'static> = Box::into_raw(worker);

        let task = unsafe {
            task::create(
                Self::work,
                task_name,
                stack_size,
                worker as *mut _,
                priority.unwrap_or(6),
                pin_to_core,
            )
        }
        .inspect_err(|_| unsafe {
            Box::from_raw(worker);
        })?;

        Ok(Self { receiver, task })
    }

    pub async fn exec_in_out(&mut self) -> Option<&mut T> {
        self.receiver.get_shared_async().await
    }

    pub async fn do_exec(&mut self) {
        self.receiver.done();
    }

    extern "C" fn work(arg: *mut core::ffi::c_void) {
        let worker: Box<Box<dyn FnOnce() + Send + 'static>> =
            unsafe { Box::from_raw(arg as *mut _) };

        worker();
    }
}

impl<T> Drop for Unblocker<T>
where
    T: Send + 'static,
{
    fn drop(&mut self) {
        self.receiver.done();

        unsafe {
            task::destroy(self.task);
        }
    }
}
