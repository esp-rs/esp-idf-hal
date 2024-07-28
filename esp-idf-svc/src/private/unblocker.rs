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
}

impl<T> Unblocker<T>
where
    T: Send + 'static,
{
    /// Create a new Unblocker
    ///
    /// # Arguments
    ///
    /// * `task_name` - The name of the task.
    /// * `stack_size` - The size of the stack for the task, in bytes.
    /// * `priority` - The priority of the task.
    /// * `pin_to_core` - The core to pin the task to, if any.
    /// * `worker` - The function to run in the task. A channel will be passed to the worker with all events.
    ///              The channel will be closed when the Unblocker is dropped, at which point the worker should exit.
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

        let _task = unsafe {
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
            // Avoid memory leak if task creation fails
            Box::from_raw(worker);
        })?;

        Ok(Self { receiver })
    }

    pub async fn exec_in_out(&mut self) -> Option<&mut T> {
        self.receiver.get_shared_async().await
    }

    pub async fn do_exec(&mut self) {
        self.receiver.done();
    }

    extern "C" fn work(arg: *mut core::ffi::c_void) {
        {
            let worker: Box<Box<dyn FnOnce() + Send + 'static>> =
                unsafe { Box::from_raw(arg as *mut _) };

            worker();
        }
        unsafe {
            // FreeRTOS tasks must delete themselves. Returning from the task function
            // without deleting the task will cause a crash.
            // This function will immediately stop running. This means we have to ensure
            // that all memory is freed before calling vTaskDelete.
            // Fortunately, this function does not manage any memory at this point.
            // The scope above guarantees that all relevant variables are dropped at this point.
            // See https://www.freertos.org/implementing-a-FreeRTOS-task.html
            task::destroy(core::ptr::null_mut());
        }
    }
}

impl<T> Drop for Unblocker<T>
where
    T: Send + 'static,
{
    fn drop(&mut self) {
        // This should cause the worker task to exit
        self.receiver.done();
    }
}
