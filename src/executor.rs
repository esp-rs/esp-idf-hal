use core::sync::atomic::{AtomicPtr, Ordering};
use core::{mem, ptr};

extern crate alloc;
use alloc::sync::{Arc, Weak};

use crate::task;

pub use edge_executor::*;

pub type EspExecutor<'a, const C: usize, S> = Executor<'a, C, TaskHandle, CurrentTaskWait, S>;
pub type EspBlocker = Blocker<TaskHandle, CurrentTaskWait>;

pub struct CurrentTaskWait(());

impl CurrentTaskWait {
    pub const fn new() -> Self {
        Self(())
    }
}

impl Default for CurrentTaskWait {
    fn default() -> Self {
        Self::new()
    }
}

impl Wait for CurrentTaskWait {
    fn wait(&self) {
        task::wait_any_notification();
    }
}

pub struct TaskHandle(Arc<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>);

impl TaskHandle {
    pub fn new() -> Self {
        Self(Arc::new(AtomicPtr::new(ptr::null_mut())))
    }
}

impl Default for TaskHandle {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for TaskHandle {
    fn drop(&mut self) {
        let mut arc = mem::replace(&mut self.0, Arc::new(AtomicPtr::new(ptr::null_mut())));

        // Busy loop until we can destroy the Arc - which means that nobody is actively holding a strong reference to it
        // and thus trying to notify our FreeRtos task, which will likely be destroyed afterwards
        loop {
            arc = match Arc::try_unwrap(arc) {
                Ok(_) => break,
                Err(a) => a,
            }
        }
    }
}

impl NotifyFactory for TaskHandle {
    type Notify = SharedTaskHandle;

    fn notifier(&self) -> Self::Notify {
        SharedTaskHandle(Arc::downgrade(&self.0))
    }
}

impl RunContextFactory for TaskHandle {
    fn prerun(&self) {
        let current_task = task::current().unwrap();
        let stored_task = self.0.load(Ordering::SeqCst);

        if stored_task.is_null() {
            self.0.store(current_task, Ordering::SeqCst);
        } else if stored_task != current_task {
            panic!("Cannot call prerun() twice from two diffeent threads");
        }
    }
}

pub struct SharedTaskHandle(Weak<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>);

impl Notify for SharedTaskHandle {
    fn notify(&self) {
        if let Some(notify) = self.0.upgrade() {
            let freertos_task = notify.load(Ordering::SeqCst);

            if !freertos_task.is_null() {
                unsafe {
                    task::notify(freertos_task, 1);
                }
            }
        }
    }
}
