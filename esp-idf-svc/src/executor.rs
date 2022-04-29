#[cfg(all(feature = "isr-async-executor", feature = "alloc"))]
pub mod asyncs {
    use core::sync::atomic::{AtomicPtr, Ordering};
    use core::{mem, ptr};

    extern crate alloc;
    use alloc::sync::{Arc, Weak};

    use embedded_svc::utils::asyncs::executor::*;

    use esp_idf_hal::interrupt;

    pub type EspLocalExecutor<'a> = ISRExecutor<'a, TaskHandle, CurrentTaskWait, Local>;
    pub type EspSendableExecutor<'a> = ISRExecutor<'a, TaskHandle, CurrentTaskWait, Sendable>;

    pub struct CurrentTaskWait;

    impl Wait for CurrentTaskWait {
        fn wait(&self) {
            interrupt::task::wait_any_notification();
        }
    }

    pub struct TaskHandle(Arc<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>);

    impl TaskHandle {
        fn new() -> Self {
            Self(Arc::new(AtomicPtr::new(ptr::null_mut())))
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
            let current_task = interrupt::task::current().unwrap();
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
                        interrupt::task::notify(freertos_task, 1);
                    }
                }
            }
        }
    }

    pub fn local<'a>(max_tasks: usize) -> EspLocalExecutor<'a> {
        ISRExecutor::<_, _, Local>::new(max_tasks, TaskHandle::new(), CurrentTaskWait)
    }

    pub fn sendable<'a>(max_tasks: usize) -> EspSendableExecutor<'a> {
        ISRExecutor::<_, _, Sendable>::new(max_tasks, TaskHandle::new(), CurrentTaskWait)
    }
}
