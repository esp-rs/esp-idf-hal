#[cfg(all(feature = "isr-async-executor", feature = "alloc"))]
pub mod asyncs {
    use core::ptr;
    use core::sync::atomic::{AtomicPtr, Ordering};

    extern crate alloc;
    use alloc::sync::Arc;

    use embedded_svc::utils::asyncs::executor::*;

    use esp_idf_hal::interrupt;

    pub type EspLocalExecutor<'a> = LocalExecutor<'a, CurrentTaskWait, SharedTaskHandle>;
    pub type EspSendableExecutor<'a> = SendableExecutor<'a, CurrentTaskWait, SharedTaskHandle>;

    pub struct CurrentTaskWait;

    impl Wait for CurrentTaskWait {
        fn wait(&self) {
            interrupt::task::wait_any_notification();
        }
    }

    pub struct SharedTaskHandle(Arc<AtomicPtr<esp_idf_sys::tskTaskControlBlock>>);

    impl SharedTaskHandle {
        fn new() -> Self {
            Self(Arc::new(AtomicPtr::new(ptr::null_mut())))
        }
    }

    impl Notify for SharedTaskHandle {
        fn prerun(&self) {
            self.0
                .store(interrupt::task::current().unwrap(), Ordering::SeqCst);
        }

        fn notify(&self) {
            let freertos_task = self.0.load(Ordering::SeqCst);

            if !freertos_task.is_null() {
                unsafe {
                    interrupt::task::notify(freertos_task, 1);
                }
            }
        }

        fn postrun(&self) {
            self.0.store(ptr::null_mut(), Ordering::SeqCst);
        }
    }

    impl Clone for SharedTaskHandle {
        fn clone(&self) -> Self {
            Self(self.0.clone())
        }
    }

    pub fn local<'a>(max_tasks: usize) -> EspLocalExecutor<'a> {
        let task_handle = SharedTaskHandle::new();

        LocalExecutor::new(max_tasks, CurrentTaskWait, task_handle)
    }

    pub fn sendable<'a>(max_tasks: usize) -> EspSendableExecutor<'a> {
        let task_handle = SharedTaskHandle::new();

        SendableExecutor::new(max_tasks, CurrentTaskWait, task_handle)
    }
}
