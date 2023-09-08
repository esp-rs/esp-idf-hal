use core::ptr;
use core::time::Duration;

extern crate alloc;
use alloc::boxed::Box;
use alloc::ffi::CString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use embedded_svc::event_bus::{ErrorType, EventBus, Postbox, PostboxProvider};

use crate::hal::cpu::Core;
use crate::hal::task;

use crate::sys::*;

use crate::handle::RawHandle;
use crate::private::mutex::Mutex;

pub use asyncify::*;

#[allow(clippy::type_complexity)]
pub struct EspSubscriptionsRegistry {
    next_subscription_id: Mutex<usize>,
    subscriptions: Mutex<
        Vec<(
            usize,
            Arc<Mutex<Box<dyn for<'a> FnMut(&'a u32) + Send + 'static>>>,
        )>,
    >,
}

unsafe impl Send for EspSubscriptionsRegistry {}
unsafe impl Sync for EspSubscriptionsRegistry {}

impl EspSubscriptionsRegistry {
    fn new() -> Self {
        Self {
            next_subscription_id: Mutex::new(0),
            subscriptions: Mutex::new(Vec::new()),
        }
    }

    fn subscribe(
        &self,
        callback: impl for<'a> FnMut(&'a u32) + Send + 'static,
    ) -> Result<usize, EspError> {
        let subscription_id = {
            let mut guard = self.next_subscription_id.lock();

            let current = *guard;

            *guard = current + 1;

            current
        };

        self.subscriptions
            .lock()
            .push((subscription_id, Arc::new(Mutex::new(Box::new(callback)))));

        Ok(subscription_id)
    }

    fn notify(&self, notification: u32) {
        let max_id = self
            .subscriptions
            .lock()
            .iter()
            .map(|(subscription_id, _)| *subscription_id)
            .max_by_key(|s| *s);

        if let Some(max_id) = max_id {
            let mut prev_id = None;

            loop {
                let next = self
                    .subscriptions
                    .lock()
                    .iter()
                    .find(|(subscription_id, _)| {
                        *subscription_id <= max_id
                            && prev_id
                                .map(|prev_id| prev_id < *subscription_id)
                                .unwrap_or(true)
                    })
                    .map(|(subscription_id, f)| (*subscription_id, f.clone()));

                if let Some((subscription_id, f)) = next {
                    f.lock()(&notification);

                    prev_id = Some(subscription_id);
                } else {
                    break;
                }
            }
        }
    }
}

pub struct EspSubscription {
    subscription_id: usize,
    state: Arc<EspSubscriptionsRegistry>,
}

impl Drop for EspSubscription {
    fn drop(&mut self) {
        self.state
            .subscriptions
            .lock()
            .retain(|(subscription_id, _)| *subscription_id != self.subscription_id);
    }
}

unsafe impl Send for EspSubscription {}

#[derive(Debug)]
pub struct Configuration<'a> {
    pub task_name: &'a str,
    pub task_priority: u8,
    pub task_stack_size: usize,
    pub task_pin_to_core: Option<Core>,
}

impl<'a> Default for Configuration<'a> {
    fn default() -> Self {
        Self {
            task_name: "Notify",
            task_priority: 0,
            task_stack_size: 3072,
            task_pin_to_core: None,
        }
    }
}

pub struct EspNotify {
    task: Arc<TaskHandle_t>,
    registry: Arc<EspSubscriptionsRegistry>,
}

impl EspNotify {
    pub fn new(conf: &Configuration<'_>) -> Result<Self, EspError> {
        let registry = Arc::new(EspSubscriptionsRegistry::new());
        let registry_weak_ptr = Arc::downgrade(&registry).into_raw();

        let task_name = CString::new(conf.task_name).unwrap();

        let result = unsafe {
            task::create(
                Self::background_loop,
                task_name.as_c_str(),
                conf.task_stack_size,
                registry_weak_ptr as *const _ as *mut _,
                conf.task_priority,
                conf.task_pin_to_core,
            )
        };

        #[allow(clippy::all)]
        {
            if let Ok(task) = result {
                Ok(Self {
                    task: Arc::new(task),
                    registry,
                })
            } else {
                unsafe { Weak::from_raw(registry_weak_ptr) };

                Err(EspError::from_infallible::<ESP_FAIL>())
            }
        }
    }

    extern "C" fn background_loop(registry: *mut core::ffi::c_void) {
        let registry: *const EspSubscriptionsRegistry = registry as *const _;
        let registry = unsafe { Weak::from_raw(registry) };

        loop {
            let notification = task::wait_notification(Some(Duration::from_millis(100)));

            if let Some(registry) = Weak::upgrade(&registry) {
                if let Some(notification) = notification {
                    registry.notify(notification);
                }
            } else {
                break;
            }
        }

        unsafe {
            task::destroy(ptr::null_mut());
        }
    }

    pub fn subscribe(
        &self,
        callback: impl for<'a> FnMut(&'a u32) + Send + 'static,
    ) -> Result<EspSubscription, EspError> {
        self.registry
            .subscribe(callback)
            .map(|subscription_id| EspSubscription {
                subscription_id,
                state: self.registry.clone(),
            })
    }

    pub fn post(&self, payload: &u32) -> Result<bool, EspError> {
        Ok(unsafe { task::notify(*self.task, *payload) })
    }
}

unsafe impl Send for EspNotify {}

impl Clone for EspNotify {
    fn clone(&self) -> Self {
        Self {
            task: self.task.clone(),
            registry: self.registry.clone(),
        }
    }
}

impl RawHandle for EspNotify {
    type Handle = TaskHandle_t;

    fn handle(&self) -> Self::Handle {
        *self.task
    }
}

impl ErrorType for EspNotify {
    type Error = EspError;
}

impl EventBus<u32> for EspNotify {
    type Subscription = EspSubscription;

    fn subscribe(
        &self,
        callback: impl for<'a> FnMut(&'a u32) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        EspNotify::subscribe(self, callback)
    }
}

impl Postbox<u32> for EspNotify {
    fn post(&self, payload: &u32, _wait: Option<Duration>) -> Result<bool, Self::Error> {
        EspNotify::post(self, payload)
    }
}

impl PostboxProvider<u32> for EspNotify {
    type Postbox = Self;

    fn postbox(&self) -> Result<Self::Postbox, Self::Error> {
        Ok(self.clone())
    }
}

mod asyncify {
    use embedded_svc::utils::asyncify::event_bus::AsyncEventBus;
    use embedded_svc::utils::asyncify::Asyncify;

    use crate::private::mutex::RawCondvar;

    impl Asyncify for super::EspNotify {
        type AsyncWrapper<S> = AsyncEventBus<(), RawCondvar, S>;
    }
}
