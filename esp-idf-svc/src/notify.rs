use core::cell::RefCell;
use core::ptr;
use core::time::Duration;

extern crate alloc;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use embedded_svc::errors::Errors;
use embedded_svc::event_bus::{EventBus, Postbox, PostboxProvider};

use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::TickType;
use esp_idf_hal::interrupt;
use esp_idf_hal::mutex::Mutex;

use esp_idf_sys::*;

use crate::private::cstr::RawCstrs;

pub type EspBackgroundNotify = EspNotify<Background>;

#[derive(Debug)]
pub struct BackgroundNotifyConfiguration<'a> {
    pub task_name: &'a str,
    pub task_priority: u8,
    pub task_stack_size: usize,
    pub task_pin_to_core: Option<Core>,
}

impl<'a> Default for BackgroundNotifyConfiguration<'a> {
    fn default() -> Self {
        Self {
            task_name: "Notify",
            task_priority: 0,
            task_stack_size: 3072,
            task_pin_to_core: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Background(TaskHandle_t);

// TODO: Implement Explit + Pinned

unsafe impl Send for Background {}
unsafe impl Sync for Background {}

pub struct EspNotifyState {
    next_subscription_id: Mutex<usize>,
    subscriptions: Mutex<
        Vec<(
            usize,
            Arc<RefCell<dyn for<'a> FnMut(&'a u32) + Send + 'static>>,
        )>,
    >,
}

unsafe impl Send for EspNotifyState {}
unsafe impl Sync for EspNotifyState {}

impl EspNotifyState {
    fn new() -> Self {
        Self {
            next_subscription_id: Mutex::new(0),
            subscriptions: Mutex::new(Vec::new()),
        }
    }

    extern "C" fn background_loop(notify: *mut c_types::c_void) {
        let notify: *const EspNotifyState = notify as *const _;
        let notify = unsafe { Weak::from_raw(notify) };

        loop {
            let bits = Self::wait(Duration::from_millis(100));

            if let Some(notify) = Weak::upgrade(&notify) {
                if let Some(bits) = bits {
                    notify.notify_subscribers(bits);
                }
            } else {
                break;
            }
        }

        unsafe {
            vTaskDelete(ptr::null_mut());
        }
    }

    fn wait(duration: Duration) -> Option<u32> {
        let mut bits = 0_u32;

        let notified = unsafe {
            xTaskGenericNotifyWait(
                0,
                0,
                u32::MAX,
                &mut bits as *mut _,
                TickType::from(duration).0,
            )
        } != 0;

        if notified {
            Some(bits)
        } else {
            None
        }
    }

    fn notify_subscribers(&self, value: u32) {
        let max_id = self
            .subscriptions
            .lock()
            .iter()
            .map(|(subscription_id, _)| subscription_id.clone())
            .max_by_key(|s| s.clone());

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
                    .map(|(subscription_id, f)| (subscription_id.clone(), f.clone()));

                if let Some((subscription_id, f)) = next {
                    f.borrow_mut()(&value);

                    prev_id = Some(subscription_id);
                } else {
                    break;
                }
            }
        }
    }
}

pub struct EspNotify<T> {
    notify_type: T,
    state: Arc<EspNotifyState>,
}

impl EspNotify<Background> {
    pub fn new(conf: &BackgroundNotifyConfiguration<'_>) -> Result<Self, EspError> {
        let mut rcs = RawCstrs::new();

        let state = Arc::new(EspNotifyState::new());
        let state_weak_ptr = Arc::downgrade(&state).into_raw();

        let mut task: TaskHandle_t = ptr::null_mut();

        let created = unsafe {
            xTaskCreatePinnedToCore(
                Some(EspNotifyState::background_loop),
                rcs.as_ptr(conf.task_name),
                conf.task_stack_size as _,
                state_weak_ptr as *const _ as *mut _,
                conf.task_priority as _,
                &mut task as *mut _,
                conf.task_pin_to_core
                    .map(|core| core as u32)
                    .unwrap_or(tskNO_AFFINITY) as _,
            ) != 0
        };

        if created {
            Ok(Self {
                notify_type: Background(task),
                state,
            })
        } else {
            unsafe { Weak::from_raw(state_weak_ptr) };

            Err(EspError::from(ESP_FAIL).unwrap())
        }
    }
}

impl<T> Clone for EspNotify<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            notify_type: self.notify_type.clone(),
            state: self.state.clone(),
        }
    }
}

impl<T> Errors for EspNotify<T> {
    type Error = EspError;
}

impl<T> EventBus<u32> for EspNotify<T> {
    type Subscription = EspSubscription;

    fn subscribe(
        &mut self,
        callback: impl for<'a> FnMut(&'a u32) + Send + 'static,
    ) -> Result<Self::Subscription, Self::Error> {
        let subscription_id = {
            let mut guard = self.state.next_subscription_id.lock();

            let current = *guard;

            *guard = current + 1;

            current
        };

        self.state
            .subscriptions
            .lock()
            .push((subscription_id, Arc::new(RefCell::new(callback))));

        Ok(EspSubscription {
            subscription_id,
            state: self.state.clone(),
        })
    }
}

impl Postbox<u32> for EspNotify<Background> {
    fn post(&mut self, payload: &u32, _wait: Option<Duration>) -> Result<bool, Self::Error> {
        let notified = if interrupt::active() {
            let mut higher_prio_task_woken: BaseType_t = Default::default();

            let notified = unsafe {
                xTaskGenericNotifyFromISR(
                    self.notify_type.0,
                    0,
                    *payload,
                    eNotifyAction_eSetBits,
                    ptr::null_mut(),
                    &mut higher_prio_task_woken as *mut _,
                )
            };

            if higher_prio_task_woken != 0 {
                interrupt::do_yield();
            }

            notified
        } else {
            unsafe {
                xTaskGenericNotify(
                    self.notify_type.0,
                    0,
                    *payload,
                    eNotifyAction_eSetBits,
                    ptr::null_mut(),
                )
            }
        };

        Ok(notified != 0)
    }
}

impl PostboxProvider<u32> for EspNotify<Background> {
    type Postbox = Self;

    fn postbox(&mut self) -> Result<Self::Postbox, Self::Error> {
        Ok(self.clone())
    }
}

pub struct EspSubscription {
    subscription_id: usize,
    state: Arc<EspNotifyState>,
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

#[cfg(feature = "experimental")]
mod asyncify {
    use embedded_svc::utils::asyncify::event_bus::AsyncEventBus;
    use embedded_svc::utils::asyncify::Asyncify;

    use esp_idf_hal::mutex::Condvar;

    impl Asyncify for super::EspNotify<super::Background> {
        type AsyncWrapper<S> = AsyncEventBus<(), Condvar, S>;
    }
}
