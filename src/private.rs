pub mod notification {
    // use core::cell::UnsafeCell;
    use core::future::Future;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::{Context, Poll};

    use atomic_waker::AtomicWaker;

    // use crate::interrupt::IsrCriticalSection;

    // /// Utility struct to register and wake a waker.
    // #[derive(Debug, Default)]
    // pub struct WakerRegistration {
    //     waker: Option<Waker>,
    // }

    // impl WakerRegistration {
    //     /// Create a new `WakerRegistration`.
    //     pub const fn new() -> Self {
    //         Self { waker: None }
    //     }

    //     /// Register a waker. Overwrites the previous waker, if any.
    //     pub fn register(&mut self, w: &Waker) {
    //         match self.waker {
    //             // Optimization: If both the old and new Wakers wake the same task, we can simply
    //             // keep the old waker, skipping the clone. (In most executor implementations,
    //             // cloning a waker is somewhat expensive, comparable to cloning an Arc).
    //             Some(ref w2) if (w2.will_wake(w)) => {}
    //             _ => {
    //                 // clone the new waker and store it
    //                 if let Some(old_waker) = core::mem::replace(&mut self.waker, Some(w.clone())) {
    //                     // We had a waker registered for another task. Wake it, so the other task can
    //                     // reregister itself if it's still interested.
    //                     //
    //                     // If two tasks are waiting on the same thing concurrently, this will cause them
    //                     // to wake each other in a loop fighting over this WakerRegistration. This wastes
    //                     // CPU but things will still work.
    //                     //
    //                     // If the user wants to have two tasks waiting on the same thing they should use
    //                     // a more appropriate primitive that can store multiple wakers.
    //                     old_waker.wake()
    //                 }
    //             }
    //         }
    //     }

    //     /// Wake the registered waker, if any.
    //     pub fn wake(&mut self) {
    //         if let Some(w) = self.waker.take() {
    //             w.wake()
    //         }
    //     }

    //     /// Returns true if a waker is currently registered
    //     pub fn occupied(&self) -> bool {
    //         self.waker.is_some()
    //     }
    // }

    // /// Utility struct to register and wake multiple wakers.
    // pub struct MultiWakerRegistration<const N: usize> {
    //     wakers: [WakerRegistration; N],
    // }

    // impl<const N: usize> MultiWakerRegistration<N> {
    //     /// Create a new empty instance
    //     pub const fn new() -> Self {
    //         const WAKER: WakerRegistration = WakerRegistration::new();
    //         Self { wakers: [WAKER; N] }
    //     }

    //     /// Register a waker. If the buffer is full the function returns it in the error
    //     pub fn register<'a>(&mut self, w: &'a Waker) -> Result<(), &'a Waker> {
    //         if let Some(waker_slot) = self
    //             .wakers
    //             .iter_mut()
    //             .find(|waker_slot| !waker_slot.occupied())
    //         {
    //             waker_slot.register(w);
    //             Ok(())
    //         } else {
    //             Err(w)
    //         }
    //     }

    //     /// Wake all registered wakers. This clears the buffer
    //     pub fn wake(&mut self) -> bool {
    //         let mut woken = false;

    //         for waker_slot in self.wakers.iter_mut() {
    //             woken = waker_slot.occupied();
    //             waker_slot.wake();
    //         }

    //         woken
    //     }
    // }

    pub struct Notification {
        waker: AtomicWaker,
        notified: AtomicBool,
    }

    impl Notification {
        pub const fn new() -> Self {
            Self {
                waker: AtomicWaker::new(),
                notified: AtomicBool::new(false),
            }
        }

        pub fn notify(&self) -> bool {
            self.notified.store(true, Ordering::SeqCst);

            if let Some(waker) = self.waker.take() {
                waker.wake();

                true
            } else {
                false
            }
        }

        pub fn clear(&self) {
            self.notified.store(false, Ordering::SeqCst);
        }

        #[allow(unused)]
        pub fn wait(&self) -> impl Future<Output = ()> + '_ {
            core::future::poll_fn(move |cx| self.poll_wait(cx))
        }

        pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<()> {
            self.waker.register(cx.waker());

            if self.notified.swap(false, Ordering::SeqCst) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    // pub struct MultiNotificationInner<const N: usize> {
    //     waker: MultiWakerRegistration<N>,
    //     notified: bool,
    // }

    // pub struct MultiNotification<const N: usize> {
    //     cs: IsrCriticalSection,
    //     inner: UnsafeCell<MultiNotificationInner<N>>,
    // }

    // impl<const N: usize> MultiNotification<N> {
    //     pub const fn new() -> Self {
    //         Self {
    //             cs: IsrCriticalSection::new(),
    //             inner: UnsafeCell::new(MultiNotificationInner {
    //                 waker: MultiWakerRegistration::new(),
    //                 notified: false,
    //             }),
    //         }
    //     }

    //     pub fn notify(&self) -> bool {
    //         let _cs = self.cs.enter();

    //         let inner = unsafe { self.inner.get().as_mut() }.unwrap();

    //         inner.notified = true;

    //         inner.waker.wake()
    //     }

    //     pub fn wait(&self) -> impl Future<Output = ()> + '_ {
    //         core::future::poll_fn(move |cx| self.poll_wait(cx))
    //     }

    //     pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<()> {
    //         let _cs = self.cs.enter();

    //         let inner = unsafe { self.inner.get().as_mut() }.unwrap();

    //         inner.waker.register(cx.waker()).unwrap();

    //         if inner.notified {
    //             Poll::Ready(())
    //         } else {
    //             Poll::Pending
    //         }
    //     }
    // }

    // unsafe impl<const N: usize> Send for MultiNotification<N> {}
    // unsafe impl<const N: usize> Sync for MultiNotification<N> {}
}

pub mod completion {
    use core::future::Future;

    pub async fn with_completion<F, D>(fut: F, dtor: D) -> F::Output
    where
        F: Future,
        D: FnMut(bool),
    {
        struct Completion<D>
        where
            D: FnMut(bool),
        {
            dtor: D,
            completed: bool,
        }

        impl<D> Drop for Completion<D>
        where
            D: FnMut(bool),
        {
            fn drop(&mut self) {
                (self.dtor)(self.completed);
            }
        }

        let mut completion = Completion {
            dtor,
            completed: false,
        };

        let result = fut.await;

        completion.completed = true;

        result
    }
}
