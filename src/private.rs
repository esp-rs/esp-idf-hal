pub mod notification {
    use core::cell::UnsafeCell;
    use core::future::Future;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::{Context, Poll, Waker};

    use atomic_waker::AtomicWaker;

    use embassy_sync::waitqueue::WakerRegistration;

    use crate::interrupt::IsrCriticalSection;

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

        pub fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<()> {
            self.waker.register(cx.waker());

            if self.notified.swap(false, Ordering::SeqCst) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub struct MultiNotificationInner<const N: usize> {
        waker: MultiWakerRegistration<N>,
        notified: bool,
    }

    pub struct MultiNotification<const N: usize> {
        cs: IsrCriticalSection,
        inner: UnsafeCell<MultiNotificationInner<N>>,
    }

    impl<const N: usize> MultiNotification<N> {
        pub const fn new() -> Self {
            Self {
                cs: IsrCriticalSection::new(),
                inner: UnsafeCell::new(MultiNotificationInner {
                    waker: MultiWakerRegistration::new(),
                    notified: false,
                }),
            }
        }

        pub fn notify(&self) -> bool {
            let _cs = self.cs.enter();

            let inner = unsafe { self.inner.get().as_mut() }.unwrap();

            inner.notified = true;

            inner.waker.wake()
        }

        pub fn wait(&self) -> impl Future<Output = ()> + '_ {
            core::future::poll_fn(move |cx| self.poll_wait(cx))
        }

        pub fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<()> {
            let _cs = self.cs.enter();

            let inner = unsafe { self.inner.get().as_mut() }.unwrap();

            inner.waker.register(cx.waker()).unwrap();

            if inner.notified {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    unsafe impl<const N: usize> Send for MultiNotification<N> {}
    unsafe impl<const N: usize> Sync for MultiNotification<N> {}

    /// Utility struct to register and wake multiple wakers.
    pub struct MultiWakerRegistration<const N: usize> {
        wakers: [WakerRegistration; N],
    }

    impl<const N: usize> MultiWakerRegistration<N> {
        /// Create a new empty instance
        pub const fn new() -> Self {
            const WAKER: WakerRegistration = WakerRegistration::new();
            Self { wakers: [WAKER; N] }
        }

        /// Register a waker. If the buffer is full the function returns it in the error
        pub fn register<'a>(&mut self, w: &'a Waker) -> Result<(), &'a Waker> {
            if let Some(waker_slot) = self
                .wakers
                .iter_mut()
                .find(|waker_slot| !waker_slot.occupied())
            {
                waker_slot.register(w);
                Ok(())
            } else {
                Err(w)
            }
        }

        /// Wake all registered wakers. This clears the buffer
        pub fn wake(&mut self) -> bool {
            let mut woken = false;

            for waker_slot in self.wakers.iter_mut() {
                woken = waker_slot.occupied();
                waker_slot.wake();
            }

            woken
        }
    }
}

pub mod completion {
    use core::{
        future::Future,
        pin::Pin,
        task::{Context, Poll},
    };

    pub struct Completion<F, D>
    where
        D: FnMut(bool),
    {
        fut: F,
        destr: D,
        completed: bool,
    }

    impl<F, D> Completion<F, D>
    where
        D: FnMut(bool),
    {
        pub const fn new(fut: F, destr: D) -> Self {
            Self {
                fut,
                destr,
                completed: false,
            }
        }
    }

    impl<F, D> Future for Completion<F, D>
    where
        F: Future,
        D: FnMut(bool),
    {
        type Output = F::Output;

        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            let this = unsafe { self.get_unchecked_mut() };

            let fut = unsafe { Pin::new_unchecked(&mut this.fut) };

            let poll = fut.poll(cx);

            this.completed |= matches!(&poll, Poll::Ready(_));

            poll
        }
    }

    impl<F, D> Drop for Completion<F, D>
    where
        D: FnMut(bool),
    {
        fn drop(&mut self) {
            (self.destr)(self.completed);
        }
    }
}
