pub mod notification {
    use core::future::Future;
    use core::sync::atomic::{AtomicU32, Ordering};
    use core::task::{Context, Poll, Waker};

    use atomic_waker::AtomicWaker;

    use crate::interrupt::asynch::{WakeRunner, HAL_WAKE_RUNNER};

    pub struct Notification {
        waker: AtomicWaker,
        notified: AtomicU32,
    }

    impl Notification {
        pub const fn new() -> Self {
            Self::new_with_value(0)
        }

        pub const fn new_with_value(value: u32) -> Self {
            Self {
                waker: AtomicWaker::new(),
                notified: AtomicU32::new(value),
            }
        }

        pub fn notify(&self) -> bool {
            self.signal(1)
        }

        pub fn signal(&self, value: u32) -> bool {
            if let Some(waker) = self.signal_waker(value) {
                waker.wake();

                true
            } else {
                false
            }
        }

        pub fn signal_waker(&self, value: u32) -> Option<Waker> {
            if value != 0 {
                self.notified.fetch_or(value, Ordering::SeqCst);

                self.waker.take()
            } else {
                None
            }
        }

        pub fn clear(&self) {
            self.waker.take();
            self.notified.store(0, Ordering::SeqCst);
        }

        #[allow(unused)]
        pub fn wait(&self) -> impl Future<Output = u32> + '_ {
            core::future::poll_fn(move |cx| self.poll_wait(cx))
        }

        pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<u32> {
            self.waker.register(cx.waker());

            let value = self.notified.swap(0, Ordering::SeqCst);

            if value != 0 {
                Poll::Ready(value)
            } else {
                Poll::Pending
            }
        }
    }

    impl Drop for Notification {
        fn drop(&mut self) {
            self.clear();
        }
    }

    pub struct IsrNotification<'a, const N: usize> {
        inner: Notification,
        runner: &'a WakeRunner<N>,
    }

    impl<'a, const N: usize> IsrNotification<'a, N> {
        pub const fn new(runner: &'a WakeRunner<N>) -> Self {
            Self {
                inner: Notification::new(),
                runner,
            }
        }

        pub const fn new_with_value(value: u32, runner: &'a WakeRunner<N>) -> Self {
            Self {
                inner: Notification::new_with_value(value),
                runner,
            }
        }

        pub fn notify(&self) -> bool {
            self.signal(1)
        }

        pub fn signal(&self, value: u32) -> bool {
            if let Some(waker) = self.inner.signal_waker(value) {
                self.runner.schedule(waker);

                true
            } else {
                false
            }
        }

        pub fn clear(&self) {
            self.inner.clear();
        }

        #[allow(unused)]
        pub fn wait(&self) -> impl Future<Output = u32> + '_ {
            self.runner.start().unwrap();

            self.inner.wait()
        }

        pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<u32> {
            self.inner.poll_wait(cx)
        }
    }

    pub struct HalIsrNotification {
        inner: Notification,
    }

    impl HalIsrNotification {
        pub const fn new() -> Self {
            Self {
                inner: Notification::new(),
            }
        }

        pub const fn new_with_value(value: u32) -> Self {
            Self {
                inner: Notification::new_with_value(value),
            }
        }

        pub fn notify(&self) -> bool {
            self.signal(1)
        }

        pub fn signal(&self, value: u32) -> bool {
            if let Some(waker) = self.inner.signal_waker(value) {
                HAL_WAKE_RUNNER.schedule(waker);

                true
            } else {
                false
            }
        }

        pub fn clear(&self) {
            self.inner.clear();
        }

        #[allow(unused)]
        pub fn wait(&self) -> impl Future<Output = u32> + '_ {
            HAL_WAKE_RUNNER.start().unwrap();

            self.inner.wait()
        }

        pub fn poll_wait(&self, cx: &Context<'_>) -> Poll<u32> {
            self.inner.poll_wait(cx)
        }
    }
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
