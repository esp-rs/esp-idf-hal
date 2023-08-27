pub mod notification {
    use core::future::Future;
    use core::sync::atomic::{AtomicU32, Ordering};
    use core::task::{Context, Poll};

    use atomic_waker::AtomicWaker;

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
            if value != 0 {
                self.notified.fetch_or(value, Ordering::SeqCst);

                if let Some(waker) = self.waker.take() {
                    waker.wake();

                    true
                } else {
                    false
                }
            } else {
                false
            }
        }

        pub fn clear(&self) {
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
