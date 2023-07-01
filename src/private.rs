pub mod notification {
    use core::future::Future;
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::{Context, Poll};

    use atomic_waker::AtomicWaker;

    pub struct Notification {
        waker: AtomicWaker,
        triggered: AtomicBool,
    }

    impl Notification {
        pub const fn new() -> Self {
            Self {
                waker: AtomicWaker::new(),
                triggered: AtomicBool::new(false),
            }
        }

        pub fn notify(&self) {
            self.triggered.store(true, Ordering::SeqCst);
            self.waker.wake();
        }

        pub fn wait(&self) -> impl Future<Output = ()> + '_ {
            core::future::poll_fn(move |cx| self.poll_wait(cx))
        }

        pub fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<()> {
            self.waker.register(cx.waker());

            if self.triggered.swap(false, Ordering::SeqCst) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }
}
