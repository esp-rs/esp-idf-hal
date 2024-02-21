/// Zero-copy blocking SPSC primitive for sharing a mutable reference owned by one thread into another.
/// Useful as a rendezvous point between two threads: one - sharing, and the other - using the shared mutable reference.
///
/// The using thread can wait for the shared reference in an asynchronous way as well.
///
/// Note that - strictly speaking - the priitive is MPSC in the sense that multiple threads can share (i.e. produce) mutable references.
use super::mutex::{Condvar, Mutex};

extern crate alloc;
use alloc::sync::{Arc, Weak};

use esp_idf_hal::task::asynch::Notification;

use log::info;

pub struct Receiver<T>(Weak<Channel<T>>)
where
    T: Send + 'static;

impl<T> Receiver<T>
where
    T: Send + 'static,
{
    pub fn get_shared(&mut self) -> Option<&mut T> {
        if let Some(channel) = Weak::upgrade(&self.0) {
            let mut guard = channel.state.lock();

            loop {
                match &mut *guard {
                    State::Empty => guard = channel.blocking_notify.wait(guard),
                    State::Quit => break None,
                    State::Data(data) => break unsafe { data.as_mut() },
                }
            }
        } else {
            None
        }
    }

    pub async fn get_shared_async(&mut self) -> Option<&mut T> {
        if let Some(channel) = Weak::upgrade(&self.0) {
            loop {
                {
                    let mut guard = channel.state.lock();

                    match &mut *guard {
                        State::Empty => (),
                        State::Quit => return None,
                        State::Data(data) => return unsafe { data.as_mut() },
                    }
                }

                channel.notify_full.wait().await;
            }
        } else {
            None
        }
    }

    pub fn done(&mut self) {
        if let Some(channel) = Weak::upgrade(&self.0) {
            let mut guard = channel.state.lock();

            if matches!(&*guard, State::Data(_)) {
                *guard = State::Empty;
                channel.blocking_notify.notify_all();
            }
        }
    }
}

impl<T> Drop for Receiver<T>
where
    T: Send + 'static,
{
    fn drop(&mut self) {
        if let Some(channel) = Weak::upgrade(&self.0) {
            let mut guard = channel.state.lock();

            *guard = State::Quit;
            channel.blocking_notify.notify_all();
        }
    }
}

unsafe impl<T> Send for Receiver<T> where T: Send + 'static {}

pub struct QuitOnDrop<T>(Arc<Channel<T>>)
where
    T: Send + 'static;

impl<T> QuitOnDrop<T>
where
    T: Send + 'static,
{
    pub const fn new(channel: Arc<Channel<T>>) -> Self {
        Self(channel)
    }

    pub fn channel(&self) -> &Channel<T> {
        &self.0
    }
}

impl<T> Drop for QuitOnDrop<T>
where
    T: Send + 'static,
{
    fn drop(&mut self) {
        self.0.set(State::Quit);
    }
}

unsafe impl<T> Send for QuitOnDrop<T> where T: Send + 'static {}

pub struct Channel<T>
where
    T: Send + 'static,
{
    state: Mutex<State<T>>,
    blocking_notify: Condvar,
    notify_full: Notification,
}

impl<T> Channel<T>
where
    T: Send + 'static,
{
    pub fn new() -> (Arc<Self>, Receiver<T>) {
        let this = Arc::new(Self {
            state: Mutex::new(State::Empty),
            blocking_notify: Condvar::new(),
            notify_full: Notification::new(),
        });

        let receiver = Receiver(Arc::downgrade(&this));

        (this, receiver)
    }

    pub fn share(&self, mut data: &mut T) -> bool {
        self.set(State::Data(data))
    }

    fn set(&self, data: State<T>) -> bool {
        let mut guard = self.state.lock();

        loop {
            match &*guard {
                State::Empty => {
                    self.set_and_notify(&mut guard, data);
                    break;
                }
                State::Quit => return false,
                State::Data(_) => guard = self.blocking_notify.wait(guard),
            }
        }

        loop {
            match &*guard {
                State::Empty | State::Quit => break,
                State::Data(_) => guard = self.blocking_notify.wait(guard),
            }
        }

        true
    }

    fn set_and_notify(&self, cell: &mut State<T>, data: State<T>) {
        *cell = data;
        self.blocking_notify.notify_all();
        self.notify_full.notify_lsb();
    }
}

impl<T> Drop for Channel<T>
where
    T: Send + 'static,
{
    fn drop(&mut self) {
        self.set(State::Quit);
    }
}

unsafe impl<T> Send for Channel<T> where T: Send + 'static {}
unsafe impl<T> Sync for Channel<T> where T: Send + 'static {}

#[derive(Copy, Clone, Debug)]
enum State<T> {
    Empty,
    Data(*mut T),
    Quit,
}
