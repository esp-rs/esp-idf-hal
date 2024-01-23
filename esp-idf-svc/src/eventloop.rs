//! Event loop library

use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use core::marker::PhantomData;
use core::result::Result;
use core::time::Duration;
use core::{ffi, mem, ptr, slice};

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};

use embedded_svc::event_bus::{self, asynch, ErrorType};
use embedded_svc::unblock::Unblocker;

use ::log::*;

use crate::hal::cpu::Core;
use crate::hal::delay::TickType;
use crate::hal::interrupt;

use crate::sys::*;

use crate::handle::RawHandle;
use crate::private::cstr::RawCstrs;
use crate::private::mutex;
use crate::private::waitable::Waitable;
use crate::private::zerocopy::{Channel, Receiver};

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub use async_wait::*;

pub type EspSystemSubscription<'a> = EspSubscription<'a, System>;
pub type EspBackgroundSubscription<'a> = EspSubscription<'a, User<Background>>;
pub type EspExplicitSubscription<'a> = EspSubscription<'a, User<Explicit>>;

pub type EspSystemAsyncSubscription<P> = EspAsyncSubscription<P, System>;
pub type EspBackgroundAsyncSubscription<P> = EspAsyncSubscription<P, User<Background>>;
pub type EspExplicitAsyncSubscription<P> = EspAsyncSubscription<P, User<Explicit>>;

pub type EspSystemEventLoop = EspEventLoop<System>;
pub type EspBackgroundEventLoop = EspEventLoop<User<Background>>;
pub type EspExplicitEventLoop = EspEventLoop<User<Explicit>>;

pub type EspSystemAsyncPostbox<U, P> = EspAsyncPostbox<U, P, EspEventLoop<System>>;
pub type EspBackgroundAsyncPostbox<U, P> = EspAsyncPostbox<U, P, EspEventLoop<User<Background>>>;
pub type EspExplicitAsyncPostbox<U, P> = EspAsyncPostbox<U, P, EspEventLoop<User<Explicit>>>;

#[derive(Debug)]
pub struct BackgroundLoopConfiguration<'a> {
    pub queue_size: usize,
    pub task_name: &'a str,
    pub task_priority: u8,
    pub task_stack_size: usize,
    pub task_pin_to_core: Core,
}

impl<'a> Default for BackgroundLoopConfiguration<'a> {
    fn default() -> Self {
        Self {
            queue_size: 64,
            task_name: "EventLoop",
            task_priority: 0,
            task_stack_size: 3072,
            task_pin_to_core: Core::Core0,
        }
    }
}

impl<'a> TryFrom<&BackgroundLoopConfiguration<'a>> for (esp_event_loop_args_t, RawCstrs) {
    type Error = EspError;

    fn try_from(conf: &BackgroundLoopConfiguration<'a>) -> Result<Self, Self::Error> {
        let mut rcs = RawCstrs::new();

        let ela = esp_event_loop_args_t {
            queue_size: conf.queue_size as _,
            task_name: rcs.as_ptr(conf.task_name)?,
            task_priority: conf.task_priority as _,
            task_stack_size: conf.task_stack_size as _,
            task_core_id: conf.task_pin_to_core as _,
        };

        Ok((ela, rcs))
    }
}

#[derive(Debug)]
pub struct ExplicitLoopConfiguration {
    pub queue_size: usize,
}

impl Default for ExplicitLoopConfiguration {
    fn default() -> Self {
        Self { queue_size: 8192 }
    }
}

impl From<&ExplicitLoopConfiguration> for esp_event_loop_args_t {
    fn from(conf: &ExplicitLoopConfiguration) -> Self {
        esp_event_loop_args_t {
            queue_size: conf.queue_size as _,
            ..Default::default()
        }
    }
}

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

#[derive(Clone, Debug)]
pub struct System;
#[derive(Clone, Debug)]
pub struct User<T>(esp_event_loop_handle_t, PhantomData<fn() -> T>);
#[derive(Clone, Debug)]
pub struct Background;
#[derive(Clone, Debug)]
pub struct Explicit;

unsafe impl Send for User<Background> {}
unsafe impl Sync for User<Background> {}

unsafe impl Send for User<Explicit> {}
unsafe impl Sync for User<Explicit> {}

pub trait EspEventLoopType {
    fn is_system() -> bool;
}

impl EspEventLoopType for System {
    fn is_system() -> bool {
        true
    }
}

impl<T> EspEventLoopType for User<T> {
    fn is_system() -> bool {
        false
    }
}

pub struct EspEventPostData {
    pub source: *const ffi::c_char,
    pub event_id: i32,
    pub payload: *const ffi::c_void,
    pub payload_len: usize,
}

impl EspEventPostData {
    /// # Safety
    ///
    /// Care should be taken to only call this function with payload reference that lives at least as long as
    /// the call that will post this data to the event loop
    pub unsafe fn new<P: Copy>(
        source: *const ffi::c_char,
        event_id: Option<i32>,
        payload: &P,
    ) -> EspEventPostData {
        Self {
            source,
            event_id: event_id.unwrap_or(0),
            payload: payload as *const _ as *const _,
            payload_len: mem::size_of::<P>(),
        }
    }

    /// # Safety
    ///
    /// Care should be taken to only call this function with payload reference that lives at least as long as
    /// the call that will post this data to the event loop
    pub unsafe fn new_raw(
        source: *const ffi::c_char,
        event_id: Option<i32>,
        payload: &[u32],
    ) -> EspEventPostData {
        Self {
            source,
            event_id: event_id.unwrap_or(0),
            payload: payload.as_ptr() as *const _,
            payload_len: payload.len(),
        }
    }
}

pub struct EspEventFetchData {
    pub source: *const ffi::c_char,
    pub event_id: i32,
    pub payload: *const ffi::c_void,
}

impl EspEventFetchData {
    /// # Safety
    ///
    /// Care should be taken to only call this function on fetch data that one is certain to be
    /// of type `P`
    pub unsafe fn as_payload<P: Copy>(&self) -> &P {
        let payload: &P = if mem::size_of::<P>() > 0 {
            self.payload as *const P
        } else {
            ptr::NonNull::dangling().as_ptr() as *const P
        }
        .as_ref()
        .unwrap();

        payload
    }

    /// # Safety
    ///
    /// Care should be taken to only call this function on fetch data that one is certain to be
    /// of length `len`
    pub unsafe fn as_raw_payload(&self, len: usize) -> &[u8] {
        slice::from_raw_parts(self.payload as *const _, len)
    }
}

struct UnsafeCallback<'a>(*mut Box<dyn FnMut(&EspEventFetchData) + Send + 'a>);

impl<'a> UnsafeCallback<'a> {
    #[allow(clippy::type_complexity)]
    fn from(boxed: &mut Box<Box<dyn FnMut(&EspEventFetchData) + Send + 'a>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut ffi::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut ffi::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: &EspEventFetchData) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

pub struct EspSubscription<'a, T>
where
    T: EspEventLoopType,
{
    event_loop_handle: Weak<EventLoopHandle<T>>,
    handler_instance: esp_event_handler_instance_t,
    source: *const ffi::c_char,
    event_id: i32,
    #[allow(clippy::type_complexity)]
    _callback: Box<Box<dyn FnMut(&EspEventFetchData) + Send + 'a>>,
}

impl<'a, T> EspSubscription<'a, T>
where
    T: EspEventLoopType,
{
    extern "C" fn handle(
        event_handler_arg: *mut ffi::c_void,
        event_base: esp_event_base_t,
        event_id: i32,
        event_data: *mut ffi::c_void,
    ) {
        let data = EspEventFetchData {
            source: event_base,
            event_id,
            payload: event_data,
        };

        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(&data);
        }
    }
}

unsafe impl<'a, T> Send for EspSubscription<'a, T> where T: EspEventLoopType {}

impl<'a, T> Drop for EspSubscription<'a, T>
where
    T: EspEventLoopType,
{
    fn drop(&mut self) {
        if T::is_system() {
            unsafe {
                esp!(esp_event_handler_instance_unregister(
                    self.source,
                    self.event_id,
                    self.handler_instance
                ))
                .unwrap();
            }
        } else if let Some(handle) = Weak::upgrade(&self.event_loop_handle) {
            unsafe {
                let handle: &T = &handle.0;
                let user: &User<Background> = mem::transmute(handle);

                esp!(esp_event_handler_instance_unregister_with(
                    user.0,
                    self.source,
                    self.event_id,
                    self.handler_instance
                ))
                .unwrap();
            }
        }
    }
}

impl<'a, T> RawHandle for EspSubscription<'a, User<T>>
where
    T: EspEventLoopType,
{
    type Handle = esp_event_handler_instance_t;

    fn handle(&self) -> Self::Handle {
        self.handler_instance
    }
}

pub struct EspAsyncSubscription<P, T>
where
    T: EspEventLoopType,
{
    receiver: Receiver<P>,
    subscription: EspSubscription<'static, T>,
}

impl<P, T> EspAsyncSubscription<P, T>
where
    P: Clone,
    T: EspEventLoopType,
{
    pub async fn recv(&mut self) -> Result<P, EspError> {
        if let Some(data) = self.receiver.get_async().await {
            Ok(data.clone())
        } else {
            Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>()) // TODO
        }
    }
}

impl<P, T> RawHandle for EspAsyncSubscription<P, User<T>>
where
    T: EspEventLoopType,
{
    type Handle = esp_event_handler_instance_t;

    fn handle(&self) -> Self::Handle {
        self.subscription.handle()
    }
}

impl<P, T> event_bus::ErrorType for EspAsyncSubscription<P, T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<P, T> asynch::Receiver for EspAsyncSubscription<P, T>
where
    P: Send + Clone,
    T: EspEventLoopType,
{
    type Data = P;

    async fn recv(&mut self) -> Result<Self::Data, Self::Error> {
        EspAsyncSubscription::recv(self).await
    }
}

#[derive(Debug)]
struct EventLoopHandle<T>(T)
where
    T: EspEventLoopType;

impl EventLoopHandle<System> {
    fn new() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            return Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>());
        }

        esp!(unsafe { esp_event_loop_create_default() })?;

        *taken = true;

        Ok(Self(System))
    }
}

impl<T> EventLoopHandle<User<T>> {
    fn new_internal(conf: &esp_event_loop_args_t) -> Result<Self, EspError> {
        let mut handle: esp_event_loop_handle_t = ptr::null_mut();

        esp!(unsafe { esp_event_loop_create(conf as *const _, &mut handle as _) })?;

        Ok(Self(User(handle, PhantomData)))
    }
}

impl EventLoopHandle<User<Background>> {
    fn new(conf: &BackgroundLoopConfiguration) -> Result<Self, EspError> {
        let (nconf, _rcs) = conf.try_into()?;

        Self::new_internal(&nconf)
    }
}

impl EventLoopHandle<User<Explicit>> {
    fn new(conf: &ExplicitLoopConfiguration) -> Result<Self, EspError> {
        Self::new_internal(&conf.into())
    }
}

impl<T> Drop for EventLoopHandle<T>
where
    T: EspEventLoopType,
{
    fn drop(&mut self) {
        if T::is_system() {
            let mut taken = TAKEN.lock();

            unsafe {
                esp!(esp_event_loop_delete_default()).unwrap();
            }

            *taken = false;

            info!("System event loop dropped");
        } else {
            unsafe {
                let handle: &T = &self.0;
                let user: &User<Background> = mem::transmute(handle);

                esp!(esp_event_loop_delete(user.0)).unwrap();
            }

            info!("Event loop dropped");
        }
    }
}

#[derive(Debug)]
pub struct EspEventLoop<T>(Arc<EventLoopHandle<T>>)
where
    T: EspEventLoopType;

impl<T> EspEventLoop<T>
where
    T: EspEventLoopType,
{
    #[allow(clippy::not_unsafe_ptr_arg_deref)]
    fn subscribe_raw<'a, F>(
        &self,
        source: *const ffi::c_char,
        event_id: i32,
        mut callback: F,
    ) -> Result<EspSubscription<'a, T>, EspError>
    where
        F: FnMut(&EspEventFetchData) + Send + 'a,
    {
        let mut handler_instance: esp_event_handler_instance_t = ptr::null_mut();

        let callback: Box<dyn FnMut(&EspEventFetchData) + Send + 'a> =
            Box::new(move |data| callback(data));
        let mut callback = Box::new(callback);

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        if T::is_system() {
            esp!(unsafe {
                esp_event_handler_instance_register(
                    source,
                    event_id,
                    Some(EspSubscription::<System>::handle),
                    unsafe_callback.as_ptr(),
                    &mut handler_instance as *mut _,
                )
            })?;
        } else {
            esp!(unsafe {
                let handle: &T = &self.0 .0;
                let user: &User<Background> = mem::transmute(handle);

                esp_event_handler_instance_register_with(
                    user.0,
                    source,
                    event_id,
                    Some(EspSubscription::<User<T>>::handle),
                    unsafe_callback.as_ptr(),
                    &mut handler_instance as *mut _,
                )
            })?;
        }

        Ok(EspSubscription {
            event_loop_handle: Arc::downgrade(&self.0),
            handler_instance,
            source,
            event_id,
            _callback: callback,
        })
    }

    pub fn post_raw(
        &self,
        data: &EspEventPostData,
        wait: Option<Duration>,
    ) -> Result<bool, EspError> {
        let result = if T::is_system() {
            unsafe {
                esp_event_post(
                    data.source,
                    data.event_id,
                    data.payload as *mut _,
                    data.payload_len as _,
                    TickType::from(wait).0,
                )
            }
        } else {
            unsafe {
                let handle: &T = &self.0 .0;
                let user: &User<Background> = mem::transmute(handle);

                esp_event_post_to(
                    user.0,
                    data.source,
                    data.event_id,
                    data.payload as *mut _,
                    data.payload_len as _,
                    TickType::from(wait).0,
                )
            }
        };

        if result == ESP_ERR_TIMEOUT {
            Ok(false)
        } else {
            esp_result!(result, true)
        }
    }

    #[cfg(esp_idf_esp_event_post_from_isr)]
    pub fn isr_post_raw(&self, data: &EspEventPostData) -> Result<bool, EspError> {
        let mut higher_prio_task_woken: BaseType_t = Default::default();

        let result = if T::is_system() {
            unsafe {
                esp_event_isr_post(
                    data.source,
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    &mut higher_prio_task_woken as *mut _,
                )
            }
        } else {
            unsafe {
                let handle: &T = &self.0 .0;
                let user: &User<Background> = mem::transmute(handle);

                esp_event_isr_post_to(
                    user.0,
                    data.source,
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    &mut higher_prio_task_woken as *mut _,
                )
            }
        };

        if higher_prio_task_woken != 0 {
            crate::hal::task::do_yield();
        }

        if result == ESP_FAIL {
            Ok(false)
        } else {
            esp!(result)?;

            Ok(true)
        }
    }

    pub fn subscribe<P, F>(&self, mut callback: F) -> Result<EspSubscription<'static, T>, EspError>
    where
        P: EspTypedEventDeserializer<P>,
        F: FnMut(&P) + Send + 'static,
    {
        self.subscribe_raw(
            P::source(),
            P::event_id().unwrap_or(ESP_EVENT_ANY_ID),
            move |raw_event| P::deserialize(raw_event, &mut callback),
        )
    }

    pub fn subscribe_async<P>(&self) -> Result<EspAsyncSubscription<P, T>, EspError>
    where
        P: EspTypedEventDeserializer<P> + Send + Clone + 'static,
    {
        let (channel, receiver) = Channel::<P>::new();

        let subscription = self.subscribe::<P, _>(move |event| {
            channel.set(event.clone());
        })?;

        Ok(EspAsyncSubscription {
            receiver,
            subscription,
        })
    }

    /// # Safety
    ///
    /// This method - in contrast to method `subscribe` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    /// scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent to a hidden ESP IDF thread.
    /// This means that if the service is forgotten, Rust is free to e.g. unwind the stack
    /// and the closure now owned by this other thread will end up with references to variables that no longer exist.
    ///
    /// The destructor of the service takes care - prior to the service being dropped and e.g.
    /// the stack being unwind - to remove the closure from the hidden thread and destroy it.
    /// Unfortunately, when the service is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    pub unsafe fn subscribe_nonstatic<'a, P, F>(
        &self,
        mut callback: F,
    ) -> Result<EspSubscription<'a, T>, EspError>
    where
        P: EspTypedEventDeserializer<P>,
        F: FnMut(&P) + Send + 'a,
    {
        self.subscribe_raw(
            P::source(),
            P::event_id().unwrap_or(ESP_EVENT_ANY_ID),
            move |raw_event| P::deserialize(raw_event, &mut callback),
        )
    }

    pub fn post<P>(&self, payload: &P, wait: Option<Duration>) -> Result<bool, EspError>
    where
        P: EspTypedEventSerializer<P>,
    {
        if interrupt::active() {
            #[cfg(not(esp_idf_esp_event_post_from_isr))]
            panic!("Trying to post from an ISR handler. Enable `CONFIG_ESP_EVENT_POST_FROM_ISR` in `sdkconfig.defaults`");

            #[cfg(esp_idf_esp_event_post_from_isr)]
            P::serialize(payload, |raw_event| self.isr_post_raw(raw_event))
        } else {
            P::serialize(payload, |raw_event| self.post_raw(raw_event, wait))
        }
    }
}

impl<T> EspEventLoop<User<T>> {
    pub fn spin(&mut self, duration: Option<Duration>) -> Result<(), EspError> {
        esp!(unsafe { esp_event_loop_run(self.0 .0 .0, TickType::from(duration).0,) })
    }
}

impl<T> RawHandle for EspEventLoop<User<T>> {
    type Handle = esp_event_loop_handle_t;

    fn handle(&self) -> Self::Handle {
        self.0 .0 .0
    }
}

impl EspEventLoop<System> {
    pub fn take() -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<System>::new()?)))
    }
}

impl EspEventLoop<User<Background>> {
    pub fn new(conf: &BackgroundLoopConfiguration) -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<User<Background>>::new(
            conf,
        )?)))
    }
}

impl EspEventLoop<User<Explicit>> {
    pub fn new(conf: &ExplicitLoopConfiguration) -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<User<Explicit>>::new(
            conf,
        )?)))
    }
}

impl<T> Clone for EspEventLoop<T>
where
    T: EspEventLoopType,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

unsafe impl<T> Send for EspEventLoop<T> where T: EspEventLoopType + Send {}
unsafe impl<T> Sync for EspEventLoop<T> where T: EspEventLoopType + Sync {}

impl<T> ErrorType for EspEventLoop<T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<T> event_bus::Spin for EspEventLoop<User<T>> {
    fn spin(&mut self, duration: Option<Duration>) -> Result<(), Self::Error> {
        EspEventLoop::spin(self, duration)
    }
}

/// # Safety
///
/// By implementing this trait, the user guarantees that the binary format serialized/deserialized by
/// `EspTypedEventSerializer` and `EspTypedEventDeserializer` is indeed THE binary format that stands behind
/// the source ID returned by the `source` method in this trait
/// (and that other producers/consumers of the ESP IDF event loop also recognize as the binary format corresponbding
/// to this source ID).
///
/// Providing the wrong source ID for a binary format, or the wrong binary format for a given source ID
/// would lead to a runtime crash, hence this trait can only be implemented unsafely, as the guarantee lies with the user
/// and not with the compiler, that cannot enforce this contract.
pub unsafe trait EspTypedEventSource {
    fn source() -> *const ffi::c_char;

    fn event_id() -> Option<i32> {
        None
    }
}

pub trait EspTypedEventSerializer<P>: EspTypedEventSource {
    fn serialize<R>(payload: &P, f: impl for<'a> FnOnce(&'a EspEventPostData) -> R) -> R;
}

pub trait EspTypedEventDeserializer<P>: EspTypedEventSource {
    fn deserialize<R>(data: &EspEventFetchData, f: &mut impl for<'a> FnMut(&'a P) -> R) -> R;
}

impl<P, T> event_bus::Postbox<P> for EspEventLoop<T>
where
    P: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, Self::Error> {
        EspEventLoop::post(self, payload, wait)
    }
}

impl<P, T> event_bus::EventBus<P> for EspEventLoop<T>
where
    P: EspTypedEventDeserializer<P>,
    T: EspEventLoopType,
{
    type Subscription<'a> = EspSubscription<'static, T> where Self: 'a;

    fn subscribe<F>(&self, callback: F) -> Result<Self::Subscription<'_>, Self::Error>
    where
        F: FnMut(&P) + Send + 'static,
    {
        EspEventLoop::subscribe(self, callback)
    }
}

impl<P, T> asynch::EventBus<P> for EspEventLoop<T>
where
    P: EspTypedEventDeserializer<P> + Send + Clone + 'static,
    T: EspEventLoopType,
{
    type Subscription<'a> = EspAsyncSubscription<P, T> where Self: 'a;

    async fn subscribe(&self) -> Result<Self::Subscription<'_>, Self::Error> {
        EspEventLoop::subscribe_async(self)
    }
}

impl<P, T> event_bus::PostboxProvider<P> for EspEventLoop<T>
where
    P: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    type Postbox<'a> = Self where Self: 'a;

    fn postbox(&self) -> Result<Self::Postbox<'_>, Self::Error>
    where
        P: EspTypedEventSerializer<P>,
    {
        Ok(self.clone())
    }
}

#[derive(Clone)]
pub struct EspAsyncPostbox<U, P, T>
where
    T: EspEventLoopType,
{
    unblocker: U,
    event_loop: EspEventLoop<T>,
    _type: PhantomData<fn(&P)>,
}

impl<U, P, T> EspAsyncPostbox<U, P, T>
where
    U: Unblocker,
    P: EspTypedEventSerializer<P> + Send + 'static,
    T: EspEventLoopType + Send,
{
    pub const fn new(event_loop: EspEventLoop<T>, unblocker: U) -> Self {
        Self {
            unblocker,
            event_loop,
            _type: PhantomData,
        }
    }

    pub async fn send(&self, value: P) -> Result<(), EspError> {
        let event_loop = self.event_loop.clone();

        self.unblocker
            .unblock(move || event_loop.post(&value, Some(Duration::MAX)).map(|_| ()))
            .await
    }
}

impl<U, P, T> ErrorType for EspAsyncPostbox<U, P, T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<U, P, T> asynch::PostboxProvider<P> for EspAsyncPostbox<U, P, T>
where
    U: Unblocker + Clone,
    P: EspTypedEventSerializer<P> + Clone + Send + 'static,
    T: EspEventLoopType + Clone + Send,
{
    type Postbox<'a> = Self where Self: 'a;

    async fn postbox(&self) -> Result<Self::Postbox<'_>, Self::Error> {
        Ok(self.clone())
    }
}

impl<U, P, T> asynch::Sender for EspAsyncPostbox<U, P, T>
where
    U: Unblocker,
    P: EspTypedEventSerializer<P> + Send + 'static,
    T: EspEventLoopType + Send,
{
    type Data = P;

    async fn send(&mut self, value: Self::Data) -> Result<(), Self::Error> {
        EspAsyncPostbox::send(self, value).await
    }
}

pub struct Wait<'a, E, T>
where
    T: EspEventLoopType,
{
    _subscription: EspSubscription<'a, T>,
    waitable: Arc<Waitable<()>>,
    _event: PhantomData<fn() -> E>,
}

impl<E, T> Wait<'static, E, T>
where
    E: EspTypedEventDeserializer<E> + Debug,
    T: EspEventLoopType,
{
    pub fn new<F: FnMut(&E) -> bool + Send + 'static>(
        event_loop: &EspEventLoop<T>,
        waiter: F,
    ) -> Result<Self, EspError> {
        Self::internal_new(event_loop, waiter)
    }
}

impl<'a, E, T> Wait<'a, E, T>
where
    E: EspTypedEventDeserializer<E> + Debug,
    T: EspEventLoopType,
{
    /// # Safety
    ///
    /// This method - in contrast to method `new` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    /// scope where the service is created.
    ///
    /// HOWEVER: care should be taken NOT to call `core::mem::forget()` on the service,
    /// as that would immediately lead to an UB (crash).
    /// Also note that forgetting the service might happen with `Rc` and `Arc`
    /// when circular references are introduced: https://github.com/rust-lang/rust/issues/24456
    ///
    /// The reason is that the closure is actually sent to a hidden ESP IDF thread.
    /// This means that if the service is forgotten, Rust is free to e.g. unwind the stack
    /// and the closure now owned by this other thread will end up with references to variables that no longer exist.
    ///
    /// The destructor of the service takes care - prior to the service being dropped and e.g.
    /// the stack being unwind - to remove the closure from the hidden thread and destroy it.
    /// Unfortunately, when the service is forgotten, the un-subscription does not happen
    /// and invalid references are left dangling.
    ///
    /// This "local borrowing" will only be possible to express in a safe way once/if `!Leak` types
    /// are introduced to Rust (i.e. the impossibility to "forget" a type and thus not call its destructor).
    pub unsafe fn new_nonstatic<F: FnMut(&E) -> bool + Send + 'a>(
        event_loop: &EspEventLoop<T>,
        waiter: F,
    ) -> Result<Self, EspError> {
        Self::internal_new(event_loop, waiter)
    }

    fn internal_new<F: FnMut(&E) -> bool + Send + 'a>(
        event_loop: &EspEventLoop<T>,
        mut waiter: F,
    ) -> Result<Self, EspError> {
        let waitable: Arc<Waitable<()>> = Arc::new(Waitable::new(()));

        let s_waitable = waitable.clone();
        let subscription = unsafe {
            event_loop.subscribe_nonstatic(move |event: &E| {
                Self::on_event(&s_waitable, event, &mut waiter)
            })?
        };

        Ok(Self {
            waitable,
            _subscription: subscription,
            _event: PhantomData,
        })
    }

    pub fn wait_while<F: FnMut() -> Result<bool, EspError>>(
        &self,
        mut matcher: F,
        duration: Option<Duration>,
    ) -> Result<(), EspError> {
        if let Some(duration) = duration {
            debug!("About to wait for duration {:?}", duration);

            let (timeout, _) =
                self.waitable
                    .wait_timeout_while_and_get(duration, |_| matcher(), |_| ())?;

            if !timeout {
                debug!("Waiting done - success");
                Ok(())
            } else {
                debug!("Timeout while waiting");
                esp!(ESP_ERR_TIMEOUT)
            }
        } else {
            debug!("About to wait");

            self.waitable.wait_while(|_| matcher())?;

            debug!("Waiting done - success");

            Ok(())
        }
    }

    fn on_event<F: FnMut(&E) -> bool>(waitable: &Waitable<()>, event: &E, waiter: &mut F) {
        debug!("Got event: {:?}", event);

        if waiter(event) {
            waitable.cvar.notify_all();
        }
    }
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
mod async_wait {
    use core::time::Duration;

    use log::debug;

    use super::{EspAsyncSubscription, EspEventLoop, EspEventLoopType, EspTypedEventDeserializer};
    use crate::sys::{esp, EspError, ESP_ERR_TIMEOUT};
    use crate::timer::{EspAsyncTimer, EspTimerService, Task};

    pub struct AsyncWait<P, T>
    where
        T: EspEventLoopType,
    {
        subscription: EspAsyncSubscription<P, T>,
        timer: EspAsyncTimer,
    }

    impl<P, T> AsyncWait<P, T>
    where
        P: EspTypedEventDeserializer<P> + Send + Clone + 'static,
        T: EspEventLoopType + Send,
    {
        pub fn new(
            event_loop: &EspEventLoop<T>,
            timer_service: &EspTimerService<Task>,
        ) -> Result<Self, EspError> {
            Ok(Self {
                subscription: event_loop.subscribe_async()?,
                timer: timer_service.timer_async()?,
            })
        }

        pub async fn wait_while<F: FnMut() -> Result<bool, EspError>>(
            &mut self,
            matcher: F,
            duration: Option<Duration>,
        ) -> Result<(), EspError> {
            if let Some(duration) = duration {
                debug!("About to wait for duration {:?}", duration);

                let timer_wait = self.timer.after(duration);
                let subscription_wait = Self::wait_sub(&mut self.subscription, matcher);

                match embassy_futures::select::select(subscription_wait, timer_wait).await {
                    embassy_futures::select::Either::First(_) => {
                        debug!("Waiting done - success");
                        Ok(())
                    }
                    embassy_futures::select::Either::Second(_) => {
                        debug!("Timeout while waiting");
                        esp!(ESP_ERR_TIMEOUT)
                    }
                }
            } else {
                debug!("About to wait");

                Self::wait_sub(&mut self.subscription, matcher).await?;

                debug!("Waiting done - success");

                Ok(())
            }
        }

        #[allow(clippy::all)]
        async fn wait_sub<PP, TT, F: FnMut() -> Result<bool, EspError>>(
            subscription: &mut EspAsyncSubscription<PP, TT>,
            mut matcher: F,
        ) -> Result<(), EspError>
        where
            PP: EspTypedEventDeserializer<P> + Send + Clone + 'static,
            TT: EspEventLoopType + Send,
        {
            while matcher()? {
                subscription.recv().await?;
            }

            Ok(())
        }
    }
}
