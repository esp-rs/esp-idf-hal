//! Event loop library

use core::fmt::Debug;
use core::marker::PhantomData;
use core::time::Duration;
use core::{ffi, mem, ptr, slice};

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};

use embedded_svc::channel;

use ::log::*;

use crate::hal::cpu::Core;
use crate::hal::delay;
use crate::hal::interrupt;

use crate::sys::*;

use crate::handle::RawHandle;
use crate::private::cstr::RawCstrs;
use crate::private::mutex;
use crate::private::waitable::Waitable;
use crate::private::zerocopy::{Channel, QuitOnDrop, Receiver};

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

#[derive(Debug)]
pub struct BackgroundLoopConfiguration<'a> {
    pub queue_size: usize,
    pub task_name: &'a str,
    pub task_priority: u8,
    pub task_stack_size: usize,
    pub task_pin_to_core: Core,
}

impl Default for BackgroundLoopConfiguration<'_> {
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
pub unsafe trait EspEventSource {
    fn source() -> Option<&'static ffi::CStr>;

    fn event_id() -> Option<i32> {
        None
    }
}

pub trait EspEventSerializer: EspEventSource {
    type Data<'a>;

    fn serialize<F, R>(data: &Self::Data<'_>, f: F) -> R
    where
        F: FnOnce(&EspEventPostData) -> R;
}

pub trait EspEventDeserializer: EspEventSource {
    type Data<'a>;

    fn deserialize<'a>(data: &EspEvent<'a>) -> Self::Data<'a>;
}

#[derive(Debug, Clone)]
pub struct EspEventPostData<'a> {
    source: &'static ffi::CStr,
    event_id: i32,
    payload: &'a ffi::c_void,
    payload_len: usize,
}

impl<'a> EspEventPostData<'a> {
    /// # Safety
    ///
    /// Care should be taken to only call this function with payload reference that lives at least as long as
    /// the call that will post this data to the event loop
    pub unsafe fn new<P: Copy + Send + 'static>(
        source: &'static ffi::CStr,
        event_id: Option<i32>,
        payload: &'a P,
    ) -> Self {
        Self {
            source,
            event_id: event_id.unwrap_or(0),
            payload: unsafe {
                (payload as *const _ as *const ffi::c_void)
                    .as_ref()
                    .unwrap()
            },
            payload_len: mem::size_of::<P>(),
        }
    }

    /// # Safety
    ///
    /// Care should be taken to only call this function with payload reference that lives at least as long as
    /// the call that will post this data to the event loop
    pub unsafe fn new_raw(
        source: &'static ffi::CStr,
        event_id: Option<i32>,
        payload: &'a [u8],
    ) -> Self {
        Self {
            source,
            event_id: event_id.unwrap_or(0),
            payload: unsafe {
                (payload.as_ptr() as *const _ as *const ffi::c_void)
                    .as_ref()
                    .unwrap()
            },
            payload_len: payload.len(),
        }
    }
}

unsafe impl EspEventSource for EspEventPostData<'_> {
    fn source() -> Option<&'static ffi::CStr> {
        None
    }
}

impl EspEventSerializer for EspEventPostData<'_> {
    type Data<'d> = EspEventPostData<'d>;

    fn serialize<F, R>(data: &Self::Data<'_>, f: F) -> R
    where
        F: FnOnce(&EspEventPostData) -> R,
    {
        f(data)
    }
}

#[derive(Debug, Clone)]
pub struct EspEvent<'a> {
    pub source: &'static ffi::CStr,
    pub event_id: i32,
    pub payload: Option<&'a ffi::c_void>,
}

impl<'a> EspEvent<'a> {
    /// # Safety
    ///
    /// Care should be taken to only call this function on fetch data that one is certain to be
    /// of type `P`
    pub unsafe fn as_payload<P: Copy + Send + 'static>(&self) -> &'a P {
        let payload: &P = if mem::size_of::<P>() > 0 {
            self.payload.unwrap() as *const _ as *const P
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
    pub unsafe fn as_raw_payload(&self, len: usize) -> Option<&[u8]> {
        self.payload
            .map(|payload| slice::from_raw_parts(payload as *const _ as *const _, len))
    }
}

unsafe impl EspEventSource for EspEvent<'_> {
    fn source() -> Option<&'static ffi::CStr> {
        None
    }
}

impl EspEventDeserializer for EspEvent<'_> {
    type Data<'d> = EspEvent<'d>;

    fn deserialize<'d>(data: &EspEvent<'d>) -> Self::Data<'d> {
        data.clone()
    }
}

struct UnsafeCallback<'a>(*mut Box<dyn FnMut(EspEvent) + Send + 'a>);

impl<'a> UnsafeCallback<'a> {
    #[allow(clippy::type_complexity)]
    fn from(boxed: &mut Box<Box<dyn FnMut(EspEvent) + Send + 'a>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut ffi::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut ffi::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: EspEvent) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

enum EventLoopHandleRef<T>
where
    T: EspEventLoopType,
{
    Strong(Arc<EventLoopHandle<T>>),
    Weak(Weak<EventLoopHandle<T>>),
}

impl<T> EventLoopHandleRef<T>
where
    T: EspEventLoopType,
{
    fn make_weak(&mut self) {
        if matches!(self, Self::Strong(_)) {
            *self = Self::Weak(Arc::downgrade(&self.upgrade().unwrap()))
        }
    }

    fn upgrade(&self) -> Option<Arc<EventLoopHandle<T>>> {
        match self {
            Self::Strong(handle) => Some(handle.clone()),
            Self::Weak(handle) => handle.upgrade(),
        }
    }
}

pub struct EspSubscription<'a, T>
where
    T: EspEventLoopType,
{
    event_loop_handle: EventLoopHandleRef<T>,
    handler_instance: esp_event_handler_instance_t,
    source: Option<&'static ffi::CStr>,
    event_id: i32,
    #[allow(clippy::type_complexity)]
    _callback: Box<Box<dyn FnMut(EspEvent) + Send + 'a>>,
}

impl<T> EspSubscription<'_, T>
where
    T: EspEventLoopType,
{
    pub fn make_weak(&mut self) {
        self.event_loop_handle.make_weak();
    }

    extern "C" fn handle(
        event_handler_arg: *mut ffi::c_void,
        event_base: esp_event_base_t,
        event_id: i32,
        event_data: *mut ffi::c_void,
    ) {
        let data = EspEvent {
            source: unsafe { ffi::CStr::from_ptr(event_base) },
            event_id,
            payload: unsafe { (event_data as *const ffi::c_void).as_ref() },
        };

        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(data);
        }
    }
}

unsafe impl<T> Send for EspSubscription<'_, T> where T: EspEventLoopType {}

impl<T> Drop for EspSubscription<'_, T>
where
    T: EspEventLoopType,
{
    fn drop(&mut self) {
        if let Some(handle) = self.event_loop_handle.upgrade() {
            if T::is_system() {
                unsafe {
                    esp!(esp_event_handler_instance_unregister(
                        self.source.map(ffi::CStr::as_ptr).unwrap_or(ptr::null()),
                        self.event_id,
                        self.handler_instance
                    ))
                    .unwrap();
                }
            } else {
                unsafe {
                    let handle: &T = &handle.0;
                    let user: &User<Background> = mem::transmute(handle);

                    esp!(esp_event_handler_instance_unregister_with(
                        user.0,
                        self.source.map(ffi::CStr::as_ptr).unwrap_or(ptr::null()),
                        self.event_id,
                        self.handler_instance
                    ))
                    .unwrap();
                }
            }
        }
    }
}

impl<T> RawHandle for EspSubscription<'_, User<T>>
where
    T: EspEventLoopType,
{
    type Handle = esp_event_handler_instance_t;

    fn handle(&self) -> Self::Handle {
        self.handler_instance
    }
}

pub struct EspAsyncSubscription<D, T>
where
    D: EspEventDeserializer,
    T: EspEventLoopType,
{
    receiver: Receiver<EspEvent<'static>>,
    subscription: EspSubscription<'static, T>,
    given: bool,
    _deserializer: PhantomData<fn() -> D>,
}

impl<D, T> EspAsyncSubscription<D, T>
where
    D: EspEventDeserializer,
    T: EspEventLoopType,
{
    pub fn make_weak(&mut self) {
        self.subscription.make_weak();
    }

    pub async fn recv(&mut self) -> Result<D::Data<'_>, EspError> {
        if self.given {
            self.receiver.done();
            self.given = false;
        }

        while let Some(data) = self.receiver.get_shared_async().await {
            if Some(data.source) != D::source() {
                self.receiver.done();
                continue;
            }
            self.given = true;
            return Ok(D::deserialize(data));
        }

        Err(EspError::from_infallible::<ESP_ERR_INVALID_STATE>())
    }
}

impl<D, T> RawHandle for EspAsyncSubscription<D, User<T>>
where
    D: EspEventDeserializer,
    T: EspEventLoopType,
{
    type Handle = esp_event_handler_instance_t;

    fn handle(&self) -> Self::Handle {
        self.subscription.handle()
    }
}

impl<D, T> channel::ErrorType for EspAsyncSubscription<D, T>
where
    D: EspEventDeserializer,
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<D, T> channel::asynch::Receiver for EspAsyncSubscription<D, T>
where
    D: EspEventDeserializer + 'static,
    T: EspEventLoopType + 'static,
{
    type Data<'a> = D::Data<'a>;

    async fn recv(&mut self) -> Result<Self::Data<'_>, Self::Error> {
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
    pub fn subscribe_async<D>(&self) -> Result<EspAsyncSubscription<D, T>, EspError>
    where
        D: EspEventDeserializer,
    {
        let (channel, receiver) = Channel::new();

        let sender = QuitOnDrop::new(channel);

        let subscription = self.subscribe::<EspEvent, _>(move |event| {
            let mut event = unsafe { mem::transmute::<EspEvent<'_>, EspEvent<'_>>(event) };

            sender.channel().share(&mut event);
        })?;

        Ok(EspAsyncSubscription {
            receiver,
            subscription,
            given: false,
            _deserializer: PhantomData,
        })
    }

    pub fn subscribe<D, F>(&self, mut callback: F) -> Result<EspSubscription<'static, T>, EspError>
    where
        D: EspEventDeserializer,
        F: for<'a> FnMut(D::Data<'a>) + Send + 'static,
    {
        self.subscribe_raw::<D, _>(move |event| callback(D::deserialize(&event)))
    }

    /// # Safety
    ///
    /// This method - in contrast to method `subscribe` - allows the user to pass
    /// a non-static callback/closure. This enables users to borrow
    /// - in the closure - variables that live on the stack - or more generally - in the same
    ///   scope where the service is created.
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
    pub unsafe fn subscribe_nonstatic<'a, D, F>(
        &self,
        mut callback: F,
    ) -> Result<EspSubscription<'a, T>, EspError>
    where
        D: EspEventDeserializer,
        F: for<'d> FnMut(D::Data<'d>) + Send + 'a,
    {
        self.subscribe_raw::<D, _>(move |event| callback(D::deserialize(&event)))
    }

    pub async fn post_async<S>(&self, payload: &S::Data<'_>) -> Result<(), EspError>
    where
        S: EspEventSerializer,
    {
        loop {
            if self.post::<S>(payload, delay::NON_BLOCK)? {
                break Ok(());
            }

            crate::hal::task::yield_now().await;
        }
    }

    pub fn post<S>(&self, payload: &S::Data<'_>, timeout: TickType_t) -> Result<bool, EspError>
    where
        S: EspEventSerializer,
    {
        if interrupt::active() {
            #[cfg(not(esp_idf_esp_event_post_from_isr))]
            panic!("Trying to post from an ISR handler. Enable `CONFIG_ESP_EVENT_POST_FROM_ISR` in `sdkconfig.defaults`");

            #[cfg(esp_idf_esp_event_post_from_isr)]
            S::serialize(payload, |event| self.isr_post_raw(event))
        } else {
            S::serialize(payload, |event| self.post_raw(event, timeout))
        }
    }

    fn subscribe_raw<'a, S, F>(&self, callback: F) -> Result<EspSubscription<'a, T>, EspError>
    where
        S: EspEventSource,
        F: FnMut(EspEvent) + Send + 'a,
    {
        let mut handler_instance: esp_event_handler_instance_t = ptr::null_mut();

        let callback: Box<dyn FnMut(EspEvent) + Send + 'a> = Box::new(callback);
        let mut callback = Box::new(callback);

        let unsafe_callback = UnsafeCallback::from(&mut callback);

        if T::is_system() {
            esp!(unsafe {
                esp_event_handler_instance_register(
                    S::source().map(ffi::CStr::as_ptr).unwrap_or(ptr::null()),
                    S::event_id().unwrap_or(ESP_EVENT_ANY_ID),
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
                    S::source().map(ffi::CStr::as_ptr).unwrap_or(ptr::null()),
                    S::event_id().unwrap_or(ESP_EVENT_ANY_ID),
                    Some(EspSubscription::<User<T>>::handle),
                    unsafe_callback.as_ptr(),
                    &mut handler_instance as *mut _,
                )
            })?;
        }

        Ok(EspSubscription {
            event_loop_handle: EventLoopHandleRef::Strong(self.0.clone()),
            handler_instance,
            source: S::source(),
            event_id: S::event_id().unwrap_or(ESP_EVENT_ANY_ID),
            _callback: callback,
        })
    }

    fn post_raw(&self, data: &EspEventPostData, timeout: TickType_t) -> Result<bool, EspError> {
        let result = if T::is_system() {
            unsafe {
                esp_event_post(
                    data.source.as_ptr(),
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    timeout,
                )
            }
        } else {
            unsafe {
                let handle: &T = &self.0 .0;
                let user: &User<Background> = mem::transmute(handle);

                esp_event_post_to(
                    user.0,
                    data.source.as_ptr(),
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    timeout,
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
    fn isr_post_raw(&self, data: &EspEventPostData) -> Result<bool, EspError> {
        let mut higher_prio_task_woken: BaseType_t = Default::default();

        let result = if T::is_system() {
            unsafe {
                esp_event_isr_post(
                    data.source.as_ptr(),
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
                    data.source.as_ptr(),
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
}

impl<T> EspEventLoop<User<T>> {
    pub fn spin(&mut self, timeout: TickType_t) -> Result<(), EspError> {
        esp!(unsafe { esp_event_loop_run(self.0 .0 .0, timeout) })
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

pub struct Wait<T>
where
    T: EspEventLoopType,
{
    waitable: Arc<Waitable<()>>,
    _subscription: EspSubscription<'static, T>,
}

impl<T> Wait<T>
where
    T: EspEventLoopType,
{
    pub fn new<S>(event_loop: &EspEventLoop<T>) -> Result<Self, EspError>
    where
        S: EspEventSource,
    {
        let waitable: Arc<Waitable<()>> = Arc::new(Waitable::new(()));

        let s_waitable = waitable.clone();
        let subscription = event_loop.subscribe_raw::<S, _>(move |_| {
            s_waitable.cvar.notify_all();
        })?;

        Ok(Self {
            waitable,
            _subscription: subscription,
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
}

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
mod async_wait {
    use core::marker::PhantomData;
    use core::pin::pin;
    use core::time::Duration;

    extern crate alloc;
    use alloc::sync::Arc;

    use esp_idf_hal::task::asynch::Notification;

    use ::log::debug;

    use super::{EspEventDeserializer, EspEventLoop, EspEventLoopType, EspSubscription};
    use crate::sys::{esp, EspError, ESP_ERR_TIMEOUT};
    use crate::timer::{EspAsyncTimer, EspTimerService, Task};

    pub struct AsyncWait<D, T>
    where
        D: EspEventDeserializer,
        T: EspEventLoopType,
    {
        notification: Arc<Notification>,
        timer: EspAsyncTimer,
        _subscription: EspSubscription<'static, T>,
        _deserializer: PhantomData<fn() -> D>,
    }

    impl<D, T> AsyncWait<D, T>
    where
        D: EspEventDeserializer,
        T: EspEventLoopType + Send,
    {
        pub fn new(
            event_loop: &EspEventLoop<T>,
            timer_service: &EspTimerService<Task>,
        ) -> Result<Self, EspError> {
            let notification = Arc::new(Notification::new());

            Ok(Self {
                _subscription: {
                    let notification = notification.clone();
                    event_loop.subscribe::<D, _>(move |_| {
                        notification.notify_lsb();
                    })?
                },
                notification,
                timer: timer_service.timer_async()?,
                _deserializer: PhantomData,
            })
        }

        pub async fn wait_while<F: FnMut() -> Result<bool, EspError>>(
            &mut self,
            mut matcher: F,
            duration: Option<Duration>,
        ) -> Result<(), EspError> {
            let notification = &self.notification;

            let subscription_wait = pin!(async move {
                while matcher()? {
                    notification.wait().await;
                }

                Result::<(), EspError>::Ok(())
            });

            if let Some(duration) = duration {
                debug!("About to wait for duration {:?}", duration);

                let timer_wait = self.timer.after(duration);

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

                subscription_wait.await?;

                debug!("Waiting done - success");

                Ok(())
            }
        }
    }
}
