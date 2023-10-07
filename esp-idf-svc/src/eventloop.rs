//! Event loop library

use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use core::marker::PhantomData;
use core::result::Result;
use core::time::Duration;
use core::{ffi, mem, ptr, slice};

extern crate alloc;
use alloc::boxed::Box;
use alloc::sync::Arc;

use ::log::*;

use embedded_svc::event_bus::{self, ErrorType};

use crate::hal::cpu::Core;
use crate::hal::delay::TickType;
use crate::hal::interrupt;

use crate::sys::*;

use crate::handle::RawHandle;
use crate::private::cstr::RawCstrs;
use crate::private::mutex;
use crate::private::waitable::Waitable;

pub use asyncify::*;

#[cfg(all(feature = "alloc", esp_idf_comp_esp_timer_enabled))]
pub use async_wait::*;

pub type EspSystemSubscription<'a> = EspSubscription<'a, System>;
pub type EspBackgroundSubscription<'a> = EspSubscription<'a, User<Background>>;
pub type EspExplicitSubscription<'a> = EspSubscription<'a, User<Explicit>>;

pub type EspSystemEventLoop = EspEventLoop<System>;
pub type EspBackgroundEventLoop = EspEventLoop<User<Background>>;
pub type EspExplicitEventLoop = EspEventLoop<User<Explicit>>;

pub type EspSystemPostbox = EspPostbox<EspEventLoop<System>>;
pub type EspBackgroundPostbox = EspPostbox<EspEventLoop<User<Background>>>;
pub type EspExplicitPostbox = EspPostbox<EspEventLoop<User<Explicit>>>;

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

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::wrap(mutex::RawMutex::new(), false);

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
    event_loop_handle: Arc<EventLoopHandle<T>>,
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
        } else {
            unsafe {
                let handle: &T = &self.event_loop_handle.0;
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
    pub fn subscribe_raw<'a, F>(
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
            event_loop_handle: self.0.clone(),
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

    pub fn subscribe<'a, P, F>(&self, mut callback: F) -> Result<EspSubscription<'a, T>, EspError>
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

    pub fn into_typed<M, P>(self) -> EspTypedEventLoop<M, P, Self> {
        EspTypedEventLoop::new(self)
    }

    pub fn as_typed<M, P>(&mut self) -> EspTypedEventLoop<M, P, &mut Self> {
        EspTypedEventLoop::new(self)
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

pub trait EspTypedEventSource {
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
    type Subscription<'a> = EspSubscription<'a, T> where Self: 'a;

    fn subscribe<'a, F>(&'a self, callback: F) -> Result<Self::Subscription<'a>, Self::Error>
    where
        F: FnMut(&P) + Send + 'a,
    {
        EspEventLoop::subscribe(self, callback)
    }
}

pub struct EspPostbox<T>(EspEventLoop<T>)
where
    T: EspEventLoopType;

unsafe impl<T> Send for EspPostbox<T> where T: EspEventLoopType {}
unsafe impl<T> Sync for EspPostbox<T> where T: EspEventLoopType {}

impl<T> Clone for EspPostbox<T>
where
    T: EspEventLoopType,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T> ErrorType for EspPostbox<T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<P, T> event_bus::Postbox<P> for EspPostbox<T>
where
    P: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, Self::Error> {
        self.0.post(payload, wait)
    }
}

impl<P, T> event_bus::PostboxProvider<P> for EspEventLoop<T>
where
    P: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    type Postbox<'a> = EspPostbox<T> where Self: 'a;

    fn postbox(&self) -> Result<Self::Postbox<'_>, Self::Error>
    where
        P: EspTypedEventSerializer<P>,
    {
        Ok(EspPostbox(self.clone()))
    }
}

pub struct EspTypedEventLoop<M, P, L> {
    untyped_event_loop: L,
    _serde: PhantomData<fn() -> M>,
    _payload: PhantomData<fn() -> P>,
}

impl<M, P, L> EspTypedEventLoop<M, P, L> {
    pub fn new(untyped_event_loop: L) -> Self {
        Self {
            untyped_event_loop,
            _serde: PhantomData,
            _payload: PhantomData,
        }
    }
}

impl<M, P, T> EspTypedEventLoop<M, P, EspEventLoop<T>>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    pub fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, EspError> {
        M::serialize(payload, |raw_event| {
            self.untyped_event_loop.post_raw(raw_event, wait)
        })
    }
}

impl<M, P, L> Clone for EspTypedEventLoop<M, P, L>
where
    L: Clone,
{
    fn clone(&self) -> Self {
        Self {
            untyped_event_loop: self.untyped_event_loop.clone(),
            _serde: PhantomData,
            _payload: PhantomData,
        }
    }
}

impl<M, P, L> ErrorType for EspTypedEventLoop<M, P, L>
where
    L: ErrorType,
{
    type Error = L::Error;
}

impl<M, P, T> event_bus::Postbox<P> for EspTypedEventLoop<M, P, EspEventLoop<T>>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, Self::Error> {
        EspTypedEventLoop::post(self, payload, wait)
    }
}

impl<'a, M, P, T> event_bus::Postbox<P> for EspTypedEventLoop<M, P, &'a EspEventLoop<T>>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, Self::Error> {
        M::serialize(payload, |raw_event| {
            self.untyped_event_loop.post_raw(raw_event, wait)
        })
    }
}

impl<M, P, T> event_bus::EventBus<P> for EspTypedEventLoop<M, P, EspEventLoop<T>>
where
    M: EspTypedEventDeserializer<P>,
    T: EspEventLoopType,
{
    type Subscription<'a> = EspSubscription<'a, T> where Self: 'a;

    fn subscribe<'a, F>(&'a self, mut callback: F) -> Result<Self::Subscription<'a>, EspError>
    where
        F: FnMut(&P) + Send + 'a,
    {
        self.untyped_event_loop.subscribe_raw(
            M::source(),
            M::event_id().unwrap_or(ESP_EVENT_ANY_ID),
            move |raw_event| M::deserialize(raw_event, &mut callback),
        )
    }
}

impl<'r, M, P, T> event_bus::EventBus<P> for EspTypedEventLoop<M, P, &'r EspEventLoop<T>>
where
    M: EspTypedEventDeserializer<P>,
    T: EspEventLoopType,
{
    type Subscription<'a> = EspSubscription<'a, T> where Self: 'a;

    fn subscribe<'a, F>(&'a self, mut callback: F) -> Result<Self::Subscription<'a>, EspError>
    where
        F: FnMut(&P) + Send + 'a,
    {
        self.untyped_event_loop.subscribe_raw(
            M::source(),
            M::event_id().unwrap_or(ESP_EVENT_ANY_ID),
            move |raw_event| M::deserialize(raw_event, &mut callback),
        )
    }
}

pub struct EspTypedPostbox<M, P, T>(EspTypedEventLoop<M, P, EspEventLoop<T>>)
where
    T: EspEventLoopType;

unsafe impl<M, P, T> Send for EspTypedPostbox<M, P, T> where T: EspEventLoopType {}
unsafe impl<M, P, T> Sync for EspTypedPostbox<M, P, T> where T: EspEventLoopType {}

impl<M, P, T> Clone for EspTypedPostbox<M, P, T>
where
    T: EspEventLoopType,
{
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<M, P, T> ErrorType for EspTypedPostbox<M, P, T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<M, P, T> event_bus::Postbox<P> for EspTypedPostbox<M, P, T>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    fn post(&self, payload: &P, wait: Option<Duration>) -> Result<bool, Self::Error> {
        self.0.post(payload, wait)
    }
}

impl<M, P, T> event_bus::PostboxProvider<P> for EspTypedEventLoop<M, P, EspEventLoop<T>>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    type Postbox<'a> = EspTypedPostbox<M, P, T> where Self: 'a;

    fn postbox(&self) -> Result<Self::Postbox<'_>, Self::Error> {
        Ok(EspTypedPostbox(Self::new(self.untyped_event_loop.clone())))
    }
}

impl<'e, M, P, T> event_bus::PostboxProvider<P> for EspTypedEventLoop<M, P, &'e EspEventLoop<T>>
where
    M: EspTypedEventSerializer<P>,
    T: EspEventLoopType,
{
    type Postbox<'a> = EspTypedPostbox<M, P, T> where Self: 'a;

    fn postbox(&self) -> Result<Self::Postbox<'_>, Self::Error> {
        Ok(EspTypedPostbox(EspTypedEventLoop::new(
            self.untyped_event_loop.clone(),
        )))
    }
}

mod asyncify {
    use embedded_svc::utils::asyncify::event_bus::AsyncEventBus;
    use embedded_svc::utils::asyncify::{Asyncify, UnblockingAsyncify};

    use crate::private::mutex::RawCondvar;

    impl<T> Asyncify for super::EspEventLoop<T>
    where
        T: super::EspEventLoopType,
    {
        type AsyncWrapper<S> = AsyncEventBus<(), RawCondvar, S>;
    }

    impl<T> UnblockingAsyncify for super::EspEventLoop<T>
    where
        T: super::EspEventLoopType,
    {
        type AsyncWrapper<U, S> = AsyncEventBus<U, RawCondvar, S>;
    }

    impl<M, P, L> Asyncify for super::EspTypedEventLoop<M, P, L> {
        type AsyncWrapper<S> = AsyncEventBus<(), RawCondvar, S>;
    }

    impl<M, P, L> UnblockingAsyncify for super::EspTypedEventLoop<M, P, L> {
        type AsyncWrapper<U, S> = AsyncEventBus<U, RawCondvar, S>;
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

impl<'a, E, T> Wait<'a, E, T>
where
    E: EspTypedEventDeserializer<E> + Debug,
    T: EspEventLoopType,
{
    pub fn new<F: FnMut(&E) -> bool + Send + 'a>(
        event_loop: &EspEventLoop<T>,
        mut waiter: F,
    ) -> Result<Self, EspError> {
        let waitable: Arc<Waitable<()>> = Arc::new(Waitable::new(()));

        let s_waitable = waitable.clone();
        let subscription = event_loop
            .subscribe(move |event: &E| Self::on_event(&s_waitable, event, &mut waiter))?;

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
    use core::fmt::Debug;
    use core::marker::PhantomData;
    use core::time::Duration;

    use embedded_svc::utils::asyncify::event_bus::{AsyncEventBus, AsyncSubscription};
    use embedded_svc::utils::asyncify::timer::{AsyncTimer, AsyncTimerService};

    use crate::sys::{esp, EspError, ESP_ERR_TIMEOUT};

    use log::debug;

    use crate::timer::{EspTaskTimerService, EspTimer};

    pub struct AsyncWait<'a, E, T>
    where
        E: super::EspTypedEventDeserializer<E> + Send,
        T: super::EspEventLoopType,
    {
        subscription:
            AsyncSubscription<crate::private::mutex::RawCondvar, E, super::EspSubscription<'a, T>>,
        timer: AsyncTimer<EspTimer<'a>>,
        _event: PhantomData<fn() -> E>,
    }

    impl<'a, E, T> AsyncWait<'a, E, T>
    where
        E: super::EspTypedEventDeserializer<E> + Debug + Send + Clone + 'static,
        T: super::EspEventLoopType + 'static,
    {
        pub fn new(
            event_loop: &'a AsyncEventBus<
                (),
                crate::private::mutex::RawCondvar,
                super::EspEventLoop<T>,
            >,
            timer_service: &'a AsyncTimerService<EspTaskTimerService>,
        ) -> Result<Self, EspError> {
            Ok(Self {
                subscription: event_loop.subscribe()?,
                timer: timer_service.timer()?,
                _event: PhantomData,
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
        async fn wait_sub<'s, EE, TT, F: FnMut() -> Result<bool, EspError>>(
            subscription: &mut AsyncSubscription<
                crate::private::mutex::RawCondvar,
                EE,
                super::EspSubscription<'s, TT>,
            >,
            mut matcher: F,
        ) -> Result<(), EspError>
        where
            EE: Send + Clone,
            TT: super::EspEventLoopType,
        {
            while matcher()? {
                subscription.recv().await;
            }

            Ok(())
        }
    }
}
