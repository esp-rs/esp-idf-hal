use core::fmt::{Debug, Display};
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::result::Result;
use core::time::Duration;

extern crate alloc;
use alloc::sync::Arc;

use ::log::*;

use embedded_svc::{event_bus, service};

use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::TickType;
use esp_idf_hal::mutex;

use esp_idf_sys::*;

use crate::private::cstr::RawCstrs;

pub type EspSystemEventLoop = EspEventLoop<System>;
pub type EspBackgroundEventLoop = EspEventLoop<User<Background>>;
pub type EspExplicitEventLoop = EspEventLoop<User<Explicit>>;
pub type EspPinnedEventLoop = EspEventLoop<User<Pinned>>;

#[derive(Debug)]
pub struct BackgroundConfiguration<'a> {
    pub queue_size: usize,
    pub task_name: &'a str,
    pub task_priority: u8,
    pub task_stack_size: usize,
    pub task_pin_to_core: Core,
}

impl<'a> Default for BackgroundConfiguration<'a> {
    fn default() -> Self {
        Self {
            queue_size: 8192,
            task_name: "(unknown)",
            task_priority: 0,
            task_stack_size: 3072,
            task_pin_to_core: Core::Core0,
        }
    }
}

impl<'a> From<&BackgroundConfiguration<'a>> for (esp_event_loop_args_t, RawCstrs) {
    fn from(conf: &BackgroundConfiguration<'a>) -> Self {
        let mut rcs = RawCstrs::new();

        let ela = esp_event_loop_args_t {
            queue_size: conf.queue_size as _,
            task_name: rcs.as_ptr(conf.task_name),
            task_priority: conf.task_priority as _,
            task_stack_size: conf.task_stack_size as _,
            task_core_id: conf.task_pin_to_core as _,
        };

        (ela, rcs)
    }
}

#[derive(Debug)]
pub struct Configuration {
    pub queue_size: usize,
}

impl Default for Configuration {
    fn default() -> Self {
        Self { queue_size: 8192 }
    }
}

impl From<&Configuration> for esp_event_loop_args_t {
    fn from(conf: &Configuration) -> Self {
        esp_event_loop_args_t {
            queue_size: conf.queue_size as _,
            ..Default::default()
        }
    }
}

static TAKEN: mutex::Mutex<bool> = mutex::Mutex::new(false);

#[derive(Clone)]
pub struct System;
#[derive(Clone)]
pub struct User<T>(esp_event_loop_handle_t, PhantomData<T>);
#[derive(Clone)]
pub struct Background;
#[derive(Clone)]
pub struct Explicit;
#[derive(Clone)]
pub struct Pinned;

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

pub trait EspEventSubscribeMetadata {
    fn metadata() -> (*const c_types::c_char, i32);
}

pub struct EspEventPostData {
    pub source: *const c_types::c_char,
    pub event_id: i32,
    pub payload: *const c_types::c_void,
    pub payload_len: usize,
}

pub struct EspEventFetchData {
    pub source: *const c_types::c_char,
    pub event_id: i32,
    pub payload: *const c_types::c_void,
}

struct UnsafeCallback(*mut Box<dyn FnMut(EspEventFetchData) + 'static>);

impl UnsafeCallback {
    fn from(boxed: &mut Box<Box<dyn FnMut(EspEventFetchData) + 'static>>) -> Self {
        Self(boxed.as_mut())
    }

    unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    unsafe fn call(&self, data: EspEventFetchData) {
        let reference = self.0.as_mut().unwrap();

        (reference)(data);
    }
}

pub struct EspSubscription<T>
where
    T: EspEventLoopType,
{
    event_loop_handle: Arc<EventLoopHandle<T>>,
    handler_instance: esp_event_handler_instance_t,
    source: *const c_types::c_char,
    event_id: i32,
    _callback: Box<Box<dyn FnMut(EspEventFetchData) + 'static>>,
}

impl<T> EspSubscription<T>
where
    T: EspEventLoopType,
{
    extern "C" fn handle(
        event_handler_arg: *mut c_types::c_void,
        event_base: esp_event_base_t,
        event_id: i32,
        event_data: *mut c_types::c_void,
    ) {
        let data = EspEventFetchData {
            source: event_base,
            event_id,
            payload: event_data,
        };

        unsafe {
            UnsafeCallback::from_ptr(event_handler_arg).call(data);
        }
    }
}

unsafe impl Send for EspSubscription<System> {}
unsafe impl Send for EspSubscription<User<Background>> {}
unsafe impl Send for EspSubscription<User<Explicit>> {}

impl<T> Drop for EspSubscription<T>
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
                let user: &User<T> = mem::transmute(&*self.event_loop_handle);

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

struct EventLoopHandle<T>(T)
where
    T: EspEventLoopType;

impl EventLoopHandle<System> {
    fn new() -> Result<Self, EspError> {
        let mut taken = TAKEN.lock();

        if *taken {
            esp!(ESP_ERR_INVALID_STATE as i32)?;
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
    fn new(conf: &BackgroundConfiguration) -> Result<Self, EspError> {
        let (nconf, _rcs) = conf.into();

        Self::new_internal(&nconf)
    }
}

impl EventLoopHandle<User<Explicit>> {
    fn new(conf: &Configuration) -> Result<Self, EspError> {
        Self::new_internal(&conf.into())
    }
}

impl EventLoopHandle<User<Pinned>> {
    fn new(conf: &Configuration) -> Result<Self, EspError> {
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
        } else {
            unsafe {
                let user: &User<T> = mem::transmute(&mut self.0);

                esp!(esp_event_loop_delete(user.0)).unwrap();
            }
        }

        info!("Dropped");
    }
}

pub struct EspEventLoop<T>(Arc<EventLoopHandle<T>>)
where
    T: EspEventLoopType;

impl<T> EspEventLoop<T>
where
    T: EspEventLoopType,
{
    pub fn subscribe<E>(
        &mut self,
        source: *const c_types::c_char,
        event_id: i32,
        mut callback: impl FnMut(EspEventFetchData) -> Result<(), E> + 'static,
    ) -> Result<EspSubscription<T>, EspError>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        let mut handler_instance: esp_event_handler_instance_t = ptr::null_mut();

        let callback: Box<dyn FnMut(EspEventFetchData) + 'static> =
            Box::new(move |data| callback(data).unwrap());
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
                let user: &User<T> = mem::transmute(&mut self.0);

                esp_event_handler_instance_register_with(
                    user.0,
                    source,
                    event_id,
                    Some(EspSubscription::<User<T>>::handle),
                    &unsafe_callback as *const _ as *mut _,
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

    pub fn post(&mut self, data: &EspEventPostData) -> Result<(), EspError> {
        // TODO: Handle the case where data size is < 4 as an optimization

        if T::is_system() {
            esp!(unsafe {
                esp_event_post(
                    data.source,
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    TickType_t::max_value(),
                )
            })
        } else {
            esp!(unsafe {
                let user: &User<T> = mem::transmute(&mut self.0);

                esp_event_post_to(
                    user.0,
                    data.source,
                    data.event_id,
                    data.payload as *const _ as *mut _,
                    data.payload_len as _,
                    TickType_t::max_value(),
                )
            })
        }
    }
}

impl EspEventLoop<System> {
    pub fn new() -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<System>::new()?)))
    }
}

impl EspEventLoop<User<Background>> {
    pub fn new(conf: &BackgroundConfiguration) -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<User<Background>>::new(
            conf,
        )?)))
    }
}

impl EspEventLoop<User<Explicit>> {
    pub fn new(conf: &Configuration) -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<User<Explicit>>::new(
            conf,
        )?)))
    }
}

impl EspEventLoop<User<Pinned>> {
    pub fn new(conf: &Configuration) -> Result<Self, EspError> {
        Ok(Self(Arc::new(EventLoopHandle::<User<Pinned>>::new(conf)?)))
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

unsafe impl Send for EspEventLoop<System> {}
unsafe impl Sync for EspEventLoop<System> {}

unsafe impl Send for EspEventLoop<User<Background>> {}
unsafe impl Sync for EspEventLoop<User<Background>> {}

unsafe impl Send for EspEventLoop<User<Explicit>> {}
unsafe impl Sync for EspEventLoop<User<Explicit>> {}

impl<T> service::Service for EspEventLoop<T>
where
    T: EspEventLoopType,
{
    type Error = EspError;
}

impl<T> event_bus::Spin for EspEventLoop<User<T>> {
    fn spin(&mut self, duration: Option<Duration>) -> Result<(), EspError> {
        esp!(unsafe { esp_event_loop_run(self.0 .0 .0, TickType::from(duration).0,) })
    }
}

impl<P, T> event_bus::Postbox<P> for EspEventLoop<T>
where
    P: Into<EspEventPostData>,
    T: EspEventLoopType,
{
    fn post(&mut self, payload: P) -> Result<(), Self::Error> {
        self.post(&payload.into())
    }
}

impl<P, T> event_bus::EventBus<P> for EspEventLoop<T>
where
    P: From<EspEventFetchData> + 'static,
    P: Into<EspEventPostData>,
    P: EspEventSubscribeMetadata,
    T: EspEventLoopType,
{
    type Subscription = EspSubscription<T>;

    type Postbox = Self;

    fn subscribe<E>(
        &mut self,
        mut callback: impl for<'b> FnMut(&'b P) -> Result<(), E> + Send + 'static,
    ) -> Result<Self::Subscription, EspError>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        let (source, event_id) = P::metadata();

        self.subscribe(source, event_id, move |data| callback(&data.into()))
    }

    fn postbox(&mut self) -> Result<Self::Postbox, Self::Error> {
        Ok(self.clone())
    }
}

impl<P> event_bus::PinnedEventBus<P> for EspEventLoop<User<Pinned>>
where
    P: From<EspEventFetchData> + 'static,
    P: Into<EspEventPostData>,
    P: EspEventSubscribeMetadata,
{
    type Subscription = EspSubscription<User<Pinned>>;

    type Postbox = Self;

    fn subscribe<E>(
        &mut self,
        mut callback: impl for<'b> FnMut(&'b P) -> Result<(), E> + 'static,
    ) -> Result<Self::Subscription, EspError>
    where
        E: Display + Debug + Send + Sync + 'static,
    {
        let (source, event_id) = P::metadata();

        self.subscribe(source, event_id, move |data| callback(&data.into()))
    }

    fn postbox(&mut self) -> Result<Self::Postbox, Self::Error> {
        Ok(self.clone())
    }
}
